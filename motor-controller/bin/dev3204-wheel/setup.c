/**
 * @file setup.c
 * @author Will Stuckey
 * @brief setup function for the STSPIN32
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 * Relevant Documents:
 * stm32f031x4/stm32f031x6 reference "stspin reference"
 *      ARM-Based 32-bit MCU with up to 32 Kbyte Flash, 9 timers, ADC and communication interfaces, 2.0 - 3.6V, 106 pages
 * stm32f0x1/stm32f0x2/stm32f0x8 reference "m0 reference"
 *      1004 pages 
 * 
 */

#include <stm32f031x6.h>

#include "setup.h"
#include "system.h"

void NMI_Handler() {
    NVIC_SystemReset();
}

void HardFault_Handler() {
    NVIC_SystemReset();
}

/**
 * @brief Setup the clock tree
 * 
 * Clock tree diagram: pg 14 / 106 of the "stspin reference"
 * Register info: pg 108+ / 1004 of the "m0 reference"
 * 
 */
__attribute__((optimize("O0")))
__attribute__((always_inline))
inline void setup_clocks() {
    // start HSI
    RCC->CR |= RCC_CR_HSION;
    RCC->CFGR = 0x00000000;
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_HSEON);
	RCC->CFGR2 = 0;
	RCC->CFGR3 = 0;
	RCC->CIR = 0;

    // wait for HSI to be stable
    while ((RCC->CR & RCC_CR_HSIRDY) == 0) {
        asm volatile("nop");
    }

    // PLL src -> HSI, follows a hard coded /2 div -> 4MHz
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;
    // PLL mul 12 -> 4 * 12 -> 48 MHz which is the sysclk max
    RCC->CFGR |= RCC_CFGR_PLLMUL12;
    // turn on the PLL
    RCC->CR |= RCC_CR_PLLON;
    // wait for PLL stability (48 MHz)
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
        asm volatile("nop");
    }

    // set flash latency for 48MHz sysclk before we switch the source over
    FLASH->ACR |= FLASH_ACR_LATENCY;
    // enable the prefetch buffer
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    // wait for confirmation (they do this in the HAL)
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY) {
        asm volatile("nop");
    }

    // set sysclk src PLL output (48 MHz)
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    // wait for the system to confirm the source has switched over
    while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL) {
        asm volatile("nop");
    }

    // Set AHB clock freq to sysclk (48MHz)
    // HPRE -> 1
    RCC->CFGR |= RCC_CFGR_HPRE_0;

    // Set APB clock freq to AHB clk (48MHz)
    // PPRE div -> 1
    RCC->CFGR |= RCC_CFGR_PPRE_0;
    // PPRE mul -> 1
    // ??? dont see this cfg
    // USART1SW src -> PCLK
    RCC->CFGR3 |= RCC_CFGR3_USART1SW_PCLK;

    // LSI enable
    RCC->CSR |= RCC_CSR_LSION;
    // wait for LSI stability (40KHz)
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0) {
        asm volatile("nop");
    }

    // Set the RTC to use the internal low speed oscillator
    // RTCSEL -> LSI (40Khz)
    RCC->BDCR |= RCC_BDCR_RTCSEL_LSI;

    // HSI14 enable (for ADC)
    // on by default?? 

    // Setup TIM14 sysclk source 
    RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;
    // MCOPRE -> 1
    RCC->CFGR |= RCC_CFGR_MCOPRE_DIV1;

    // https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-timer--systick/systick-reload-value-register
    // set source to sysclk (48MHz) 
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    // timer counts down to fire approximately every 1ms 
    SysTick->LOAD = (F_SYS_CLK_HZ / 1000UL);
    // current value set to 0, e.g. no trigger event
    SysTick->VAL = (F_SYS_CLK_HZ / 1000UL) - 1;
    // enable the interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    // enable the counter
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief sets up base IO
 * 
 */
__attribute__((optimize("O0")))
inline void setup_io() {
    // turn on IO bank clock domains
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

    // turn on DMA clock domain
    RCC->AHBENR |= RCC_AHBENR_DMAEN;

    // turn on CRC clock domain
    RCC->AHBENR |= RCC_AHBENR_CRCEN;

    // enable timer 3 source on the peripheral bus
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // enable timer 2 source on the peripherial bus
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // enable timer 1 source on the peripherial bus
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // enable timer 16
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

    // enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    GPIOB->MODER |= GPIO_MODER_MODER8_0;
    GPIOB->MODER |= GPIO_MODER_MODER9_0;
    //GPIOB->OTYPER &= ~GPIO_OTYPER_OT_8;
}

/**
 * @brief setups UART IO, UART, and UART DMA
 * 
 * TODO move to uart.c
 */
__attribute__((optimize("O0")))
void setup_uart() {
    /////////////////
    //  Pin Setup  //
    /////////////////

    // PA14 USART1_TX
    // PA15 USART1_RX

    // clear PA13 and PA14 mode which start in AF for SWD
    GPIOA->MODER &= ~(GPIO_MODER_MODER13_0 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER14_1);
    // clear PA13 speed mode which starts at 50Mhz (0b11) for SWD
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR13_0 | GPIO_OSPEEDR_OSPEEDR13_1);
    // clear PA13 and PA14 pull up status
    GPIOA->PUPDR &= !(GPIO_PUPDR_PUPDR13_0 | GPIO_PUPDR_PUPDR13_1 | GPIO_PUPDR_PUPDR14_0 | GPIO_PUPDR_PUPDR14_1);

    // configure PA14 and PA15 AF mode
    GPIOA->MODER |= (GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);
    // configure PA14 and PA15 pin speed to 10Mhz
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR14_0 | GPIO_OSPEEDR_OSPEEDR15_0); 
    // configure PA14 and PA15 pin pullup to UP
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR14_0 | GPIO_PUPDR_PUPDR15_0);
    // configure PA14 and PA15 alternate function
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL14_Pos);
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL15_Pos);

    // enable bus clock to the UART
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /////////////////////////////
    //  UART Peripheral Setup  //
    /////////////////////////////

    // make sure USART1 is disabled
    USART1->CR1 &= ~(USART_CR1_UE);
    // 9-bits with parity (PCE) insert parity bit at the 9th bit
    // defaults to even parity
	USART1->CR1 |= (USART_CR1_M | USART_CR1_PCE);
    // enable transmision
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    // we don't need anything here
	USART1->CR2 = 0;
    // enable DMA for transmission and receive
	USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    // set baud rate
	USART1->BRR = 0x18; // => 2 Mbaud/s

    // enable the module
	USART1->CR1 |= USART_CR1_UE;

    // Enable idle line interrupt
    USART1->ICR |= USART_ICR_IDLECF;
    // USART1->CR1 |= USART_CR1_IDLEIE;

    //////////////////////////////////////
    //  Transmission DMA Channel Setup  //
    //////////////////////////////////////

	// medium priority, memory increment, memory to peripheral
	DMA1_Channel2->CCR = DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_DIR;
    // clear buffer base addr
	DMA1_Channel2->CMAR = 0; // transmit buffer base addr, set at transmission time
    // clear transmission length
    DMA1_Channel2->CNDTR = 0; // transmit length, set at transmission time
    // set destination address as UART periperal transmission shift register
	DMA1_Channel2->CPAR = (uint32_t) &USART1->TDR; // USART1 data transmit register address
    // enable the transfer complete (TCIE) and transfer error (TEIE) interrupts
    DMA1_Channel2->CCR |= (DMA_CCR_TEIE | DMA_CCR_TCIE);
    // clear channel 2, global IF, transfer error IF, half-transfer IF, and transfer complete IF
    DMA1->IFCR |= DMA_IFCR_CGIF2;

    /////////////////////////////////
    //  Receive DMA Channel Setup  //
    /////////////////////////////////

	// USART1_RX, low priority, memory increment, peripheral to memory
	// DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_CIRC;
	DMA1_Channel3->CCR = DMA_CCR_MINC;
    DMA1_Channel3->CCR |= (0x3U << DMA_CCR_PL_Pos);
	DMA1_Channel3->CMAR = (uint32_t) 0 ;
	DMA1_Channel3->CPAR = (uint32_t) &USART1->RDR;
	DMA1_Channel3->CNDTR = 0;
    // DMA1_Channel3->CCR |= (DMA_CCR_TEIE | DMA_CCR_TCIE);
    DMA1->IFCR |= DMA_IFCR_CGIF3;

    /////////////////
    //  DMA Setup  //
    /////////////////

    NVIC_SetPriority(USART1_IRQn, 10);
    NVIC_EnableIRQ(USART1_IRQn);

    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 10);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/**
 * @brief performs startup system configuration
 * 
 */
__attribute__((optimize("O0")))
//__attribute__((always_inline))
void setup() {
    setup_clocks();

    setup_io();
#ifdef UART_ENABLED
    setup_uart();
#endif
}