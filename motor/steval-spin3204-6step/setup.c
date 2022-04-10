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
    // confirm reset values
    RCC->CFGR = 0;
    RCC->CFGR2 = 0;
    RCC->CFGR3 = 0;
    RCC->CIR = 0;

    // start HSI
    RCC->CR |= RCC_CR_HSION;
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
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);

    // set flash latency for 48MHz sysclk before we switch the source over
    FLASH->ACR |= FLASH_ACR_LATENCY;
    // enable the prefetch buffer
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    // wait for confirmation
    while ((FLASH->ACR & FLASH_ACR_PRFTBS) == 0) {
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
}

__attribute__((optimize("O0")))
__attribute__((always_inline))
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

    // reset io banks
    RCC->AHBRSTR |= (RCC_AHBRSTR_GPIOARST | RCC_AHBRSTR_GPIOBRST | RCC_AHBRSTR_GPIOCRST | RCC_AHBRSTR_GPIOFRST);

    GPIOB->MODER |= GPIO_MODER_MODER8_1;
    //GPIOB->OTYPER &= ~GPIO_OTYPER_OT_8;
}

__attribute__((optimize("O0")))
void setup() {
    setup_clocks();

    setup_io();
}