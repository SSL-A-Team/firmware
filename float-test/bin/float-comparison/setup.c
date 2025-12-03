/**
 * @file setup.c
 * @brief System setup for float-comparison test on STM32G071RB
 */

#include <stm32g071xx.h>
#include "setup.h"

/**
 * @brief Configure system clocks to 64MHz using HSI + PLL
 *
 * STM32G071RB supports up to 64MHz system clock
 * HSI = 16 MHz
 * PLL: HSI / M * N / R = 16 / 1 * 8 / 2 = 64 MHz
 */
void setup(void) {
    // Enable HSI (16MHz internal RC oscillator)
    RCC->CR |= RCC_CR_HSION;

    // Wait for HSI to be ready
    while ((RCC->CR & RCC_CR_HSIRDY) == 0) {
        asm volatile("nop");
    }

    // Configure Flash latency for 64MHz (2 wait states for 2.7-3.6V)
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    // Configure PLL:
    // - Source: HSI (16 MHz)
    // - M divider: 1 (VCO input = 16 MHz)
    // - N multiplier: 8 (VCO output = 128 MHz)
    // - R divider: 2 (System clock = 64 MHz)
    RCC->PLLCFGR = 0; // Clear PLL configuration
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;        // PLL source = HSI
    RCC->PLLCFGR |= (1 - 1) << RCC_PLLCFGR_PLLM_Pos; // PLLM = 1
    RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLN_Pos;     // PLLN = 8
    RCC->PLLCFGR |= (2 - 1) << RCC_PLLCFGR_PLLR_Pos; // PLLR = 2
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;            // Enable PLLR output

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;

    // Wait for PLL to be ready
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
        asm volatile("nop");
    }

    // Select PLL as system clock
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLLRCLK;

    // Wait for PLL to be used as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLLRCLK) {
        asm volatile("nop");
    }
}
