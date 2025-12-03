/**
 * @file system_stm32g0xx.c
 * @brief CMSIS Cortex-M0+ Device Peripheral Access Layer System Source File
 */

#include <stm32g071xx.h>

/**
 * @brief System Clock Frequency (Core Clock)
 */
uint32_t SystemCoreClock = 16000000UL; /*!< System Clock Frequency (Default HSI) */

/**
 * @brief Setup the microcontroller system
 * This is called from startup_stm32g071xx.s before main()
 */
void SystemInit(void) {
    // Nothing needed here - setup() will configure clocks
    // This function is required by CMSIS startup code
}

/**
 * @brief Update SystemCoreClock variable
 * Updates the SystemCoreClock with current core Clock retrieved from cpu registers.
 */
void SystemCoreClockUpdate(void) {
    uint32_t tmp;
    uint32_t pllvco;
    uint32_t pllr;
    uint32_t pllsource;
    uint32_t pllm;
    uint32_t plln;

    /* Get SYSCLK source */
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp) {
        case RCC_CFGR_SWS_HSISYS:  /* HSI used as system clock */
            SystemCoreClock = 16000000UL;
            break;

        case RCC_CFGR_SWS_HSE:  /* HSE used as system clock */
            SystemCoreClock = 8000000UL; // Assuming 8MHz HSE
            break;

        case RCC_CFGR_SWS_PLLRCLK:  /* PLL used as system clock */
            pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
            pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1UL;

            if (pllsource == RCC_PLLCFGR_PLLSRC_HSI) {
                /* HSI used as PLL clock source */
                pllvco = (16000000UL / pllm);
            } else if (pllsource == RCC_PLLCFGR_PLLSRC_HSE) {
                /* HSE used as PLL clock source */
                pllvco = (8000000UL / pllm);
            } else {
                pllvco = 0;
            }

            plln = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
            pllvco = pllvco * plln;
            pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1UL);
            SystemCoreClock = pllvco / pllr;
            break;

        case RCC_CFGR_SWS_LSI:  /* LSI used as system clock */
            SystemCoreClock = 32000UL;
            break;

        case RCC_CFGR_SWS_LSE:  /* LSE used as system clock */
            SystemCoreClock = 32768UL;
            break;

        default:
            SystemCoreClock = 16000000UL;
            break;
    }
}
