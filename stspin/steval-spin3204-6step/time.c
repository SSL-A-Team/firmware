/**
 * @file time.c
 * @author Will Stuckey
 * @brief defines all functions related to time keeping
 * @version 0.1
 * @date 2022-04-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stm32f031x6.h>

#include <stdbool.h>

#include "time.h"

/**
 * @brief blocks for a number of milliseconds
 * 
 */
__attribute__((optimize("O0")))
void wait_ms(uint32_t time_ms) {
    for (uint32_t count = 0; count < time_ms; count++) {
        // reading SysTick->CTRL clears the COUNTFLAG bit, so we don't have to do it manually
        // the SysTick timer will continue running and we assume it was set to begin with
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    }
}

/**
 * @brief blocks until SysTick fires
 * 
 * @return true if the tick has already elapsed and no waiting occurred
 *          can be used to determine if a core process is not meeting timing slack requirements
 */
__attribute__((optimize("O0")))
bool sync_ms() {
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0) {
        return true;
    }

    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    return false;
}