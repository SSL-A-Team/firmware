/**
 * @file time.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdbool.h>

#include <stm32f031x6.h>

/**
 * @brief blocks for a number of milliseconds. Assumes SysTick is already running
 * 
 * @param time_ms time, in milliseconds, to block
 */
void wait_ms(uint32_t time_ms);

/**
 * @brief 
 * 
 */
bool sync_ms();