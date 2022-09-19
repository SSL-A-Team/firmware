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

#include <stdbool.h>

#include <stm32f031x6.h>

#include "system.h"
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
bool sync_systick() {
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0) {
        return true;
    }

    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    return false;
}


volatile uint32_t epoch_seconds = 0;
volatile uint32_t uptime_ticks_since_epoch = 0;
volatile uint32_t uptime_ticks = 0;

void SysTick_Handler() {
    uptime_ticks++;
    uptime_ticks_since_epoch++;
}

uint32_t time_get_uptime_ms() {
    return (uptime_ticks * 1000) / SYSTICK_PER_S;
}

void time_set_epoch_seconds(uint32_t epoch_sec) {
    epoch_seconds = epoch_sec;
    uptime_ticks_since_epoch = 0;
}

uint32_t time_local_epoch_s() {
    return epoch_seconds + (uptime_ticks_since_epoch / SYSTICK_PER_S);
}

/**
 * @brief 
 * 
 * @param time_sync 
 * @param ticks 
 */
void time_sync_init(SyncTimer_t *time_sync, uint32_t ticks) {
    time_sync->sync_time_ticks = ticks;
    time_sync->prev_time_ticks = uptime_ticks;
}

void time_sync_reset(SyncTimer_t *time_sync) {
    time_sync->prev_time_ticks = uptime_ticks;
}

bool time_sync_ready(SyncTimer_t *time_sync) {
    return (uptime_ticks - time_sync->prev_time_ticks >= time_sync->sync_time_ticks);
}

bool time_sync_ready_rst(SyncTimer_t *time_sync) {
    if (time_sync_ready(time_sync)) {
        time_sync_reset(time_sync);
        return true;
    }

    return false;
}

bool time_sync_block(SyncTimer_t *time_sync) {
    while (!time_sync_ready(time_sync)) {
        asm volatile("nop");
    }

    return (uptime_ticks - time_sync->prev_time_ticks > time_sync->sync_time_ticks);
}

bool time_sync_block_rst(SyncTimer_t *time_sync) {
    bool res = time_sync_block(time_sync);
    time_sync_reset(time_sync);
    return res;
}


