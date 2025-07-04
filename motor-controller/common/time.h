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

#pragma once

#include <stdbool.h>

#include <stm32f031x6.h>

/**
 * @brief blocks for a number of milliseconds. Assumes SysTick is already running
 * 
 * @param time_ms time, in milliseconds, to block
 */
void wait_ms(uint32_t time_ms);

uint32_t time_get_uptime_ms();
void time_set_epoch_seconds(uint32_t epoch_sec);
uint32_t time_local_epoch_s();

/**
 * @brief 
 * 
 */
bool sync_systick();

typedef struct SyncTimer {
    uint32_t sync_time_ticks;
    uint32_t prev_time_ticks;
} SyncTimer_t;

/**
 * @brief 
 * 
 * @param time_sync 
 * @param ticks 
 */
void time_sync_init(SyncTimer_t *time_sync, uint32_t ticks);

/**
 * @brief 
 * 
 * @param time_sync 
 */
void time_sync_reset(SyncTimer_t *time_sync);

/**
 * @brief 
 * 
 * @param time_sync 
 * @return true 
 * @return false 
 */
bool time_sync_ready(SyncTimer_t *time_sync);

/**
 * @brief 
 * 
 * @param time_sync 
 * @return true 
 * @return false 
 */
bool time_sync_ready_rst(SyncTimer_t *time_sync);

/**
 * @brief 
 * 
 * @param time_sync 
 * @return true 
 * @return false 
 */
bool time_sync_block(SyncTimer_t *time_sync);

/**
 * @brief 
 * 
 * @param time_sync 
 * @return true 
 * @return false 
 */
bool time_sync_block_rst(SyncTimer_t *time_sync);