/**
 * @file git.h
 * @author Nicholas Witten
 * @brief 
 * @version 0.1
 * @date 2025-01-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include <stdint.h>

typedef struct {
    uint32_t git_struct_id;
    uint32_t git_dirty;  // Only requires 1 bit, but using 32 bit to keep things word aligned
    uint32_t git_hash;
} GitStatus_t;

uint32_t get_git_hash(void);
uint32_t get_git_dirty(void);