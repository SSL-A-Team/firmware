/**
 * @file image_hash.h
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

/// Stores the firmware image hash, the value is injected into the binary after compilation
typedef struct ImgHash {
    char img_hash_magic[16];
    char img_hash[16];
} ImgHash_t;