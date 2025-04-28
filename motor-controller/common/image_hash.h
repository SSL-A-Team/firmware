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

typedef struct WheelImgHash {
    char wheel_img_hash_magic[16];
    char wheel_img_hash[16];
} WheelImgHash_t;

const char* get_wheel_img_hash(void);