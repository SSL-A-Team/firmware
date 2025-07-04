/**
 * @file image_hash.c
 * @author Nicholas Witten
 * @brief Struct instantiation to hold embeddable git information
 * @version 0.1
 * @date 2022-05-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "image_hash.h"


static volatile WheelImgHash_t wheel_img_hash_struct = {
    "WheelImgHashMotr",
    {0},
};  // Motor bin hash will be injected upon build

const char* get_wheel_img_hash(void) {
    return wheel_img_hash_struct.wheel_img_hash;
}