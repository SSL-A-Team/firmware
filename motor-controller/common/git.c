/**
 * @file git.c
 * @author Nicholas Witten
 * @brief Struct instantiation to hold embeddable git information
 * @version 0.1
 * @date 2022-05-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "git.h"


static volatile GitStatus_t git_status = {0xAABBCCDD, 0, 0};  // Git hash and dirty will be embedded upon build


uint32_t get_git_hash(void) {
    return git_status.git_hash;
}

uint32_t get_git_dirty(void) {
    return git_status.git_dirty;
}

// Keeps compiler from optimizing this field out
uint32_t get_git_struct_id(void) {
    return git_status.git_struct_id;
}