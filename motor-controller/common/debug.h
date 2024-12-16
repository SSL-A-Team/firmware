#pragma once

#include <stdint.h>

void turn_on_red_led();
void turn_off_red_led();
void turn_on_yellow_led();
void turn_off_yellow_led();
void turn_on_green_led();
void turn_off_green_led();

void _debug_value_manchester_8(uint8_t val);
void _debug_value_manchester(uint16_t val);
void _debug_value_manchester_32(int32_t val);