
#include "debug.h"
#include "time.h"
#include <stm32f031x6.h>

void turn_on_red_led() {
    GPIOB->BSRR |= GPIO_BSRR_BS_6;
}

void turn_off_red_led() {
    GPIOB->BSRR |= GPIO_BSRR_BR_6;
}

void turn_on_yellow_led() {
    GPIOB->BSRR |= GPIO_BSRR_BS_8;
}

void turn_off_yellow_led() {
    GPIOB->BSRR |= GPIO_BSRR_BR_8;
}

void turn_on_green_led() {
    GPIOB->BSRR |= GPIO_BSRR_BS_7;
}

void turn_off_green_led() {
    GPIOB->BSRR |= GPIO_BSRR_BR_7;
}

void _debug_value_manchester_8(uint8_t val) {
    turn_off_yellow_led();
    wait_ms(1);

    for (int i = 8; i >= 0; i--) {
        turn_on_yellow_led();
        wait_ms(1);
        if (((val >> i) & 0x1) != 0) {
            wait_ms(1);
            turn_off_yellow_led();
        } else {
            turn_off_yellow_led();
            wait_ms(1);
        }

        wait_ms(1);
    }
}

void _debug_value_manchester(uint16_t val) {
    turn_off_yellow_led();
    wait_ms(1);

    for (int i = 15; i >= 0; i--) {
        turn_on_yellow_led();
        wait_ms(1);
        if (((val >> i) & 0x1) != 0) {
            wait_ms(1);
            turn_off_yellow_led();
        } else {
            turn_off_yellow_led();
            wait_ms(1);
        }

        wait_ms(1);
    }
}

void _debug_value_manchester_32(int32_t val) {
    turn_off_yellow_led();
    wait_ms(1);

    for (int i = 31; i >= 0; i--) {
        turn_on_yellow_led();
        wait_ms(1);
        if (((val >> i) & 0x1) != 0) {
            wait_ms(1);
            turn_off_yellow_led();
        } else {
            turn_off_yellow_led();
            wait_ms(1);
        }

        wait_ms(1);
    }
}