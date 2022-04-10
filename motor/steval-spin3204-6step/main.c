/**
 * @file main.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stm32f031x6.h>
#include <stdbool.h>
#include <stdint.h>

#include "setup.h"

int main() {
    // Setups clocks
    setup();

    // toggle J1-1
    while (true) {
        for (long int i = 0; i < 20000000; i++);
        GPIOB->ODR = GPIO_ODR_8;
        for (long int i = 0; i < 20000000; i++);
        GPIOB->ODR = 0;
    }
}