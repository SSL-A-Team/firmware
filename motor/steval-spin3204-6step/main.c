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
#include "uart.h"
#include "time.h"

__attribute__((optimize("O0")))
int main() {
    // Setups clocks
    setup();

    GPIOB->BSRR |= GPIO_BSRR_BR_8;

    const char* hello = "hello";
    int len = 6;

    // toggle J1-1
    while (true) {
        // GPIOB->BSRR |= GPIO_BSRR_BS_8;
        // //for (int32_t i = 0; i < 2000000L; i++);
        // wait_ms(50);
        // //GPIOB->BSRR &= ~(GPIO_BSRR_BS_8);
        // GPIOB->BSRR |= GPIO_BSRR_BR_8;
        // //for (int32_t i = 0; i < 2000000L; i++);
        // wait_ms(50);

        // GPIOB->BSRR |= GPIO_BSRR_BS_8;
        // wait_ms(1000);
        // GPIOB->BSRR |= GPIO_BSRR_BR_8;
        // wait_ms(1000);

        GPIOB->BSRR |= GPIO_BSRR_BS_8;
        uart_transmit_dma((uint8_t *) hello, len);
        GPIOB->BSRR |= GPIO_BSRR_BR_8;

        uart_wait_for_transmission();
        // 35us transmission time for 6 bytes

        wait_ms(500);
    }
}
