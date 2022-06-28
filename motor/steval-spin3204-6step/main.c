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

#include "quadrature_encoder.h"
#include "setup.h"
#include "uart.h"
#include "time.h"
#include "6step.h"

__attribute__((optimize("O0")))
int main() {
    // Setups clocks
    setup();

    GPIOB->BSRR |= GPIO_BSRR_BR_8;
    GPIOB->BSRR |= GPIO_BSRR_BR_9;

    pwm6step_setup();

    quadenc_setup();
    quadenc_reset_encoder_delta();

    pwm6step_set_duty_cycle(16384);

    wait_ms(10000);
    
    pwm6step_estop();

    // const char* hello = "hello";
    // int len = 6;

    // toggle J1-1
    while (true) {
        // GPIOB->BSRR |= GPIO_BSRR_BS_8;
        // //for (int32_t i = 0; i < 2000000L; i++);
        // wait_ms(50);
        // //GPIOB->BSRR &= ~(GPIO_BSRR_BS_8);
        // GPIOB->BSRR |= GPIO_BSRR_BR_8;
        // //for (int32_t i = 0; i < 2000000L; i++);
        // wait_ms(50);

        //GPIOB->BSRR |= GPIO_BSRR_BS_8;
        //wait_ms(1000);
        //GPIOB->BSRR |= GPIO_BSRR_BR_8;
        //wait_ms(1000);

        // Send counter to UART
        // uint16_t count = quadenc_get_counter();
        // uint8_t lower_counter = (uint8_t)(count & 0xFFU);
        // uint8_t upper_counter = (uint8_t)((count & 0xFF00U) >> 0x8U);
        // uint8_t dir = 0x00;
        // if (TIM3->CR1 & TIM_CR1_DIR_Msk) {
        //     dir = 0xFF;
        // } else {
        //     dir = 0x00;
        // }
        //uint8_t data[2] = {lower_counter, upper_counter};
        // uint8_t data[5] = {0x3C, lower_counter, upper_counter, 0x3C, dir};
        // uart_transmit_dma(data, (uint16_t)5);
        // uart_wait_for_transmission();

        GPIOB->BSRR |= GPIO_BSRR_BS_8;
        wait_ms(1);
        GPIOB->BSRR |= GPIO_BSRR_BR_8;
        wait_ms(1);

        // int32_t delta = quadenc_get_encoder_delta();
        // for (int i = 15; i >= 0; i--) {
        //     GPIOB->BSRR |= GPIO_BSRR_BS_8;
        //     wait_ms(1);
        //     bool bitval = (((((int16_t) delta) >> i) & 0x1) != 0);
        //     if (!bitval) {
        //         GPIOB->BSRR |= GPIO_BSRR_BR_8;
        //     }
        //     wait_ms(1);
        //     GPIOB->BSRR |= GPIO_BSRR_BR_8;
        //     wait_ms(1);
        // }

        // GPIOB->BSRR |= GPIO_BSRR_BS_8;
        // uart_transmit_dma((uint8_t *) hello, len);
        // GPIOB->BSRR |= GPIO_BSRR_BR_8;

        // uart_wait_for_transmission();
        // 35us transmission time for 6 bytes

        // wait_ms(200);
        //sync_ms();
    }
}
