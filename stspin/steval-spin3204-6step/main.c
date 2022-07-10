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

#include "stspin_packets.h"

#include "6step.h"
#include "current_sensing.h"
#include "quadrature_encoder.h"
#include "setup.h"
#include "time.h"
#include "uart.h"

static int slipped_control_frame_count = 0;

void _debug_value_manchester(uint16_t val) {
    GPIOB->BSRR |= GPIO_BSRR_BR_8;
    wait_ms(1);

    for (int i = 15; i >= 0; i--) {
        GPIOB->BSRR |= GPIO_BSRR_BS_8;
        wait_ms(1);
        if ((val >> i) & 0x1 != 0) {
            wait_ms(1);
            GPIOB->BSRR |= GPIO_BSRR_BR_8;
        } else {
            GPIOB->BSRR |= GPIO_BSRR_BR_8;
            wait_ms(1);
        }

        wait_ms(1);
    }
}

__attribute__((optimize("O0")))
int main() {
    // Setups clocks
    setup();

    GPIOB->BSRR |= GPIO_BSRR_BR_8;
    GPIOB->BSRR |= GPIO_BSRR_BR_9;

    pwm6step_setup();

    quadenc_setup();
    quadenc_reset_encoder_delta();

    ADC_Result_t res;
    currsen_setup();

    // while (true) {
    //     wait_ms(100);
    //     GPIOB->BSRR |= GPIO_BSRR_BS_8;
    //     currsen_read(&res);
    //     GPIOB->BSRR |= GPIO_BSRR_BR_8;

    //     wait_ms(1);

    //     _debug_value_manchester(res.pot);

    //     wait_ms(1);
    // }

    while(true) {
        currsen_read(&res);
        int32_t raw_set_point = ((int32_t) res.pot) - 511;
        int32_t scaled_set_point = raw_set_point * 127;
        pwm6step_set_duty_cycle(scaled_set_point);
        wait_ms(10);
    }



    // toggle J1-1
    while (true) {
        // read control state
        //  - encoders
        //  - hall
        //  - current

        // execute control loops
        //  - vel
        //  - toruqe

        //////////////////////////////////
        //  Transmit Motion Data Frame  //
        //////////////////////////////////

        // read error states
        const MotorErrors_t reported_motor_errors = pwm6step_get_motor_errors();

        // transmit data frame

        MotorResponsePacket_t response_packet;
        response_packet.type = MCP_MOTION;
        response_packet.motion.master_error = false; // TODO update any error

        // bldc errors
        response_packet.motion.hall_power_error = reported_motor_errors.hall_power;
        response_packet.motion.hall_disconnected_error = reported_motor_errors.hall_disconnected;
        response_packet.motion.bldc_transition_error = reported_motor_errors.invalid_transitions;
        response_packet.motion.bldc_commutation_watchdog_error = reported_motor_errors.commutation_watchdog_timeout;

        // encoder errors
        response_packet.motion.enc_disconnected = false;
        response_packet.motion.enc_decoding_error = false;

        // velocity checks
        response_packet.motion.hall_enc_vel_disagreement_error = false;

        // ADC errors
        response_packet.motion.overcurrent_error = false;
        response_packet.motion.undervoltage_error = false;
        response_packet.motion.overvoltage_error = false;

        // torque limiting
        response_packet.motion.torque_limited = false;

        // loop time
        response_packet.motion.control_loop_time_error = false;

        // timestamp
        response_packet.motion.timestamp = 0U;

        // motion data
        response_packet.motion.encoder_deltas = 0U;
        response_packet.motion.enc_vel_estimate = 0U;
        response_packet.motion.hall_vel_estimate = 0U;
        response_packet.motion.current_estimate = 0U;

        //uart_transmit_dma((uint8_t *) &response_packet, sizeof(MotorResponsePacket_t));

        if (sync_ms()) {
            slipped_control_frame_count++;
        }
    }
}
