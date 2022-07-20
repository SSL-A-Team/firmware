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
#include <string.h>

#include "ateam-common-packets/include/stspin_packets.h"

#include "6step.h"
#include "current_sensing.h"
#include "main.h"
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

void _debug_value_manchester_32(int32_t val) {
    GPIOB->BSRR |= GPIO_BSRR_BR_8;
    wait_ms(1);

    for (int i = 31; i >= 0; i--) {
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

    MotorResponsePacket_t response_packet;
    memset(&response_packet, 0, sizeof(MotorResponsePacket_t));

    // while (true) {
    //     wait_ms(100);
    //     GPIOB->BSRR |= GPIO_BSRR_BS_8;
    //     currsen_read(&res);
    //     GPIOB->BSRR |= GPIO_BSRR_BR_8;

    //     wait_ms(1);

    //     _debug_value_manchester(res.pot);

    //     wait_ms(1);
    // }

    // pwm6step_set_duty_cycle(16384);

    // while(true) {
    //     currsen_read(&res);
    //     _debug_value_manchester(res.pot);
    //     wait_ms(10);
    // }

    const float u = 10.0f;
    float last_speed = 0.0f;
    float last_speed_2 = 0.0f;

    // toggle J1-1
    while (true) {
        GPIOB->BSRR |= GPIO_BSRR_BS_9;

        // read control state
        //  - encoders
        //  - hall
        //  - current
        // int32_t enc_count = quadenc_get_encoder_delta();
        // _debug_value_manchester_32(enc_count);
        // wait_ms(5);
        int32_t enc_delta = quadenc_get_encoder_delta();
        float enc_vel_rev_per_s = quadenc_delta_to_w(enc_delta, 0.001);


        float filt_speed = (enc_vel_rev_per_s + last_speed + last_speed_2)/3.0f;
        last_speed_2 = last_speed;
        last_speed = enc_vel_rev_per_s;
        
        // execute control loops
        //  - vel
        //  - toruqe
        
        const float kP = 0.8f;
        float vel_err = (u - filt_speed) / MAX_MOTOR_REV_PER_S * 65535.0f;
        float vel_setpoint = kP * vel_err;
        // float setpoint = u / MAX_MOTOR_REV_PER_S * 65535.0f;

        pwm6step_set_duty_cycle((int32_t) vel_setpoint);

        //////////////////////////////////
        //  Transmit Motion Data Frame  //
        //////////////////////////////////

        // read error states
        const MotorErrors_t reported_motor_errors = pwm6step_get_motor_errors();

        // transmit data frame

        response_packet.type = MCP_MOTION;
        response_packet.motion.master_error = false; // TODO update any error

        // bldc errors
        response_packet.motion.hall_power_error = reported_motor_errors.hall_power;
        response_packet.motion.hall_disconnected_error = reported_motor_errors.hall_disconnected;
        response_packet.motion.bldc_transition_error = reported_motor_errors.invalid_transitions;
        response_packet.motion.bldc_commutation_watchdog_error = reported_motor_errors.commutation_watchdog_timeout;

        // encoder errors
        response_packet.motion.enc_disconnected_error = false;
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
        response_packet.motion.encoder_delta = enc_delta;
        response_packet.motion.vel_enc_estimate = enc_vel_rev_per_s;
        response_packet.motion.vel_hall_estimate = 0U;
        response_packet.motion.vel_computed_error = vel_err;
        response_packet.motion.vel_computed_setpoint = vel_setpoint;

        response_packet.motion.current_estimate = 0.0f;
        response_packet.motion.current_computed_error = 0.0f;
        response_packet.motion.current_computed_setpoint = 0.0f;

        uart_transmit_dma((uint8_t *) &response_packet, sizeof(MotorResponsePacket_t));
        uart_wait_for_transmission();

        GPIOB->BSRR |= GPIO_BSRR_BR_9;

        if (sync_ms()) {
            slipped_control_frame_count++;
        }
    }
}
