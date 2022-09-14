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

#include "ateam-common-packets/include/stspin.h"

#include "6step.h"
#include "current_sensing.h"
#include "iir.h"
#include "main.h"
#include "quadrature_encoder.h"
#include "pid.h"
#include "setup.h"
#include "system.h"
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


    SyncTimer_t vel_loop_timer;
    SyncTimer_t torque_loop_timer;
    time_sync_init(&vel_loop_timer, 10);
    time_sync_init(&torque_loop_timer, 1);

    IIRFilter_t encoder_filter;
    iir_filter_init(&encoder_filter, iir_filter_alpha_from_Tf(ENCODER_IIR_TF_MS, VELOCITY_LOOP_RATE_MS));

    IIRFilter_t dc_filter;
    iir_filter_init(&dc_filter, iir_filter_alpha_from_Tf(DC_IIR_TF_MS, VELOCITY_LOOP_RATE_MS));

    const float r_rpm = 500.0f;
    const float r_rad_s = rpm_to_rad_s(r_rpm);

    PidConstants_t vel_pid_constants;
    pid_constants_initialize(&vel_pid_constants);
    Pid_t vel_pid;
    pid_initialize(&vel_pid, &vel_pid_constants);

    vel_pid_constants.kP = 0.74;
    // vel_pid_constants.kI = 0.0001;
    // vel_pid_constants.kD = 0.1;
    // vel_pid_constants.kI_max = 550.0;
    // vel_pid_constants.kI_min = -550.0;

    // toggle J1-1
    while (true) {
        //GPIOB->BSRR |= GPIO_BSRR_BR_8;
        //GPIOB->BSRR |= GPIO_BSRR_BR_9;

#ifdef UART_ENABLED
        // TODO: receive commands
#endif

        bool run_torque_loop = time_sync_ready_rst(&torque_loop_timer);
        bool run_vel_loop = time_sync_ready_rst(&vel_loop_timer);

        if (run_torque_loop) {
            //GPIOB->BSRR |= GPIO_BSRR_BS_8;
        }

        if (run_vel_loop) {
            //GPIOB->BSRR |= GPIO_BSRR_BS_9;

            // read control state
            //  - encoders
            //  - hall
            //  - current
            // int32_t enc_count = quadenc_get_encoder_delta();
            // _debug_value_manchester_32(enc_count);
            // wait_ms(5);
            int32_t enc_delta = quadenc_get_encoder_delta();
            float enc_vel_rad_s = quadenc_delta_to_w(enc_delta, VELOCITY_LOOP_RATE_MS);

            float enc_rad_s_filt = iir_filter_update(&encoder_filter, enc_vel_rad_s);
            
            // float vel_err = (r_rad_s - enc_rad_s_filt);
            // float vel_setpoint = vel_kP * vel_err;

            float vel_setpoint = pid_calculate(&vel_pid, r_rad_s, enc_rad_s_filt);
            
            float dc_pct = vel_setpoint / MOTOR_MAXIMUM_RAD_S;

            float dc_pct_filt = iir_filter_update(&dc_filter, dc_pct);
            //pwm6step_set_duty_cycle_f(dc_pct_filt);
            pwm6step_set_duty_cycle_f(0.25f);
            //pwm6step_set_duty_cycle(-16000);


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
            response_packet.motion.vel_enc_estimate = enc_rad_s_filt;
            response_packet.motion.vel_hall_estimate = 0U;
            response_packet.motion.vel_computed_error = vel_pid.prev_err;
            response_packet.motion.vel_computed_setpoint = vel_setpoint;

            response_packet.motion.current_estimate = 0.0f;
            response_packet.motion.current_computed_error = 0.0f;
            response_packet.motion.current_computed_setpoint = 0.0f;

#ifdef UART_ENABLED
            uart_transmit_dma((uint8_t *) &response_packet, sizeof(MotorResponsePacket_t));
            uart_wait_for_transmission();
#endif
        }

        //GPIOB->BSRR |= GPIO_BSRR_BR_8;
        //GPIOB->BSRR |= GPIO_BSRR_BR_9;

        if (sync_systick()) {
            slipped_control_frame_count++;
        }
    }
}
