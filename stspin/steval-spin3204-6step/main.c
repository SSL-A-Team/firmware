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
#include "io_queue.h"
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
    bool params_return_packet_requested = false;

    SyncTimer_t vel_loop_timer;
    SyncTimer_t torque_loop_timer;
    time_sync_init(&vel_loop_timer, 10);
    time_sync_init(&torque_loop_timer, 1);

    IIRFilter_t encoder_filter;
    iir_filter_init(&encoder_filter, iir_filter_alpha_from_Tf(ENCODER_IIR_TF_MS, VELOCITY_LOOP_RATE_MS));

    MotorCommand_MotionType_t motion_control_type = VELOCITY;
    
    float r_motor_board = 0.0f;
    float u_vel_loop;
    float u_torque_loop;
    float cur_limit = 0.0f;

    const float r_rpm = 500.0f;
    r_motor_board = rpm_to_rad_s(r_rpm);

    PidConstants_t vel_pid_constants;
    pid_constants_initialize(&vel_pid_constants);
    Pid_t vel_pid;
    pid_initialize(&vel_pid, &vel_pid_constants);

    vel_pid_constants.kP = 1.0;
    // vel_pid_constants.kI = 0.0001;
    // vel_pid_constants.kD = 0.1;
    // vel_pid_constants.kI_max = 550.0;
    // vel_pid_constants.kI_min = -550.0;

    PidConstants_t torque_pid_constants;
    pid_constants_initialize(&torque_pid_constants);
    Pid_t torque_pid;
    pid_initialize(&torque_pid, &torque_pid_constants);

    // toggle J1-1
    while (true) {
        //GPIOB->BSRR |= GPIO_BSRR_BR_8;
        //GPIOB->BSRR |= GPIO_BSRR_BR_9;

#ifdef UART_ENABLED
        while (uart_can_read()) {
            MotorCommandPacket_t motor_command_packet;
            uint8_t bytes_moved = uart_read(&motor_command_packet, sizeof(MotorCommandPacket_t));
            if (bytes_moved != sizeof(MotorCommandPacket_t)) {
                // something went wrong, just purge all of the data
                uart_discard();
            }

            if (motor_command_packet.type == MCP_MOTION) {
                // we got a motion packet!

                if (motor_command_packet.motion.reset) {
                    // TODO handle hardware reset

                    while (true); // block, hardware reset flagged
                }

                r_motor_board = motor_command_packet.motion.setpoint;
            } else if (motor_command_packet.type == MCP_PARAMS) {
                // we got some params
                params_return_packet_requested = true;

                if (motor_command_packet.params.update_timestamp) {
                    time_set_epoch_seconds(motor_command_packet.params.update_timestamp);
                }

                if (motor_command_packet.params.update_vel_p) {
                    vel_pid_constants.kP = motor_command_packet.params.vel_p;
                }

                if (motor_command_packet.params.update_vel_i) {
                    vel_pid_constants.kI = motor_command_packet.params.vel_i;
                }

                if (motor_command_packet.params.update_vel_d) {
                    vel_pid_constants.kD = motor_command_packet.params.vel_d;
                }

                if (motor_command_packet.params.update_vel_i_max) {
                    vel_pid_constants.kI_max = motor_command_packet.params.vel_i_max;
                    vel_pid_constants.kI_min = -motor_command_packet.params.vel_i_max;
                }

                if (motor_command_packet.params.update_cur_p) {
                    torque_pid_constants.kP = motor_command_packet.params.cur_p;
                }

                if (motor_command_packet.params.update_cur_i) {
                    torque_pid_constants.kI = motor_command_packet.params.cur_i;
                }

                if (motor_command_packet.params.update_cur_d) {
                    torque_pid_constants.kD = motor_command_packet.params.cur_d;
                }

                if (motor_command_packet.params.update_cur_i_max) {
                    torque_pid_constants.kI_max = motor_command_packet.params.cur_i_max;
                    torque_pid_constants.kI_min = -motor_command_packet.params.cur_i_max;
                }

                if (motor_command_packet.params.update_cur_clamp) {
                    cur_limit = motor_command_packet.params.cur_clamp;
                }
            }
        }
#endif

        bool run_torque_loop = time_sync_ready_rst(&torque_loop_timer);
        bool run_vel_loop = time_sync_ready_rst(&vel_loop_timer);

        if (run_torque_loop) {
            float cur_measurement = 0.0f; // TODO: get current from dma
            // TODO: estimate torque from current
            // TODO: filter?

            float r;
            if (motion_control_type == TORQUE) {
                r = r_motor_board;
            } else {
                r = u_vel_loop;
            }

            float torque_setpoint = pid_calculate(&torque_pid, r, cur_measurement);
            // TODO: transform desired torque to current
            u_torque_loop = torque_setpoint;

            // torque control data
            response_packet.motion.current_setpoint = 0.0f;
            response_packet.motion.current_estimate = 0.0f;
            response_packet.motion.current_computed_error = 0.0f;
            response_packet.motion.current_computed_setpoint = 0.0f;
        }

        if (run_vel_loop) {
            int32_t enc_delta = quadenc_get_encoder_delta();
            float enc_vel_rad_s = quadenc_delta_to_w(enc_delta, VELOCITY_LOOP_RATE_MS);
            float enc_rad_s_filt = iir_filter_update(&encoder_filter, enc_vel_rad_s);
        
            float vel_setpoint = pid_calculate(&vel_pid, r_motor_board, enc_rad_s_filt);
            u_vel_loop = vel_setpoint / MOTOR_MAXIMUM_RAD_S;

            // velocity control data
            response_packet.motion.vel_setpoint = 0.0f;
            response_packet.motion.encoder_delta = enc_delta;
            response_packet.motion.vel_enc_estimate = enc_rad_s_filt;
            response_packet.motion.vel_hall_estimate = 0U;
            response_packet.motion.vel_computed_error = vel_pid.prev_err;
            response_packet.motion.vel_computed_setpoint = vel_setpoint;
        }


        if (run_torque_loop || run_vel_loop) {
            // set the motor duty cycle
            if (motion_control_type == OPEN_LOOP) {
                pwm6step_set_duty_cycle_f(r_motor_board);
            } else if (motion_control_type == VELOCITY) {
                pwm6step_set_duty_cycle_f(u_vel_loop);
            } else {
                pwm6step_set_duty_cycle_f(u_torque_loop);
            }



            // load system state for transmit

            // read error states
            const MotorErrors_t reported_motor_errors = pwm6step_get_motor_errors();

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
            response_packet.motion.timestamp = time_local_epoch_s();



            // transmit packets
#ifdef UART_ENABLED
            // GPIOB->BSRR |= GPIO_BSRR_BS_8;
            uart_wait_for_transmission();
            // takes ~270uS, mostly hardware DMA
            uart_transmit((uint8_t *) &response_packet, sizeof(MotorResponsePacket_t));
            // GPIOB->BSRR |= GPIO_BSRR_BR_8;
#endif

            if (params_return_packet_requested) {
                params_return_packet_requested = false;

                response_packet.type = MCP_PARAMS;

                response_packet.params.vel_p = vel_pid_constants.kP;
                response_packet.params.vel_i = vel_pid_constants.kI;
                response_packet.params.vel_d = vel_pid_constants.kD;
                response_packet.params.vel_i_max = vel_pid_constants.kI_max;
                response_packet.params.cur_p = torque_pid_constants.kP;
                response_packet.params.cur_i = torque_pid_constants.kI;
                response_packet.params.cur_d = torque_pid_constants.kD;
                response_packet.params.torque_i_max = torque_pid_constants.kI_max;
                response_packet.params.cur_clamp = (uint16_t) cur_limit;

#ifdef UART_ENABLED
                uart_transmit((uint8_t *) &response_packet, sizeof(MotorResponsePacket_t));
#endif
            }
        }

        // limit loop rate to smallest time step
        if (sync_systick()) {
            slipped_control_frame_count++;
        }
    }
}
