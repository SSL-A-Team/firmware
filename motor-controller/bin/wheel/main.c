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
#include "conversions.h"
#include "current_sensing.h"
#include "iir.h"
#include "io_queue.h"
#include "main.h"
#include "motor_model.h"
#include "quadrature_encoder.h"
#include "pid.h"
#include "setup.h"
#include "system.h"
#include "time.h"
#include "uart.h"

static int slipped_control_frame_count = 0;

__attribute__((optimize("O0")))
int main() {
    uint32_t rcc_csr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;

    // turn off LEDs
    GPIOB->BSRR |= GPIO_BSRR_BR_6;
    GPIOB->BSRR |= GPIO_BSRR_BR_7;

    // turn on Red LED
    GPIOB->BSRR |= GPIO_BSRR_BS_6;

    // Setups clocks
    setup();

    // start watchdog
    IWDG->KR = 0x0000CCCC; // enable the module
    IWDG->KR = 0x00005555; // enable register writes
    IWDG->PR = 0x4; // set prescaler to 64, 40kHz -> 625Hz, 1.6ms per tick
    IWDG->RLR = 5; // count to 10 ticks, 16ms then trigger a system reset
    while (IWDG->SR) {} // wait for value to take
    IWDG->KR = 0x0000AAAA; // feed the watchdog

    GPIOB->BSRR |= GPIO_BSRR_BR_6;
    GPIOB->BSRR |= GPIO_BSRR_BR_7;

    GPIOB->BSRR |= GPIO_BSRR_BS_8;

    uint32_t ticks_since_last_command_packet = 0;
    bool telemetry_enabled = false;

    // initialize current sensing setup
    ADC_Result_t res;
    currsen_setup(ADC_MODE, &res, ADC_NUM_CHANNELS, ADC_CH_MASK, ADC_SR_MASK);

    // initialize motor driver
    pwm6step_setup();
    pwm6step_set_duty_cycle_f(0.0f);

    // enable ADC hardware trigger (tied to 6step timer)
    currsen_enable_ht();

    // setup encoder
    quadenc_setup();
    quadenc_reset_encoder_delta();

    // pwm6step_set_duty_cycle_f(0.05f);

    // while (1) {
    //     IWDG->KR = 0x0000AAAA; // feed the watchdog
    // }

    MotorCommandPacket motor_command_packet;
    memset(&motor_command_packet, 0, sizeof(MotorCommandPacket));

    MotorResponsePacket response_packet;
    memset(&response_packet, 0, sizeof(MotorResponsePacket));

    response_packet.data.motion.reset_watchdog_independent = rcc_csr & RCC_CSR_IWDGRSTF != 0;
    response_packet.data.motion.reset_watchdog_window = rcc_csr & RCC_CSR_WWDGRSTF != 0;
    response_packet.data.motion.reset_low_power = rcc_csr & RCC_CSR_LPWRRSTF != 0;
    response_packet.data.motion.reset_software = rcc_csr & RCC_CSR_SFTRSTF != 0;
    response_packet.data.motion.reset_pin = rcc_csr & RCC_CSR_PINRSTF != 0;

    bool params_return_packet_requested = false;

    // setup the loop rate regulators
    SyncTimer_t vel_loop_timer;
    SyncTimer_t torque_loop_timer;
    time_sync_init(&vel_loop_timer, 10);
    time_sync_init(&torque_loop_timer, 1);

    // setup the velocity filter
    IIRFilter_t encoder_filter;
    iir_filter_init(&encoder_filter, iir_filter_alpha_from_Tf(ENCODER_IIR_TF_MS, VELOCITY_LOOP_RATE_MS));

    IIRFilter_t torque_filter;
    iir_filter_init(&torque_filter, iir_filter_alpha_from_Tf(TORQUE_IIR_TF_MS, TORQUE_LOOP_RATE_MS));

    // set the default command mode to open loop (no PID)
    MotorCommand_MotionType motion_control_type = OPEN_LOOP;
    
    // define the control points the loops use to interact
    float r_motor_board = 0.0f;
    float u_vel_loop = 0.0f;
    float u_torque_loop = 0.0f;
    float cur_limit = 0.0f;

    // recovered velocities (helps torque recover direction)
    float enc_vel_rads = 0.0f;
    float enc_rad_s_filt = 0.0f;

    // initialize the motor model for the Nanotec DF45 50W (this is our drive motor)
    MotorModel_t df45_model;
    mm_initialize(&df45_model);
    df45_model.max_vel_rads = DF45_MAX_MOTOR_RAD_PER_S;
    df45_model.enc_ticks_per_rev = DF45_ENC_TICKS_PER_REV;
    df45_model.rated_current = DF45_RATED_CURRENT;
    df45_model.voltage_to_current_linear_map_m = CS_AMP_NETWORK_V_TO_I_LINEAR_M;
    df45_model.voltage_to_current_linear_map_b = CS_AMP_NETWORK_V_TO_I_LINEAR_B;
    df45_model.torque_to_current_linear_model_m = DF45_TORQUE_TO_CURRENT_LINEAR_M;
    df45_model.torque_to_current_linear_model_b = DF45_TORQUE_TO_CURRENT_LINEAR_B;
    df45_model.current_to_torque_linear_model_m = DF45_CURRENT_TO_TORQUE_LINEAR_M;
    df45_model.current_to_torque_linear_model_b = DF45_CURRENT_TO_TORQUE_LINEAR_B;

    // setup the velocity PID
    PidConstants_t vel_pid_constants;
    pid_constants_initialize(&vel_pid_constants);
    Pid_t vel_pid;
    pid_initialize(&vel_pid, &vel_pid_constants);

    vel_pid_constants.kP = 1.0f;
    // vel_pid_constants.kI = 0.0001;
    // vel_pid_constants.kD = 0.1;
    // vel_pid_constants.kI_max = 550.0;
    // vel_pid_constants.kI_min = -550.0;

    // setup the torque PID
    PidConstants_t torque_pid_constants;
    pid_constants_initialize(&torque_pid_constants);
    Pid_t torque_pid;
    pid_initialize(&torque_pid, &torque_pid_constants);

    torque_pid_constants.kP = 1.0f;

    // turn off LEDs
    GPIOB->BSRR |= GPIO_BSRR_BR_6;
    GPIOB->BSRR |= GPIO_BSRR_BS_7;

    // toggle J1-1
    while (true) {
        IWDG->KR = 0x0000AAAA; // feed the watchdog

        //GPIOB->BSRR |= GPIO_BSRR_BR_8;
        //GPIOB->BSRR |= GPIO_BSRR_BR_9;

        // watchdog on receiving a command packet
        ticks_since_last_command_packet++;

        GPIOB->BSRR |= GPIO_BSRR_BS_9;
#ifdef UART_ENABLED
        // increment the soft watchdog
        ticks_since_last_command_packet++;

        // process all available packets
        while (uart_can_read()) {
            uint8_t bytes_moved = uart_read(&motor_command_packet, sizeof(MotorCommandPacket));
            if (bytes_moved != sizeof(MotorCommandPacket)) {
                // something went wrong, just purge all of the data
                uart_discard();

                continue;
            }

            if (motor_command_packet.type == MCP_MOTION) {

                // we got a motion packet!
                ticks_since_last_command_packet = 0;

                if (motor_command_packet.data.motion.reset) {
                    // TODO handle hardware reset

                    while (true); // block, hardware reset flagged
                }

                telemetry_enabled = motor_command_packet.data.motion.enable_telemetry;
                r_motor_board = motor_command_packet.data.motion.setpoint;
            } else if (motor_command_packet.type == MCP_PARAMS) {
                // a params update is issued (or the upstream just wants to readback current params)

                // everytime a params packet is received, we echo back the current params state
                params_return_packet_requested = true;

                if (motor_command_packet.data.params.update_timestamp) {
                    time_set_epoch_seconds(motor_command_packet.data.params.update_timestamp);
                }

                if (motor_command_packet.data.params.update_vel_p) {
                    vel_pid_constants.kP = motor_command_packet.data.params.vel_p;
                }

                if (motor_command_packet.data.params.update_vel_i) {
                    vel_pid_constants.kI = motor_command_packet.data.params.vel_i;
                }

                if (motor_command_packet.data.params.update_vel_d) {
                    vel_pid_constants.kD = motor_command_packet.data.params.vel_d;
                }

                if (motor_command_packet.data.params.update_vel_i_max) {
                    vel_pid_constants.kI_max = motor_command_packet.data.params.vel_i_max;
                    vel_pid_constants.kI_min = -motor_command_packet.data.params.vel_i_max;
                }

                if (motor_command_packet.data.params.update_cur_p) {
                    torque_pid_constants.kP = motor_command_packet.data.params.cur_p;
                }

                if (motor_command_packet.data.params.update_cur_i) {
                    torque_pid_constants.kI = motor_command_packet.data.params.cur_i;
                }

                if (motor_command_packet.data.params.update_cur_d) {
                    torque_pid_constants.kD = motor_command_packet.data.params.cur_d;
                }

                if (motor_command_packet.data.params.update_cur_i_max) {
                    torque_pid_constants.kI_max = motor_command_packet.data.params.cur_i_max;
                    torque_pid_constants.kI_min = -motor_command_packet.data.params.cur_i_max;
                }

                if (motor_command_packet.data.params.update_cur_clamp) {
                    cur_limit = motor_command_packet.data.params.cur_clamp;
                }
            }
        }
#endif

        // if upstream isn't listening or its been too long since we got a command packet, turn off the motor
        if (!telemetry_enabled || ticks_since_last_command_packet > COMMAND_PACKET_TIMEOUT_TICKS) {
            r_motor_board = 0.0f;
        }

        // if any critical error is latched, coast the motor
        if (response_packet.data.motion.master_error) {
            r_motor_board = 0.0f;
        }

        // determine which loops need to be run
        bool run_torque_loop = time_sync_ready_rst(&torque_loop_timer);
        bool run_vel_loop = time_sync_ready_rst(&vel_loop_timer);

        // run the torque loop if applicable
        if (run_torque_loop) {
            // recover torque using the shunt voltage drop, amplification network model and motor model
            // pct of voltage range 0-3.3V
            float current_sense_shunt_v = ((float) res.I_motor_filt / (float) UINT16_MAX) * AVDD_V;
            // map voltage given by the amp network to current
            float current_sense_I = mm_voltage_to_current(&df45_model, current_sense_shunt_v);
            // map current to torque using the motor model
            float measured_torque_Nm = mm_current_to_torque(&df45_model, current_sense_I);
            // filter torque
            measured_torque_Nm = iir_filter_update(&torque_filter, measured_torque_Nm);

            // TODO: add filter? 

            // correct torque sign from recovered velocity
            if (enc_vel_rads < 0.0f) {
                measured_torque_Nm = -fabs(measured_torque_Nm);
            }

            // choose setpoint
            float r_Nm;
            if (motion_control_type == TORQUE) {
                r_Nm = r_motor_board;
            } else {
                r_Nm = u_vel_loop;
            }

            // calculate PID on the torque in Nm
            float torque_setpoint_Nm = pid_calculate(&torque_pid, r_Nm, measured_torque_Nm);

            // convert desired torque to desired current
            float current_setpoint = mm_torque_to_current(&df45_model, fabs(torque_setpoint_Nm));
            // convert desired current to desired duty cycle
            u_torque_loop = mm_pos_current_to_pos_dc(&df45_model, current_setpoint);

            if (torque_setpoint_Nm < 0.0f) {
                u_torque_loop = -fabs(u_torque_loop);
            }

            // load data frame
            // torque control data
            response_packet.data.motion.current_setpoint = r_Nm;
            // response_packet.data.motion.current_estimate = current_sense_I;
            response_packet.data.motion.current_estimate = measured_torque_Nm;
            response_packet.data.motion.current_computed_error = torque_pid.prev_err;
            response_packet.data.motion.current_computed_setpoint = torque_setpoint_Nm;
        }

        // run velocity loop if applicable
        if (run_vel_loop) {
            int32_t enc_delta = quadenc_get_encoder_delta();
            float rads_delta = mm_enc_ticks_to_rad(&df45_model, enc_delta);
            // float enc_vel_rad_s = quadenc_delta_to_w(enc_delta, VELOCITY_LOOP_RATE_S);
            enc_vel_rads = discrete_time_derivative(rads_delta, VELOCITY_LOOP_RATE_S);

            // filter the recovered velocity
            enc_rad_s_filt = iir_filter_update(&encoder_filter, enc_vel_rads);
        
            // compute the velcoity PID
            float vel_setpoint_rads = pid_calculate(&vel_pid, r_motor_board, enc_rad_s_filt);
            // back convert rads to duty cycle
            u_vel_loop = mm_rads_to_dc(&df45_model, vel_setpoint_rads);
            // u_vel_loop = vel_setpoint / DF45_MAX_MOTOR_RAD_PER_S;

            // velocity control data
            response_packet.data.motion.vel_setpoint = vel_setpoint_rads;
            response_packet.data.motion.encoder_delta = enc_delta;
            response_packet.data.motion.vel_enc_estimate = enc_rad_s_filt;
            response_packet.data.motion.vel_hall_estimate = 0U;
            response_packet.data.motion.vel_computed_error = vel_pid.prev_err;
            response_packet.data.motion.vel_computed_setpoint = vel_setpoint_rads;
        }


        if (run_torque_loop || run_vel_loop) {
            // detect if the encoder is not pulling the detect pin down
            bool encoder_disconnected = (GPIOA->IDR & GPIO_IDR_5) != 0;

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

            // bldc errors
            response_packet.data.motion.hall_power_error = reported_motor_errors.hall_power;
            response_packet.data.motion.hall_disconnected_error = reported_motor_errors.hall_disconnected;
            response_packet.data.motion.bldc_transition_error = reported_motor_errors.invalid_transitions;
            response_packet.data.motion.bldc_commutation_watchdog_error = reported_motor_errors.commutation_watchdog_timeout;

            // encoder errors
            response_packet.data.motion.enc_disconnected_error = encoder_disconnected;
            response_packet.data.motion.enc_decoding_error = false;

            // velocity checks
            response_packet.data.motion.hall_enc_vel_disagreement_error = false;

            // ADC errors
            response_packet.data.motion.overcurrent_error = false;
            response_packet.data.motion.undervoltage_error = false;
            response_packet.data.motion.overvoltage_error = false;

            // torque limiting
            response_packet.data.motion.torque_limited = false;

            // loop time
            response_packet.data.motion.control_loop_time_error = false;

            // master error
            response_packet.data.motion.master_error = response_packet.data.motion.hall_power_error
                    || response_packet.data.motion.hall_power_error
                    || response_packet.data.motion.hall_disconnected_error
                    || response_packet.data.motion.bldc_transition_error
                    || response_packet.data.motion.bldc_commutation_watchdog_error
                    || response_packet.data.motion.enc_disconnected_error
                    || response_packet.data.motion.enc_decoding_error
                    || response_packet.data.motion.hall_enc_vel_disagreement_error
                    || response_packet.data.motion.overcurrent_error
                    || response_packet.data.motion.undervoltage_error
                    || response_packet.data.motion.overvoltage_error
                    || response_packet.data.motion.control_loop_time_error;

            // timestamp
            response_packet.data.motion.timestamp = time_local_epoch_s();

            // transmit packets
#ifdef UART_ENABLED
            // GPIOB->BSRR |= GPIO_BSRR_BS_8;
            uart_wait_for_transmission();
            // takes ~270uS, mostly hardware DMA
            // if (telemetry_enabled) {
                uart_transmit((uint8_t *) &response_packet, sizeof(MotorResponsePacket));
            // }
            // GPIOB->BSRR |= GPIO_BSRR_BR_8;
#endif

            if (params_return_packet_requested) {
                params_return_packet_requested = false;

                response_packet.type = MCP_PARAMS;

                response_packet.data.params.version_major = VERSION_MAJOR;
                response_packet.data.params.version_major = VERSION_MINOR;
                response_packet.data.params.version_major = VERSION_PATCH;
                response_packet.data.params.timestamp = time_local_epoch_s();

                response_packet.data.params.vel_p = vel_pid_constants.kP;
                response_packet.data.params.vel_i = vel_pid_constants.kI;
                response_packet.data.params.vel_d = vel_pid_constants.kD;
                response_packet.data.params.vel_i_max = vel_pid_constants.kI_max;
                response_packet.data.params.cur_p = torque_pid_constants.kP;
                response_packet.data.params.cur_i = torque_pid_constants.kI;
                response_packet.data.params.cur_d = torque_pid_constants.kD;
                response_packet.data.params.torque_i_max = torque_pid_constants.kI_max;
                response_packet.data.params.cur_clamp = (uint16_t) cur_limit;

#ifdef UART_ENABLED
                uart_transmit((uint8_t *) &response_packet, sizeof(MotorResponsePacket));
#endif
            }
        }

        // limit loop rate to smallest time step
        if (sync_systick()) {
            slipped_control_frame_count++;
        }
    }
}
