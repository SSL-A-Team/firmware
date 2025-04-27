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

#include "debug.h"
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
#include "image_hash.h"

static int slipped_control_frame_count = 0;

__attribute__((optimize("O0")))
int main() {
    uint32_t rcc_csr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;

    // turn off LEDs
    turn_off_red_led();
    turn_off_yellow_led();
    turn_off_green_led();

    // turn on Red/Yellow LED
    turn_on_red_led();
    turn_on_yellow_led();

    // Setups clocks
    setup();

    // start watchdog
    IWDG->KR = 0x0000CCCC; // enable the module
    IWDG->KR = 0x00005555; // enable register writes
    IWDG->PR = 0x4; // set prescaler to 64, 40kHz -> 625Hz, 1.6ms per tick
    IWDG->RLR = 5; // count to 10 ticks, 16ms then trigger a system reset
    while (IWDG->SR) {} // wait for value to take
    IWDG->KR = 0x0000AAAA; // feed the watchdog

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

    // Initialized response_packet here to capture the reset method.
    MotorResponsePacket response_packet;
    memset(&response_packet, 0, sizeof(MotorResponsePacket));

    response_packet.data.motion.reset_watchdog_independent = (rcc_csr & RCC_CSR_IWDGRSTF) != 0;
    response_packet.data.motion.reset_watchdog_window = (rcc_csr & RCC_CSR_WWDGRSTF) != 0;
    response_packet.data.motion.reset_low_power = (rcc_csr & RCC_CSR_LPWRRSTF) != 0;
    response_packet.data.motion.reset_software = (rcc_csr & RCC_CSR_SFTRSTF) != 0;
    response_packet.data.motion.reset_pin = (rcc_csr & RCC_CSR_PINRSTF) != 0;

    bool params_return_packet_requested = false;

    // setup the loop rate regulators
    SyncTimer_t vel_loop_timer;
    time_sync_init(&vel_loop_timer, VELOCITY_LOOP_RATE_MS);
    SyncTimer_t torque_loop_timer;
    time_sync_init(&torque_loop_timer, TORQUE_LOOP_RATE_MS);
    SyncTimer_t telemetry_timer;
    time_sync_init(&telemetry_timer, TELEMETRY_LOOP_RATE_MS);

    // setup the velocity filter
    IIRFilter_t encoder_filter;
    iir_filter_init(&encoder_filter, iir_filter_alpha_from_Tf(ENCODER_IIR_TF_MS, VELOCITY_LOOP_RATE_MS));

    IIRFilter_t torque_filter;
    iir_filter_init(&torque_filter, iir_filter_alpha_from_Tf(TORQUE_IIR_TF_MS, TORQUE_LOOP_RATE_MS));

    // set the default command mode to open loop (no PID)
    MotorCommand_MotionType motion_control_type = OPEN_LOOP;

    // define the control points the loops use to interact
    float r_motor_board = 0.0f;
    float control_setpoint_vel_duty = 0.0f;
    float control_setpoint_vel_rads = 0.0f;
    float control_setpoint_vel_rads_prev = 0.0f;
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
    df45_model.rads_to_dc_linear_map_m = DF45_RADS_TO_DC_LINEAR_M;
    df45_model.rads_to_dc_linear_map_b = DF45_RADS_TO_DC_LINEAR_B;

    // setup the velocity PID
    PidConstants_t vel_pid_constants;
    pid_constants_initialize(&vel_pid_constants);
    Pid_t vel_pid;
    pid_initialize(&vel_pid, &vel_pid_constants);

    vel_pid_constants.kP = 2.0f;
    vel_pid_constants.kI = 0.0f;
    vel_pid_constants.kD = 0.0f;
    vel_pid_constants.kI_max = 20.0;
    vel_pid_constants.kI_min = -20.0;

    // setup the torque PID
    PidConstants_t torque_pid_constants;
    pid_constants_initialize(&torque_pid_constants);
    Pid_t torque_pid;
    pid_initialize(&torque_pid, &torque_pid_constants);

    torque_pid_constants.kP = 1.0f;

    // Turn off Red/Yellow LED after booting.
    turn_off_red_led();
    turn_off_yellow_led();

    // Initialize UART and logging status.
    uart_initialize();
    uart_logging_status_rx_t uart_logging_status_receive;
    uart_logging_status_tx_t uart_logging_status_send;

    // toggle J1-1
    while (true) {
        IWDG->KR = 0x0000AAAA; // feed the watchdog

#ifdef UART_ENABLED
        // increment the soft watchdog
        ticks_since_last_command_packet++;

        // process all available packets
        while (uart_can_read()) {
            // Make a new packet and clear out before reading
            // in the new data.
            MotorCommandPacket motor_command_packet;
            memset(&motor_command_packet, 0, sizeof(MotorCommandPacket));

            // Read in the new packet data.
            uart_read(&motor_command_packet, sizeof(MotorCommandPacket));

            // If something goes wrong with the UART, we need to flag it.
            if (uart_rx_get_logging_status() != UART_LOGGING_OK) {
                // Capture the status of the UART.
                uart_logging_status_receive = uart_rx_get_logging_status();

                // If something went wrong, just purge all of the data.
                uart_discard();

                // Go through the loop again to get the next packet.
                continue;
            } else {
                // If we are in COMP_MODE, don't latch the status
                // and clear it out if we get UART working.
                #ifdef COMP_MODE
                uart_logging_status_receive = UART_LOGGING_OK;
                #endif
            }

            // Clear the logging for the next UART receive.
            uart_rx_clear_logging_status();

            if (motor_command_packet.type == MCP_MOTION) {
                // We got a motion packet!
                ticks_since_last_command_packet = 0;

                if (motor_command_packet.data.motion.reset) {
                    // Does a software reset.
                    NVIC_SystemReset();
                }

                telemetry_enabled = motor_command_packet.data.motion.enable_telemetry;
                r_motor_board = motor_command_packet.data.motion.setpoint;
                motion_control_type = motor_command_packet.data.motion.motion_control_type;
            } else if (motor_command_packet.type == MCP_PARAMS) {
                // a params update is issued (or the upstream just wants to read back current params)

                // Every time a params packet is received, we echo back the current params state
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

        // Coast motor based on failure states.
        if (!telemetry_enabled ||
            ticks_since_last_command_packet > COMMAND_PACKET_TIMEOUT_TICKS ||
            response_packet.data.motion.master_error ||
            uart_logging_status_receive != UART_LOGGING_OK) {
            // If telemetry is disabled, that means the upstream
            // isn't ready.
            // If we haven't received a command packet in a while,
            // that means the upstream isn't ready or locked up.
            // If we have a master error, means the motor is in a bad state.
            // If we have a UART error, means something happened coming down
            // stream from the upstream or something locally went wrong.
            r_motor_board = 0.0f;
        }

        // determine which loops need to be run
        bool run_torque_loop = time_sync_ready_rst(&torque_loop_timer);
        bool run_vel_loop = time_sync_ready_rst(&vel_loop_timer);
        bool run_telemetry = time_sync_ready_rst(&telemetry_timer);

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
            // TODO: this should probably be acceleration based not velocity
            // e.g. F = ma
            if (enc_vel_rads < 0.0f) {
                measured_torque_Nm = -fabs(measured_torque_Nm);
            }

            // choose setpoint
            float r_Nm;
            if (motion_control_type == TORQUE) {
                r_Nm = r_motor_board;
            } else {
                r_Nm = control_setpoint_vel_duty;
            }

            // calculate PID on the torque in Nm
            float torque_setpoint_Nm = pid_calculate(&torque_pid, r_Nm, measured_torque_Nm, TORQUE_LOOP_RATE_S);

            // convert desired torque to desired current
            float current_setpoint = mm_torque_to_current(&df45_model, fabs(torque_setpoint_Nm));
            // convert desired current to desired duty cycle
            u_torque_loop = mm_pos_current_to_pos_dc(&df45_model, current_setpoint);

            if (torque_setpoint_Nm < 0.0f) {
                u_torque_loop = -fabs(u_torque_loop);
            }

            // load data frame
            // torque control data
            response_packet.data.motion.torque_setpoint = r_Nm;
            response_packet.data.motion.current_estimate = current_sense_I;
            response_packet.data.motion.torque_estimate = measured_torque_Nm;
            response_packet.data.motion.torque_computed_error = torque_pid.prev_err;
            response_packet.data.motion.torque_computed_setpoint = torque_setpoint_Nm;
        }

        // run velocity loop if applicable
        if (run_vel_loop) {
            int32_t enc_delta = quadenc_get_encoder_delta();
            float rads_delta = mm_enc_ticks_to_rad(&df45_model, enc_delta);
            // float enc_vel_rad_s = quadenc_delta_to_w(enc_delta, VELOCITY_LOOP_RATE_S);
            enc_vel_rads = discrete_time_derivative(rads_delta, VELOCITY_LOOP_RATE_S);

            // filter the recovered velocity
            enc_rad_s_filt = iir_filter_update(&encoder_filter, enc_vel_rads);

            // compute the velocity PID
            control_setpoint_vel_rads = pid_calculate(&vel_pid, r_motor_board, enc_rad_s_filt, VELOCITY_LOOP_RATE_S);

            // Clamp setpoint acceleration
            float setpoint_accel_rads_2 = (control_setpoint_vel_rads - control_setpoint_vel_rads_prev)/VELOCITY_LOOP_RATE_S;
            if (setpoint_accel_rads_2 > MOTOR_MAXIMUM_ACCEL) {
                setpoint_accel_rads_2 = MOTOR_MAXIMUM_ACCEL;
            } else if (setpoint_accel_rads_2 < -MOTOR_MAXIMUM_ACCEL) {
                setpoint_accel_rads_2 = -MOTOR_MAXIMUM_ACCEL;
            }

            control_setpoint_vel_rads = control_setpoint_vel_rads_prev + (setpoint_accel_rads_2 * VELOCITY_LOOP_RATE_S);
            control_setpoint_vel_rads_prev = control_setpoint_vel_rads;

            // back convert rads to duty cycle
            control_setpoint_vel_duty = mm_rads_to_dc(&df45_model, control_setpoint_vel_rads);

            // velocity control data
            response_packet.data.motion.vel_setpoint = r_motor_board;
            response_packet.data.motion.vel_setpoint_clamped = control_setpoint_vel_rads;
            response_packet.data.motion.encoder_delta = enc_delta;
            response_packet.data.motion.vel_enc_estimate = enc_rad_s_filt;
            response_packet.data.motion.vel_computed_error = vel_pid.prev_err;
            response_packet.data.motion.vel_computed_setpoint = control_setpoint_vel_duty;
        }

        if (run_torque_loop || run_vel_loop) {
            // detect if the encoder is not pulling the detect pin down
            // bool encoder_disconnected = (GPIOA->IDR & GPIO_IDR_5) != 0;
            bool encoder_disconnected = false;

            // set the motor duty cycle
            if (motion_control_type == OPEN_LOOP) {
                float r_motor = mm_rads_to_dc(&df45_model, r_motor_board);
                response_packet.data.motion.vel_setpoint = r_motor_board;
                response_packet.data.motion.vel_computed_setpoint = r_motor;
                pwm6step_set_duty_cycle_f(r_motor);
            } else if (motion_control_type == VELOCITY) {
                pwm6step_set_duty_cycle_f(control_setpoint_vel_duty);
            } else {
                pwm6step_set_duty_cycle_f(u_torque_loop);
            }

            // load system state for transmit

            // read error states
            const MotorErrors_t reported_motor_errors = pwm6step_get_motor_errors();

            response_packet.type = MRP_MOTION;

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
            response_packet.data.motion.control_loop_time_error = slipped_control_frame_count > 10;

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

            // transmit packets
#ifdef UART_ENABLED
            if (telemetry_enabled && run_telemetry) {
                // If previous UART transmit is still occurring,
                // wait for it to finish.
                uart_wait_for_transmission();
                // takes ~270uS, mostly hardware DMA, but should be cleared out by now.
                uart_transmit((uint8_t *) &response_packet, sizeof(MotorResponsePacket));
                // Capture the status for the response packet / LED.
                if (uart_tx_get_logging_status() != UART_LOGGING_OK) {
                    uart_logging_status_send = uart_tx_get_logging_status();
                } else {
                    // If we are in COMP_MODE, don't latch the status if
                    // we are able to send a packet successfully later.
                    #ifdef COMP_MODE
                    uart_logging_status_send = UART_LOGGING_OK;
                    #endif
                }

                // Clear the logging for the next UART transmit / receive.
                uart_tx_clear_logging_status();
            }
#endif

            if (params_return_packet_requested) {
                params_return_packet_requested = false;

                response_packet.type = MRP_PARAMS;

                response_packet.data.params.version_major = VERSION_MAJOR;
                response_packet.data.params.version_minor = VERSION_MINOR;
                response_packet.data.params.version_patch = VERSION_PATCH;
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

                memcpy(response_packet.data.params.wheel_img_hash, get_wheel_img_hash(), sizeof(response_packet.data.params.wheel_img_hash));
#ifdef UART_ENABLED
                uart_transmit((uint8_t *) &response_packet, sizeof(MotorResponsePacket));
                // Capture the status for the response packet / LED.
                if (uart_tx_get_logging_status() != UART_LOGGING_OK) {
                    uart_logging_status_send = uart_tx_get_logging_status();
                } else {
                    // If we are in COMP_MODE, don't latch the status if
                    // we are able to send a packet successfully later.
                    #ifdef COMP_MODE
                    uart_logging_status_send = UART_LOGGING_OK;
                    #endif
                }

                // Clear the logging for the next UART transmit / receive.
                uart_tx_clear_logging_status();
#endif
            }
        }

        // limit loop rate to smallest time step
        if (sync_systick()) {
            // Track if we are slipping control frames.
            slipped_control_frame_count++;
        }

        // Set all status LEDs

        // Red LED means we are in an error state.
        // This latches and requires resetting the robot to clear.
        if (response_packet.data.motion.master_error ||
            (telemetry_enabled && ticks_since_last_command_packet > COMMAND_PACKET_TIMEOUT_TICKS)) {
            turn_on_red_led();
        }

        // Yellow LED means we are in an warning state.
        // Will clear if resolved.
        if (uart_logging_status_receive != UART_LOGGING_OK ||
            uart_logging_status_send != UART_LOGGING_OK) {
            turn_on_yellow_led();
        } else {
            turn_off_yellow_led();
        }

        // Green LED means we are able to send telemetry upstream.
        // This means the upstream sent a packet downstream with telemetry enabled.
        if (!telemetry_enabled) {
            turn_off_green_led();
        } else {
            turn_on_green_led();
        }

        #ifdef COMP_MODE
        if (telemetry_enabled &&
            ticks_since_last_command_packet > COMMAND_PACKET_TIMEOUT_TICKS) {
            // If have telemetry enabled (meaning we at one
            // point received a message from upstream) and haven't
            // received a command packet in a while, we need to reset
            // the system when in COMP_MODE.
            // This is a safety feature to prevent the robot from
            // running away if the upstream controller locks up.
            while (true);
        }
        #endif
    }
}
