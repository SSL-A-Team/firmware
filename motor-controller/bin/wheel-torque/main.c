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

// timeout and error counters
static uint32_t slipped_control_frame_count = 0;
static uint32_t ticks_since_last_command_packet = 0;

// Wheel image hash, saved in the wheel image
static volatile ImgHash_t wheel_img_hash_struct = {
    "WheelImgHashWeel",
    {0},
};

// communications data
static motor_command_packet;
static response_packet;
static uart_logging_status_rx_t uart_logging_status_receive;
static uart_logging_status_tx_t uart_logging_status_send;
static bool params_return_packet_requested = false;


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

    // zero out packet initial states
    memset(&motor_command_packet, 0, sizeof(MotorCommandPacket));
    memset(&response_packet, 0, sizeof(MotorResponse));
#ifdef UART_ENABLED
    // Initialize UART and logging status.
    uart_initialize();
#endif
    // load reset vector data to response packet
    response_packet.data.motion.reset_watchdog_independent = (rcc_csr & RCC_CSR_IWDGRSTF) != 0;
    response_packet.data.motion.reset_watchdog_window = (rcc_csr & RCC_CSR_WWDGRSTF) != 0;
    response_packet.data.motion.reset_low_power = (rcc_csr & RCC_CSR_LPWRRSTF) != 0;
    response_packet.data.motion.reset_software = (rcc_csr & RCC_CSR_SFTRSTF) != 0;
    response_packet.data.motion.reset_pin = (rcc_csr & RCC_CSR_PINRSTF) != 0;

    // setup encoder
    quadenc_setup();
    quadenc_reset_encoder_delta();

    // initialize current sensing setup
    CS_Status_t cs_status = currsen_setup(ADC_CH_MASK);
    if (cs_status != CS_OK) {
        // turn on red LED to indicate error
        turn_on_red_led();
    }

    // start watchdog
    IWDG->KR = 0x0000CCCC; // enable the module
    IWDG->KR = 0x00005555; // enable register writes
    IWDG->PR = 0x4; // set prescaler to 64, 40kHz -> 625Hz, 1.6ms per tick
    IWDG->RLR = 10; // count to 10 ticks, 16ms then trigger a system reset
    while (IWDG->SR) {} // wait for value to take
    IWDG->KR = 0x0000AAAA; // feed the watchdog

    // watchdog primarily protects the motor from stalling and burning out
    // if critical errors occur. "softer" errors like crashed coms etc
    // are not critical to the safety of the motor and handled via error
    // counter and timeouts. From this point forward the WD is ON, so critical
    // motor functions can be initialized

    // initialize motor driver
    pwm6step_setup();
    pwm6step_set_duty_cycle_f(0.0f);

    // enable ADC hardware trigger (tied to 6step timer)
    currsen_enable_ht();

    wait_ms(5);

    // calibrate current
    while (!currsen_calibrate_sense()) {
        wait_ms(1);
        IWDG->KR = 0x0000AAAA; // feed the watchdog
    }

    // Turn off Red/Yellow LED after booting.
    turn_off_red_led();
    turn_off_yellow_led();


    /////////////////
    //  Main Loop  //
    /////////////////

    while (true) {
        IWDG->KR = 0x0000AAAA; // feed the watchdog
        
        // increment the soft watchdog
        ticks_since_last_command_packet++;
#ifdef COMP_MODE
        if (ticks_since_last_command_packet > COMMAND_PACKET_TIMEOUT_TICKS) {
            while (true);
        }
#endif

        // process any command or parameter packets
        read_packets();

        // "soft" error conditions zero out the motor setpoint
        if (allow_motor_to_run()) {
            motor_command_packet.motion_control_type = MOTOR_OFF;
            motor_command_packet.current_setpoint_ma = 0;
            motor_command_packet.velocity_setpoint_rads = 0.0f;
        }

        // run the velocity control loop
        vel_control_loop();

        switch (motor_command_packet.motor_control_type) {
            case MOTOR_OFF:
                pwm6step_set_duty_cycle(0);
                break;
            case DUTY_OPENLOOP:
                pwm6step_set_duty_cycle_f(motor_command_packet.duty_setpoint_f);
            case VOLTAGE_OPENLOOP:
                pwm6step_set_voltage((int32_t) motor_command_packet.voltage_setpoint_mv);
                break;
            case CURRENT:
                pwm6step_set_current(motor_command_packet.current_setpoint_ma);
                break;
            case VELOCITY:
                break;
            case VELOCITY_CURRENT:
                pwm6step_set_current(motor_command_packet.current_setpoint_ma);
                break;
        }

        // load errors into packets and set LEDs
        update_errors();

        // send response packets
        send_packets();

        // TODO sync time from ADC?

        // limit loop rate to smallest time step
        if (sync_systick()) {
            // Track if we are slipping control frames.
            slipped_control_frame_count++;
        }
    }
}

bool allow_motor_to_run() {
    return motor_command_packet.telemetry_enabled 
        && ticks_since_last_command_packet <= COMMAND_PACKET_TIMEOUT_TICKS 
        && !response_packet.data.motion.master_error
        && uart_logging_status_receive == UART_LOGGING_OK
        && uart_logging_status_send == UART_LOGGING_OK;
}

void read_packets() {
    // process all available packets
    while (uart_can_read()) {
        // Make a new packet and clear out before reading
        // in the new data.
        // memset(&motor_command_packet, 0, sizeof(MotorCommandPacket));

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

            // TODO remote PID updates are off for gain scheduled PID

            // if (motor_command_packet.data.params.update_vel_p) {
            //     vel_pid_constants.kP = motor_command_packet.data.params.vel_p;
            // }

            // if (motor_command_packet.data.params.update_vel_i) {
            //     vel_pid_constants.kI = motor_command_packet.data.params.vel_i;
            // }

            // if (motor_command_packet.data.params.update_vel_d) {
            //     vel_pid_constants.kD = motor_command_packet.data.params.vel_d;
            // }

            // if (motor_command_packet.data.params.update_vel_i_max) {
            //     vel_pid_constants.kI_max = motor_command_packet.data.params.vel_i_max;
            //     vel_pid_constants.kI_min = -motor_command_packet.data.params.vel_i_max;
            // }

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
}

void do_vel_control() {
    int32_t enc_delta = quadenc_get_encoder_delta();
    float rads_delta = mm_enc_ticks_to_rad(&df45_model, enc_delta);
    // float enc_vel_rad_s = quadenc_delta_to_w(enc_delta, VELOCITY_LOOP_RATE_S);
    enc_vel_rads = discrete_time_derivative(rads_delta, VELOCITY_LOOP_RATE_S);

    // filter the recovered velocity
    enc_rad_s_filt = iir_filter_update(&encoder_filter, enc_vel_rads);

    // reset integrator when commanded to stop
    if(fabsf(r_motor_board) < 1e-3f) {
        vel_pid.eI = 0.0f;
    }

    // compute the velocity PID
    control_setpoint_vel_rads = gspid_calculate(&vel_pid, r_motor_board, enc_rad_s_filt, VELOCITY_LOOP_RATE_S);

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
    if(r_motor_board == 0.0f) {
        control_setpoint_vel_duty = 0.0f;
    } else {
        control_setpoint_vel_duty = mm_rads_to_dc(&df45_model, control_setpoint_vel_rads);
    }

    // velocity control data
    response_packet.data.motion.vel_setpoint = r_motor_board;
    response_packet.data.motion.vel_computed_rads = control_setpoint_vel_rads;
    response_packet.data.motion.encoder_delta = enc_delta;
    response_packet.data.motion.vel_enc_estimate = enc_rad_s_filt;
    response_packet.data.motion.vel_computed_error = vel_pid.prev_err;
    response_packet.data.motion.vel_computed_duty = control_setpoint_vel_duty;
}

void update_errors() {
    // detect if the encoder is not pulling the detect pin down
#ifdef HAS_EXTERNAL_ENCODER
    bool encoder_disconnected = (GPIOA->IDR & GPIO_IDR_5) != 0;
#else
    bool encoder_disconnected = false;
#endif

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

    response_packet.data.motion.gain_stage_index = gspid_get_cur_gain_stage_index(&vel_pid) & 0xFF;
}

void send_packets() {
    // If previous UART transmit is still occurring,
    // wait for it to finish.
    uart_wait_for_transmission();
    // takes ~270uS, mostly hardware DMA, but should be cleared out by now.
    uart_transmit((uint8_t *) &response_packet, sizeof(MotorResponse));
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
    
    if (params_return_packet_requested) {
        params_return_packet_requested = false;

        response_packet.type = MRP_PARAMS;

        response_packet.data.params.version_major = VERSION_MAJOR;
        response_packet.data.params.version_minor = VERSION_MINOR;
        response_packet.data.params.version_patch = VERSION_PATCH;
        response_packet.timestamp = time_local_epoch_s();

        // TODO parameter updates are off for gain scheduled PID
        // response_packet.data.params.vel_p = vel_pid_constants.kP;
        // response_packet.data.params.vel_i = vel_pid_constants.kI;
        // response_packet.data.params.vel_d = vel_pid_constants.kD;
        // response_packet.data.params.vel_i_max = vel_pid_constants.kI_max;

        response_packet.data.params.cur_p = torque_pid_constants.kP;
        response_packet.data.params.cur_i = torque_pid_constants.kI;
        response_packet.data.params.cur_d = torque_pid_constants.kD;
        response_packet.data.params.torque_i_max = torque_pid_constants.kI_max;
        response_packet.data.params.cur_clamp = (uint16_t) cur_limit;

        memcpy(response_packet.data.params.firmware_img_hash, wheel_img_hash_struct.img_hash, sizeof(response_packet.data.params.firmware_img_hash));
        uart_transmit((uint8_t *) &response_packet, sizeof(MotorResponse));
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
}

void set_leds() {
    static uint16_t green_led_ctr = 0;

    // Red LED means we are in an error state.
    // This latches and requires resetting the robot to clear.
    if (response_packet.data.motion.master_error ||
        ticks_since_last_command_packet > COMMAND_PACKET_TIMEOUT_TICKS) {
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

    // Green LED means system is up. Flicker means we're acknowledging that
    // control has enabled the system (non-zero motor commands allowed)
    if (!motor_command_packet.telemetry_enabled) {
        turn_on_green_led();
    } else {
        // 5Hz flicker
        if (green_led_ctr > 200) {
            green_led_ctr = 0;
        } else if (green_led_ctr > 100) {
            turn_off_green_led();
        } else {
            turn_on_green_led();
        }

        green_led_ctr++;
    }
}
