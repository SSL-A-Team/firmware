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

#include "ateam-common-packets/include/stspin_current.h"

#include "debug.h"
#include "6step_current.h"
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
static CcmMotionCommand motor_command_packet;
static CcmTelemetry response_packet;
static uart_logging_status_rx_t uart_logging_status_receive;
static uart_logging_status_tx_t uart_logging_status_send;
static bool params_return_packet_requested = false;

// encoder filter
static IIRFilter_t encoder_filter;

// motor model
static MotorModel_t df45_model;

// joint velocity + current controller
// static FixedPointS12F4_PiConstants_t vel_curvel_controller_constants = {
//     // 1000 Hz bandwidth -> 6283 rads

//     // .kP = 2123,
//     // .kI = 910,

//     // KNOWN GOOD
//     .kP = 338 * 3,      // S07F10, 6283 * 0.00033 H = 2.07339 => 2123
//     .kI = 145 * 3,  // S05F13, 6283 * (0.7ohm coil + 0.007 ohm wire) * (1 / 40000) = 0.11105 => 910 
//     .kI_max = 4095,  // S12F0
//     .kI_min = -(4095),  // S12F0
//     .anti_jitter_thresh = 0,
//     .anti_jitter_thresh_inv = 0,
// };

// static FixedPointS12F4_PiController_t vel_curvel_controller;

const PidConstants_t vel_velcur_controller_constants = {
    .kP = 0.1f,
    .kI = 0.0f,
    .kD = 0.0f,
    .kI_max = 0.0f,
    .kI_min = 0.0f,
};

static Pid_t vel_velcur_controller;
static float vel_cur_component = 0.0;

// legacy velocity control model
static GainScheduledPid_t vel_pid;
const PidConstants_t vel_gains[3] = {
    {
        .kP = 6.0f,
        .kI = 12.0f,
        .kD = 0.4f,
        .kI_max = 20.0f,
        .kI_min = -20.0f,
    },
    {
        .kP = 7.0f,
        .kI = 0.0f,
        .kD = 0.5f,
        .kI_max = 0.0f,
        .kI_min = 0.0f,
    },
    {
        .kP = 2.0f,
        .kI = 0.0f,
        .kD = 0.1f,
        .kI_max = 0.0f,
        .kI_min = 0.0f,
    }
};
const float vel_gain_schedule[3] = {
    3.0,
    7.0f,
    30.0f,
}; // rad/s

////////////////////////////
//  Function Definitions  //
////////////////////////////

static bool allow_motor_to_run();
static void update_wheel_vel_est();
static float do_vel_control();
static void do_vel_cur_control();

static void update_errors();
static void read_packets();
static void send_packets();
static void set_leds();

////////////
//  Main  //
////////////

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
    memset(&motor_command_packet, 0, sizeof(CcmMotionCommand));
    memset(&response_packet, 0, sizeof(CcmTelemetry));

#ifdef UART_ENABLED
    // Initialize UART and logging status.
    uart_initialize();
#endif
    // load reset vector data to response packet
    response_packet.reset_watchdog_independent = (rcc_csr & RCC_CSR_IWDGRSTF) != 0;
    response_packet.reset_watchdog_window = (rcc_csr & RCC_CSR_WWDGRSTF) != 0;
    response_packet.reset_low_power = (rcc_csr & RCC_CSR_LPWRRSTF) != 0;
    response_packet.reset_software = (rcc_csr & RCC_CSR_SFTRSTF) != 0;
    response_packet.reset_pin = (rcc_csr & RCC_CSR_PINRSTF) != 0;

    //////////////////////////////
    //  Encoder Initialization  //
    //////////////////////////////

    quadenc_setup();
    quadenc_reset_encoder_delta();

    // setup the velocity filter
    iir_filter_init(&encoder_filter, iir_filter_alpha_from_cutoff_hz(200.0f, VELOCITY_LOOP_RATE_S));

    ////////////////////////////////////
    //  Current Sense Initialization  //
    ////////////////////////////////////

    // initialize current sensing setup
    CS_Status_t cs_status = currsen_setup(ADC_CH_MASK);
    if (cs_status != CS_OK) {
        // turn on red LED to indicate error
        turn_on_red_led();
    }

    ////////////////////////////////////////////
    //  Legacy Velocity Model Initialization  //
    ////////////////////////////////////////////

    // initialize the motor model for the Nanotec DF45 50W (this is our drive motor)
    // this is used for legacy wheel velocity control
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

    gspid_initialize(&vel_pid, 3, vel_gains, vel_gain_schedule, 0.2f, true);

    ///////////////////////////////
    //  Watchdog Initialization  //
    ///////////////////////////////

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

    pid_initialize(&vel_velcur_controller, &vel_velcur_controller_constants);

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
        if (!allow_motor_to_run()) {
            motor_command_packet.current_setpoint_ma = 0;
            motor_command_packet.setpoint = 0.0f;
        }

        // update wheel velocity est
        update_wheel_vel_est();

        response_packet.motion_control_type = motor_command_packet.motion_control_type;

        switch (motor_command_packet.motion_control_type) {
            case CCM_MCT_MOTOR_OFF:
                pwm6step_set_duty_cycle(0);
                break;
            case CCM_MCT_DUTY_OPENLOOP:
                pwm6step_set_duty_cycle_f(motor_command_packet.setpoint);
                break;
            case CCM_MCT_VOLTAGE_OPENLOOP:
                pwm6step_set_voltage((int32_t) motor_command_packet.setpoint);
                break;
            case CCM_MCT_CURRENT:
                pwm6step_set_current(motor_command_packet.current_setpoint_ma);
                response_packet.current_telemetry.current_setpoint_ma = motor_command_packet.current_setpoint_ma;
                break;
            case CCM_MCT_VELOCITY:
                float dc_f = do_vel_control();
                pwm6step_set_duty_cycle_f(dc_f);

                break;
            case CCM_MCT_VELOCITY_CURRENT:
                do_vel_cur_control();

                // add the feedforward current to the velocity PID output
                int16_t motor_current = motor_command_packet.current_setpoint_ma + (int16_t) vel_cur_component;

                pwm6step_set_current(motor_current);

                break;
        }

        response_packet.current_telemetry.bus_voltage_mv = pwm6step_get_vbus_voltage();
        response_packet.current_telemetry.motor_voltage_cmd_mv = pwm6step_get_voltage_command();

        memcpy(response_packet.current_telemetry.current_samples_ma, pwm6step_get_current_log(), 40);

        // load errors into packets and set LEDs
        update_errors();

        // send response packets
        send_packets();

        set_leds();

        // sync to ADC
        while (!pwm6step_1ms_flag()) {}
    }
}

static bool allow_motor_to_run() {
    return motor_command_packet.enable_telemetry 
        && ticks_since_last_command_packet <= COMMAND_PACKET_TIMEOUT_TICKS 
        && !response_packet.master_error
        && uart_logging_status_receive == UART_LOGGING_OK
        && uart_logging_status_send == UART_LOGGING_OK;
}

static void read_packets() {
    // process all available packets
    CcmCommand command_packet;
    // memset(&command_packet, 0, sizeof(CurrentControlledMotor_Command));

    while (uart_can_read()) {
        // Read in the new packet data.
        uart_read(&command_packet, sizeof(CcmCommand));

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

        if (command_packet.type == CCM_CMD_MOTION) {
            // We got a motion packet!
            ticks_since_last_command_packet = 0;

            if (motor_command_packet.reset) {
                // Does a software reset.
                NVIC_SystemReset();
            }

            motor_command_packet = command_packet.data.motion;
        } else if (command_packet.type == CCM_CMD_PARAMS) {
            // just sends the firmware hash back by default for now
            params_return_packet_requested = true;
        }
    }
}

static void update_wheel_vel_est() {
    int32_t enc_delta = quadenc_get_encoder_delta();
    float rads_delta = mm_enc_ticks_to_rad(&df45_model, enc_delta);
    // float enc_vel_rad_s = quadenc_delta_to_w(enc_delta, VELOCITY_LOOP_RATE_S);
    float enc_vel_rads = discrete_time_derivative(rads_delta, VELOCITY_LOOP_RATE_S);

    // filter the recovered velocity
    float enc_rad_s_filt = iir_filter_update(&encoder_filter, enc_vel_rads);

    // set the value
    response_packet.velocity_telemetry.wheel_vel_rads = enc_rad_s_filt;
}

static void do_vel_cur_control() {
    vel_cur_component = pid_calculate(&vel_velcur_controller, 0, response_packet.velocity_telemetry.wheel_vel_rads, VELOCITY_LOOP_RATE_S);
}

static float do_vel_control() {
    float r_motor_board = motor_command_packet.setpoint;
    float cur_wheel_vel_est_rads = response_packet.velocity_telemetry.wheel_vel_rads;
    static float control_setpoint_vel_rads_prev = 0.0f;

    // reset integrator when commanded to stop
    if(fabsf(r_motor_board) < 1e-3f) {
        vel_pid.eI = 0.0f;
    }

    // compute the velocity PID
    float control_setpoint_vel_rads = gspid_calculate(&vel_pid, r_motor_board, cur_wheel_vel_est_rads, VELOCITY_LOOP_RATE_S);

    // Clamp setpoint acceleration
    float setpoint_accel_rads_2 = (control_setpoint_vel_rads - control_setpoint_vel_rads_prev) / VELOCITY_LOOP_RATE_S;
    if (setpoint_accel_rads_2 > MOTOR_MAXIMUM_ACCEL) {
        setpoint_accel_rads_2 = MOTOR_MAXIMUM_ACCEL;
    } else if (setpoint_accel_rads_2 < -MOTOR_MAXIMUM_ACCEL) {
        setpoint_accel_rads_2 = -MOTOR_MAXIMUM_ACCEL;
    }

    control_setpoint_vel_rads = control_setpoint_vel_rads_prev + (setpoint_accel_rads_2 * VELOCITY_LOOP_RATE_S);
    control_setpoint_vel_rads_prev = control_setpoint_vel_rads;

    float control_setpoint_vel_duty;
    // back convert rads to duty cycle
    if(r_motor_board == 0.0f) {
        control_setpoint_vel_duty = 0.0f;
    } else {
        control_setpoint_vel_duty = mm_rads_to_dc(&df45_model, control_setpoint_vel_rads);
    }

    return control_setpoint_vel_duty;
}

static void update_errors() {
    // detect if the encoder is not pulling the detect pin down
#ifdef HAS_EXTERNAL_ENCODER
    bool encoder_disconnected = (GPIOA->IDR & GPIO_IDR_5) != 0;
#else
    bool encoder_disconnected = false;
#endif

    // read error states
    const MotorErrors_t reported_motor_errors = pwm6step_get_motor_errors();

    // bldc errors
    response_packet.hall_power_error = reported_motor_errors.hall_power;
    response_packet.hall_disconnected_error = reported_motor_errors.hall_disconnected;
    response_packet.bldc_transition_error = reported_motor_errors.invalid_transitions;
    response_packet.bldc_commutation_watchdog_error = reported_motor_errors.commutation_watchdog_timeout;

    // encoder errors
    response_packet.enc_disconnected_error = encoder_disconnected;

    // ADC errors
    response_packet.overcurrent_error = false;
    response_packet.undervoltage_error = false;
    response_packet.overvoltage_error = false;

    // torque limiting
    response_packet.torque_limited = false;

    // loop time
    response_packet.control_loop_time_error = slipped_control_frame_count > 10;

    // master error
    response_packet.master_error = response_packet.hall_power_error
            || response_packet.hall_power_error
            || response_packet.hall_disconnected_error
            || response_packet.bldc_transition_error
            || response_packet.bldc_commutation_watchdog_error
            || response_packet.enc_disconnected_error
            || response_packet.overcurrent_error
            || response_packet.undervoltage_error
            || response_packet.overvoltage_error
            || response_packet.control_loop_time_error;

    response_packet.gain_stage_index = gspid_get_cur_gain_stage_index(&vel_pid) & 0xFF;
}

static uint8_t params_seq_ctr = 0;
static uint8_t seq_ctr = 0;
static void send_packets() {
    CcmResponse response_pkt;
    response_pkt.type = CCM_RESP_TELEM;
    response_pkt.timestamp = time_local_epoch_s();
    response_pkt.seq_num = seq_ctr;  // intentionally overflow
    seq_ctr++;

    response_pkt.data.motion = response_packet;

    // If previous UART transmit is still occurring,
    // wait for it to finish.
    uart_wait_for_transmission();
    // takes ~270uS, mostly hardware DMA, but should be cleared out by now.
    uart_transmit((uint8_t *) &response_pkt, sizeof(CcmResponse));
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

        response_pkt.type = CCM_RESP_PARAMS;
        response_pkt.timestamp = time_local_epoch_s();
        response_pkt.seq_num = params_seq_ctr;  // intentionally overflow
        params_seq_ctr++;

        // responding with firmware image hash, read operation, as a reply message
        response_pkt.data.params.parameter = CCM_PARAM_FIRMWARE_IMAGE_HASH;
        response_pkt.data.params.parameter_operation = CCM_PARAMOP_READ;
        response_pkt.data.params.parameter_direction = CCM_PARAMDIR_REPLY;

        // load the hash
        memcpy(response_pkt.data.params.value.val_u8x4, wheel_img_hash_struct.img_hash, sizeof(response_pkt.data.params.value));

        // send the packet
        uart_transmit((uint8_t *) &response_pkt, sizeof(CcmResponse));

        // TODO check if this works, uart_transmit doesn't block so status may be stale
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

static void set_leds() {
    static uint16_t green_led_ctr = 0;

    // Red LED means we are in an error state.
    // This latches and requires resetting the robot to clear.
    if (response_packet.master_error ||
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
    if (!motor_command_packet.enable_telemetry) {
        turn_on_green_led();
    } else {
        // 1Hz flicker
        if (green_led_ctr > 1000) {
            green_led_ctr = 0;
        } else if (green_led_ctr > 500) {
            turn_off_green_led();
        } else {
            turn_on_green_led();
        }

        green_led_ctr++;
    }
}
