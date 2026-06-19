#include <stm32f031x6.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ateam-common-packets/include/stspin_current.h"

#include "debug.h"
#include "6step_current.h"
#include "current_sensing.h"
#include "iir.h"
#include "io_queue.h"
#include "main.h"
#include "pid.h"
#include "setup.h"
#include "system.h"
#include "time.h"
#include "uart.h"
#include "image_hash.h"

// timeout and error counters
static uint32_t slipped_control_frame_count = 0;
static uint32_t ticks_since_last_command_packet = 0;

// Dribbler torque image hash, saved in the dribbler torque image
static volatile ImgHash_t dribbler_torque_img_hash_struct = {
    "DrbTrqImgHashDrb",
    {0},
};

// communications data
static CcmMotionCommand motor_command_packet;
static CcmTelemetry response_packet;
static uart_logging_status_rx_t uart_logging_status_receive;
static uart_logging_status_tx_t uart_logging_status_send;
static bool params_return_packet_requested = false;

static bool current_limited = false;
static uint8_t params_seq_ctr = 0;
static uint8_t seq_ctr = 0;

static int16_t prev_current_cmd = 0;
static float prev_vel_setpoint = 0.0f;

// joint velocity + current controller
// kP in mA/(rad/s); tuned experimentally for ECU22048H24-S101
PidConstants_t vel_velcur_controller_constants = {
    .kP = 0.1f,
    .kI = 2.0f,
    .kD = 0.0f,
    .kI_max = 500.0f,
    .kI_min = -500.0f,
    .anti_jitter_thresh = (float) M_PI / 2.0f,
};

static Pid_t vel_velcur_controller;
static float vel_cur_component = 0.0;

// hall-based velocity estimate in rad/s
static float hall_vel_rads = 0.0f;
static IIRFilter_t hall_vel_filter;

////////////////////////////
//  Function Definitions  //
////////////////////////////

static bool allow_motor_to_run();
static int16_t apply_current_limits(int16_t);
static int16_t apply_current_slew_rate(int16_t);
static float apply_vel_setpoint_slew_rate(float);
static void update_dribbler_vel_est();
static void do_vel_cur_control(float);

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

    ////////////////////////////////////
    //  Current Sense Initialization  //
    ////////////////////////////////////

    // initialize current sensing setup
    CS_Status_t cs_status = currsen_setup(ADC_CH_MASK);
    if (cs_status != CS_OK) {
        // turn on red LED to indicate error
        turn_on_red_led();
    }

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
    // ECU22048H24-S101: R_L-L = 0.94 ohm, L_L-L = 0.104 mH, 1 pole pair
    // Base bandwidth 159 Hz (999 rad/s) scaled by 2 for ~318 Hz effective bandwidth
    // .kP = 106,
    // .kI = 192,
    static const FixedPointS12F4_PiConstants_t motor_current_pi_constants = {
        .kP = 106 * 2,      // S07F10, 999 * 0.000104 H = 0.10390 => 106
        .kI = 192 * 2,      // S05F13, 999 * 0.94 ohm * (1 / 40000) = 0.023477 => 192
        .kI_max = 4095,     // S12F0
        .kI_min = -(4095),  // S12F0
        .anti_jitter_thresh = 0,
        .anti_jitter_thresh_inv = 0,
    };
    pwm6step_setup(&motor_current_pi_constants);
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
    iir_filter_init(&hall_vel_filter, iir_filter_alpha_from_cutoff_hz(HALL_VEL_FILTER_CUTOFF_HZ, VELOCITY_LOOP_RATE_S));

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

        // update hall-based velocity estimate
        update_dribbler_vel_est();

        response_packet.motion_control_type = motor_command_packet.motion_control_type;

        switch (motor_command_packet.motion_control_type) {
            case CCM_MCT_MOTOR_OFF:
                prev_current_cmd = 0;
                prev_vel_setpoint = 0.0f;
                pwm6step_set_duty_cycle(0);
                break;
            case CCM_MCT_DUTY_OPENLOOP:
                pwm6step_set_duty_cycle_f(motor_command_packet.setpoint);
                break;
            case CCM_MCT_VOLTAGE_OPENLOOP:
                pwm6step_set_voltage((int16_t) motor_command_packet.setpoint);
                break;
            case CCM_MCT_CURRENT: {
                int16_t motor_current = motor_command_packet.current_setpoint_ma;
                motor_current = apply_current_limits(motor_current);
                motor_current = apply_current_slew_rate(motor_current);

                response_packet.current_telemetry.current_setpoint_ma = motor_current;
                pwm6step_set_current(motor_current);
                break;
            }
            case CCM_MCT_VELOCITY:
                // hall-only velocity control not implemented; motor off
                pwm6step_set_duty_cycle(0);
                break;
            case CCM_MCT_VELOCITY_CURRENT: {
                float effective_setpoint = apply_vel_setpoint_slew_rate(motor_command_packet.setpoint);
                do_vel_cur_control(effective_setpoint);

                float vel_component_clamped = fmaxf((float)INT16_MIN, fminf((float)INT16_MAX, vel_cur_component));
                int16_t joint_motor_current = motor_command_packet.current_setpoint_ma + (int16_t)vel_component_clamped;
                joint_motor_current = apply_current_limits(joint_motor_current);
                joint_motor_current = apply_current_slew_rate(joint_motor_current);

                response_packet.current_telemetry.current_setpoint_ma = joint_motor_current;
                pwm6step_set_current(joint_motor_current);
                break;
            }
        }

        response_packet.velocity_telemetry.vel_setpoint_rads = prev_vel_setpoint;
        response_packet.current_telemetry.bus_voltage_mv = pwm6step_get_vbus_voltage();
        response_packet.current_telemetry.motor_voltage_cmd_mv = pwm6step_get_voltage_command();

        memcpy(response_packet.current_telemetry.current_samples_ma, pwm6step_get_current_log(), sizeof(response_packet.current_telemetry.current_samples_ma));

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

static int16_t apply_current_limits(int16_t desired_current) {
    int16_t applied_current = desired_current;

    bool motor_turning = abs(response_packet.current_telemetry.hall_vel_est_drads) > DRIBBLER_TURNING_VEL_THRESH_DRADS;
    int16_t limit = motor_turning ? MAX_CURR_DRIBBLER_TURNING : MAX_CURR_DRIBBLER_NOT_TURNING;

    if (desired_current > limit) {
        applied_current = limit;
    }

    if (desired_current < -limit) {
        applied_current = -limit;
    }

    current_limited = (applied_current != desired_current);

    return applied_current;
}

static void update_dribbler_vel_est() {
    int16_t hall_drads = pwm6step_hall_get_rps_estimate();
    response_packet.current_telemetry.hall_vel_est_drads = hall_drads;
    hall_vel_rads = iir_filter_update(&hall_vel_filter, hall_drads / 10.0f);
    response_packet.velocity_telemetry.wheel_vel_rads = hall_vel_rads;
}

static void do_vel_cur_control(float effective_setpoint) {
    vel_cur_component = pid_calculate_err_resp_only(
        &vel_velcur_controller,
        effective_setpoint,
        hall_vel_rads,
        VELOCITY_LOOP_RATE_S);
}

static int16_t apply_current_slew_rate(int16_t target) {
    int16_t delta = target - prev_current_cmd;
    if (delta >  MAX_CURR_SLEW_RATE_MA_PER_MS) delta =  MAX_CURR_SLEW_RATE_MA_PER_MS;
    if (delta < -MAX_CURR_SLEW_RATE_MA_PER_MS) delta = -MAX_CURR_SLEW_RATE_MA_PER_MS;
    prev_current_cmd += delta;
    return prev_current_cmd;
}

static float apply_vel_setpoint_slew_rate(float target) {
    float delta = target - prev_vel_setpoint;
    if (delta >  MAX_VEL_SETPOINT_SLEW_RATE_RADS_PER_MS) delta =  MAX_VEL_SETPOINT_SLEW_RATE_RADS_PER_MS;
    if (delta < -MAX_VEL_SETPOINT_SLEW_RATE_RADS_PER_MS) delta = -MAX_VEL_SETPOINT_SLEW_RATE_RADS_PER_MS;
    prev_vel_setpoint += delta;
    return prev_vel_setpoint;
}

static void update_errors() {
    // read error states
    const MotorErrors_t reported_motor_errors = pwm6step_get_motor_errors();

    // bldc errors
    response_packet.hall_power_error = reported_motor_errors.hall_power;
    response_packet.hall_disconnected_error = reported_motor_errors.hall_disconnected;
    response_packet.bldc_transition_error = reported_motor_errors.invalid_transitions;
    response_packet.bldc_commutation_watchdog_error = reported_motor_errors.commutation_watchdog_timeout;

    // no encoder on dribbler
    response_packet.enc_disconnected_error = false;

    // ADC errors
    response_packet.overcurrent_error = false;
    response_packet.undervoltage_error = false;
    response_packet.overvoltage_error = false;

    // torque limiting
    response_packet.torque_limited = current_limited;

    // loop time
    response_packet.control_loop_time_error = slipped_control_frame_count > 10;

    // master error
    response_packet.master_error = response_packet.hall_power_error
            || response_packet.hall_disconnected_error
            || response_packet.bldc_transition_error
            || response_packet.bldc_commutation_watchdog_error
            || response_packet.enc_disconnected_error
            || response_packet.overcurrent_error
            || response_packet.undervoltage_error
            || response_packet.overvoltage_error
            || response_packet.control_loop_time_error;

    response_packet.gain_stage_index = 0;
}

static void read_packets() {
    // process all available packets
    CcmCommand command_packet;

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

            motor_command_packet = command_packet.data.motion;

            if (motor_command_packet.reset) {
                // Does a software reset.
                NVIC_SystemReset();
            }
        } else if (command_packet.type == CCM_CMD_PARAMS) {
            // just sends the firmware hash back by default for now
            params_return_packet_requested = true;
        }
    }
}

static void send_packets() {
    CcmResponse response_pkt;

    // If previous UART transmit is still occurring,
    // wait for it to finish.
    uart_wait_for_transmission();

    // Send only one packet per frame to avoid back-to-back transmissions
    // that merge into a single UART idle-line frame on the receiver.
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
        memcpy(response_pkt.data.params.value.val_u8x4, (const char *)dribbler_torque_img_hash_struct.img_hash, sizeof(response_pkt.data.params.value));
    } else {
        response_pkt.type = CCM_RESP_TELEM;
        response_pkt.timestamp = time_local_epoch_s();
        response_pkt.seq_num = seq_ctr;  // intentionally overflow
        seq_ctr++;

        response_pkt.data.motion = response_packet;
    }

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
