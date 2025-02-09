#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <stm32f031x6.h>

#include "ateam-common-packets/include/stspin.h"

#include "debug.h"
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

__attribute__((optimize("O0")))
int main() {
    uint32_t rcc_csr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;

    // turn off LEDs
    turn_off_red_led();
    turn_off_yellow_led();
    turn_off_green_led();

    // turn on Red LED
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


    // Initialized response_packet here to capture the reset method.
    MotorResponsePacket response_packet;
    memset(&response_packet, 0, sizeof(MotorResponsePacket));

    response_packet.data.motion.reset_watchdog_independent = (rcc_csr & RCC_CSR_IWDGRSTF) != 0;
    response_packet.data.motion.reset_watchdog_window = (rcc_csr & RCC_CSR_WWDGRSTF) != 0;
    response_packet.data.motion.reset_low_power = (rcc_csr & RCC_CSR_LPWRRSTF) != 0;
    response_packet.data.motion.reset_software = (rcc_csr & RCC_CSR_SFTRSTF) != 0;
    response_packet.data.motion.reset_pin = (rcc_csr & RCC_CSR_PINRSTF) != 0;

    bool params_return_packet_requested = false;

    SyncTimer_t torque_loop_timer;
    time_sync_init(&torque_loop_timer, TORQUE_LOOP_RATE_MS);
    SyncTimer_t telemetry_timer;
    time_sync_init(&telemetry_timer, TELEMETRY_LOOP_RATE_MS);

    IIRFilter_t torque_filter;
    iir_filter_init(&torque_filter, iir_filter_alpha_from_Tf(TORQUE_IIR_TF_MS, TORQUE_LOOP_RATE_MS));

    MotorCommand_MotionType motion_control_type = OPEN_LOOP;

    float r_motor_board = 0.0f;
    float u_torque_loop;
    float cur_limit = 0.0f;

    PidConstants_t torque_pid_constants;
    pid_constants_initialize(&torque_pid_constants);
    Pid_t torque_pid;
    pid_initialize(&torque_pid, &torque_pid_constants);

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
        // watchdog on receiving a command packet
        ticks_since_last_command_packet++;

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
                // we got a motion packet!
                ticks_since_last_command_packet = 0;

                if (motor_command_packet.data.motion.reset) {
                    // Does a software reset.
                    NVIC_SystemReset();
                }

                telemetry_enabled = motor_command_packet.data.motion.enable_telemetry;
                r_motor_board = motor_command_packet.data.motion.setpoint;
                motion_control_type = motor_command_packet.data.motion.motion_control_type;
            } else if (motor_command_packet.type == MCP_PARAMS) {
                // Every time a params packet is received, we echo back the current params state
                params_return_packet_requested = true;

                if (motor_command_packet.data.params.update_timestamp) {
                    time_set_epoch_seconds(motor_command_packet.data.params.update_timestamp);
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

        bool run_torque_loop = time_sync_ready_rst(&torque_loop_timer);
        bool run_telemetry = time_sync_ready_rst(&telemetry_timer);

        if (run_torque_loop) {
            float cur_measurement = ((float) res.I_motor / (float) UINT16_MAX) * AVDD_V;
            // TODO: recover current from voltage
            // TODO: estimate torque from current
            // TODO: filter?

            cur_measurement = iir_filter_update(&torque_filter, cur_measurement);

            float r = r_motor_board;

            float torque_setpoint = pid_calculate(&torque_pid, r, cur_measurement, TORQUE_LOOP_RATE_S);
            // TODO: transform desired torque to current
            u_torque_loop = torque_setpoint;

            // torque control data
            response_packet.data.motion.torque_setpoint = r;
            response_packet.data.motion.torque_estimate = cur_measurement;
            response_packet.data.motion.torque_computed_error = torque_pid.prev_err;
            response_packet.data.motion.torque_computed_setpoint = torque_setpoint;
        }

        if (run_torque_loop) {
            // set the motor duty cycle
            if (motion_control_type == OPEN_LOOP) {
                pwm6step_set_duty_cycle_f(r_motor_board);
            } else {
                pwm6step_set_duty_cycle_f(u_torque_loop);
            }

            // load system state for transmit

            // read error states
            const MotorErrors_t reported_motor_errors = pwm6step_get_motor_errors();

            response_packet.type = MRP_MOTION;
            response_packet.data.motion.master_error = false; // TODO update any error

            // bldc errors
            response_packet.data.motion.hall_power_error = reported_motor_errors.hall_power;
            response_packet.data.motion.hall_disconnected_error = reported_motor_errors.hall_disconnected;
            response_packet.data.motion.bldc_transition_error = reported_motor_errors.invalid_transitions;
            response_packet.data.motion.bldc_commutation_watchdog_error = reported_motor_errors.commutation_watchdog_timeout;

            // encoder errors
            response_packet.data.motion.enc_disconnected_error = false;
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

                // Clear the logging for the next UART transmit.
                uart_tx_clear_logging_status();
            }
#endif

            if (params_return_packet_requested) {
                params_return_packet_requested = false;

                response_packet.type = MRP_PARAMS;

                response_packet.data.params.vel_p = FLT_MIN;
                response_packet.data.params.vel_i = FLT_MIN;
                response_packet.data.params.vel_d = FLT_MIN;
                response_packet.data.params.vel_i_max = FLT_MIN;
                response_packet.data.params.cur_p = torque_pid_constants.kP;
                response_packet.data.params.cur_i = torque_pid_constants.kI;
                response_packet.data.params.cur_d = torque_pid_constants.kD;
                response_packet.data.params.torque_i_max = torque_pid_constants.kI_max;
                response_packet.data.params.cur_clamp = (uint16_t) cur_limit;

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

                // Clear the logging for the next UART transmit.
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
