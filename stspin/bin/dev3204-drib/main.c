#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <stm32f031x6.h>

#include "ateam-common-packets/include/stspin.h"

#include "6step.h"
#include "current_sensing.h"
#include "debug.h"
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
    // Setups clocks
    setup();

    GPIOB->BSRR |= GPIO_BSRR_BR_8;
    GPIOB->BSRR |= GPIO_BSRR_BR_9;

    ADC_Result_t res;
    currsen_setup(ADC_MODE, &res, ADC_NUM_CHANNELS, ADC_CH_MASK, ADC_SR_MASK);

    pwm6step_setup();
    pwm6step_set_duty_cycle_f(0.0f);

    // enable ADC hardware trigger (tied to 6step timer)
    currsen_enable_ht();

    MotorResponsePacket_t response_packet;
    memset(&response_packet, 0, sizeof(MotorResponsePacket_t));
    bool params_return_packet_requested = false;

    SyncTimer_t torque_loop_timer;
    time_sync_init(&torque_loop_timer, 1);

    MotorCommand_MotionType_t motion_control_type = OPEN_LOOP;
    
    float r_motor_board = 0.0f;
    float u_torque_loop;
    float cur_limit = 0.0f;

    PidConstants_t torque_pid_constants;
    pid_constants_initialize(&torque_pid_constants);
    Pid_t torque_pid;
    pid_initialize(&torque_pid, &torque_pid_constants);

    // toggle J1-1
    while (true) {
        GPIOB->BSRR |= GPIO_BSRR_BR_8;
        GPIOB->BSRR |= GPIO_BSRR_BR_9;

#ifdef UART_ENABLED
        while (uart_can_read()) {
            MotorCommandPacket_t motor_command_packet;
            uint8_t bytes_moved = uart_read(&motor_command_packet, sizeof(MotorCommandPacket_t));
            if (bytes_moved != sizeof(MotorCommandPacket_t)) {
                // something went wrong, just purge all of the data
                uart_discard();
            }

            if (motor_command_packet.type == MCP_MOTION) {
                GPIOB->BSRR |= GPIO_BSRR_BS_8;

                // we got a motion packet!

                if (motor_command_packet.motion.reset) {
                    // TODO handle hardware reset

                    while (true); // block, hardware reset flagged
                }

                r_motor_board = motor_command_packet.motion.setpoint;
            } else if (motor_command_packet.type == MCP_PARAMS) {
                GPIOB->BSRR |= GPIO_BSRR_BS_9;

                // we got some params
                params_return_packet_requested = true;

                if (motor_command_packet.params.update_timestamp) {
                    time_set_epoch_seconds(motor_command_packet.params.update_timestamp);
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

        if (run_torque_loop) {
            float cur_measurement = (float) res.I_motor; // TODO: get current from dma
            // TODO: estimate torque from current
            // TODO: filter?

            float r = r_motor_board;

            float torque_setpoint = pid_calculate(&torque_pid, r, cur_measurement);
            // TODO: transform desired torque to current
            u_torque_loop = torque_setpoint;

            // torque control data
            response_packet.motion.current_setpoint = r;
            response_packet.motion.current_estimate = cur_measurement;
            response_packet.motion.current_computed_error = torque_pid.prev_err;
            response_packet.motion.current_computed_setpoint = torque_setpoint;
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

                response_packet.params.vel_p = FLT_MIN;
                response_packet.params.vel_i = FLT_MIN;
                response_packet.params.vel_d = FLT_MIN;
                response_packet.params.vel_i_max = FLT_MIN;
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
