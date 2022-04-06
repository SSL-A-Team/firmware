
#include <ateam_ext.h>
#include "Robot.h"

#ifdef __cplusplus
extern "C" {
#endif

void robot_entry(UART_HandleTypeDef *radio_uart, UART_HandleTypeDef *serial_uart) {
	ateam::Robot robot(radio_uart, serial_uart);
	robot.run_forever();
}

void dma_transfer_complete_interrupt(UART_HandleTypeDef *huart) {
	ateam::Robot::instance->__uart_transfer_complete_int_cb(huart);
}

void dma_receive_complete_interrupt(UART_HandleTypeDef *huart) {
	ateam::Robot::instance->__uart_receive_complete_int_cb(huart);
}

void dma_receive_line_idle_interrupt(UART_HandleTypeDef *huart, uint16_t len) {
	ateam::Robot::instance->__uart_receive_line_idle_int_cb(huart, len);
}

void dma_error_interrupt(UART_HandleTypeDef *huart) {
	ateam::Robot::instance->__uart_error_int_cb(huart);
}

#ifdef __cplusplus
}
#endif
