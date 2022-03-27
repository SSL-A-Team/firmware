
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

void dma_transfer_error_interrupt(UART_HandleTypeDef *huart) {
	ateam::Robot::instance->__uart_transfer_error_int_cb(huart);
}

#ifdef __cplusplus
}
#endif
