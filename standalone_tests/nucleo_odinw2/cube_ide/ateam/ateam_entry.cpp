
#include "ateam_entry.h"

#include "Robot.h"

#ifdef __cplusplus
extern "C" {
#endif
int robot_entry(UART_HandleTypeDef *radio_uart, UART_HandleTypeDef *serial_uart) {
	ateam::Robot robot(radio_uart, serial_uart);
	robot.run_forever();
}
#ifdef __cplusplus
}
#endif
