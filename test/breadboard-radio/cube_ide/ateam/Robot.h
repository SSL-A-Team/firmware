/*
 * Robot.h
 *
 *  Created on: Feb 27, 2022
 *      Author: guyfl
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "main.h"

#include "OdinW2Radio.h"

namespace ateam {

class Robot {
public:
	Robot(UART_HandleTypeDef *radio_uart, UART_HandleTypeDef *serial_uart);
	virtual ~Robot();

	static Robot *instance;

	UART_HandleTypeDef *radio_uart;
	UART_HandleTypeDef *serial_uart;

	OdinW2Radio *radio;

	void run_forever();

	void __uart_transfer_complete_int_cb(UART_HandleTypeDef *uart_inst);
	void __uart_receive_complete_int_cb(UART_HandleTypeDef *uart_inst);
	void __uart_receive_line_idle_int_cb(UART_HandleTypeDef *uart_inst, uint16_t len);
	void __uart_error_int_cb(UART_HandleTypeDef *uart_inst);
};

} /* namespace ateam */

#endif /* ROBOT_H_ */
