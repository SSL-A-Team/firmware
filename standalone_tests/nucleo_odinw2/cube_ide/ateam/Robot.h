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

	static Robot *robot;

	UART_HandleTypeDef *radio_uart;
	UART_HandleTypeDef *serial_uart;

	void run_forever();
};

} /* namespace ateam */

#endif /* ROBOT_H_ */
