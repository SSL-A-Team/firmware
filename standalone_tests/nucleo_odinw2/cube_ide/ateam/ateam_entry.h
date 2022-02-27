/*
 * robot.h
 *
 *  Created on: Feb 27, 2022
 *      Author: guyfl
 */

#ifndef ATEAM_ENTRY_H_
#define ATEAM_ENTRY_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
int robot_entry(UART_HandleTypeDef *radio_uart, UART_HandleTypeDef *serial_uart);
#ifdef __cplusplus
}
#endif

#endif /* ATEAM_ENTRY_H_ */
