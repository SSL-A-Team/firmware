/*
 * ateam_entry.h
 *
 *  Created on: Feb 27, 2022
 *      Author: guyfl
 *
 *  This header/cpp file bridges the C HAL into our C++ code. This
 *  appears to fix syscall linking for some critical functions
 */

#ifndef ATEAM_ENTRY_H_
#define ATEAM_ENTRY_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
void robot_entry(UART_HandleTypeDef *radio_uart, UART_HandleTypeDef *serial_uart);
#ifdef __cplusplus
}
#endif

#endif /* ATEAM_ENTRY_H_ */
