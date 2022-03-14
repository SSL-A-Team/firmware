/*
 * Robot.cpp
 *
 *  Created on: Feb 27, 2022
 *      Author: guyfl
 */

#include "Robot.h"

#include <stdio.h>
#include <unistd.h>

namespace ateam {

Robot *Robot::instance;

Robot::Robot(UART_HandleTypeDef *radio_uart, UART_HandleTypeDef *serial_uart) {
	this->instance = this;
	this->radio_uart = radio_uart;
	this->serial_uart = serial_uart;
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}

void Robot::run_forever() {
	// LD1=Green, LD2=Blue, LD3=Red
	HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);

	OdinW2Radio radio(this->radio_uart, USART2_RST_GPIO_Port, USART2_RST_Pin);
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
	radio.hard_reset();
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);

	printf("stdout == %d\r\n", STDOUT_FILENO);
	fprintf(stderr, "stdout == %d\r\n", STDERR_FILENO);
//	fprintf(stdout, "stdout == %d\r\n", STDIN_FILENO);
	fprintf((FILE*)0xFFFF, "invalid fh\r\n");

	//while(true);

	int nLoop=0;
	while (true) {
		  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
		  HAL_Delay(500);
		  nLoop++;
		  //fprintf(stdout, "nLoop == %d\r\n", nLoop);
		  //fprintf((FILE*) 0xFFFF, "stdout == %d\r\n", stdout);
	}
}

void Robot::__uart_transfer_complete_int_cb(UART_HandleTypeDef *uart_instance) {
	if (uart_instance == radio_uart) {
		radio->__dma_interrupt_tx_complete();
	}
}

void Robot::__uart_transfer_error_int_cb(UART_HandleTypeDef *uart_instance) {
	if (uart_instance == radio_uart) {
		radio->__dma_interrupt_tx_error();
	}
}

} /* namespace ateam */
