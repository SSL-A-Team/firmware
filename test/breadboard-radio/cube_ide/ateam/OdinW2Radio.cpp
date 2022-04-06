/*
 * OdinW2Radio.cpp
 *
 *  Created on: Feb 13, 2022
 *      Author: guyfl
 */

#include "OdinW2Radio.h"

#include <stdio.h>
#include <string.h>
#include <memory.h>

namespace ateam {

OdinW2Radio::OdinW2Radio(UART_HandleTypeDef *uart, GPIO_TypeDef *reset_pin_bank, uint16_t reset_pin_ind, const int rx_buf_len, const int tx_buf_len) {
	this->uart = uart;
	this->uart_dma = nullptr;
	this->rx_buf_len = rx_buf_len;
	this->tx_buf_len = tx_buf_len;
}


OdinW2Radio::OdinW2Radio(UART_HandleTypeDef *uart, DMA_HandleTypeDef *uart_dma, GPIO_TypeDef *reset_pin_bank, uint16_t reset_pin_ind, const int rx_buf_len, const int tx_buf_len) {
	this->uart = uart;
	this->uart_dma = uart_dma;
	this->rx_buf_len = rx_buf_len;
	this->tx_buf_len = tx_buf_len;
}

OdinW2Radio::~OdinW2Radio() {
	// TODO Auto-generated destructor stub
}

void OdinW2Radio::allocate_uart_buffers() {
	rx_buf = new uint8_t[rx_buf_len];
	tx_buf = new uint8_t[tx_buf_len];
}

void OdinW2Radio::start() {
	printf("Sending AT_CGMI cmd\r\n");
	const char* get_mfg = "AT+CGMI\r\n";
	int len = strlen(get_mfg);
	memcpy(tx_buf, get_mfg, len);
	write_uart_dma_b(len);
	printf("Sent AT+CGMI cmd\r\n");
	bool status = read_uart_dma_b(this->rx_buf_len);
	if (status) {
		printf("Error on reply read\r\n");
	} else {
		printf("Read reply\r\n");
		//printf("Read %d bytes\r\n", this->rx_rx_len);
		//printf("Read data %s\r\n", this->rx_buf);
	}
}

void OdinW2Radio::hard_reset(int hold_time_ms) {
	HAL_GPIO_WritePin(this->reset_pin_bank, this->reset_pin_ind, GPIO_PIN_RESET);
	HAL_Delay(hold_time_ms);
	HAL_GPIO_WritePin(this->reset_pin_bank, this->reset_pin_ind, GPIO_PIN_SET);

	read_uart_dma_b(this->rx_buf_len);
	printf("received data %s", this->rx_buf);
}

void OdinW2Radio::__dma_interrupt_tx_complete() {
	this->uart_tx_state = UartTxState::DMA_TRANSMIT_COMPLETE_SUCC;
}

void OdinW2Radio::__dma_interrupt_rx_complete() {
	this->uart_rx_state = UartRxState::AT_LINE_RECEIVED_SUCC;
}

void OdinW2Radio::__dma_interrupt_error() {
	// make sure the fsm never allowed tx and rx at the same time
	if (this->uart_tx_state != UartTxState::IDLE) {
		this->uart_tx_state = UartTxState::DMA_TRANSMIT_COMPLETE_ERROR;
	} else {
		this->uart_rx_state = UartRxState::AT_LINE_RECEIVED_ERROR;
	}
}

void OdinW2Radio::__dma_interrupt_rx_line_idle(uint16_t len) {
	this->rx_rx_len = len;
	
	// if we didn't receive a full length packet, null terminate it just in case
	// someone wants to print it
	// ofc printing raw data packets can have nulls anyway, but we expect this to happen
	if (len < rx_buf_len) {
		this->rx_buf[len] = '\0';
	}

	this->uart_rx_state = UartRxState::AT_LINE_RECEIVED_SUCC;
}

bool OdinW2Radio::write_uart_dma_nb(int len) {
	if (this->uart_tx_state != UartTxState::IDLE) {
		return false;
	}

	this->uart_tx_state = UartTxState::DMA_TRANSMITTING;
	HAL_UART_Transmit_DMA(this->uart, this->tx_buf, len);
	return true;
}

bool OdinW2Radio::write_uart_dma_b(int len) {
	if (!write_uart_dma_nb(len)) {
		return false;
	}

	while (this->uart_tx_state != UartTxState::DMA_TRANSMIT_COMPLETE_SUCC && this->uart_tx_state != UartTxState::DMA_TRANSMIT_COMPLETE_ERROR) {
		HAL_Delay(5);
	}

	return this->uart_tx_state == UartTxState::DMA_TRANSMIT_COMPLETE_SUCC;
}

bool OdinW2Radio::read_uart_dma_nb(int len) {
	this->rx_req_len = len;
	this->rx_rx_len = 0;
	if (this->uart_rx_state != UartRxState::IDLE) {
		return false;
	}

	this->uart_rx_state = UartRxState::AT_RECEIVING_LINE;
	HAL_UARTEx_ReceiveToIdle_DMA(this->uart, this->rx_buf, len);
	return true;
}

bool OdinW2Radio::read_uart_dma_b(int len) {
	if (!read_uart_dma_nb(len)) {
		return false;
	}

	while (this->uart_rx_state != UartRxState::AT_LINE_RECEIVED_SUCC && this->uart_rx_state != UartRxState::AT_LINE_RECEIVED_ERROR) {
		HAL_Delay(5);
	}

	return this->uart_rx_state == UartRxState::AT_LINE_RECEIVED_SUCC;
}

} /* namespace ateam */
