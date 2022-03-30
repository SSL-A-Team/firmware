/*
 * OdinW2Radio.cpp
 *
 *  Created on: Feb 13, 2022
 *      Author: guyfl
 */

#include "OdinW2Radio.h"

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

}

void OdinW2Radio::hard_reset(int hold_time_ms) {
	HAL_GPIO_WritePin(this->reset_pin_bank, this->reset_pin_ind, GPIO_PIN_RESET);
	HAL_Delay(hold_time_ms);
	HAL_GPIO_WritePin(this->reset_pin_bank, this->reset_pin_ind, GPIO_PIN_SET);
}

void OdinW2Radio::__dma_interrupt_tx_complete() {
	this->uart_tx_state = UartTxState::DMA_TRANSMIT_COMPLETE_SUCC;
}

void OdinW2Radio::__dma_interrupt_tx_error() {
	this->uart_tx_state = UartTxState::DMA_TRANSMIT_COMPLETE_ERROR;
}

bool OdinW2Radio::write_uart_dma_nb(int len) {
	if (this->uart_tx_state != UartTxState::IDLE) {
		return false;
	}

	HAL_UART_Transmit_DMA(this->uart, this->tx_buf, len);
	this->uart_tx_state = UartTxState::DMA_TRANSMITTING;
	return true;
}

bool OdinW2Radio::write_uart_dma_b(int len) {
	write_uart_dma_nb(len);

	while (this->uart_tx_state != UartTxState::DMA_TRANSMIT_COMPLETE_SUCC || this->uart_tx_state != UartTxState::DMA_TRANSMIT_COMPLETE_ERROR) {
		HAL_Delay(5);
	}

	return this->uart_tx_state == UartTxState::DMA_TRANSMIT_COMPLETE_SUCC;
}

} /* namespace ateam */
