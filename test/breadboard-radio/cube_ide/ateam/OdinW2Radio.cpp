/*
 * OdinW2Radio.cpp
 *
 *  Created on: Feb 13, 2022
 *      Author: guyfl
 */

#include "OdinW2Radio.h"
#include "OdinW2Defines.h"

#include <stdio.h>
#include <string.h>
#include <memory.h>

namespace ateam {

/**
 * @brief Construct a new Odin W 2 Radio:: Odin W 2 Radio object
 * 
 * @param uart pointer to the STM32 uart connected to this radio
 * @param reset_pin_bank STM32 HAL reset pin bank
 * @param reset_pin_ind STM32 HAL reset pin index
 * @param rx_buf_len receive buffer size in elements
 * @param tx_buf_len transmit buffer size in elements
 */
OdinW2Radio::OdinW2Radio(UART_HandleTypeDef *uart, GPIO_TypeDef *reset_pin_bank, uint16_t reset_pin_ind, const int rx_buf_len, const int tx_buf_len) {
	this->uart = uart;
	this->uart_dma = nullptr;
	this->rx_buf_len = rx_buf_len;
	this->tx_buf_len = tx_buf_len;
}

/**
 * @brief Construct a new Odin W 2 Radio:: Odin W 2 Radio object
 * 
 * @param uart pointer to the STM32 uart connected to this radio
 * @param uart_dma pointer to the STM32 dma connected to the @link{uart} parameter
 * @param reset_pin_bank STM32 HAL reset pin bank
 * @param reset_pin_ind STM32 HAL reset pin index
 * @param rx_buf_len receive buffer size in elements
 * @param tx_buf_len transmit buffer size in elements
 */
OdinW2Radio::OdinW2Radio(UART_HandleTypeDef *uart, DMA_HandleTypeDef *uart_dma, GPIO_TypeDef *reset_pin_bank, uint16_t reset_pin_ind, const int rx_buf_len, const int tx_buf_len) {
	this->uart = uart;
	this->uart_dma = uart_dma;
	this->rx_buf_len = rx_buf_len;
	this->tx_buf_len = tx_buf_len;
}

/**
 * @brief Destroy the Odin W 2 Radio:: Odin W 2 Radio object
 * 
 */
OdinW2Radio::~OdinW2Radio() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief initialize the transmit and receive buffers on the heap
 * 
 */
void OdinW2Radio::allocate_uart_buffers() {
	rx_buf = new uint8_t[rx_buf_len];
	tx_buf = new uint8_t[tx_buf_len];
}

/**
 * @brief initialize the radio
 * 
 */
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

/**
 * @brief perform a hardware reset
 * 
 * 1. toggle the reset line for the manufacturere required minimum time (hold_time_ms)
 * 2. wait for the radio to send the Unsolicited Responce Code (URC) "+STARTUP"
 * 
 * @param hold_time_ms 
 */
void OdinW2Radio::hard_reset(int hold_time_ms) {
	HAL_GPIO_WritePin(this->reset_pin_bank, this->reset_pin_ind, GPIO_PIN_RESET);
	HAL_Delay(hold_time_ms);
	HAL_GPIO_WritePin(this->reset_pin_bank, this->reset_pin_ind, GPIO_PIN_SET);

	read_uart_dma_b(this->rx_buf_len);
	printf("received data %s", this->rx_buf);
}

/**
 * @brief perform a software reset
 * 
 * 1. toggle the reset line for the manufacturere required minimum time (hold_time_ms)
 * 2. wait for the radio to send the Unsolicited Responce Code (URC) "+STARTUP"
 * 
 * @param hold_time_ms 
 */
void OdinW2Radio::soft_reset() {
	int len = strlen(AT_SOFT_RESET);
	memcpy(tx_buf, AT_SOFT_RESET, len);
	write_uart_dma_b(len);

	read_uart_dma_b(this->rx_buf_len);
	printf("received data %s", this->rx_buf);
}

/**
 * @brief radio uart full-transfer completed hardware interrupt callback
 * 
 * Because the hardware invoked this call watch any prohibited actions
 * 	1. sending any bytes and blocking for a wait
 * 	2. using *printf
 * 	3. locking or unlocking any HAL lock
 * 
 */
void OdinW2Radio::__dma_interrupt_tx_complete() {
	this->uart_tx_state = UartTxState::DMA_TRANSMIT_COMPLETE_SUCC;
}

/**
 * @brief radio uart full-receive completed hardware interrupt callback
 * 
 * Because the hardware invoked this call watch any prohibited actions
 * 	1. sending any bytes and blocking for a wait
 * 	2. using *printf
 * 	3. locking or unlocking any HAL lock
 * 
 */
void OdinW2Radio::__dma_interrupt_rx_complete() {
	this->uart_rx_state = UartRxState::AT_LINE_RECEIVED_SUCC;
}

/**
 * @brief radio uart error hardware interrupt callback
 * 
 * Because the hardware invoked this call watch any prohibited actions
 * 	1. sending any bytes and blocking for a wait
 * 	2. using *printf
 * 	3. locking or unlocking any HAL lock
 * 
 * This call back is context dependent based on if the transmission
 * is using interrupts or DMA interrupts. If using interrupts and
 * a non-terminal error code is asserted (e.g. noise framing or parity)
 * the byte will still be transferred. If using DMA the DMA transfer should
 * be aborted by hal. You may want to manually abort here again. Then a DMA
 * rx should be re-initialized to capture any remaining bits of the
 * corrupted frame, this will need to be discarded. Then normal transfer
 * can be resumed again.
 * 
 */
void OdinW2Radio::__dma_interrupt_error() {
	// make sure the fsm never allowed tx and rx at the same time
	if (this->uart_tx_state != UartTxState::IDLE) {
		this->uart_tx_state = UartTxState::DMA_TRANSMIT_COMPLETE_ERROR;
	} else {
		this->uart_rx_state = UartRxState::AT_LINE_RECEIVED_ERROR;
	}
}

/**
 * @brief radio uart line idle hardware interrupt callback
 * 
 * @param len number of byte transferred into the buffer by the DMA action
 * 
 * This interrupt is called when the DMA receive experiences an idle frame, meaning
 * the radio sent fewer bytes than requested. This is most likely from a URC or manual
 * staging. Using the provided transfer length to determine the valid range of the buffer
 * and handle the data/URC.
 */
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
