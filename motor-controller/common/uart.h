/**
 * @file uart.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-04-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#define UART_LOGGING_OK 0

typedef enum {
    UART_LOGGING_DMA_TX_ERROR = 1 << 0,
    UART_LOGGING_UART_TX_BUFFER_FULL = 1 << 1,
    UART_LOGGING_UART_TX_SIZE_MISMATCH = 1 << 2
} uart_logging_status_tx_t;

typedef enum {
    UART_LOGGING_DMA_RX_ERROR = 1 << 0,
    UART_LOGGING_DMA_RX_BUFFER_FULL = 1 << 1,
    UART_LOGGING_UART_RX_BUFFER_FULL = 1 << 2,
    UART_LOGGING_UART_RX_PARITY_ERROR = 1 << 3,
    UART_LOGGING_UART_RX_BUFFER_EMPTY = 1 << 4,
    UART_LOGGING_UART_RX_SIZE_MISMATCH = 1 << 5,
} uart_logging_status_rx_t;

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void uart_initialize();

void uart_wait_for_transmission();
void uart_transmit(uint8_t *data_buf, uint16_t len);

bool uart_can_read();
void uart_discard();
void uart_read(void *dest, uint16_t len);

void uart_tx_clear_logging_status();
uart_logging_status_tx_t uart_tx_get_logging_status();

void uart_rx_clear_logging_status();
uart_logging_status_rx_t uart_rx_get_logging_status();