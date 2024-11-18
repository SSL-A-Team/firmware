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

typedef enum {
    UART_LOGGING_OK = 0,
    UART_LOGGING_DMA_TX_ERROR = 1,
    UART_LOGGING_DMA_RX_ERROR = 2,
    UART_LOGGING_DMA_RX_BUFFER_FULL = 3,
    UART_LOGGING_UART_RX_BUFFER_FULL = 4,
    UART_LOGGING_UART_RX_PARITY_ERROR = 5,
    UART_LOGGING_UART_RX_BUFFER_EMPTY = 6,
    UART_LOGGING_UART_RX_SIZE_MISMATCH = 7,
    UART_LOGGING_UART_TX_BUFFER_FULL = 8,
    UART_LOGGING_UART_TX_SIZE_MISMATCH = 9
} uart_logging_status_t;

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void uart_initialize();

void uart_wait_for_transmission();
void uart_transmit(uint8_t *data_buf, uint16_t len);

bool uart_can_read();
void uart_discard();
void uart_read(void *dest, uint16_t len);

static volatile uart_logging_status_t uart_logging_status;
void uart_clear_logging_status();