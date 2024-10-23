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
    UART_LOGGING_DMA_TX_ERROR,
    UART_LOGGING_DMA_RX_ERROR,
    UART_LOGGING_DMA_RX_BUFFER_FULL,
    UART_LOGGING_UART_RX_BUFFER_FULL
} uart_logging_status_t;

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void uart_initialize();

bool uart_transmit_dma_pending();
void uart_wait_for_transmission();
bool uart_transmit(uint8_t *data_buf, uint16_t len);

bool uart_can_read();
void uart_discard();
bool uart_read(void *dest, uint16_t len, uint16_t* num_bytes_read);

static volatile uart_logging_status_t uart_logging_status;
