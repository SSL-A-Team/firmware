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
#define COMP_MODE

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void uart_initialize();

bool uart_transmit_dma_pending();
bool uart_wait_for_transmission();
bool uart_transmit(uint8_t *data_buf, uint16_t len);

bool uart_can_read();
uint8_t uart_read(void *dest, uint8_t len);
void uart_discard();

volatile uart_logging_status_t uart_logging_status;

typedef enum {
    UART_LOGGING_OK = 0,
    UART_LOGGING_DMA_TX_ERROR,
    UART_LOGGING_DMA_RX_ERROR
} uart_logging_status_t;
