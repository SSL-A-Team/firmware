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

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void uart_initialize();

bool uart_can_transmit();
bool uart_transmit_dma_pending();
bool uart_wait_for_transmission();
bool uart_transmit(uint8_t *data_buf, uint16_t len);

bool uart_can_read();
uint8_t uart_read(void *dest, uint8_t len);

