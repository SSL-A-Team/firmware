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

bool uart_transmit_dma_pending();
bool uart_wait_for_transmission();
bool uart_transmit_dma(uint8_t *data_buf, uint16_t len);
bool uart_recv_dma(uint8_t *data_buf, uint16_t len);
