/**
 * @file uart.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <stm32f031x6.h>

#include "uart.h"

volatile bool uart_dma_tx_active = false;
uint8_t uart_tx_dma_buffer[DMA_TX_BUFFER_CAPACITY];

volatile bool uart_dma_rx_active = false;
uint8_t uart_rx_dma_buffer[DMA_RX_BUFFER_DEPTH][DMA_RX_BUFFER_CAPACITY];

/**
 * @brief check if a UART DMA transmission is pending
 * 
 * @return true if a tranmission is pending
 * @return false if the engine is free for transmission
 */
__attribute((__optimize__("O0")))
bool uart_transmit_dma_pending() {
    return uart_dma_tx_active;
}

/**
 * @brief waits for a DMA transmission to complete
 * 
 */
__attribute__((optimize("O0")))
bool uart_wait_for_transmission() {
    while (uart_transmit_dma_pending());
}

/**
 * @brief transmit a buffer via UART DMA
 * 
 * @param data_buf the buffer to transmit
 * @param len the length to transmit
 */
bool uart_transmit_dma(uint8_t *data_buf, uint16_t len) {
    // check if dma is already sending another set of data
    if (uart_dma_tx_active) {
        return false;
    }

    if (data_buf == NULL) {
        return false;
    }

    memcpy(uart_tx_dma_buffer, data_buf, len);

    // prevent nested/concurrent transfers
    uart_dma_tx_active = true;

    // clear all interrupt flags on the tx dma channel
    DMA1->IFCR = DMA_IFCR_CGIF2;

    // set the transmit buffer and length
    DMA1_Channel2->CMAR = (uint32_t) uart_tx_dma_buffer;
    DMA1_Channel2->CNDTR = len;

    // clear the transfer complete flag
    USART1->ICR = USART_ICR_TCCF;
    // enable transmission complete interrupt flag
    USART1->CR1 |= USART_CR1_TCIE;

    // enable DMA
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    return true;
}

/**
 * @brief receive a buffer via UART DMA
 * 
 * @param data_buf the buffer to recv
 * @param len the length to recv
 * @return bool true on success, false on failure
 */
bool uart_recv_dma(uint8_t *data_buf, uint16_t len) {
    // check if dma is already receiving another set of data
    if (uart_dma_rx_active) {
        return false;
    }

    uart_dma_rx_active = true;

    // Set USART RDR register address to source of transfer
    DMA1_Channel2->CMAR = USART_RDR_RDR;

    // Write mem addr in DMA control register as dest of transfer
    DMA1_Channel2->CMAR = (uint32_t) data_buf;

    // Set number of bytes to transfer to DMA control register
    DMA1_Channel2->CNDTR = len;

    // Set priority
    DMA1_Channel2->CCR |= (0x3U << DMA_CCR_PL_Pos);

    // Enable 

    // Activate channel
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    return true;
}

/**
 * @brief Process data once it's received on UART
 * 
 * @param data 
 * @param len 
 */
void usart_process_data(const void *data, size_t len) {
    const uint8_t *d = data;

    // TODO something
}

/**
 * @brief Check for new data from DMA
 * 
 */
void usart_rx_check() {
    static size_t old_pos;
    size_t pos;

    // Calculate position in buffer
    pos = ARRAY_LEN(uart_rx_dma_buffer) - (DMA1_Channel2->CNDTR);
    if (pos != old_pos) {
        if (pos < old_pos) {
            usart_process_data(&uart_rx_dma_buffer[old_pos], pos - old_pos);
        }
        else {
            usart_process_data(&uart_rx_dma_buffer[old_pos], ARRAY_LEN(uart_rx_dma_buffer) - old_pos);
            if (pos > 0) {
                usart_process_data(&uart_rx_dma_buffer[0], pos);
            }
        }
    }

    old_pos = pos;
}

/**
 * @brief callback handler for DMA RX
 * 
 */
void DMA1_Channel2_3_IRQHandler(void) {

    // Check half-transfer complete interrupt
    if (DMA1->ISR & DMA_ISR_HTIF2_Msk) {
        DMA1->ISR &= DMA_ISR_HTIF2_Msk;         // Clear half-transfer complete flag
        usart_rx_check();                       // Check for data to process
    }

    // Check transfer-complete interrupt
    if (DMA1->ISR & DMA_ISR_TCIF2_Msk) {
        DMA1->ISR &= DMA_ISR_TCIF2_Msk;         // Clear transfer complete flag
        usart_rx_check();                       // Check for data to process
    }

}

/**
 * @brief callback handler for uart
 * 
 */
__attribute((__optimize__("O0")))
void USART1_IRQHandler() {
    uint32_t uart_status_register = USART1->ISR;


    ////////////////////
    //   Reception    //
    ////////////////////
    if (uart_status_register & USART_ISR_RXNE) {
        // disable DMA channel
        DMA1_Channel2->CCR &= ~(DMA_CCR_EN);

        // restore the ready flag
        uart_dma_rx_active = false;
    }


    ////////////////////
    //  Transmission  //
    ////////////////////

    if (uart_status_register & USART_ISR_TC) {
        // disable transfer complete interrupts 
        USART1->CR1 &= ~(USART_CR1_TCIE);
        // disable DMA channel
        DMA1_Channel2->CCR &= ~(DMA_CCR_EN);

        // restore the ready flag
        uart_dma_tx_active = false;
    }
}