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

#include <stm32f031x6.h>

volatile bool uart_dma_tx_active = false;
volatile bool uart_dma_rx_active = false;

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

    // prevent nested/concurrent transfers
    uart_dma_tx_active = true;

    // clear all interrupt flags on the tx dma channel
    DMA1->IFCR = DMA_IFCR_CGIF2;

    // set the transmit buffer and length
    DMA1_Channel2->CMAR = (uint32_t) data_buf;
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

    // Activate channel
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    return true;
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