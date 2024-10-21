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

#include "io_queue.h"
#include "uart.h"

volatile bool uart_dma_tx_active = false;
IoQueue_t uart_tx_queue;

volatile bool uart_dma_rx_active = false;
volatile int uart_dma_rx_num_bytes = 0;
IoQueue_t uart_rx_queue;
volatile IoBuf_t backing_sto[2];

/////////////////////////
//  PRIVATE FUNCTIONS  //
/////////////////////////

void _uart_start_transmit_dma();
void _uart_start_receive_dma();
void _uart_receive_dma();

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void uart_initialize() {

    ioq_initialize(&uart_tx_queue);
    ioq_initialize(&uart_rx_queue);

    // Make sure SYSCFG bit(s) are cleared for USART TX and RX
    // to be on DMA Ch2 and Ch3, respectively.
    SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART1TX_DMA_RMP);
    SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART1RX_DMA_RMP);

    _uart_start_receive_dma();
}

//////////
//  TX  //
//////////

bool uart_is_transmit_dma_pending() {
    return uart_dma_tx_active;
}

bool uart_wait_for_transmission() {
    while (uart_transmit_dma_pending());
}

bool uart_transmit(uint8_t *data_buf, uint16_t len) {
    if (!ioq_write(&uart_tx_queue, data_buf, len)) {
        // Queue is either full or the data length is invalid.
        return false;
    }

    // dma transmission isn't in progress to keep scheduling dma writes
    // manually start the first/only transfer
    if (!uart_is_transmit_dma_pending()) {
        _uart_start_transmit_dma();
    }

    return true;
}

void _uart_start_transmit_dma() {
    // Get the next data to read and send down the line.
    IoBuf_t *tx_buf;
    if (!ioq_peek_read(&uart_tx_queue, &tx_buf)) {
        // This would fail if the queue is empty, so just don't
        // start the DMA.
        return;
    }

    // Prevent nested/concurrent transfers
    uart_dma_tx_active = true;

    // clear the transfer complete flag
    USART1->ICR = USART_ICR_TCCF;
    // clear all interrupt flags on the tx dma channel
    DMA1->IFCR = DMA_IFCR_CGIF2;

    // set the transmit buffer and length
    DMA1_Channel2->CMAR = (uint32_t) tx_buf->buf;
    DMA1_Channel2->CNDTR = tx_buf->len;

    // enable DMA
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

//////////
//  RX  //
//////////

bool uart_can_read() {
    return (!ioq_empty(&uart_rx_queue));
}

void uart_discard() {
    ioq_discard(&uart_rx_queue);
}

bool uart_read(void *dest, uint16_t len, uint16_t* num_bytes_read) {
    return ioq_read(&uart_rx_queue, dest, len, num_bytes_read);
}

void _uart_start_receive_dma() {
    // get the next data to read and send down the line
    IoBuf_t *rx_buf;
    ioq_peek_write(&uart_rx_queue, &rx_buf);

    DMA1_Channel3->CMAR = (uint32_t) rx_buf->buf;
    DMA1_Channel3->CNDTR = IOQ_BUF_LENGTH;

    // enable DMA
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    USART1->ICR |= USART_ICR_IDLECF;
    USART1->CR1 |= USART_CR1_IDLEIE;
}

void _uart_receive_dma() {
    // get the next data to read and send down the line
    IoBuf_t *rx_buf;
    ioq_peek_write(&uart_rx_queue, &rx_buf);

    // Peak write discards the last entry if we are full,
    // so we will always be good on transfers.

    // CNDTR is number of bytes left so
    // (max tranfer size - size left) is transfer size.
    uint16_t transmitted_bytes = (IOQ_BUF_LENGTH - DMA1_Channel3->CNDTR);
    rx_buf->len = transmitted_bytes;

    // data and len now correct, finalize write
    // TODO JOE Need a hardware failure/indicator if this returns false.
    // Implies RX happens twice without handling
    ioq_finalize_peek_write(&uart_rx_queue);

    // Assigns the place and length for the write.
    DMA1_Channel3->CMAR = (uint32_t) rx_buf->buf;
    DMA1_Channel3->CNDTR = IOQ_BUF_LENGTH;
}

//////////////////////////
//  INTERRUPT HANDLERS  //
//////////////////////////

/**
 * @brief callback handler for USART DMA1 Ch2 (TX) and Ch3 (RX)
 *
 */
void DMA1_Channel2_3_IRQHandler() {
    // Check if DMA1 Ch2 (TX) has any interrupts.
    if (DMA1->ISR & DMA_ISR_GIF2) {
        // Transmit had a DMA error. Occurs when the USART is
        // reading / writing at a reserved address.
        if (DMA1->ISR & DMA_ISR_TEIF2) {
            uart_logging_status = UART_LOGGING_DMA_TX_ERROR;
            // In COMP_MODE, try to just clear the error and then
            // continue with transfers.
            #ifdef COMP_MODE
            // If TEIF2 is set, the CCR is disabled automatically.
            // First need to clear Transfer Error Flag.
            DMA1->IFCR |= DMA_IFCR_CTEIF2;
            // Then reset the DMA channel control (CCR).
            DMA1_Channel2->CCR |= DMA_CCR_EN;
            #endif
        }

        // DMA finished transfer to USART
        if (DMA1->ISR & DMA_ISR_TCIF2) {
            // Expect the USART1 Transfer Complete interrupt to
            // fire once last byte is out of
            // the FIFO/Transmit Register.
            // Since USART1 handler disables the Transfer Complete
            // interrupt, we need to re-enable it here.
            USART1->CR1 |= USART_CR1_TCIE;
        }

        // Clears the interrupt flags for Ch2.
        DMA1->IFCR |= DMA_IFCR_CGIF2;
    }

    // Check if DMA1 Ch3 (RX) has any interrupt.
    if (DMA1->ISR & DMA_ISR_GIF3) {
        // Receive had a DMA error
        if (DMA1->ISR & DMA_ISR_TEIF3) {
            uart_logging_status = UART_LOGGING_DMA_RX_ERROR;
            // In COMP_MODE, try to just clear the error and then
            // continue with transfers.
            #ifdef COMP_MODE
            // If TEIF3 is set, the CCR is disabled automatically.
            // First need to clear Transfer Error Flag.
            DMA1->IFCR |= DMA_IFCR_CTEIF3;
            // Then reset the DMA channel control (CCR).
            DMA1_Channel3->CCR |= DMA_CCR_EN;
            #endif
        }

        // receive, got a full buffer
        // TODO JOE look into this.
        // this is sort of unexpected, we generally expect a packet less than
        // max buffer length, which fires USART line idle. We probably got
        // two packets back to back and need to sort that out
        if (DMA1->ISR & DMA_ISR_TCIF3) {
            // _uart_receive_dma();
        }

        // Clears the interrupt flags for Ch3.
        DMA1->IFCR |= DMA_IFCR_CGIF3;
    }
}

/**
 * @brief callback handler for uart
 *
 */
__attribute((__optimize__("O0")))
void USART1_IRQHandler() {
    const uint32_t uart_status_register = USART1->ISR;

    ////////////////////
    //  Transmission  //
    ////////////////////

    // If it's a USART transmit complete interrupt:
    if (uart_status_register & USART_ISR_TC) {
        // Disable TX DMA channel until we
        // know we have a payload to send.
        DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
        // Disable transfer complete interrupts.
        USART1->CR1 &= ~(USART_CR1_TCIE);
        // Clear transmission complete flag.
        USART1->ICR |= USART_ICR_TCCF;

        // finalize tx queue read
        // check if queue is empty
        // conditionally recall transmit
        ioq_finalize_peek_read(&uart_tx_queue, NULL);
        if (!ioq_empty(&uart_tx_queue)) {
            _uart_start_transmit_dma();
        } else {
            // restore the ready flag
            uart_dma_tx_active = false;
        }
    }

    ////////////////////
    //   Reception    //
    ////////////////////
    // TODO Check parity?
    // TODO Finish looking over this.
    // detected idle line before full buf
    if (uart_status_register & USART_ISR_IDLE) {
        // disable DMA
        DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
        // disable idle interrupts
        // USART1->CR1 &= ~(USART_CR1_IDLEIE);

        // clear idle line int flag
        USART1->ICR |= USART_ICR_IDLECF;

        _uart_receive_dma();

        // enable idle interrupts
        // USART1->CR1 |= USART_CR1_IDLEIE;

        // enable DMA
        DMA1_Channel3->CCR |= DMA_CCR_EN;
    }
}