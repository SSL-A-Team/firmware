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

/////////////////////////
//  PRIVATE FUNCTIONS  //
/////////////////////////

void _uart_start_transmit_dma();
void _uart_start_receive_dma();
void _uart_receive_dma(bool parity_error);

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

void uart_wait_for_transmission() {
    while (uart_is_transmit_dma_pending());
}

void uart_transmit(uint8_t *data_buf, uint16_t len) {
    if (!ioq_write(&uart_tx_queue, data_buf, len)) {
        // Queue is either full or the length is too long.
        uart_logging_status = UART_LOGGING_UART_TX_BUFFER_FULL;
        if (len > IOQ_BUF_LENGTH) {
            uart_logging_status = UART_LOGGING_UART_TX_SIZE_MISMATCH;
        }
        return;
    }

    // dma transmission isn't in progress to keep scheduling dma writes
    // manually start the first/only transfer
    if (!uart_is_transmit_dma_pending()) {
        _uart_start_transmit_dma();
    }
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
    return (!ioq_is_empty(&uart_rx_queue));
}

void uart_discard() {
    uint8_t discard_runs = 0;
    // Try to empty the current queue. If it's empty, we're done.
    while (!ioq_is_empty(&uart_rx_queue)) {
        ioq_discard_write_back(&uart_rx_queue);

        // Don't want to get caught in an infinite loop,
        // so just clear at least the buffer depth.
        if (discard_runs++ > IOQ_BUF_DEPTH) {
            break;
        }
    }
}

void uart_clear_logging_status() {
    uart_logging_status = UART_LOGGING_OK;
}

void uart_read(void *dest, uint16_t len) {
    uint16_t num_bytes_to_read = 0;
    if (!ioq_read(&uart_rx_queue, dest, len, &num_bytes_to_read)) {
        // Can fail from empty queue or size mismatch.
        uart_logging_status = UART_LOGGING_UART_RX_BUFFER_EMPTY;
        // If the size number of bytes to read is not zero and
        // not matching, then we have a size mismatch.
        if (num_bytes_to_read != 0 && len != num_bytes_to_read) {
            uart_logging_status = UART_LOGGING_UART_RX_SIZE_MISMATCH;
        }
    }
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

void _uart_receive_dma(bool parity_error) {
    // If we have a parity error, we can just overwrite the queue
    // with the next packet.
    if (!parity_error) {
        // Get the buffer location that was written into with
        // last DMA contents.
        IoBuf_t *rx_buf;
        ioq_peek_write(&uart_rx_queue, &rx_buf);

        // Peek write discards the last entry if we are full,
        // so we will always be good on transfers.

        // CNDTR is number of bytes left in the DMA buffer so
        // (max transfer size - size left) is packet transfer size.
        uint16_t transmitted_bytes = (IOQ_BUF_LENGTH - DMA1_Channel3->CNDTR);
        rx_buf->len = transmitted_bytes;

        // data and len now correct, finalize write
        // If this returns false, implies RX happens twice without handling.
        if (!ioq_finalize_peek_write(&uart_rx_queue)) {
            uart_logging_status = UART_LOGGING_UART_RX_BUFFER_FULL;
        }
    } else {
        // If we have a parity error, we can just overwrite the queue
        // with the next packet.
        uart_logging_status = UART_LOGGING_UART_RX_PARITY_ERROR;
    }

    // Get the NEXT buffer for the DMA to write into.
    IoBuf_t* rx_buf_next;
    ioq_peek_write(&uart_rx_queue, &rx_buf_next);

    // Assigns the location and length for the NEXT DMA write.
    DMA1_Channel3->CMAR = (uint32_t) rx_buf_next->buf;
    DMA1_Channel3->CNDTR = IOQ_BUF_LENGTH;

    // DMA is now ready to receive the next payload, so enable DMA.
    DMA1_Channel3->CCR |= DMA_CCR_EN;
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

        // Receive, got a full buffer
        // This is sort of unexpected, we generally expect less than
        // max buffer length, which fires USART line idle. We probably got
        // two packets back to back and need to sort that out.
        if (DMA1->ISR & DMA_ISR_TCIF3) {
            uart_logging_status = UART_LOGGING_DMA_RX_BUFFER_FULL;
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
        if (!ioq_is_empty(&uart_tx_queue)) {
            _uart_start_transmit_dma();
        } else {
            // restore the ready flag
            uart_dma_tx_active = false;
        }
    }

    ////////////////////
    //   Reception    //
    ////////////////////
    // detected idle line before full buf
    if (uart_status_register & USART_ISR_IDLE) {
        // disable DMA
        DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
        // FUTURE: Need to fix pull-ups to disable idle interrupts
        // USART1->CR1 &= ~(USART_CR1_IDLEIE);

        // clear idle line int flag
        USART1->ICR |= USART_ICR_IDLECF;

        // Moves the received DMA data to the queue
        // and prepares the next DMA transfer.
        bool parity_error = (uart_status_register & USART_ISR_PE);
        _uart_receive_dma(parity_error);

        // Clear the parity error flag for the next transfer.
        USART1->ICR |= USART_ICR_PECF;

        // FUTURE: Need to fix pull-ups to enable idle interrupts
        // enable idle interrupts
        // USART1->CR1 |= USART_CR1_IDLEIE;
    }
}