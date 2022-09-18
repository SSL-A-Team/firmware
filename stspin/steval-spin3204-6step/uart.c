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

bool _uart_start_transmit_dma();
void _uart_start_rx_dma();
void _uart_rx_dma();

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void uart_initialize() {
    ioq_initialize(&uart_tx_queue);
    ioq_initialize(&uart_rx_queue);

    _uart_start_rx_dma();
}

//////////
//  TX  //
//////////

bool uart_can_transmit() {
    return false;
}

bool uart_transmit_dma_pending() {
    return uart_dma_tx_active;
}

bool uart_wait_for_transmission() {
    while (uart_transmit_dma_pending());
}

bool uart_transmit(uint8_t *data_buf, uint16_t len) {
    ioq_write(&uart_tx_queue, data_buf, len);

    // dma transmission isn't in progress to keep scheduling dma writes
    // manually start the first/only transfer
    if (!uart_dma_tx_active) {
        _uart_start_transmit_dma();
    }
}

bool _uart_start_transmit_dma() {
    // get the next data to read and send down the line
    IoBuf_t *tx_buf;
    ioq_peek_read(&uart_tx_queue, &tx_buf);

    // prevent nested/concurrent transfers
    uart_dma_tx_active = true;

    // TODO this should be done by the interrupt callback,
    // maybe remove this
    // clear the transfer complete flag
    USART1->ICR = USART_ICR_TCCF;
    // clear all interrupt flags on the tx dma channel
    DMA1->IFCR = DMA_IFCR_CGIF2;

    // set the transmit buffer and length
    DMA1_Channel2->CMAR = (uint32_t) tx_buf->buf;
    DMA1_Channel2->CNDTR = tx_buf->len;

    // enable DMA
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    return true;
}

//////////
//  RX  //
//////////

bool uart_can_read() {
    return (!ioq_empty(&uart_rx_queue));
}

uint8_t uart_read(void *dest, uint8_t len) {
    return ioq_read(&uart_rx_queue, dest, len);
}

void _uart_start_rx_dma() {
    // get the next data to read and send down the line
    IoBuf_t *rx_buf;
    ioq_peek_write(&uart_rx_queue, &rx_buf);

    DMA1_Channel3->CMAR = (uint32_t) rx_buf->buf;
    DMA1_Channel3->CNDTR = IOQ_BUF_LENGTH;

    // DMA1_Channel3->CMAR = ( uint32_t) backing_sto[0].buf;
    // DMA1_Channel3->CNDTR = IOQ_BUF_LENGTH;

    // enable DMA
    DMA1_Channel3->CCR |= DMA_CCR_EN; 

    USART1->ICR |= USART_ICR_IDLECF;
    USART1->CR1 |= USART_CR1_IDLEIE;
}

void _uart_rx_dma() {
    // get the next data to read and send down the line
    IoBuf_t *rx_buf;
    ioq_peek_write(&uart_rx_queue, &rx_buf);

    // check if were filling the last slot
    // if (ioq_cur_size(&uart_rx_queue) < (IOQ_BUF_DEPTH - 1)) {
        uint8_t transmitted_bytes = (IOQ_BUF_LENGTH - DMA1_Channel3->CNDTR);
        rx_buf->len = transmitted_bytes;

        // data and len now correct, finalize write
        ioq_finalize_peek_write(&uart_rx_queue, NULL);

        // re-peek after potential finalization
        ioq_peek_write(&uart_rx_queue, &rx_buf);
    // }

    DMA1_Channel3->CMAR = (uint32_t) rx_buf->buf;
    DMA1_Channel3->CNDTR = IOQ_BUF_LENGTH;
}

//////////////////////////
//  INTERRUPT HANDLERS  //
//////////////////////////

/**
 * @brief callback handler for DMA RX
 * 
 */
void DMA1_Channel2_3_IRQHandler() {
    // check if DMA1 CH2 has any interrupt, this is for transmit
    if (DMA1->ISR & DMA_ISR_GIF2) {
        // transmit had a DMA error
        if (DMA1->ISR & DMA_ISR_TEIF2) {
            // TODO log error
        }

        // DMA finished transfer to USART
        if (DMA1->ISR & DMA_ISR_TCIF2) {
            // expect the USART1 TC interrupt to fire once last byte is out of the FIFO/TR
            // make sure tx complete inerrupts are on
            USART1->CR1 |= USART_CR1_TCIE;
        }

        DMA1->IFCR |= DMA_IFCR_CGIF2;
    }

    // check if DMA1 CH3 has any interrupt, this is for receive
    if (DMA1->ISR & DMA_ISR_GIF3) {
        // receive had a dma error
        if (DMA1->ISR & DMA_ISR_TEIF3) {
            // TODO: log error
        }

        // receive, got a full buffer
        // this is sort of unexpected, we generally expect a packet less than
        // max buffer length, which fires USART line idle. We probably got 
        // two packets back to back and need to sort that out
        if (DMA1->ISR & DMA_ISR_TCIF3) {
            // _uart_rx_dma();
        }

        DMA1->IFCR |= DMA_IFCR_CGIF3;
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
    //  Transmission  //
    ////////////////////

    if (uart_status_register & USART_ISR_TC) {
        // disable DMA channel
        DMA1_Channel2->CCR &= ~(DMA_CCR_EN);
        // disable transfer complete interrupts 
        USART1->CR1 &= ~(USART_CR1_TCIE);

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

    // detected idle line before full buf
    if (uart_status_register & USART_ISR_IDLE) {
        // disable DMA
        DMA1_Channel3->CCR &= ~(DMA_CCR_EN);
        // disable idle interrupts
        // USART1->CR1 &= ~(USART_CR1_IDLEIE);

        // clear idle line int flag
        USART1->ICR |= USART_ICR_IDLECF;

        _uart_rx_dma();

        // enable idle interrupts
        // USART1->CR1 |= USART_CR1_IDLEIE;

        // enable DMA
        DMA1_Channel3->CCR |= DMA_CCR_EN;
    }
}