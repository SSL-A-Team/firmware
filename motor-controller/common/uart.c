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

static bool init_run = false;

static volatile uart_logging_status_tx_t uart_logging_status_tx;
static volatile uart_logging_status_rx_t uart_logging_status_rx;
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

    /////////////////
    //  Pin Setup  //
    /////////////////

    // PA14 USART1_TX
    // PA15 USART1_RX

    // clear PA13 and PA14 mode which start in AF for SWD
    GPIOA->MODER &= ~(GPIO_MODER_MODER13_0 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER14_1);
    // clear PA13 speed mode which starts at 50Mhz (0b11) for SWD
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR13_0 | GPIO_OSPEEDR_OSPEEDR13_1);
    // clear PA13 and PA14 pull up status
    GPIOA->PUPDR &= !(GPIO_PUPDR_PUPDR13_0 | GPIO_PUPDR_PUPDR13_1 | GPIO_PUPDR_PUPDR14_0 | GPIO_PUPDR_PUPDR14_1);

    // configure PA14 and PA15 AF mode
    GPIOA->MODER |= (GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);
    // configure PA14 and PA15 pin speed to 10Mhz
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR14_0 | GPIO_OSPEEDR_OSPEEDR15_0);
    // configure PA14 and PA15 pin pullup to UP
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR14_0 | GPIO_PUPDR_PUPDR15_0);
    // configure PA14 and PA15 alternate function
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL14_Pos);
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL15_Pos);

    // enable bus clock to the UART
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /////////////////////////////
    //  UART Peripheral Setup  //
    /////////////////////////////

    // make sure USART1 is disabled while configuring.
    USART1->CR1 &= ~(USART_CR1_UE);
    // 9-bits with parity (PCE) insert parity bit at the 9th bit
    // defaults to even parity
    USART1->CR1 |= (USART_CR1_M | USART_CR1_PCE);
    // Enable transmit and receive functionality.
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    // we don't need anything here
    USART1->CR2 = 0;
    // enable DMA for transmission and receive
    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    // set baud rate
    USART1->BRR = 0x18; // => 2 Mbaud/s

    // Enable the module now that configuration is finished.
    USART1->CR1 |= USART_CR1_UE;

    // Clear idle line interrupt
    USART1->ICR |= USART_ICR_IDLECF;
    // FUTURE: We can probably enable the idle line interrupt here
    // and leave it enabled after we add pullups to the bus.
    // USART1->CR1 |= USART_CR1_IDLEIE;

    //////////////////////////////////////
    //  Transmission DMA Channel Setup  //
    //////////////////////////////////////

    // Memory increment, memory to peripheral
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR;
    // DMA set to Medium Priority
    DMA1_Channel2->CCR |= DMA_CCR_PL_0;
    // clear buffer base addr
    DMA1_Channel2->CMAR = 0; // transmit buffer base addr, set at transmission time
    // clear transmission length
    DMA1_Channel2->CNDTR = 0; // transmit length, set at transmission time
    // set destination address as UART periperal transmission shift register
    DMA1_Channel2->CPAR = (uint32_t) &USART1->TDR; // USART1 data transmit register address
    // Enable the transfer complete (TCIE) and transfer error (TEIE) interrupts
    DMA1_Channel2->CCR |= (DMA_CCR_TEIE | DMA_CCR_TCIE);
    // Clear the Global Ch2 interrupt flag.
    DMA1->IFCR |= DMA_IFCR_CGIF2;

    /////////////////////////////////
    //  Receive DMA Channel Setup  //
    /////////////////////////////////

    // USART1_RX memory increment, peripheral to memory
    // Sets memory increment mode.
    DMA1_Channel3->CCR = DMA_CCR_MINC;
    // DMA set to High Priority
    DMA1_Channel3->CCR |= DMA_CCR_PL_1;
    // clear buffer base addr
    DMA1_Channel3->CMAR = (uint32_t) 0 ;
    // Set destination address as UART periperal receive register
    DMA1_Channel3->CPAR = (uint32_t) &USART1->RDR;
    // clear transmission length
    DMA1_Channel3->CNDTR = 0;
    // Clear the Global Ch3 interrupt flag.
    DMA1->IFCR |= DMA_IFCR_CGIF3;

    /////////////////
    //  DMA Setup  //
    /////////////////

    NVIC_SetPriority(USART1_IRQn, 10);
    NVIC_EnableIRQ(USART1_IRQn);

    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 10);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    // Make sure SYSCFG bit(s) are cleared for USART TX and RX
    // to be on DMA Ch2 and Ch3, respectively.
    SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART1TX_DMA_RMP);
    SYSCFG->CFGR1 &= ~(SYSCFG_CFGR1_USART1RX_DMA_RMP);

    _uart_start_receive_dma();

    init_run = true;
}

//////////
//  TX  //
//////////

bool uart_is_transmit_dma_pending() {
    return uart_dma_tx_active;
}

void uart_wait_for_transmission() {
    while (!init_run || uart_is_transmit_dma_pending());
}

void uart_transmit(uint8_t *data_buf, uint16_t len) {
    if (!ioq_write(&uart_tx_queue, data_buf, len)) {
        // Queue is either full or the length is too long.
        if (len > IOQ_BUF_LENGTH) {
            uart_logging_status_tx |= UART_LOGGING_UART_TX_SIZE_MISMATCH;
        }
        else {
            uart_logging_status_tx |= UART_LOGGING_UART_TX_BUFFER_FULL;
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
    USART1->ICR |= USART_ICR_TCCF;
    // clear all interrupt flags on the tx dma channel
    DMA1->IFCR |= DMA_IFCR_CGIF2;

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
    return (init_run && !ioq_is_empty(&uart_rx_queue));
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

void uart_tx_clear_logging_status() {
    uart_logging_status_tx = UART_LOGGING_OK;
}

uart_logging_status_tx_t uart_tx_get_logging_status() {
    return uart_logging_status_tx;
}

void uart_rx_clear_logging_status() {
    uart_logging_status_rx = UART_LOGGING_OK;
}

uart_logging_status_rx_t uart_rx_get_logging_status() {
    return uart_logging_status_rx;
}

void uart_read(void *dest, uint16_t len) {
    uint16_t num_bytes_to_read = 0;
    if (!ioq_read(&uart_rx_queue, dest, len, &num_bytes_to_read)) {
        // Can fail from empty queue or size mismatch.
        // If the size number of bytes to read is not zero and
        // not matching, then we have a size mismatch.
        if (num_bytes_to_read != 0 && len != num_bytes_to_read) {
            uart_logging_status_rx |= UART_LOGGING_UART_RX_SIZE_MISMATCH;
        } else {
            uart_logging_status_rx |= UART_LOGGING_UART_RX_BUFFER_EMPTY;
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
            uart_logging_status_rx |= UART_LOGGING_UART_RX_BUFFER_FULL;
        }
    } else {
        // If we have a parity error, we can just overwrite the queue
        // with the next packet.
        uart_logging_status_rx |= UART_LOGGING_UART_RX_PARITY_ERROR;
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
            uart_logging_status_tx |= UART_LOGGING_DMA_TX_ERROR;
            // Try to just clear the error and then
            // continue with transfers.
            // If TEIF2 is set, the CCR is disabled automatically.
            // First need to clear Transfer Error Flag.
            DMA1->IFCR |= DMA_IFCR_CTEIF2;
            // Then reset the DMA channel control (CCR).
            DMA1_Channel2->CCR |= DMA_CCR_EN;
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
            uart_logging_status_rx |= UART_LOGGING_DMA_RX_ERROR;
            // Try to just clear the error and then
            // continue with transfers.
            // If TEIF3 is set, the CCR is disabled automatically.
            // First need to clear Transfer Error Flag.
            DMA1->IFCR |= DMA_IFCR_CTEIF3;
            // Then reset the DMA channel control (CCR).
            DMA1_Channel3->CCR |= DMA_CCR_EN;
        }

        // Receive, got a full buffer
        // This is sort of unexpected, we generally expect less than
        // max buffer length, which fires USART line idle. We probably got
        // two packets back to back and need to sort that out.
        if (DMA1->ISR & DMA_ISR_TCIF3) {
            uart_logging_status_rx |= UART_LOGGING_DMA_RX_BUFFER_FULL;
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