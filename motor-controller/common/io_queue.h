#pragma once

#include <stdint.h>

#include "system.h"

#ifndef IOQ_BUF_LENGTH
#error "User must define IOQ_BUF_LENGTH in bytes for backing buffer upper bound"
#endif

#ifndef IOQ_BUF_DEPTH
#error "User must define IOQ_BUF_DEPTH in bytes for backing buffer upper bound"
#endif

typedef struct IoBuf {
    uint8_t buf[IOQ_BUF_LENGTH];
    // u16 is underlying register size.
    volatile uint16_t len;
} IoBuf_t;

typedef struct IoQueue {
    volatile uint8_t size;
    volatile uint8_t read_ind;
    volatile uint8_t write_ind;
    // Holders are u8, so protect against bigger.
    _Static_assert(IOQ_BUF_DEPTH < 255);
    IoBuf_t backing_sto[IOQ_BUF_DEPTH];
} IoQueue_t;

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void ioq_initialize(IoQueue_t *q);

bool ioq_empty(IoQueue_t *q);
bool ioq_full(IoQueue_t *q);
uint8_t ioq_cur_size(IoQueue_t *q);

bool ioq_write(IoQueue_t *q, uint8_t *buf, uint16_t len);
void ioq_peek_write(IoQueue_t *q, IoBuf_t **buf);
bool ioq_finalize_peek_write(IoQueue_t *q);

bool ioq_read(IoQueue_t *q, void *dest, uint16_t len, uint16_t *num_bytes_read);
bool ioq_peek_read(IoQueue_t *q, IoBuf_t **dest);
bool ioq_finalize_peek_read(IoQueue_t *q, IoBuf_t *dest);
void ioq_discard(IoQueue_t *q);