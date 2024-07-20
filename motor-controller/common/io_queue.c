
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "io_queue.h"
#include "system.h"

/////////////////////////
//  PRIVATE FUNCTIONS  //
/////////////////////////

void _increment_write_ind(IoQueue_t *q) {
    q->write_ind = (q->write_ind + 1) % IOQ_BUF_DEPTH;
}

void _increment_read_ind(IoQueue_t *q) {
    q->read_ind = (q->read_ind + 1) % IOQ_BUF_DEPTH;
}

void _decrement_read_ind(IoQueue_t *q) {
    q->read_ind = (q->read_ind - 1) % IOQ_BUF_DEPTH;
}

////////////////////////
//  PUBLIC FUNCTIONS  //
////////////////////////

void ioq_initialize(IoQueue_t *q) {
    q->size = 0;
    q->read_ind = 0;
    q->write_ind = 0;

    for (int i = 0; i < IOQ_BUF_DEPTH; i++) {
        q->backing_sto[i].len = 0;
    }
}

bool ioq_is_empty(IoQueue_t *q) {
    return (q->size == 0);
}

bool ioq_is_full(IoQueue_t *q) {
    return (q->size == IOQ_BUF_DEPTH);
}

uint8_t ioq_get_cur_size(IoQueue_t *q) {
    return (q->size);
}

bool ioq_write(IoQueue_t *q, uint8_t *buf, uint16_t len) {
    // Can't add to a full queue.
    if (ioq_full(q)) {
        return false;
    }

    // Can't write longer than our buffer size.
    if (len > IOQ_BUF_LENGTH) {
        return false;
    }

    // Do copy first and then set the length to confirm data 
    // is valid in the buffer.
    memcpy(q->backing_sto[q->write_ind].buf, buf, len);
    q->backing_sto[q->write_ind].len = len;

    // Critical. A packet can be removed from the queue between assigning size. Should be fast.
    __disable_irq();
    q->size++;
    __enable_irq();

    _increment_write_ind(q);

    return true;
}

bool ioq_peek_write(IoQueue_t *q, IoBuf_t **buf) {
    if (ioq_full(q)) {
        // If the queue is full, discard the back to maintain 
        // order and minimize data loss.
        ioq_discard_back(q);
        // TODO JOE FINISH
        return false;
    }

    // Gets the pointer for the next buffer to write to.
    *buf = &q->backing_sto[q->write_ind];

    return true;
}

bool ioq_finalize_peek_write(IoQueue_t *q, IoBuf_t *buf) {
    // Can't add to a full queue.
    if (ioq_full(q)) {
        return false;
    }

    _increment_write_ind(q);
    // Critical. A packet can be removed from the queue between assigning size. Should be fast.
    __disable_irq();
    q->size++;
    __enable_irq();

    return true;
}

///////////////
//  READING  //
///////////////

bool ioq_read(IoQueue_t *q, void *dest, uint16_t len, uint16_t* num_bytes_read) {
    // Can't read an empty queue.
    if (ioq_empty(q)) {
        return false;
    }

    uint16_t cpy_num_bytes = q->backing_sto[q->read_ind].len;
    // Intended read size is smaller than intended. 
    if (len < cpy_num_bytes) {
        return false;
    }

    memcpy(dest, q->backing_sto[q->read_ind].buf, cpy_num_bytes);
    _increment_read_ind(q);

    // Critical. A packet can be added to the queue between assigning size. Should be fast.
    __disable_irq();
    q->size--;
    __enable_irq();

    *num_bytes_read = cpy_num_bytes;
    return true;
}

bool ioq_peek_read(IoQueue_t *q, IoBuf_t **dest) {
    // Can't read an empty queue.
    if (ioq_empty(q)) {
        return false;
    }

    // Gets the pointer for the next buffer to be read from.
    *dest = &q->backing_sto[q->read_ind];

    return true;
}

bool ioq_finalize_peek_read(IoQueue_t *q, IoBuf_t *dest) {
    // Can't read an empty queue.
    if (ioq_empty(q)) {
        return false;
    }

    _increment_read_ind(q);
    // This is probably fine without disable since called from interrupt with high enough 
    // priority, but just to be safe.
    __disable_irq();
    q->size--;
    __enable_irq();

    return true;
}

void ioq_discard_back(IoQueue_t *q) {
    // Critical. Should be quick but we don't want to infinite loop 
    __disable_irq();
    _decrement_read_ind(q);
    q->size--;
    __enable_irq();
}