
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

bool ioq_empty(IoQueue_t *q) {
    return (q->size == 0);
}

bool ioq_full(IoQueue_t *q) {
    return (q->size == IOQ_BUF_DEPTH);
}

uint8_t ioq_cur_size(IoQueue_t *q) {
    return (q->size);
}

bool ioq_write(IoQueue_t *q, uint8_t *buf, uint16_t len) {
    if (ioq_full(q)) {
        return false;
    }

    if (len > IOQ_BUF_LENGTH) {
        return false;
    }

    q->backing_sto[q->write_ind].len = len;
    memcpy(q->backing_sto[q->write_ind].buf, buf, len);

    q->size++;
    _increment_write_ind(q);

    return true;
}

bool ioq_peek_write(IoQueue_t *q, IoBuf_t **buf) {
    if (ioq_full(q)) {
        return false;
    }

    *buf = &q->backing_sto[q->write_ind];

    return true;
}

bool ioq_finalize_peek_write(IoQueue_t *q, IoBuf_t *buf) {
    if (ioq_full(q)) {
        return false;
    }

    // if (buf != &q->backing_sto[q->write_ind]) {
    //     return false;
    // }

    _increment_write_ind(q);
    q->size++;

    return true;
}

///////////////
//  READING  //
///////////////

uint8_t ioq_read(IoQueue_t *q, void *dest, uint8_t len) {
    if (ioq_empty(q)) {
        return 0;
    }

    uint8_t cpy_num_bytes = q->backing_sto[q->read_ind].len; 
    if (len < cpy_num_bytes) {
        return 0;
    }

    memcpy(dest, q->backing_sto[q->read_ind].buf, cpy_num_bytes);

    _increment_read_ind(q);
    q->size--;

    return cpy_num_bytes;
}

bool ioq_peek_read(IoQueue_t *q, IoBuf_t **dest) {
    if (ioq_empty(q)) {
        return false;
    }

    *dest = &q->backing_sto[q->read_ind];

    return true;
}

bool ioq_finalize_peek_read(IoQueue_t *q, IoBuf_t *dest) {
    if (ioq_empty(q)) {
        return false;
    }

    // looks like the hardware might actually increment the
    // mem addr, so we can't do this comparison without additional
    // storage and idt that's worth it
    // if (dest != &q->backing_sto[q->read_ind]) {
    //     return false;
    // }

    _increment_read_ind(q);
    q->size--;

    return true;
}