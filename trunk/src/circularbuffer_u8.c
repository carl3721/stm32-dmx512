#include "circularbuffer_u8.h"

void circularbuffer_u8_init(circularbuffer_u8_s* cb, uint32_t size, 
        volatile uint8_t* elements) {
    cb->size = size;
    cb->start = 0;
    cb->count = 0;
    cb->elements = elements;
}

uint8_t circularbuffer_u8_full(circularbuffer_u8_s* cb) {
    return cb->count == cb->size ? 1: 0;
}

uint8_t circularbuffer_u8_empty(circularbuffer_u8_s* cb) {
    return cb->count == 0 ? 1: 0;
}

void circularbuffer_u8_push(circularbuffer_u8_s* cb, uint8_t byte) {
    uint32_t end = (cb->start + cb->count) % cb->size;
    cb->elements[end] = byte;
    if (cb->count == cb->size) {
        cb->start = (cb->start + 1) % cb->size;
    } else {
        cb->count++;
    }
}

uint8_t circularbuffer_u8_pop(circularbuffer_u8_s* cb) {
    uint8_t byte = cb->elements[cb->start];
    cb->start = (cb->start + 1) % cb->size;
    cb->count--;
    return byte;
}