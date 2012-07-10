/*
 * circularbuffer_u8.c
 * 
 * STM32 DMX512
 * Copyright (C) 2012 Erik Van Hamme, all rights reserved
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* This is the implementation file for the uint8_t circular buffer. */

/*
 * -------------------- System Includes ----------------------------------------
 */

#include <stdint.h>

/*
 * -------------------- Local Includes -----------------------------------------
 */

#include "circularbuffer_u8.h"

/*
 * -------------------- Defines ------------------------------------------------
 */

/*
 * -------------------- Type definitions ---------------------------------------
 */

/*
 * -------------------- Global variables ---------------------------------------
 */

/*
 * -------------------- Prototypes ---------------------------------------------
 */

/*
 * -------------------- Method implementations ---------------------------------
 */

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