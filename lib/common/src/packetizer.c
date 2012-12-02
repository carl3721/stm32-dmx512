/*
 * packetizer.c
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

/* This is the implementation file for the packetizer. */

/*
 * -------------------- System Includes ----------------------------------------
 */

#include <stdlib.h>

/*
 * -------------------- Local Includes -----------------------------------------
 */

#include "packetizer.h"

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

void packetizer_init(packetizer_s* p) {
    if (p && (p->init == false)) {
        uint32_t size = PACKETIZER_LENGTH * PACKETIZER_BUFFER_PACKETS;
        volatile uint8_t* pBufArray = (volatile uint8_t*) malloc(size);
        if (pBufArray) {
            p->pBuf = (circularbuffer_u8_s*) malloc(sizeof (circularbuffer_u8_s));
            if (p->pBuf) {
                circularbuffer_u8_init(p->pBuf, size, pBufArray);
                p->pBufCount = 0;
                p->pBufCount = 0;
                p->init = true;
            }
        }
    }
}

void packetizer_deinit(packetizer_s* p) {
    if (p && (p->init == true)) {
        p->init = false;
        if (p->pBuf) {
            uint8_t* pBufArray = (uint8_t*) p->pBuf->elements;
            if (pBufArray) {
                free(pBufArray);
            }
            free(p->pBuf);
        }
        p->pBufCount = 0;
        p->pBufCount = 0;
    }
}

void packetizer_rx(packetizer_s* p, uint8_t byte) {
    if (p && (p->init == true)) {
        if (circularbuffer_u8_full(p->pBuf) != 1) {
            circularbuffer_u8_push(p->pBuf, byte);
            p->pBufCount++;
            p->pBufSince = 0;
        }
    }
}

bool packetizer_complete(packetizer_s* p) {
    if (p && (p->init == true)) {
        if (p->pBufCount >= PACKETIZER_LENGTH) {
            return true;
        }
    }
    return false;
}

void packetizer_packet(packetizer_s* p, uint8_t* data) {
    if (p && (p->init == true)) {
        for(uint32_t i = 0; i < PACKETIZER_LENGTH; i++) {
            data[i] = circularbuffer_u8_pop(p->pBuf);
            p->pBufCount--;
        }
    }
}

void packetizer_tick(packetizer_s* p) {
    if (p && (p->init == true)) {
        if ((p->pBufCount > 0) && (p->pBufSince > PACKETIZER_TIMEOUT_TICKS)) {
            // Reinitialize the circular buffer, this has the effect that all
            // data in it is cleared.
            // We can clear the whole buffer because the newest byte in the
            // buffer is already older then the threshold.
            // Reuse the size and elements items.
            circularbuffer_u8_init(p->pBuf, p->pBuf->size, p->pBuf->elements);
        }
    }
}