/*
 * packetizer.h
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

/* This is the interface description for the packetizer. */

/*
 * The function of the packetizer is to group bytes together until a fixed 
 * length packet is completely received.
 * The packetizer is also responsible for coping with lost bytes. If a byte is 
 * lost in transmission, the packet will not complete, and after a configurable
 * timeout, the data received thus far is discarded.
 */

#ifndef PACKETIZER_H
#define	PACKETIZER_H

/*
 * -------------------- System Includes ----------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * -------------------- Local Includes -----------------------------------------
 */

#include "circularbuffer_u8.h"
#include "main.h"

#ifdef	__cplusplus
extern "C" {
#endif

    /*
     * -------------------- Defines --------------------------------------------
     */
    
    // set default for packet length if needed.
#ifndef PACKETIZER_LENGTH
    /**
     * @brief Default value for the packet length.
     */
#define PACKETIZER_LENGTH 8
#endif
    
    // set default for packet buffer size
#ifndef PACKETIZER_BUFFER_PACKETS
    /**
     * @brief Default amount of packets that can fit in the buffer.
     */
#define PACKETIZER_BUFFER_PACKETS 4
#endif
    
    // set default for timeout ticks if needed.
#ifndef PACKETIZER_TIMEOUT_TICKS
    /**
     * @brief Default value for the timeout ticks.
     */
#define PACKETIZER_TIMEOUT_TICKS 50
#endif

    /*
     * -------------------- Type definitions -----------------------------------
     */

    /**
     * @brief The packetizer context structure definition.
     */
    typedef struct {
        /**
         * @brief This flag indicates if the context is initialized or not.
         */
        bool init;
        /**
         * @brief Pointer to the packet buffer.
         */
        circularbuffer_u8_s* pBuf;
        /**
         * @brief Amount of bytes in the packet buffer.
         */
        uint32_t pBufCount;
        /**
         * Counts how many ticks have past since the last receive.
         */
        uint32_t pBufSince;
    } packetizer_s;
    
    /*
     * -------------------- Global variables -----------------------------------
     */

    /*
     * -------------------- Prototypes -----------------------------------------
     */
    
    /**
     * @brief Initializes a packetizer context structure.
     * 
     * @param p The packetizer context structure.
     */
    void packetizer_init(packetizer_s* p);
    
    /**
     * @brief De-initializes a packetizer context structure.
     * 
     * @param p The packetizer context structure.
     */
    void packetizer_deinit(packetizer_s* p);
    
    /**
     * @brief Receive method for a single byte.
     * 
     * @param p The packetizer context structure.
     * @param byte The byte to receive.
     */
    void packetizer_rx(packetizer_s* p, uint8_t byte);

    /**
     * @brief Checker method to see if a complete packet is in the buffer.
     * 
     * @param p The packetizer context structure.
     * 
     * @return <code>True</code> if a full packet is in the buffer, 
     * <code>false</code> if not.
     */
    bool packetizer_complete(packetizer_s* p);
    
    /**
     * @brief Retrieve method for complete packets.
     * 
     * This method assumes that it is only called after a call to 
     * @link packetizer_complete() is made.
     * The method also assumes that the data buffer is sufficiently large to 
     * hold the completed packet.
     * 
     * @param p The packetizer context structure.
     * @param data Pointer to the target buffer for the completed packet.
     */
    void packetizer_packet(packetizer_s* p, uint8_t* data);
    
    /**
     * @brief Ticker method.
     * 
     * This method is required for the packetizer to know when a packet is no 
     * longer expected to complete. If the ticker value overruns the threshold, 
     * the data in the buffer is discarded.
     * 
     * @param p The packetizer context structure.
     */
    void packetizer_tick(packetizer_s* p);
    
#ifdef	__cplusplus
}
#endif

#endif

