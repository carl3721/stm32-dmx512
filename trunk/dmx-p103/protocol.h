/*
 * protocol.h
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

/* This is the communication protocol header file. */

#ifndef PROTOCOL_H
#define	PROTOCOL_H

/*
 * -------------------- System Includes ----------------------------------------
 */

#include <stdint.h>

/*
 * -------------------- Local Includes -----------------------------------------
 */

#ifdef	__cplusplus
extern "C" {
#endif

    /*
     * -------------------- Defines --------------------------------------------
     */

    /*
     * -------------------- Type definitions -----------------------------------
     */

    /*
     * -------------------- Global variables -----------------------------------
     */

    /*
     * -------------------- Prototypes -----------------------------------------
     */

    /**
     * @brief Ticker method. Should be called every 40 milliseconds.
     * 
     * The protocol handler uses this information to determine the packet
     * timeout condition.
     */
    void protocol_tick();
    
    /**
     * @brief Receive method. 
     * 
     * Receives each byte as they come in from the communications channel.
     * 
     * @param byte The received byte.
     * @return Returns 1 if a message is complete, 0 if not.
     */
    void protocol_receive(uint8_t byte);

    /**
     * @brief Sends a single byte over the communications channel.
     * 
     * This method must be implemented by the application. It is used by the
     * protocol stack to send return messages.
     */
    extern void app_send_byte(uint8_t byte);

    void protocol_remote_set_rgb(uint16_t addr, uint8_t r, uint8_t g, uint8_t b);
    void protocol_remote_get_rgb(uint16_t addr);
    void protocol_remote_set(uint16_t from, uint16_t to, uint8_t c);
    void protocol_remote_get(uint16_t addr);
    extern void app_set_rgb(uint16_t addr, uint8_t r, uint8_t g, uint8_t b);
    extern void app_get_rgb(uint16_t addr);
    extern void app_set(uint16_t from, uint16_t to, uint8_t c);
    extern void app_get(uint16_t addr);

    void protocol_set_rgb_done(uint16_t addr);
    void protocol_get_rgb_done(uint16_t addr, uint8_t r, uint8_t g, uint8_t b);
    void protocol_set_done(uint16_t from, uint16_t to);
    void protocol_get_done(uint16_t addr, uint8_t c);

#ifdef	__cplusplus
}
#endif

#endif

