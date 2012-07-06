/*
 * protocol.c
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

/* This is the communication protocol implementation file. */

/*
 * -------------------- System Includes ----------------------------------------
 */

#include <stdint.h>

/*
 * -------------------- Local Includes -----------------------------------------
 */

#include "protocol.h"

/*
 * -------------------- Defines ------------------------------------------------
 */

/**
 * @brief Constant for the length (in bytes) of a message.
 */
#define PROTOCOL_MESSAGE_SIZE 7

/**
 * @brief Constant defining the timeout threshold in milliseconds.
 */
#define PROTOCOL_TIMEOUT_THRESHOLD 100

/*
 * -------------------- Type definitions ---------------------------------------
 */

/**
 * @brief Field structure for the message data.
 */
typedef struct {
    /**
     * @brief Bit field for a set operation.
     */
    uint8_t set : 1;
    /**
     * @brief Bit field for a get operation.
     */
    uint8_t get : 1;
    /**
     * @brief Bit field for indicating the packet is an ACK.
     */
    uint8_t ack : 1;
    /**
     * @brief Bit field for reporting an error.
     */
    uint8_t err : 1;
    /**
     * @brief Lower end of the DMX512 address range.
     * 
     * Must be non-zero.
     */
    uint16_t from : 10;
    /**
     * @brief Upper end of the DMX512 address range.
     * 
     * Set this to 0 for the RGB mode. If not 0, must be larger or equal
     * to the from address.
     */
    uint16_t to : 10;
    /**
     * @brief The color to set.
     * 
     * When setting/getting in RGB mode, this is the red component.
     */
    uint8_t col_red;
    /**
     * @brief The green component for RGB mode.
     * 
     * This is unused when not in RGB mode.
     */
    uint8_t green;
    /**
     * @brief The blue component for RGB mode.
     * 
     * This is unused when not in RGB mode.
     */
    uint8_t blue;
    /**
     * @brief The longitudinal redundancy check.
     * 
     * See http://en.wikipedia.org/wiki/Longitudinal_redundancy_check for 
     * more details.
     */
    uint8_t lrc;
} protocol_message_s;

/**
 * @brief Union to easily translate between the fields of the message and the 
 * contents of the backing byte array.
 */
typedef union {
    /**
     * @brief Message data as represented by a byte array.
     */
    uint8_t data[PROTOCOL_MESSAGE_SIZE];
    /**
     * @brief Access structure for the individual fields
     */
    protocol_message_s fields;
} protocol_message_u;

/*
 * -------------------- Global variables ---------------------------------------
 */
uint8_t protocol_rx_buf[PROTOCOL_MESSAGE_SIZE];
uint8_t protocol_rx_count;
uint8_t protocol_rx_timeout;

/*
 * -------------------- Prototypes ---------------------------------------------
 */

/**
 * @brief Calculates the LRC of the given data.
 * 
 * See http://en.wikipedia.org/wiki/Longitudinal_redundancy_check for more
 * details.
 * 
 * @param data The array containing the data.
 * @param length The length of the data to process.
 * 
 * @return The LRC of the given data.
 */
uint8_t protocol_lrc(uint8_t* data, uint8_t length);

/**
 * @brief Processes the given message.
 * 
 * The processing may result in calls to the external methods defined in 
 * @file protocol.h .
 * 
 * @param message Pointer to the message to be processed.
 */
void protocol_process_message(protocol_message_u* message);

/**
 * @brief Sends the message.
 * 
 * Calling this method will result in calls to the app_send_byte method.
 * 
 * @param message Pointer to the message to be sent.
 */
void protocol_send_message(protocol_message_u* message);

/*
 * -------------------- Method implementations ---------------------------------
 */

void protocol_tick() {

    // if there is data in the rx buffer, increment the timeout counter
    if (protocol_rx_count != 0) {
        protocol_rx_timeout += 40;
    }

    // if the timeout counter exceeds the threshold, reset rx count and timeout
    if (protocol_rx_timeout >= PROTOCOL_TIMEOUT_THRESHOLD) {
        protocol_rx_count = 0;
        protocol_rx_timeout = 0;
    }
}

void protocol_receive(uint8_t byte) {

    // store the byte in the rx buffer
    protocol_rx_buf[protocol_rx_count++] = byte;

    // reset the timeout counter
    protocol_rx_timeout = 0;

    // if we have received sufficient bytes for a complete message, process it
    if (protocol_rx_count == PROTOCOL_MESSAGE_SIZE) {

        // check the LRC, process message if valid
        uint8_t lrc = protocol_lrc(protocol_rx_buf, protocol_rx_count);
        if (lrc == 0) {

            // create a message and put the data in it
            protocol_message_u message;
            for (uint8_t i = 0; i < PROTOCOL_MESSAGE_SIZE; i++) {
                message.data[i] = protocol_rx_buf[i];
            }

            // process the message
            protocol_process_message(&message);
        }

        // reset the rx counter
        protocol_rx_count = 0;
    }
}

uint8_t protocol_lrc(uint8_t* data, uint8_t length) {
/*
    // code snatched from wikipedia
    uint8_t lrc = 0;
    while (length > 0) {
        lrc += *data++;
        length--;
    }
    return ((~lrc) + 1);
*/
    return 0;
}

void protocol_process_message(protocol_message_u* message) {

    // check the amount of set flags for validity
    uint8_t temp = message->fields.get + message->fields.set;
    if (temp != 1) {
        // message is not valid
        message->fields.err = 1;
        protocol_send_message(message);
        return;
    }

    // if we are commanded to perform a GET action
    if (message->fields.get) {

        // determine the GET mode
        if (message->fields.to == 0) {

            // get RGB mode

            // check for errors
            if ((message->fields.from == 0) || (message->fields.from > 510)) {
                // message is not valid
                message->fields.err = 1;
                protocol_send_message(message);
                return;
            }

            // call application method
            app_get_rgb(message->fields.from);

        } else {

            // normal get mode

            // check for errors
            if ((message->fields.from != message->fields.to) ||
                    (message->fields.from == 0) ||
                    (message->fields.from > 512)) {
                // message is not valid
                message->fields.err = 1;
                protocol_send_message(message);
                return;
            }

            // call application method
            app_get(message->fields.from);
        }

        // done processing the message
        return;
    }

    // if we are commanded to perform a set action
    if (message->fields.set) {

        // determine the SET mode
        if (message->fields.to == 0) {

            // set RGB mode

            // check for errors
            if ((message->fields.from == 0) || (message->fields.from > 510)) {
                // message is not valid
                message->fields.err = 1;
                protocol_send_message(message);
                return;
            }

            // call application method
            app_set_rgb(message->fields.from, message->fields.col_red,
                    message->fields.green, message->fields.blue);

        } else {

            // set normal mode

            // check for errors
            if ((message->fields.from != message->fields.to) ||
                    (message->fields.from == 0) ||
                    (message->fields.from > 512)) {
                // message is not valid
                message->fields.err = 1;
                protocol_send_message(message);
                return;
            }

            // call application method
            app_set(message->fields.from, message->fields.to,
                    message->fields.col_red);
        }

        // done processing the message
        return;
    }
}

void protocol_send_message(protocol_message_u* message) {
    // send all the bytes of the message
    for (uint8_t i = 0; i < PROTOCOL_MESSAGE_SIZE; i++) {
        app_send_byte(message->data[i]);
    }
}

void protocol_set_rgb_done(uint16_t addr) {
    // TODO: implement me
}

void protocol_get_rgb_done(uint16_t addr, uint8_t r, uint8_t g, uint8_t b) {
    // TODO: implement me
}

void protocol_set_done(uint16_t from, uint16_t to) {
    // TODO: implement me
}

void protocol_get_done(uint16_t addr, uint8_t c) {
    // TODO: implement me
}

void protocol_remote_set_rgb(uint16_t addr, uint8_t r, uint8_t g, uint8_t b) {
    
    // create the set RGB message
    protocol_message_u message;
    for (uint8_t i = 0; i < PROTOCOL_MESSAGE_SIZE; i++) {
        message.data[i] = 0;
    }
    message.fields.set = 1;
    message.fields.from = addr;
    message.fields.to = 0;
    message.fields.col_red = r;
    message.fields.green = g;
    message.fields.blue = b;
    
    // send the message
    protocol_send_message(&message);
    
    // TODO: set up a check to see if the message is ACK-ed properly
}
