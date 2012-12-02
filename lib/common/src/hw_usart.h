/*
 * hw_usart.h
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

/* This is the interface description of the hw abstraction layer for the USARTS. */

// TODO: extend this layer to allow configuration of the stopbits, databits, parity, etc...
// TODO: add a convenience method for bulk receive of bytes.

#ifndef HW_USART3_H
#define	HW_USART3_H

/*
 * -------------------- System Includes ----------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * -------------------- Local Includes -----------------------------------------
 */

#include "main.h"

#ifdef	__cplusplus
extern "C" {
#endif

    /*
     * -------------------- Defines --------------------------------------------
     */

    /**
     * @brief Default setting for the RX and TX buffer sizes.
     */
#ifndef USART_BUFFER_SIZE
#define USART_BUFFER_SIZE 512
#endif

    /**
     * @brief Defines the amount of available USART devices on the STM32F10X_MD
     *        device family.
     */
#ifdef STM32F10X_MD
#define HW_USART_COUNT 3
#endif

    /**
     * @brief Defines the amount of available USART devices on the STM32F4XX
     *        device family.
     */
#ifdef STM32F4XX
#define HW_USART_COUNT 6
#endif

    /**
     * @brief Defines the USART device count to 0 for unsupported devices.
     */
#ifndef HW_USART_COUNT
#define HW_USART_COUNT 0
#endif

    /*
     * -------------------- Type definitions -----------------------------------
     */

    /**
     * @brief Enumerates the available USART peripherals.
     * 
     * This enum will grow/shrink to fit the amount of available USART 
     * peripherals.
     */
    typedef enum {
#if HW_USART_COUNT >= 1
        /**
         * @brief USART1 device.
         */ 
        HW_USART1 = 0,
#endif
#if HW_USART_COUNT >= 2
        /**
         * @brief USART2 device.
         */ 
        HW_USART2 = 1,
#endif
#if HW_USART_COUNT >= 3
        /**
         * @brief USART3 device.
         */ 
        HW_USART3 = 2,
#endif
#if HW_USART_COUNT >= 4
        /**
         * @brief UART4 device.
         */ 
        HW_UART4 = 3,
#endif
#if HW_USART_COUNT >= 5
        /**
         * @brief UART4 device.
         */ 
        HW_UART5 = 4,
#endif
#if HW_USART_COUNT >= 6
        /**
         * @brief USART6 device.
         */ 
        HW_USART6 = 5
#endif
    } hw_usart_e;

    /*
     * -------------------- Global variables -----------------------------------
     */

    /*
     * -------------------- Prototypes -----------------------------------------
     */

    /**
     * @brief Initializes the chosen USART peripheral.
     * 
     * Some USART devices are actually UART devices. They do not support
     * synchronous mode. In these cases, the flowControl parameter is ignored.
     * 
     * @param usart The USART device to initialize.
     * @param baudrate The baudrate for the communications.
     * @param rx_buf_size The rx buffer size. Set to 0 to use default.
     * @param tx_buf_size The tx buffer size. Set to 0 to use default.
     * @param flowControl Set to <code>true</code> to enable the hardware
     *        flow control. Set to <code>false</code> to disable the hardware 
     *        flow control. This parameter may be ignored.
     */
    void hw_usart_init(hw_usart_e usart, uint32_t baudrate,
            uint32_t rx_buf_size, uint32_t tx_buf_size, bool flowControl);
    
    /**
     * @brief Initializes the hardware pins for the chosen USART device.
     * 
     * The user application must implement this method to enable the hardware
     * pins for the USART device. If you have to do remapping of pins, this 
     * method is the place to do it.
     * 
     * @param usart The chosen USART device.
     */
    extern void hw_usart_init_pins(hw_usart_e usart);
    
    /**
     * @brief Transmits a single byte over the chosen USART device.
     * 
     * If the device is not initialized, calling this method has no effect.
     * 
     * @param usart The USART to transmit on.
     * @param byte The byte to transmit.
     */
    void hw_usart_tx(hw_usart_e usart, uint8_t byte);
    
    /**
     * @brief Convenience method to transmit a whole string.
     * 
     * If the device is not initialized, calling this method has no effect.
     * 
     * @param usart The chosen USART device.
     * @param string The string to transmit.
     */
    void hw_usart_tx_string(hw_usart_e usart, const char* string);
    
    /**
     * @brief Receives a byte from the chosen USART device.
     * 
     * This method will always return <code>false</code> if the USART device is
     * not initialized.
     * 
     * @param usart The chosen USART device.
     * @param byte Pointer to the byte to receive.
     * @return <code>True</code> if a byte was transferred to the pointer, 
     *         <code>false</code> if no byte was transferred.
     */
    bool hw_usart_rx(hw_usart_e usart, uint8_t* byte);

#ifdef USART_ALLOW_DEINIT
    /**
     * @brief This method de-initializes a USART device.
     * 
     * Calling this method on an uninitialized USART device has no effect.
     * After this method completes, the USART device will be back in the state 
     * it had after a reset of the microcontroller.
     * 
     * Support for this method has to be enabled by defining the 
     * <code>USART_ALLOW_DEINIT</code> preprocessor directive. This can be done
     * in main.h.
     * 
     * @param usart The chosen USART device.
     */
    void hw_usart_deinit(hw_usart_e usart);
    
    /**
     * @brief This method de-initializes the pins used by a USART device.
     * 
     * Calling this method on an uninitialized USART device has no effect.
     * After this method completes, the hardware pins of the micro should be 
     * returned to their reset states.
     * 
     * Support for this method has to be enabled by defining the 
     * <code>USART_ALLOW_DEINIT</code> preprocessor directive. This can be done
     * in main.h.
     * 
     * @param usart The chosen USART device.
     */
    extern void hw_usart_deinit_pins(hw_usart_e usart);
#endif

#ifdef	__cplusplus
}
#endif

#endif
