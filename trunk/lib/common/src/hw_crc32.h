/*
 * hw_crc32.h
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

/* This is the interface description of the hw abstraction layer for the CRC32 
 * calculation unit present in stm32 chips. */

#ifndef HW_CRC32_H
#define	HW_CRC32_H

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
     * @brief Initializes the CRC32 calculation unit hardware.
     */
    void hw_crc32_init();
    
    /**
     * @brief Resets the CRC32 calculation unit for a new calculation.
     */
    void hw_crc32_reset();
    
    /**
     * @brief Digests a double word (32bit) variable through the CRC32 
     * algorithm.
     * 
     * @param dword The double word to digest.
     */
    void hw_crc32_digest(uint32_t dword);
    
    /**
     * @brief Digest a block of double words (32 bit) through the CRC32
     * algorithm.
     * 
     * @param block Pointer to the beginning of the double word block to
     *              digest.
     * @param length The amount of double words to digest.
     */
    void hw_crc32_digest_block(uint32_t* block, uint32_t length);
    
    /**
     * @brief Retrieves the current value of the CRC32 algorithm.
     * 
     * @return The current value of the CRC32 algorithm.
     */
    uint32_t hw_crc32_retrieve();

#ifdef	__cplusplus
}
#endif

#endif	/* HW_CRC32_H */

