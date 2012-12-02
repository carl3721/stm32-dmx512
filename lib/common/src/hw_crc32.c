/*
 * hw_crc32.c
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

/* This is the implementation file of the hw abstraction layer for the CRC32 
 * calculation unit present in stm32 chips. */

/*
 * -------------------- System Includes ----------------------------------------
 */

/*
 * -------------------- Local Includes -----------------------------------------
 */


#include "hw_crc32.h"

#ifdef STM32F10X_MD
#include "stm32f10x.h"
#include "stm32f10x_crc.h"
#include "stm32f10x_rcc.h"
#endif

#ifdef STM32F4XX
#include "stm32f4xx.h"
#include "stm32f4xx_crc.h"
#include "stm32f4xx_rcc.h"
#endif

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

void hw_crc32_init() {
#ifdef STM32F10X_MD
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
#endif
#ifdef STM32F4XX
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
#endif
}

void hw_crc32_reset() {
    CRC_ResetDR();
}

void hw_crc32_digest(uint32_t dword) {
    CRC_CalcCRC(dword);
}

void hw_crc32_digest_block(uint32_t* block, uint32_t length) {
    CRC_CalcBlockCRC(block, length);
}

uint32_t hw_crc32_retrieve() {
    return CRC_GetCRC();
}
