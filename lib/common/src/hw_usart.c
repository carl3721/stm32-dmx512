/*
 * hw_usart.c
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

/* This is the implementation file of the USART HAL. */

/*
 * -------------------- System Includes ----------------------------------------
 */

#include <stdbool.h>
#include <stdlib.h>

/*
 * -------------------- Local Includes -----------------------------------------
 */

#include "circularbuffer_u8.h"
#include "hw_usart.h"

#ifdef STM32F10X_MD
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#endif

#ifdef STM32F4XX
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#endif

/*
 * -------------------- Defines ------------------------------------------------
 */

/*
 * -------------------- Type definitions ---------------------------------------
 */

/*
 * -------------------- Global variables ---------------------------------------
 */

#ifdef STM32F10X_MD
USART_TypeDef* usarts[] = {USART1, USART2, USART3};
uint32_t irqns[] = {USART1_IRQn, USART2_IRQn, USART3_IRQn};
bool hw_usart_enabled[] = {false, false, false};
#endif

#ifdef STM32F4XX
USART_TypeDef* usarts[] = {USART1, USART2, USART3, UART4, UART5, USART6};
uint32_t irqns[] = {USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn,
    UART5_IRQn, USART6_IRQn};
bool hw_usart_enabled[] = {false, false, false, false, false, false};
#endif

circularbuffer_u8_s* hw_usart_rx_bufs[HW_USART_COUNT];
circularbuffer_u8_s* hw_usart_tx_bufs[HW_USART_COUNT];

/*
 * -------------------- Prototypes ---------------------------------------------
 */

/**
 * @brief Allocates the transmit and receive buffer for the chosen USART device.
 * 
 * @param usart The chosen USART device.
 * @param rx_buf_size The size of the rx buffer. Set to <code>0</code> to use 
 *                    the default setting.
 * @param tx_buf_size The size of the tx buffer. Set to <code>0</code> to use
 *                    the default setting.
 */
void hw_usart_init_buffers(hw_usart_e usart, uint32_t rx_buf_size, uint32_t tx_buf_size);

/**
 * @brief Catch-all IRQ handler for all USART devices.
 * 
 * This file includes implementations of the standard CMSIS USART IRQ handler. 
 * These handlers all call this catch all handler.
 * 
 * @param usart The USART device that triggered the interrupt.
 */
void hw_usart_irq(hw_usart_e usart);

// only declare deinit buffers proto if deinit is supported
#ifdef USART_ALLOW_DEINIT

/**
 * @brief De-allocates the buffers used by the chosen USART.
 * 
 * @param usart The chosen USART device.
 */
void hw_usart_deinit_buffers(hw_usart_e usart);

#endif

/*
 * -------------------- Method implementations ---------------------------------
 */

void hw_usart_init_buffers(hw_usart_e usart, uint32_t rx_buf_size, uint32_t tx_buf_size) {

    // determine the buffer sizes
    uint32_t rx_size = rx_buf_size > 0 ? rx_buf_size : USART_BUFFER_SIZE;
    uint32_t tx_size = tx_buf_size > 0 ? tx_buf_size : USART_BUFFER_SIZE;

    // allocate the buffer backing arrays
    uint8_t* rx = (uint8_t *) malloc(rx_size);
    uint8_t* tx = (uint8_t *) malloc(tx_size);

    // allocate the buffers
    hw_usart_rx_bufs[usart] = (circularbuffer_u8_s *)
            malloc(sizeof (circularbuffer_u8_s));
    hw_usart_tx_bufs[usart] = (circularbuffer_u8_s *)
            malloc(sizeof (circularbuffer_u8_s));

    // initialize the buffers
    circularbuffer_u8_init(hw_usart_rx_bufs[usart], rx_size, rx);
    circularbuffer_u8_init(hw_usart_tx_bufs[usart], tx_size, tx);
}

// only implement prototypes if deinit is supported
#ifdef USART_ALLOW_DEINIT

void hw_usart_deinit_buffers(hw_usart_e usart) {

    if (hw_usart_rx_bufs[usart]) {
        volatile uint8_t* rx = hw_usart_rx_bufs[usart]->elements;
        if (rx) {
            free((void *) rx);
        }
        free(hw_usart_rx_bufs[usart]);
        hw_usart_rx_bufs[usart] = 0;
    }

    if (hw_usart_tx_bufs[usart]) {
        volatile uint8_t* tx = hw_usart_tx_bufs[usart]->elements;
        if (tx) {
            free((void *) tx);
        }
        free(hw_usart_tx_bufs[usart]);
        hw_usart_tx_bufs[usart] = 0;
    }
}

// provide a weakly linked default implementation for pin deinit.
#pragma weak hw_usart_deinit_pins = _hw_usart_deinit_pins

void _hw_usart_deinit_pins(hw_usart_e usart) {
    uint8_t u = (uint8_t) usart;
    u = 0;
}
#endif

void hw_usart_init(hw_usart_e usart, uint32_t baudrate,
        uint32_t rx_buf_size, uint32_t tx_buf_size, bool flowControl) {

    // init the hardware pins
    hw_usart_init_pins(usart);

    // init the buffers for the interrupt
    hw_usart_init_buffers(usart, rx_buf_size, tx_buf_size);

    // enable the peripheral clock
    switch (usart) {
#if HW_USART_COUNT >= 1
        case HW_USART1:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
            break;
#endif
#if HW_USART_COUNT >=2
        case HW_USART2:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
            break;
#endif
#if HW_USART_COUNT >=3
        case HW_USART3:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
            break;
#endif
#if HW_USART_COUNT >=4
        case HW_USART4:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
            break;
#endif
#if HW_USART_COUNT >=5
        case HW_USART5:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
            break;
#endif
#if HW_USART_COUNT >=6
        case HW_USART6:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
            break;
#endif
    }

    // init and enable the usart
    USART_InitTypeDef usartCfg;
    usartCfg.USART_BaudRate = baudrate;
    if (flowControl) {
        usartCfg.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
    } else {
        usartCfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    }
    usartCfg.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usartCfg.USART_Parity = USART_Parity_No;
    usartCfg.USART_StopBits = USART_StopBits_1;
    usartCfg.USART_WordLength = USART_WordLength_8b;
    USART_Init(usarts[usart], &usartCfg);
    USART_Cmd(usarts[usart], ENABLE);

    // enable the usart irq
    NVIC_EnableIRQ(irqns[usart]);
    USART_ITConfig(usarts[usart], USART_IT_RXNE, ENABLE);

    // set enabled flag
    hw_usart_enabled[usart] = true;
}

#ifdef USART_ALLOW_DEINIT

void hw_usart_deinit(hw_usart_e usart) {

    // unset enabled flag
    hw_usart_enabled[usart] = false;

    // disable the usart irq
    USART_ITConfig(usarts[usart], USART_IT_RXNE | USART_IT_TXE, DISABLE);
    NVIC_DisableIRQ(irqns[usart]);

    // disable and deinit the usart
    USART_Cmd(usarts[usart], DISABLE);
    USART_DeInit(usarts[usart]);

    // disable the peripheral clock
    switch (usart) {
#if HW_USART_COUNT >=1
        case HW_USART1:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
            break;
#endif
#if HW_USART_COUNT >=2
        case HW_USART2:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
            break;
#endif
#if HW_USART_COUNT >=3
        case HW_USART3:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);
            break;
#endif
#if HW_USART_COUNT >=4
        case HW_USART4:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, DISABLE);
            break;
#endif
#if HW_USART_COUNT >=5
        case HW_USART5:
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, DISABLE);
            break;
#endif
#if HW_USART_COUNT >=6
        case HW_USART6:
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, DISABLE);
            break;
#endif
    }

    // deinit the buffers for the interrupt
    hw_usart_deinit_buffers(usart);

    // deinit the hardware pins
    hw_usart_deinit_pins(usart);
}
#endif

void hw_usart_tx(hw_usart_e usart, uint8_t byte) {

    // drop bytes if usart is not enabled
    if (hw_usart_enabled[usart] == false) {
        return;
    }

    // if the TX buffer is full, drop bytes
    if (circularbuffer_u8_full(hw_usart_tx_bufs[usart])) {
        return;
    }

    // add the byte to the buffer
    circularbuffer_u8_push(hw_usart_tx_bufs[usart], byte);

    // if we must, start the send
    if ((usarts[usart]->CR1 & (1 << 6)) == 0) {

        // enable the TC interrupt and send the byte
        usarts[usart]->DR = circularbuffer_u8_pop(hw_usart_tx_bufs[usart]);
        usarts[usart]->CR1 |= (1 << 6);
    }
}

bool hw_usart_rx(hw_usart_e usart, uint8_t* byte) {

    if (hw_usart_enabled[usart] == false) {
        return false;
    }

    if (circularbuffer_u8_empty(hw_usart_rx_bufs[usart])) {
        return false;
    }

    *byte = circularbuffer_u8_pop(hw_usart_rx_bufs[usart]);
    return true;
}

void hw_usart_tx_string(hw_usart_e usart, const char* string) {
    uint32_t count = 0;
    while (1) {
        uint8_t c = string[count++];
        if (c == '\0') {
            break;
        }
        hw_usart_tx(usart, c);
    }
}

void hw_usart_irq(hw_usart_e usart) {

    // RX interrupt
    if (usarts[usart]->SR & USART_FLAG_RXNE) {

        // clear interrupt bit
        usarts[usart]->SR &= !USART_FLAG_RXNE;

        // retrieve the byte
        uint8_t byte = usarts[usart]->DR;

        // store the byte in the RX buffer if there is space, otherwise drop it
        if (circularbuffer_u8_full(hw_usart_rx_bufs[usart]) == 0) {
            circularbuffer_u8_push(hw_usart_rx_bufs[usart], byte);
        }
    }

    // TX interrupt
    if (usarts[usart]->SR & USART_FLAG_TC) {

        // clear interrupt bit
        usarts[usart]->SR &= !USART_FLAG_TC;

        // if there is more data to be sent, send the next byte
        // otherwise, disable the TC interrupt
        if (circularbuffer_u8_empty(hw_usart_tx_bufs[usart]) != 1) {
            usarts[usart]->DR = circularbuffer_u8_pop(hw_usart_tx_bufs[usart]);
        } else {
            usarts[usart]->CR1 &= ~(1 << 6);
        }
    }
}

#if HW_USART_COUNT >= 1

void USART1_IRQHandler() {
    hw_usart_irq(HW_USART1);
}
#endif

#if HW_USART_COUNT >= 2

void USART2_IRQHandler() {
    hw_usart_irq(HW_USART2);
}
#endif

#if HW_USART_COUNT >= 3

void USART3_IRQHandler() {
    hw_usart_irq(HW_USART3);
}
#endif

#if HW_USART_COUNT >= 4

void UART4_IRQHandler() {
    hw_usart_irq(HW_USART4);
}
#endif

#if HW_USART_COUNT >= 5

void UART5_IRQHandler() {
    hw_usart_irq(HW_USART5);
}
#endif

#if HW_USART_COUNT >= 6

void USART6_IRQHandler() {
    hw_usart_irq(HW_USART6);
}
#endif
