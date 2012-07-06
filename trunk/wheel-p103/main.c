/*
 * main.c
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

/* This is the color wheel application entry point. */

/*
 * -------------------- System Includes ----------------------------------------
 */

#include <stdint.h>

/*
 * -------------------- Local Includes -----------------------------------------
 */

#include "circularbuffer_u8.h"
#include "protocol.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

/*
 * -------------------- Defines ------------------------------------------------
 */

/**
 * @brief Toggles the state of the run led.
 * 
 * The run led indicates that the main loop of the application is still running.
 * 
 * This method is implemented as a register call because it is highly 
 * performance critical.
 * 
 * This method is implemented as a define, to ensure that the code is inlined by
 * the compiler. This is done for performance reasons.
 * 
 * GCC treats the inline keyword as an optimization hint. The compiler may still
 * ignore the keyword and not inline the function. This is avoided by using a
 * define.
 * 
 * @param None.
 * @return None.
 */
#define toggle_runled() GPIOC->ODR ^= (1 << 12)

/**
 * @brief Buffer size for the USART2 tx and rx buffer.
 */
#define USART2_BUFFER_SIZE 256

/*
 * -------------------- Type definitions ---------------------------------------
 */

/*
 * -------------------- Global variables ---------------------------------------
 */

/**
 * @brief Backing array for the USART2 TX buffer.
 */

volatile uint8_t usart2_tx_arr[USART2_BUFFER_SIZE];

/**
 * @brief Backing array for the USART2 RX buffer.
 */
volatile uint8_t usart2_rx_arr[USART2_BUFFER_SIZE];

/**
 * @brief USART2 TX circular buffer.
 */
circularbuffer_u8_s usart2_tx_buffer;

/**
 * @brief USART2 RX circular buffer.
 */
circularbuffer_u8_s usart2_rx_buffer;

/**
 * @brief Flag indicating that an update should be sent
 */
uint8_t update;

/*
 * -------------------- Prototypes ---------------------------------------------
 */

/**
 * @brief Performs the initialization of the program.
 */
void init();

/**
 * @brief Sends out the given byte over USART2.
 * 
 * This send is non-blocking.
 * 
 * @param byte The byte to send.
 */
void usart2_send(uint8_t byte);

/*
 * -------------------- Method implementations ---------------------------------
 */

void init() {

    /*
     * Step 1: Enable the various hardware devices needed for the application.
     */

    // enable GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // enable GPIOC
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // enable USART2
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);

    // enable TIM3
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);

    /*
     * Step 2: Set the pin configurations for all required pins.
     */

    // configure LED pin
    GPIO_InitTypeDef pinCfg;
    pinCfg.GPIO_Speed = GPIO_Speed_50MHz;
    pinCfg.GPIO_Mode = GPIO_Mode_Out_PP;
    pinCfg.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOC, &pinCfg);

    // configure USART2 pins: PA0-WKUP/USART2_CTS
    pinCfg.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    pinCfg.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &pinCfg);

    // configure USART2 pins: PA1/USART2_RTS
    pinCfg.GPIO_Mode = GPIO_Mode_AF_PP;
    pinCfg.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOA, &pinCfg);

    // configure USART2 pins: PA2/USART2_TX
    pinCfg.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &pinCfg);

    // configure USART2 pins: PA3/USART2_RX
    pinCfg.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    pinCfg.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &pinCfg);

    /*
     * Step 3: Configure USART2 for communication.
     */

    // configure USART2
    // Please note that this USART is configured to use RTS/CTS hardware
    // flow control.
    // The use of flow control implies that you must close the solder
    // bridges on the PCB of the STM32-P103 development board as seen in the
    // schematic.
    USART_InitTypeDef usartCfg;
    usartCfg.USART_BaudRate = 38400;
    usartCfg.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
    usartCfg.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usartCfg.USART_Parity = USART_Parity_No;
    usartCfg.USART_StopBits = USART_StopBits_1;
    usartCfg.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &usartCfg);

    // enable USART2
    USART_Cmd(USART2, ENABLE);

    // initialize the RX and TX buffer for USART2
    circularbuffer_u8_init(&usart2_rx_buffer, USART2_BUFFER_SIZE, usart2_rx_arr);
    circularbuffer_u8_init(&usart2_tx_buffer, USART2_BUFFER_SIZE, usart2_tx_arr);

    /*
     * Step 4: Configure TIM3 for interrupt at 25 Hz.
     */

    // configure TIM3
    TIM3->PSC = 71; // prescaler of 72
    TIM3->ARR = 38000; // period of 40000 for 25Hz
    TIM3->EGR = 1; // only set update generation
    TIM3->DIER = 1; // only enable update interrupt
    TIM3->CR1 = 1; // only set counter enable

    /*
     * Step 5: Interrupt configuration. 
     */

    // configure the USART2 receive interrupt.
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    // enable USART2 interrupt and set priority
    NVIC_SetPriority(USART2_IRQn, 4);
    NVIC_EnableIRQ(USART2_IRQn);

    // enable TIM3 interrupt and set priority
    NVIC_SetPriority(TIM3_IRQn, 2);
    NVIC_EnableIRQ(TIM3_IRQn);
}

/**
 * @brief Application entry point.
 * 
 * Called by the default reset handler. The main is built as a perpetual while
 * loop. This method should never return.
 * 
 * @return Should never return.
 */
int main() {

    // vars
    uint8_t divider = 0;
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 255;
    uint16_t wheel = 0;

    // initialize the hardware
    init();

    // application main loop
    while (1) {

        // deal with the color wheel updates
        if (update) {
            update = 0;

            // send tick event to protocol stack
            protocol_tick();

            // update the color wheel at 12.5 Hz
            if (divider >= 2) {
                divider = 0;

                // calculate color wheel colors
                if (wheel <= 255) {
                    red = wheel;
                    green = 0;
                    blue = 255 - wheel;
                } else if (255 < wheel && wheel <= 511) {
                    red = 255 - (wheel - 256);
                    green = wheel - 256;
                    blue = 0;
                } else if (511 < wheel && wheel <= 767) {
                    red = 0;
                    green = 255 - (wheel - 512);
                    blue = wheel - 512;
                }
                wheel++;
                wheel %= 768;
                
                // call on protocol to send the setRGB message
                protocol_remote_set_rgb(1, red, green, blue);
                
                // toggle run led to indicate operation
                toggle_runled();
            }
            divider++;
        }

    }
    return 0;
}

void app_send_byte(uint8_t byte) {

    // send out the byte over USART2
    usart2_send(byte);
}

void USART2_IRQHandler() {

    // RX interrupt
    if (USART2->SR & USART_FLAG_RXNE) {

        // clear interrupt bit
        USART2->SR &= !USART_FLAG_RXNE;

        // retrieve the byte
        uint8_t byte = USART2->DR;

        // store the byte in the RX buffer if there is space, otherwise drop it
        if (circularbuffer_u8_full(&usart2_rx_buffer) == 0) {
            circularbuffer_u8_push(&usart2_rx_buffer, byte);
        }
    }

    // TX interrupt
    if (USART2->SR & USART_FLAG_TC) {

        // clear interrupt bit
        USART2->SR &= !USART_FLAG_TC;

        // if there is more data to be sent, send the next byte
        // otherwise, disable the TC interrupt
        if (circularbuffer_u8_empty(&usart2_tx_buffer) != 1) {
            USART2->DR = circularbuffer_u8_pop(&usart2_tx_buffer);
        } else {
            USART2->CR1 &= ~(1 << 6);
        }
    }
}

void usart2_send(uint8_t byte) {

    // if the TX buffer is full, drop bytes
    if (circularbuffer_u8_full(&usart2_tx_buffer)) {
        return;
    }

    // add the byte to the buffer
    circularbuffer_u8_push(&usart2_tx_buffer, byte);

    // if we must, start the send
    if ((USART2->CR1 & (1 << 6)) == 0) {

        // enable the TC interrupt and send the byte
        USART2->CR1 |= (1 << 6);
        USART2->DR = circularbuffer_u8_pop(&usart2_tx_buffer);
    }
}

void TIM3_IRQHandler() {

    // unset the interrupt flag
    TIM3->SR &= ~0x0001;

    // flag update
    update = 1;
}
