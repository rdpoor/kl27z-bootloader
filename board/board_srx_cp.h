/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _BOARD_SRX_CP_H_
#define _BOARD_SRX_CP_H_

#include "clock_config.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BOOTLOADER_UART_BASEADDR (uint32_t) LPUART1
#define BOOTLOADER_UART_CLKSRC kCLOCK_McgIrc48MClk
#define BOOTLOADER_UART_CLK_FREQ CLOCK_GetPeriphClkFreq()
#define BOOTLOADER_UART_IRQ LPUART1_IRQn
#define BOOTLOADER_UART_IRQ_HANDLER LPUART1_IRQHandler

/* The board name */
#define BOARD_NAME "Sensorex CP"

// The Sensorex board has a PWM output that controls current loop output.  In
// the bootloader initialization, we need to set it to zero (low level) in order
// to minimize current draw.
#define PWM_PORT PORTA
#define PWM_GPIO GPIOA
#define PWM_PIN 1U

// The board has a serial enable pin that must be activated for serial I/O.
// (It gets disabled when the device is working in 4-20MA current loop mode
// to save current.)
#define SERIAL_ENABLE_PORT PORTC
#define SERIAL_ENABLE_GPIO GPIOC
#define SERIAL_ENABLE_PIN 4U

#define SERIAL_SET_ENABLED(enabled)                                            \
   GPIO_WritePinOutput(SERIAL_ENABLE_GPIO, SERIAL_ENABLE_PIN, enabled)

#define SERIAL_ENABLE()                                                        \
    GPIO_SetPinsOutput(SERIAL_ENABLE_GPIO, 1U << SERIAL_ENABLE_PIN)

#define SERIAL_DISABLE()                                                       \
    GPIO_ClearPinsOutput(SERIAL_ENABLE_GPIO, 1U << SERIAL_ENABLE_PIN)

// The board uses RS485 serial communication.  In addition to being half duplex,
// the serial interface has a drive enable pin that must be set high prior to
// transmitting and set low thereafter.
#define RS485_DRIVE_ENABLE_PORT PORTD
#define RS485_DRIVE_ENABLE_GPIO GPIOD
#define RS485_DRIVE_ENABLE_PIN 4U

#define RS485_SET_DRIVE_ENABLED(enabled)                                       \
   GPIO_WritePinOutput(RS485_DRIVE_ENABLE_GPIO, RS485_DRIVE_ENABLE_PIN, enabled)

#define RS485_DRIVE_ENABLE()                                                   \
    GPIO_SetPinsOutput(RS485_DRIVE_ENABLE_GPIO, 1U << RS485_DRIVE_ENABLE_PIN)

#define RS485_DRIVE_DISABLE()                                                  \
    GPIO_ClearPinsOutput(RS485_DRIVE_ENABLE_GPIO, 1U << RS485_DRIVE_ENABLE_PIN)

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitRS485DriveEnable(bool initial);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_SRX_CP_H_ */
