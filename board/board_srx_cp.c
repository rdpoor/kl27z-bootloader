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

#include "board.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_port.h"

#include "fsl_common.h"

#ifdef BOARD_SRX_CP  // whole file

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////

// Constants for CLOCK_SetLpuartXClock().
//
// See KL27P64M48SF6RM.pdf section 12.3.3: System Options Register 2 (SIM_SOPT2)
// NOTE: Are these constants perhaps defined elsewhere?
typedef enum {
  kUartClockDisabled = 0,
  kUartClockIRC48M,
  kUartClockOSCERCLK,
  kUartClockMCGIRCLK
} lpuart_clock_source_t;

#define UART_RX_PIN 18
#define UART_TX_PIN 19

#define SOPT5_LPUART1RXSRC_LPUART_RX  0x00u // Select LPUART1_RX pin for UART RX
#define SOPT5_LPUART1TXSRC_LPUART_TX  0x00u // Select LPUART1_TX pin for UART TX

// bit mask for Transmit Data Register Empty and Transmit Complete
#define TDRE_TC_MASK (LPUART_STAT_TDRE_MASK | LPUART_STAT_TC_MASK)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

// Perform board-level initialization
void BOARD_InitBoard() {
  CLOCK_EnableClock(kCLOCK_PortA);
  CLOCK_EnableClock(kCLOCK_PortC);
  CLOCK_EnableClock(kCLOCK_PortD);
  BOARD_BootClockRUN();
  
  // The Sensorex board has a PWM output that controls 4-20mA current draw.
  // Initialize PWM output to zero in order to minimize current draw.
  PORT_SetPinMux(PWM_PORT, PWM_PIN, kPORT_MuxAsGpio);
  gpio_pin_config_t config = { kGPIO_DigitalOutput, 0 };
  GPIO_PinInit(PWM_GPIO, PWM_PIN, &config);
  
  // The board has a serial enable pin that must be activated for serial I/O.
  // (It gets disabled when the device is working in 4-20MA current loop mode
  // to save current.)
  PORT_SetPinMux(SERIAL_ENABLE_PORT, SERIAL_ENABLE_PIN, kPORT_MuxAsGpio);
  GPIO_PinInit(SERIAL_ENABLE_GPIO, SERIAL_ENABLE_PIN, &config);
  SERIAL_ENABLE();
  
  // The Sensorex board uses RS485 half-duplex serial communication.  Before
  // transmitting a character, the firmware must assert the drive enable gpio
  // pin.  Conversely, to receive a character, drive enable must be de-
  // asserted.
  PORT_SetPinMux(RS485_DRIVE_ENABLE_PORT,
                 RS485_DRIVE_ENABLE_PIN,
                 kPORT_MuxAsGpio);
  GPIO_PinInit(RS485_DRIVE_ENABLE_GPIO, RS485_DRIVE_ENABLE_PIN, &config);
  RS485_DRIVE_DISABLE();  // initially listening...
}

// Perform board-level cleanup
void BOARD_DeinitBoard() {
  // kPORT_PinDisabledOrAnalog
  PORT_SetPinMux(RS485_DRIVE_ENABLE_PORT,
                 RS485_DRIVE_ENABLE_PIN,
                 kPORT_PinDisabledOrAnalog);
  PORT_SetPinMux(SERIAL_ENABLE_PORT,
                 SERIAL_ENABLE_PIN,
                 kPORT_PinDisabledOrAnalog);
  PORT_SetPinMux(PWM_PORT, PWM_PIN, kPORT_PinDisabledOrAnalog);
  
  BOARD_BootClockRUN();              // TODO: should be managed by bootloader?
  CLOCK_DisableClock(kCLOCK_PortD);
  CLOCK_DisableClock(kCLOCK_PortC);
  CLOCK_DisableClock(kCLOCK_PortA);
}

// Initialize UART
// TODO: see board_frdm_kl27.c:BOARD_InitUART(), consider parameterizing
// TODO: passing in baseAddr is deceiving since the code makes hardwired
// assumptions about which uart module is being enabled.  Maybe restructure.
void BOARD_InitUART(uint32_t baseAddr, uint32_t baud) {
  LPUART_Type *base = (LPUART_Type *)baseAddr;
  lpuart_config_t config;

  // set up I/O ports and pins
  PORT_SetPinMux(PORTA, UART_RX_PIN, kPORT_MuxAlt3);
  PORT_SetPinMux(PORTA, UART_TX_PIN, kPORT_MuxAlt3);
  SIM->SOPT5 = ((SIM->SOPT5 &
                 (~(SIM_SOPT5_LPUART1TXSRC_MASK | SIM_SOPT5_LPUART1RXSRC_MASK))) /* Mask bits to zero which are setting */
                | SIM_SOPT5_LPUART1TXSRC(SOPT5_LPUART1TXSRC_LPUART_TX) /* LPUART0 Transmit Data Source Select: LPUART0_TX pin */
                | SIM_SOPT5_LPUART1RXSRC(SOPT5_LPUART1RXSRC_LPUART_RX) /* LPUART0 Receive Data Source Select: LPUART_RX pin */
                );
  
  CLOCK_SetLpuart1Clock(kUartClockIRC48M);
  
  LPUART_GetDefaultConfig(&config);
  config.baudRate_Bps = baud;
  config.enableRx = true;
  config.enableTx = true;
  if (LPUART_Init(base, &config, CLOCK_GetPeriphClkFreq()) == kStatus_Success)
    {
      LPUART_EnableInterrupts(base, kLPUART_RxDataRegFullInterruptEnable);
      EnableIRQ(LPUART1_IRQn);
    }
}

// Shut down UART.  In general, do things in reverse order from initialization.
void BOARD_DeinitUART(uint32_t baseAddr) {
  LPUART_Type *base = (LPUART_Type *)baseAddr;
  
  DisableIRQ(LPUART1_IRQn);
  LPUART_DisableInterrupts(base, kLPUART_RxDataRegFullInterruptEnable);
  LPUART_Deinit(base);
  SIM->SOPT5 = ((SIM->SOPT5 &
                 (~(SIM_SOPT5_LPUART1TXSRC_MASK | SIM_SOPT5_LPUART1RXSRC_MASK))));
  PORT_SetPinMux(PORTA, UART_TX_PIN, kPORT_PinDisabledOrAnalog);
  PORT_SetPinMux(PORTA, UART_RX_PIN, kPORT_PinDisabledOrAnalog);
}

// Sensorex UART requires a special write method for its RS485 interface.  It
// must disable reception of UART data (otherwise every transmitted character
// would be treated as a received char) and then enable the TX drive line.
// After sending the data, it reverses the process.
void BOARD_WriteUART(uint32_t baseAddr,
                     const uint8_t *buffer,
                     uint32_t byteCount) {
    LPUART_Type *base = (LPUART_Type *)baseAddr;

    // disable UART receive and set the drive enable line
    base->CTRL &= ~LPUART_CTRL_RE_MASK;
    RS485_DRIVE_ENABLE();

    // do the write.
    LPUART_WriteBlocking(base, buffer, byteCount);

    // wait until the buffer is empty and tx complete before dropping tx enable
    while((LPUART_GetStatusFlags(base) & TDRE_TC_MASK) != TDRE_TC_MASK) {
    }

    // clear the drive enable line and enable UART reception
    RS485_DRIVE_DISABLE();
    base->CTRL |= LPUART_CTRL_RE_MASK;
}

#endif // #ifdef BOARD_SRX_CP
