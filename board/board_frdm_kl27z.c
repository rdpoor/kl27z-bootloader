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
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"

#ifdef BOARD_FRDM_KL27Z // whole file

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////

#define SOPT5_LPUART0RXSRC_LPUART_RX  0x00u // Select LPUART0_RX pin for UART RX
#define SOPT5_LPUART0TXSRC_LPUART_TX  0x00u // Select LPUART0_TX pin for UART TX

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

// Perform board-level initialization
void BOARD_InitBoard() {
	CLOCK_EnableClock(kCLOCK_PortA);
	BOARD_BootClockRUN();
}

// Perform board-level cleanup
void BOARD_DeinitBoard() {
	BOARD_BootClockRUN();              // TODO: should be managed by bootloader?
	CLOCK_DisableClock(kCLOCK_PortA);
}

// Initialize UART
// TODO: see board_srx_cp.c:BOARD_InitUART(), consider parameterizing
void BOARD_InitUART(uint32_t baseAddr, uint32_t baud) {
    LPUART_Type *base = (LPUART_Type *)baseAddr;
	lpuart_config_t config;

    // set up I/O ports and pins
    PORT_SetPinMux(PORTA, PIN1_IDX, kPORT_MuxAlt2);            /* PORTA1 (pin 23) is configured as LPUART0_RX */
    PORT_SetPinMux(PORTA, PIN2_IDX, kPORT_MuxAlt2);            /* PORTA2 (pin 24) is configured as LPUART0_TX */
    SIM->SOPT5 = ((SIM->SOPT5 &
      (~(SIM_SOPT5_LPUART0TXSRC_MASK | SIM_SOPT5_LPUART0RXSRC_MASK))) /* Mask bits to zero which are setting */
        | SIM_SOPT5_LPUART0TXSRC(SOPT5_LPUART0TXSRC_LPUART_TX) /* LPUART0 Transmit Data Source Select: LPUART0_TX pin */
        | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_LPUART_RX) /* LPUART0 Receive Data Source Select: LPUART_RX pin */
      );

    CLOCK_SetLpuart0Clock(kUartClockIRC48M);

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = baud;
    config.enableRx = true;
    config.enableTx = true;
    if (LPUART_Init(base, &config, CLOCK_GetPeriphClkFreq()) == kStatus_Success)
    {
        LPUART_EnableInterrupts(base, kLPUART_RxDataRegFullInterruptEnable);
        EnableIRQ(LPUART0_IRQn);
    }
}

// Shut down UART.  In general, do things in reverse order from initialization.
void BOARD_DeinitUART(uint32_t baseAddr) {
	LPUART_Type *base = (LPUART_Type *)baseAddr;

	DisableIRQ(LPUART0_IRQn);
	LPUART_DisableInterrupts(base, kLPUART_RxDataRegFullInterruptEnable);
	LPUART_Deinit(base);
	SIM->SOPT5 = ((SIM->SOPT5 &
			(~(SIM_SOPT5_LPUART0TXSRC_MASK | SIM_SOPT5_LPUART0RXSRC_MASK))));
	PORT_SetPinMux(PORTA, PIN2_IDX, kPORT_PinDisabledOrAnalog);
	PORT_SetPinMux(PORTA, PIN1_IDX, kPORT_PinDisabledOrAnalog);
}

#ifdef DEBUG

/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    uint32_t uartClkSrcFreq;
    /* SIM_SOPT2[27:26]:
     *  00: Clock Disabled
     *  01: IRC48M
     *  10: OSCERCLK
     *  11: MCGIRCCLK
     */
    CLOCK_SetLpuart0Clock(1);

    uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
    DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}
#endif

#endif  // #ifdef BOARD_FRDM_KL27Z
