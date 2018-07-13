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

#ifndef _BOARD_H_
#define _BOARD_H_

////////////////////////////////////////////////////////////////////////////////
// Board selection
////////////////////////////////////////////////////////////////////////////////

// un-comment the desired board

// #define BOARD_FRDM_KL27Z
#define BOARD_SRX_CP

#if defined(BOARD_FRDM_KL27Z)
#include "board_frdm_kl27z.h"

#elif defined(BOARD_SRX_CP)
#include "board_srx_cp.h"

#else
#error Unknown board type

#endif

////////////////////////////////////////////////////////////////////////////////
// Common Prototypes
// These must be defined in each board_xxx.c file
////////////////////////////////////////////////////////////////////////////////

// Perform board-level initialization
void BOARD_InitBoard();

// Perform board-level cleanup
void BOARD_DeinitBoard();

// UARTs
// (note: we assume every board has one.  If not, conditionalize...)

// Initialize UART
void BOARD_InitUART(uint32_t baseAddr, uint32_t baud);

// Shut down UART
void BOARD_DeinitUART(uint32_t baseAddr);

// Set UART baud rate depending on current run mode (RUN or VPLR)
void BOARD_setBaudRate(uint32_t baud);

// Write a buffer of bytes to the UART
void BOARD_WriteUART(uint32_t baseAddr,
		const uint8_t *buffer,
		uint32_t byteCount);

#endif /* _BOARD_H_ */
