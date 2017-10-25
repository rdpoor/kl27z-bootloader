/*
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
 * All rights reserved.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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
#include "pin_mux.h"
#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "fsl_device_registers.h"
#include "fsl_uart.h"
#include "utilities/fsl_rtos_abstraction.h"


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define BOOT_PIN_NUMBER 3
#define BOOT_PIN_PORT PORTC
#define BOOT_PIN_GPIO GPIOC
#define BOOT_PIN_ALT_MODE 1
#define BOOT_PIN_DEBOUNCE_READ_COUNT 500

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void init_hardware(void)
{
	// already initialized in main.c
}

void deinit_hardware(void)
{
	// uart0_full_shutdown() has already released the HW resources.  In fact,
	// calling BOARD_DeinintPins() twice in a row generates a hard fault.
	// BOARD_DeinitPins();
}

uint32_t get_bus_clock_freq(void)
{
	return CLOCK_GetBusClkFreq();
}

bool usb_clock_init(void)
{
	// Not implemented.
	return false;
}

uint32_t get_uart_clock(uint32_t instance)
{
	// not called by anyone?
	return 0;
}

bool is_boot_pin_asserted(void)
{
	// Not implemented.
	return false;
}

void update_available_peripherals()
{
}

// @brief Initialize watchdog
void bootloader_watchdog_init(void)
{
	// Not currently used.  If it was, it would be something like this:
#if 0
	cop_config_t config;
	COP_GetDefaultConfig(&config);
	// adjust config...
	config.timeoutCycles = kCOP_2Power8CyclesOr2Power16Cycles;
	COP_Init(BOOTLOADER_SIM, &config)
#endif
}

void bootloader_watchdog_service(void)
{
	// Not currently used.  If it was, it would be something like this:
#if 0
	COP_Refresh(BOOTLOADER_SIM);
#endif
}

void bootloader_watchdog_deinit(void)
{
	// Not currently used.  If it was, it would be something like this:
#if 0
	COP_Disable(BOOTLOADER_SIM);
#endif
}

// Called from bl_shutdown_cleanup.  These two NVIC_xxx methods are not provided
// in core_cm0plus.h, so we're implementing them here.
void init_interrupts(void)
{
    // Clear any IRQs that may be enabled, we only want the IRQs we enable to be active
    // NVIC_ClearEnabledIRQs();
    NVIC->ICER[0] = 0xFFFFFFFF;

    // Clear any pending IRQs that may have been set
    // NVIC_ClearAllPendingIRQs();
    NVIC->ICPR[0] = 0xFFFFFFFF;

}

void uart_pinmux_config(uint32_t instance, pinmux_type_t pinmux) {
	// the LPUART is already initialized in BOARD_InitPins()
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
