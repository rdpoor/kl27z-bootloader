/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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
#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#include "autobaud/autobaud.h"
#include "packet/serial_packet.h"
#include "fsl_device_registers.h"
#include "fsl_lpuart.h"
#include "fsl_common.h"

#if BL_CONFIG_UART

//! @addtogroup uart0_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
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

typedef enum {
	kDisabled,
	kConfiguredForAutobaud,
	kConfiguredForUART
} serial_state_t;

static status_t uart0_full_init(const peripheral_descriptor_t *self,
		serial_byte_receive_func_t function);

static bool uart0_poll_for_activity(const peripheral_descriptor_t *self);

static void uart0_full_shutdown(const peripheral_descriptor_t *self);

static status_t uart0_write(const peripheral_descriptor_t *self,
		const uint8_t *buffer, uint32_t byteCount);

static void serial_set_state(const peripheral_descriptor_t *self,
		serial_state_t state,
		uint32_t baud);

static void serial_disable(const peripheral_descriptor_t *self);

static void serial_configure_for_autobaud(const peripheral_descriptor_t *self);

static void serial_configure_for_uart(const peripheral_descriptor_t *self,
		uint32_t baud);

#ifdef BL_FEATURE_BYPASS_AUTOBAUD
static void receive_ping_byte_callback(uint8_t byte);
static bool serial_ping_was_detected();
#endif

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

static const uint32_t g_lpsciBaseAddr[] = { BOOTLOADER_UART_BASEADDR };

const peripheral_control_interface_t g_uart0ControlInterface = {
		.init = uart0_full_init,
		.pollForActivity = uart0_poll_for_activity,
		.shutdown = uart0_full_shutdown,
		.pump = 0
};

const peripheral_byte_inteface_t g_uart0ByteInterface = {
		.init = NULL,
		.write = uart0_write
};

static serial_byte_receive_func_t s_byte_received_callback;

static serial_byte_receive_func_t s_uart0_byte_receive_callback;

static serial_state_t s_serial_state = kDisabled;

#ifdef BL_FEATURE_BYPASS_AUTOBAUD
static uint8_t s_ping_buffer[2] = {0, 0};
#endif

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*
 * In a system that supports autobaud, prepare the serial port to listen for
 * autobaud activity as a GPIO input.  Otherwise prepare the serial port with
 * the UART fully configured.
 */
status_t uart0_full_init(const peripheral_descriptor_t *self,
		serial_byte_receive_func_t function)
{
	s_byte_received_callback = function;
#ifndef BL_FEATURE_BYPASS_AUTOBAUD
	serial_set_state(self, kConfiguredForAutobaud, 0);
	autobaud_init();
#else
	serial_set_state(self, kConfiguredForUART, BL_FEATURE_BAUD_RATE);
	// we are searching for a ping byte.  install our custom handler...
	s_uart0_byte_receive_callback = receive_ping_byte_callback;
#endif
	return kStatus_Success;
}

// In a system that supports autobaud, return false unless autobaud detection
// has completed.  If autobaud detection has completed, prepare the serial
// port with the UART fully configured.
//
// In a system that does not support autobaud, return false until the system has
// received the sync sequence (0x5a 0xa6).
//
// Return true once the sync sequence is detected (whether via autobaud or not).
//
bool uart0_poll_for_activity(const peripheral_descriptor_t *self)
{
#ifndef BL_FEATURE_BYPASS_AUTOBAUD
    uint32_t baud;
    status_t autoBaudCompleted = autobaud_get_rate(self->instance, &baud);
    if (kStatusSuccess != autoBaudCompleted) {
    	return false;
    }
    serial_set_state(self, kConfiguredForUART, baud);
#else
    if (!serial_ping_was_detected()) {
    	return false;
    }
#endif

    // Arrive here when autobaud sequence was detected (if autobaud enabled)
    // or if the ping sequence was detected (non-autobaud).

    // Install the "standard" packet interface callback and
    // inform the command layer about the two sync bytes
	s_uart0_byte_receive_callback = s_byte_received_callback;
    s_uart0_byte_receive_callback(kFramingPacketStartByte);
    s_uart0_byte_receive_callback(kFramingPacketType_Ping);

    return true;
}

void uart0_full_shutdown(const peripheral_descriptor_t *self)
{
	serial_set_state(self, kDisabled, 0);
#ifndef BL_FEATURE_BYPASS_AUTOBAUD
    autobaud_deinit(self->instance);
#endif
}

status_t uart0_write(const peripheral_descriptor_t *self,
		const uint8_t *buffer,
		uint32_t byteCount)
{
    uint32_t baseAddr = g_lpsciBaseAddr[self->instance];
    BOARD_WriteUART(baseAddr, buffer, byteCount);
    return kStatus_Success;
}

/********************************************************************/
/*
 * UART0 IRQ Handler
 *
 * In case you have a hard time finding the value s_uart0_byte_receive_callback
 * was set to (I did), it is:
 * `serial_packet_queue_byte(uint8_t byte)` defined in `serial_packet.c`.
 */
void BOOTLOADER_UART_IRQ_HANDLER(void)
{
	uint8_t data = LPUART_ReadByte((LPUART_Type *)BOOTLOADER_UART_BASEADDR);
    s_uart0_byte_receive_callback(data);
}

//! @}

#endif // BL_CONFIG_UART

////////////////////////////////////////////////////////////////////////////////
// Static (local) methods
////////////////////////////////////////////////////////////////////////////////

static void serial_set_state(const peripheral_descriptor_t *self,
		serial_state_t state,
		uint32_t baud)
{
	if (state == s_serial_state) {
		return;
	}
    // Disable if previously active.
	if (s_serial_state != kDisabled) {
    	serial_disable(self);
	}

	if (state == kConfiguredForAutobaud) {
		serial_configure_for_autobaud(self);
	} else if (state == kConfiguredForUART) {
		serial_configure_for_uart(self, baud);
	}

	s_serial_state = state;
}

static void serial_disable(const peripheral_descriptor_t *self) {
    uint32_t baseAddr = g_lpsciBaseAddr[self->instance];
    // TODO: passing in baseAddr is deceiving since the board specific code
    // makes hard-wired assumptions about which uart module is being enabled.
    // Maybe restructure.
    BOARD_DeinitUART(baseAddr);
}

static void serial_configure_for_autobaud(const peripheral_descriptor_t *self) {
	// TODO: Check if this actually works...
    self->pinmuxConfig(self->instance, kPinmuxType_GPIO);
}

static void serial_configure_for_uart(const peripheral_descriptor_t *self,
		uint32_t baud) {
    uint32_t baseAddr = g_lpsciBaseAddr[self->instance];
    // TODO: passing in baseAddr is deceiving since the board specific code
    // makes hard-wired assumptions about which uart module is being enabled.
    // Maybe restructure.
	BOARD_InitUART(baseAddr, baud);
}

#ifdef BL_FEATURE_BYPASS_AUTOBAUD

// Receive a byte at interrupt level.  Save it in our two-byte fifo...
// TODO: confirm that two bytes is enough to receive the whole packet.
static void receive_ping_byte_callback(uint8_t byte) {
  	s_ping_buffer[0] = s_ping_buffer[1];
  	s_ping_buffer[1] = byte;
}

static bool serial_ping_was_detected() {
	if ((s_ping_buffer[0] == kFramingPacketStartByte) &&
		(s_ping_buffer[1] == kFramingPacketType_Ping)) {
		return true;
	}
	return false;
}

#endif

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
