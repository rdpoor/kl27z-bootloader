# Flash-Resident Bootloader for KL27Z
## Design And Implementation Notes
> Robert Poor <rdpoor@gmail.com> October 2017

## Overview
Our goal is to create a flash-resident bootloader for the KL27Z family of 
processors in order to load programs into flash memory.  It is designed to be
compatible with the Kinetis KBOOT 2.0 protocol. 

As a code base, we started with the bootloader for the KL25Z processor and used 
the following guidelines while porting the code:

1. Use the Project Wizard to create a blank Kinetis 2.2 project, specifically 
for the FRDM-KL27Z board using "all drivers".
2. Use library routines (i.e. `drivers/fsl_xxx`) from the Kinetis SDK 2.x 
whenever practical. 
3. After giving priority to 1. and 2. above, maintain file structure of the 
KL25Z bootloader sources as much as possible.

We intentionally departed from the KL25Z bootloader on a few points:

* LPUART (0 or 1) is the only supported peripheral
* Autobaud detection is disabled.  The baud rate is fixed at 19200.
* Most conditional code blocks, e.g. `#ifdef BL_FEATURE_QSPI_MODULE` have been
left in place, but are untested.
* One exception: we frequently deleted code blocks under the `BOOTLOADER_HOST`
conditional as that appears to be deprecated.

## Relevant Documents
The following documents were useful during the porting process:

* [KL27 Sub-Family Reference Manual](https://www.nxp.com/docs/en/reference-manual/KL27P64M48SF2RM.pdf)
* [KL25 Sub-Family Reference Manual](https://www.nxp.com/docs/en/reference-manual/KL25P80M48SF0RM.pdf)
* [Kinetis Bootloader v2.0.0 Reference Manual](https://www.nxp.com/docs/en/reference-manual/KBTLDR200RM.pdf)
* [Kinetis Bootloader v2.0.0 Release Notes](https://www.nxp.com/docs/en/release-note/KBTLDR200RN.pdf)
* [Getting Started with Kinetis SDK (KSDK) v.2.0](https://www.nxp.com/docs/en/user-guide/KSDK20GSUG.pdf)
* [Kinetis SDK v.2.0 API Reference Manual](https://www.nxp.com/docs/en/reference-manual/KSDK20APIRM.pdf)
* [Kinetis SDK 2.0 Transition Guide](https://www.nxp.com/docs/en/user-guide/KSDK20TUG.pdf)
* [Kinetis blhost User's Guide](https://www.nxp.com/docs/en/user-guide/KBLHOSTUG.pdf)

An executable version of `blhost` is distributed in:

    NXP_Kinetis_Bootloader_2_0_0/bin/Tools/blhost/mac/blhost
    
Useful:

    alias blhost=<root>/NXP_Kinetis_Bootloader_2_0_0/bin/Tools/blhost/mac/blhost
    
## Changes to the file structure

In the list below, (KL25Z) means the kl25z-bootloader project, (KL27Z) means
the kl27z-bootloader project.

* (KL25Z) `src/*` => (KL27Z) `source/*`
* (KL27Z) `source/main.c:main()` is the primary entry point.  After performing
standard SDK 2.2 initialization, it calls `source/bootloader/src/bl_main.c:bl_main()`
* (KL25Z) `kl24z4/*` files are now under (KL27Z) `board/*` but are heavily 
modified and in some cases replaced by other functionality.
* (KL25Z) `link/MKL25Z128xxx4_application_0x0000.ld` =>
(KL27Z) `MKL27Z64xxx4_flash.ld`
* Each file in `source/drivers/fsl_<xyzzy>.[ch]` is carefully scrutinized and 
deleted if there is a corresponding `drivers/fsl_<xyzzy>.[ch]` file.

## Little notes

* extended  `MKL27Z64xxx4_flash.ld` to include a Booloader Configuration Area
* added ../source, ../source/include to the Cross 
Arm Gnu C Compiler Includes
* changed instance of `#include "<xyzzy>/fsl_<xyzzy>.h"` to 
`#include "fsl_<xyzzy>.h"`, since the SDK 2.0 structure has those files in
`drivers/fsl_<xyzzy>.h`
* changed instances of `#include "utilities/fsl_assert.h"` to 
`#include "fsl_common.h"` 
* tweaked all of the `#include "usb_xxx"` references to qualify include path.
* Deleted the `source/startup` directory since that's all KL25Z specific.
* Deleted `source/platform` directory since that's all KL25Z specific.

## SysTick

It appears that `source/drivers/systick/*` is used to drive a watchdog timer in
`board/hardware_init_MKL25Z4.c` and `drivers/watchdoc/fsl_watchdog_cop.h`.
fsl_cop supports watchdog.  So systick can go. As can watchdog.

## LPSCI => fsl\_lpuart

The `source/drivers/lpsci` directory defines an interface to a UART.  It is 
called (exclusively) from `source/bootloader/src/uart0_peripheral_interface`,
and can be replaced by fsl_lpuart calls.

Here are the methods defined in lpsci.h and where they are referenced.

    find . -name "*.[chsS]" -exec grep -n LPSCI_TransferCreateHandle {} /dev/null \;

    status_t LPSCI_Init(UART0_Type *base, const lpsci_config_t *config, uint32_t srcClock_Hz);
    ./source/bootloader/src/uart0_peripheral_interface.c:126:        if (LPSCI_Init((UART0_Type *)baseAddr, &config, uart0_get_clock(self->instance)) == kStatus_Success)
    ./utilities/fsl_debug_console.c:272:            LPSCI_Init(s_debugConsole.base, &lpsci_config, clkSrcFreq);

    void LPSCI_Deinit(UART0_Type *base);
    ./source/bootloader/src/uart0_peripheral_interface.c:172:        LPSCI_Deinit((UART0_Type *)baseAddr);
    ./utilities/fsl_debug_console.c:354:            LPSCI_Deinit(s_debugConsole.base);

    void LPSCI_GetDefaultConfig(lpsci_config_t *config);
    ./source/bootloader/src/uart0_peripheral_interface.c:120:        LPSCI_GetDefaultConfig(&config);
    ./utilities/fsl_debug_console.c:269:            LPSCI_GetDefaultConfig(&lpsci_config);

    status_t LPSCI_SetBaudRate(UART0_Type *base, uint32_t baudRate_Bps, uint32_t srcClock_Hz);
    not referenced

    uint32_t LPSCI_GetStatusFlags(UART0_Type *base);
    not referenced

    status_t LPSCI_ClearStatusFlags(UART0_Type *base, uint32_t mask);
    not referenced

    void LPSCI_EnableInterrupts(UART0_Type *base, uint32_t mask);
    ./source/bootloader/src/uart0_peripheral_interface.c:128:            LPSCI_EnableInterrupts((UART0_Type *)baseAddr, kLPSCI_RxDataRegFullInterruptEnable);

    void LPSCI_DisableInterrupts(UART0_Type *base, uint32_t mask);
    ./source/bootloader/src/uart0_peripheral_interface.c:171:        LPSCI_DisableInterrupts((UART0_Type *)baseAddr, UART0_C2_RIE_MASK);

    uint32_t LPSCI_GetEnabledInterrupts(UART0_Type *base);
    not referenced

    static inline uint32_t LPSCI_GetDataRegisterAddress(UART0_Type *base)
    not referenced

    static inline void LPSCI_EnableTxDMA(UART0_Type *base, bool enable)
    not referenced

    static inline void LPSCI_EnableRxDMA(UART0_Type *base, bool enable)
    not referenced

    static inline void LPSCI_EnableTx(UART0_Type *base, bool enable)
    ./utilities/fsl_debug_console.c:273:            LPSCI_EnableTx(s_debugConsole.base, true);

    static inline void LPSCI_EnableRx(UART0_Type *base, bool enable)
    ./utilities/fsl_debug_console.c:274:            LPSCI_EnableRx(s_debugConsole.base, true);

    static inline void LPSCI_WriteByte(UART0_Type *base, uint8_t data)
    not referenced

    static inline uint8_t LPSCI_ReadByte(UART0_Type *base)
    not referenced

    void LPSCI_TransferCreateHandle(UART0_Type *base,
                                    lpsci_handle_t *handle,
                                    lpsci_transfer_callback_t callback,
                                    void *userData);
    not referenced

    void LPSCI_TransferStartRingBuffer(UART0_Type *base,
                                       lpsci_handle_t *handle,
                                       uint8_t *ringBuffer,
                                       size_t ringBufferSize);
    not referenced

    void LPSCI_TransferStopRingBuffer(UART0_Type *base, lpsci_handle_t *handle);
    not referenced

    void LPSCI_TransferSendBlocking(UART0_Type *base, const uint8_t *data, size_t length);
    ./source/bootloader/src/uart0_peripheral_interface.c:190:    LPSCI_TransferSendBlocking((UART0_Type *)baseAddr, buffer, byteCount);

    status_t LPSCI_TransferSendNonBlocking(UART0_Type *base, lpsci_handle_t *handle, lpsci_transfer_t *xfer);
    not referenced

    void LPSCI_TransferAbortSend(UART0_Type *base, lpsci_handle_t *handle);
    not referenced

    status_t LPSCI_TransferGetSendCount(UART0_Type *base, lpsci_handle_t *handle, uint32_t *count);
    not referenced

    status_t LPSCI_TransferReceiveBlocking(UART0_Type *base, uint8_t *data, size_t length);
    not referenced

    status_t LPSCI_TransferReceiveNonBlocking(UART0_Type *base,
                                              lpsci_handle_t *handle,
                                              lpsci_transfer_t *xfer,
                                              size_t *receivedBytes);
    not referenced

    void LPSCI_TransferAbortReceive(UART0_Type *base, lpsci_handle_t *handle);
    not referenced

    status_t LPSCI_TransferGetReceiveCount(UART0_Type *base, lpsci_handle_t *handle, uint32_t *count);
    not referenced

    void LPSCI_TransferHandleIRQ(UART0_Type *base, lpsci_handle_t *handle);
    not referenced

    void LPSCI_TransferHandleErrorIRQ(UART0_Type *base, lpsci_handle_t *handle);
    not referenced

Distilling this down, here are the methods we need to provide (or make sure
that fsl_lpuart provides...), followed by the correspoinding LPUART function.

    status_t LPSCI_Init(UART0_Type *base, const lpsci_config_t *config, uint32_t srcClock_Hz);
    status_t LPUART_Init(LPUART_Type *base, const lpuart_config_t *config, uint32_t srcClock_Hz);
    
    void LPSCI_Deinit(UART0_Type *base);
    void LPUART_Deinit(LPUART_Type *base);
    
    void LPSCI_GetDefaultConfig(lpsci_config_t *config);
    void LPUART_GetDefaultConfig(lpuart_config_t *config);
    
    void LPSCI_EnableInterrupts(UART0_Type *base, uint32_t mask);
    void LPUART_EnableInterrupts(LPUART_Type *base, uint32_t mask);
    
    void LPSCI_DisableInterrupts(UART0_Type *base, uint32_t mask);
    void LPUART_DisableInterrupts(LPUART_Type *base, uint32_t mask);
    
    static inline void LPSCI_EnableTx(UART0_Type *base, bool enable);
    static inline void LPUART_EnableTx(LPUART_Type *base, bool enable);
    
    static inline void LPSCI_EnableRx(UART0_Type *base, bool enable);
    static inline void LPUART_EnableRx(LPUART_Type *base, bool enable);
    
    void LPSCI_TransferSendBlocking(UART0_Type *base, const uint8_t *data, size_t length);
    void LPUART_WriteBlocking(LPUART_Type *base, const uint8_t *data, size_t length);  

Note that reading serial data is done though the serial interrupt routine in
`uart0_peripheral_interface.c`.

Armed with this knowledge, we have deleted source/drivers/lpsci/ with impunity.
Now all we need to do is modify `uart0_peripheral_interface.c`

## uart0\_peripheral\_interface.c

Fairly straightforward.  Created LPUART definitions in board.h of the form
`BOOTLOADER_UART_*` and changed all LPSCI references to LPUART references.
Clock source is now determined by `BOOTLOADER_UART_CLK_FREQ`, so the chip
specific clock code gets deleted.

## fsl\_smc

`bl_main.c` is referencing `smc/smc.h`, which probably can be replaced with
`fsl_smc.h` references.  It's worth mentioning the existing bl_main only calls
`enter_vlls1()` on exiting the bootloader if `BL_FEATURE_POWERDOWN` is enabled.
Our approach is to simply forbid `BL_FEATURE_POWERDOWN` (and thus eliminate
smc.h).  Before we do, let's see who else references smc.h.  

Hum!  Nobody else does.  So we deleted the smc/ directory.

Note that if you *did* want to support `BL_FEATURE_POWERDOWN` you could use the
functions defined in `drivers/fsl_smc.h`.

## fsl\_flash.h

It's worth comparing the old `source/drivers/flash/fsh_flash.h` against
the new `drivers/fsl_flash.h` -- as well as seeing which method are actually
called -- in order to understand how the SDK / API has changed.

The old and new are surprisingly similar.  Differences:

New defines `FLASH_SetProperty`, which the old lacks:

    status_t FLASH_SetProperty(flash_config_t *config, flash_property_tag_t whichProperty, uint32_t value);

New has `pflash_protection_status_t` argument types for these two functions:

    status_t FLASH_PflashSetProtection(flash_config_t *config, pflash_protection_status_t *protectStatus);

    status_t FLASH_PflashGetProtection(flash_config_t *config, pflash_protection_status_t *protectStatus);

... while old has `uint32_t *`:

    status_t FLASH_PflashSetProtection(flash_config_t *config, uint32_t protectStatus);

    status_t FLASH_PflashGetProtection(flash_config_t *config, uint32_t *protectStatus);

New defines some "speculation" methods which old lacks:

    status_t FLASH_PflashSetPrefetchSpeculation(flash_prefetch_speculation_status_t *speculationStatus);

    status_t FLASH_PflashGetPrefetchSpeculation(flash_prefetch_speculation_status_t *speculationStatus);

From all this -- unless there are some data structures that affect the operation
of the flash memory -- we can simply swap in the new methods.  

New version is 2.3.1, old is 2.1.0.

New defines:

    #if !defined(FLASH_SSD_CONFIG_ENABLE_SECONDARY_FLASH_SUPPORT)
    #define FLASH_SSD_CONFIG_ENABLE_SECONDARY_FLASH_SUPPORT 1 /*!< Enables the secondary flash support by default. */
    #endif

    #if defined(FSL_FEATURE_FLASH_HAS_MULTIPLE_FLASH) || defined(FSL_FEATURE_FLASH_PFLASH_1_START_ADDRESS)
    #define FLASH_SSD_IS_SECONDARY_FLASH_ENABLED (FLASH_SSD_CONFIG_ENABLE_SECONDARY_FLASH_SUPPORT)
    #else
    #define FLASH_SSD_IS_SECONDARY_FLASH_ENABLED (0)
    #endif

New defines two extra status constants that Old lacks:

    kStatus_FLASH_InvalidPropertyValue =
        MAKE_STATUS(kStatusGroupFlashDriver, 19), /*!< The flash property value is out of range.*/
    kStatus_FLASH_InvalidSpeculationOption =
        MAKE_STATUS(kStatusGroupFlashDriver, 20), /*!< The option of flash prefetch speculation is invalid.*/

New extends the `flash_property_tag_t` enum with `kFLASH_PropertyFlashMemoryIndex`
and `kFLASH_PropertyFlashCacheControllerIndex`

New has renamed a field in `flash_execute_in_ram_function_config_t` from
`flashCacheClearCommand` to `flashCommonBitOperation`.  **WARNING**: This is 
referenced in bl_main.c -- make sure the semantics are the same.

New defines 

* `_k3_flash_read_once_index`
* `flash_memory_index_t` 
* `flash_cache_controller_index_t`
* `flash_prefetch_speculation_option_t`
* `flash_prefetch_speculation_status_t`
* `flash_cache_clear_process_t`
* `flash_protection_config_t`
* `flash_access_config_t`
* `flash_config_t` has changed structure somewhat.  **CHECK THIS**

Meh.  I'm tempted just to swap in the new fsl_flash API and make that one change
from `flashCacheClearCommand` to `flashCommonBitOperation` and hope it works.

## fsl\_common

The "standard" `drivers/fsl_common` SDK files seem to be a superset of the old,
with the following exception:

    uint32_t InstallIRQHandler(IRQn_Type irq, uint32_t irqHandler);
    
returns the old IRQ handler address.  The previous version did not.  However,
this method is not referred to in the bootloader code.  Out with the old...

## fsl\_gpio

Old is version 2.0.0, new is version 2.1.1

New one is a superset.  Out with the old.

## fsl\_i2c

Old is version 2.0.0, new is version 2.0.3

In new, a field in `i2c_slave_transfer_event_t`:

    kI2C_SlaveStartEvent = 0x10U, /*!< A start/repeated start was detected. */

in old:

    kI2C_SlaveRepeatedStartEvent = 0x10U, /*!< A repeated start was detected. */    

In new, the `i2c_master_config_t` lacks the

        bool enableHighDrive; /*!< Controls the drive capability of the I2C pads. */ 

field.  However, it is referenced only from the fsl_i2c.c file, so it's
**probably** safe to delete it.

New defines:

    bool enableDoubleBuffering; /*!< Controls double buffer enable; notice ... */

which old lacks.

In New, `I2C_SlaveInit` has an extra argument: `srcClock_Hz`:

    void I2C_SlaveInit(I2C_Type *base, const i2c_slave_config_t *slaveConfig, uint32_t srcClock_Hz);

This **will** cause an incompatible change in:

    ./source/bootloader/src/i2c_peripheral_interface.c:243:    I2C_SlaveInit((I2C_Type *)baseAddr, &i2cSlaveConfig);

Oddly, the only difference between `I2C_SlaveInit` in the old and the new is 
that new resets the module, optionally enables double buffering and ends in a 
call to:

    I2C_SetHoldTime(base, slaveConfig->sclStopHoldTime_ns, srcClock_Hz);
    
In New, `I2C_MasterWriteBlocking` and `I2C_MasterReadBlocking` have an extra argument, `flags`:

    status_t I2C_MasterWriteBlocking(I2C_Type *base, const uint8_t *txBuff, size_t txSize, uint32_t flags);
    status_t I2C_MasterReadBlocking(I2C_Type *base, uint8_t *rxBuff, size_t rxSize, uint32_t flags);
    
However, neither of these methods are referenced in the source code.

Aside from the incompatible call to `I2C_SlaveInit`, the APIs look identical.
I'll delete the old and let the compiler warn me about fixing it.

Suggestion: Look at the examples directory i2c_interrupt.c

### Resolved

In i2c\_peripheral\_interface.c, added the following:

    // I2C_SlaveInit() in fsl_i2c driver V2.0.3 requires srcClock_Hz argument.
    uint32_t srcClock_Hz = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    I2C_SlaveInit((I2C_Type *)baseAddr, &i2cSlaveConfig, srcClock_Hz);

## source/drivers/microseconds/

This defines a microsecond clock that is used in several places in the system,
notably:

    ./source/autobaud/src/autobaud_irq.c:126
    ./source/bootloader/src/bl_main.c:343
    ./source/bootloader/src/bl_shutdown_cleanup.c:83
    ./source/packet/src/serial_packet.c:392

The microsecond module uses the PIT timer module which is available on the KL27,
so the code just might work unmodified.

Replaced `source/drivers/microseconds` with `source/systick` which is
functionally equivalent to the microseconds code, but it uses the fsl_pit API
and cleans up some of the function names.

## fsl\_port

The old and new are identical (except the new is v2.0.0).  deleting old.

## fsl\_spi

New is v 2.0.3.  Old is v 2.0.0.  

New has prototype:

    static inline void SPI_Enable(SPI_Type *base, bool enable)
    
Old has prototype:

    static inline void SPI_Enable(I2C_Type *base, bool enable)

Otherwise identical.  Deleting old.

## fsl\_uart

In old several methods of the form:

    void UART_CreateHandle(UART_Type *base, 
                       uart_handle_t *handle, 
                       uart_transfer_callback_t callback, 
                       void *userData);
            
have been renamed in new:

    void UART_TransferCreateHandle(UART_Type *base,
                               uart_handle_t *handle,
                               uart_transfer_callback_t callback,
                               void *userData);

Deleting old.

## fsl\_watchdog

The directory source/drivers/watchdog contains:

    watchdog.h
    src/watchdog.c
    
It appears that these files can (and should) be replaced by 

    drivers/fsl_cop.h

The calls are different, but the functionality is similar.  See
`driver_examples/cop.c` for usage.

## PinMux

The KL25Z bootloader seems to have a hand-rolled set of methods to set the 
GPIO pins and mux.  In general, I think most of the functionality in:

    board/pinmux_utility_common.c
    source/drivers/port/*

Call heirarchy:

    void uart_pinmux_config(uint32_t instance, pinmux_type_t pinmux)
        peripherals_MKL25Z4.c (in peripheral_descriptor pinmuxConfig)
            bootloader/src/uart0_peripheral_interface.c:98
            bootloader/src/uart0_peripheral_interface.c:123
            bootloader/src/uart0_peripheral_interface.c:150

The pinmuxConfig slot method is set to `uart_pinmux_config()` in
`peripherals_MKL25Z4.c` and called three times from 
`bootloader/src/uart0_peripheral_interface.c`: 

* in `uart0_poll_for_activity` with pinmux = kPinmuxType_Peripheral
* in `uart0_full_init` with pinmux = kPinmuxType_GPIO
* in `uart0_full_shutdown` witn pinmux = kPinmuxType_Default

`uart_pinmux_config()` is defined in `board/pinmux_utility_common.c`, and calls
these methods (also defined in `board/pinmux_utility_common.c`) using the 
`pinmux_t` argument to control what it does:

* PORT_RestoreDefault
* PORT_SetUartAutoBaudPinMode
* PORT_SetUartPinMode

## hardware\_init\_MKL25Z4.c

The original file, `MKL25Z4.c/hardware_init_MKL25Z4.c` contained a collection of
methods used by the bootloader, specifically for initializing the hardware.

We've created a `board/hardware_init_MKL27Z4.c` file and are implementing 
methods there as needed.

## autobaud

We've added a new feature to bootloader_config.h:

    // Defining BL_FEATURE_BAUD_RATE to anything other than 0 will disable autobaud
    #define BL_FEATURE_BAUD_RATE (19200)
    
    #if BL_FEATURE_BAUD_RATE != 0
    #define BL_FEATURE_BYPASS_AUTOBAUD (1)
    #else
    #undef BL_FEATURE_BYPASS_AUTOBAUD
    #endif
 
 ... and sprinkling `#ifdef BL_FEATURE_AUDOBAUD` appropriately.
 
## microseconds
 
Needs work.  Still calls timing registers directly.  Should be using fsl_pit.

Promoted it up one directory (so there's no longer a source/drivers directory).

 
# To check
 
 Stepping through the bootloader with a debugger came across the following:
 
## normal\_memory.c
 
 Calls on sram\_init\_cm0plus.c which seems too low level.  See if 
 there is an fsl_* driver file for it.
 
## bootloader\_property\_init()
 
flashEnd = 0x7688 (i.e. \_\_ROM\_END) before rounding up, 0x77ff after round
up.  Make sure this is sensible.
 
ramStart = 0x0 (i.e. \_\_RAM_START), which seems suspicious.  ramEnd = 
0x20002fff which seems okay (but check that anyway).
 
Although, a few lines later, it calls this (with sensible values):
 
    propertyStore->ramStartAddress[kIndexSRAM - 1] = map->startAddress;
    propertyStore->ramSizeInBytes[kIndexSRAM - 1] = map->endAddress - map->startAddress + 1;
 
## Unique ID
 
 Replaced this:
 
    propertyStore->UniqueDeviceId.uidmh = SIM->UIDMH;
    propertyStore->UniqueDeviceId.uidml = SIM->UIDML;
    propertyStore->UniqueDeviceId.uidl = SIM->UIDL;
 
 with a call to
 
     SIM_GetUniqueId(&propertyStore->UniqueDeviceId)
 
## bl\_main: get\_active\_peripheral()
 
Not clear why it should be calling `microseconds_convert_to_ticks()`
 
## autobaud
 
When using autobaud, the process goes like this:
 
* `uart_full_init` configures UART pin as GPIO and calls `autobaud_init()`
* `autobaud_init()` installs an IRQ callback to look for transitions on the GPIO
* `uart_poll_for_activity` repeatedly calls `autobaud_get_rate()` to see if the
autobaud process completes.  If the autobaud process has not yet completed,
`uart_poll_for_activity` returns false.
* When the autobaud process completes, `uart_poll_for_activity` fully sets up
the uart (and GPIO, and MUX) and fakes ping squence for higher level fns and
returns true.

When not using autobaud, the process is as follows:

* `uart_full_init` fully configures the UART (and GPIO and MUX) and installs the
a specialize ping detection interrupt handler.
* `uart_poll_for_activity` calls the ping detection code, which returns false 
until it has detected the ping sequence, at which point it returns true and the 
(fixed) baud rate.  It leaves the two chars in the receive buffer for the higher
level fns.

# It's alive

    $ blhost -p /dev/cu.usbmodem1421,19200 -- get-property 12
    Ping responded in 1 attempt(s)
    Inject command 'get-property'
    Response status = 0 (0x0) Success.
    Response word 1 = 0 (0x0)
    Response word 2 = 30719 (0x77ff)
    Response word 3 = 0 (0x0)
    Response word 4 = 536883199 (0x20002fff)
    Reserved Regions = Flash: 0x0-0x77FF (30 KB), RAM: 0x0-0x20002FFF (512.012 MB)

The RAM boundaries look wrong.  The corresponding response from the KL25 was:

    $ blhost --port /dev/cu.usbmodem246,19200 -- get-property 12
    Ping responded in 1 attempt(s)
    Inject command 'get-property'
    Response status = 0 (0x0) Success.
    Response word 1 = 0 (0x0)
    Response word 2 = 31743 (0x7bff)
    Response word 3 = 536866816 (0x1ffff000)
    Response word 4 = 536873671 (0x20000ac7)
    Reserved Regions = Flash: 0x0-0x7BFF (31 KB), RAM: 0x1FFFF000-0x20000AC7 (6.695 KB)

The difference MAY be that the KL25 has in its compile command:

    -Xlinker --defsym=__ram_vector_table__=1 

... yet the KL25 .map file shows `__VECTOR_RAM` being defined as zero.

# Debugging

We have some work to do:

    [~/Projects/Sensorex]$ cd workspace.kds/kl27z-blinky-8000/
    [~/Projects/Sensorex/workspace.kds/kl27z-blinky-8000]$ find . -name "*.srec" -print
    ./debug/kl27z-blinky-8000.srec
    [~/Projects/Sensorex/workspace.kds/kl27z-blinky-8000]$ cd debug
    [~/Projects/Sensorex/workspace.kds/kl27z-blinky-8000/debug]$ blhost -p /dev/cu.usbmodem1421,19200 -- flash-image kl27z-blinky-8000.srec erase
    Ping responded in 1 attempt(s)
    Inject command 'flash-image'
    Response status = 10200 (0x27d8) kStatusMemoryRangeInvalid
    [~/Projects/Sensorex/workspace.kds/kl27z-blinky-8000/debug]$ 

Run it under debugger to find the origin of the kStatusMemoryRangeInvalid error.

It appears that in `mem_erase(uint32_t address, uint32_t length)` in memory.c,
it's attempting to erase address=0, len=0x400.  It's possible that
kl27z-blinky-8000.srec is not properly constructed.  Compare it to the
kl25z-blinky-8000.srec file to see what addresses are involved.

Sure enough, the kl27z srec file's second line is:

    S113000000300020398600009D860000A186000093
    
indicating a record to be written at 0x0000 when the kl25z srec file is:

    S113800000300020398600009D860000A186000013
 
 which starts at 0x8000.
 
 Found it.  Typo in MKL27Z64xxx4_flash.ld.  
 
 But it's still getting the same error, but with an address of 0x8000 and a
 length of 0x400.  That's interesting, because the memory map in
 spexifies a `m_interrupts` section with length 0x0200:
 
       m_interrupts          (RX)  : ORIGIN = 0x00008000, LENGTH = 0x0200

The problem seems to be that the memory map for RAM starts at zero and ends at
0x20002fff, so it fails in `mem_is_block_reserved()`.  It should start at
0x1ffff000.  Note that in` mem_init()` at memory.c:288 it properly picks up those 
values from `g_bootloaderContext.memoryMap`.

But at line 230 in property.c, ramStart is 0:

    propertyStore->reservedRegions[kProperty_RamReservedRegionIndex].startAddress = ramStart;

And referenced in memory.c at line 260:

    reserved_region_t *region = &g_bootloaderContext.propertyInterface->store->reservedRegions[i];

... which is why the test fails.  Forcing ramStart to be 0x1ffff000 made the 
`flash-image` command succeed.  (Now to see if the program actually loaded...)
(and then to see why ramStart isn't set properly)

## More ramStart info...

`RAM_START` is defined to be `__VECTOR_RAM`.  In the .map file, we have:

                0x000000001ffff000                __VECTOR_RAM__ = .

and then:

                0x0000000000000000                __VECTOR_RAM = DEFINED (__ram_vector_table__)?__VECTOR_RAM__:ORIGIN (m_interrupts)

Since `__VECTOR_RAM__` is the correct value, it's evident that the probmem is
that `__ram_vector_table__` is not defined.  Let's see how KL25Z handles this...

As previously noted, the KL25Z bootloader project has this in the compilation
args:

    -Xlinker --defsym=__ram_vector_table__=1

In Project => Properties => C/C++ Build => Settings => Cross ARM C Linker => Miscellaneous =>
Other Linker Flags, we have:

    -Xlinker -z  -Xlinker muldefs   -flto  -fno-inline-small-functions  --specs=nano.specs          -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -fshort-wchar -Wl,--no-wchar-size-warning  -mapcs  -Xlinker -static  -Xlinker --defsym=__ram_vector_table__=1

The correspoinding Other Linker Flags in the k27Z project are:

    -specs=nosys.specs -Xlinker -z -Xlinker muldefs
    
I don't know the meaning of all the settings, but I'm adding

    -Xlinker --defsym=__ram_vector_table__=1

to the KL27Z project flags.

As with the KL25Z project, this doesn't seem to make a difference in the .ld
file, but I'll have ot check the runtime behavior.

Yep, that works.  Now the system will execute

    $ blhost -p /dev/cu.usbmodem1421,19200 -- flash-image kl27z-blinky-8000.srec erase
    Ping responded in 1 attempt(s)
    Inject command 'flash-image'
    Successful generic response to command 'flash-erase-region'
    Successful generic response to command 'flash-erase-region'
    Successful generic response to command 'flash-erase-region'
    Wrote 192 bytes to address 0x8000
    Successful generic response to command 'write-memory'
    (1/3)100% Completed!
    Successful generic response to command 'write-memory'
    Wrote 52 bytes to address 0x83c0
    Successful generic response to command 'write-memory'
    (2/3)100% Completed!
    Successful generic response to command 'write-memory'
    Wrote 7724 bytes to address 0x8400
    Successful generic response to command 'write-memory'
    (3/3)100% Completed!
    Successful generic response to command 'write-memory'
    Response status = 0 (0x0) Success.
    $

# It's working

I've now been able to:

* run flash-erase-all and verify that it erased memory
* run flash-image kl27z-blinky-8000.srec erase and verify the code was loaded
* use the debugger to find entry point (0x8639) and start the program there

One remaining step is to find why `-- reset` doesn't start the downloaded
program.  Or maybe it does...  Nope.  It seems to get back into the bootloader.
Need to debug.

On second thought, maybe it's doing the right thing: what *should* it do on reset?

Lets see if we can use the blhost to start the app.

    $ blhost -p /dev/cu.usbmodem1421,19200 -- call 0x8639 0
    Ping responded in 1 attempt(s)
    Inject command 'call'

Yes, this starts the app but blhost hangs -- that's to be expected.

I think the answer lies in the Bootloader Configuration Area:

Ah!  Check `bootloader_configuration_data_t bca->bootFlags` -- if set
to 0xFE, that requests "direct boot".  Currently, directBoot is NOT
set, so it falls over to the flash-resident bootloader.

The fix is in the kl27z_blinky_8000 app -- see
`sources/bootloader_support.c` -- the `.bootFlags` field is set to
0xff, which means "defer to the bootloader".  If set to 0xfe, it
the bootloader would automatically run the app.  Let's try it...

Almost.  Got a trap (div by zero?):

In blhost:

    $ blhost -p /dev/cu.usbmodem1421,19200 -- reset
    Ping responded in 1 attempt(s)
    Inject command 'reset'
    Successful generic response to command 'reset'
    Response status = 0 (0x0) Success.
    $
    
In the debugger:


      Thread #1 <main> (Suspended : Signal : SIGTRAP:Trace/breakpoint trap)	
	__udivsi3() at 0x668	
	LPUART_Init() at fsl_lpuart.c:265 0x6368	
	serial_configure_for_uart() at uart0_peripheral_interface.c:279 0x4af4	
	serial_set_state() at uart0_peripheral_interface.c:236 0x4a30	
	uart0_full_init() at uart0_peripheral_interface.c:138 0x48f0	
	get_active_peripheral() at bl_main.c:305 0x4414	
	bootloader_init() at bl_main.c:566 0x464e	
	bl_main() at bl_main.c:613 0x4754	
	main() at main.c:23 0x4b74	

It's curious that it's trying to configure the UART.  I would have thought it
would go straight to the app.  Let's try it again w single stepping.

Squashed a couple of bugs.  The biggest issue was that it was was calling
BOARD_DeinitPins() twice.  The GPIO port was disabled in the first call, and in
the second call an attempt to change the mux settings caused a hard fault, which
caused the bootloader to restart.

## Next bug: need to give the bootloader a chance to take over.

Evidently when directBoot is enabled, it's jumping immediately into the user
code.  Instead, we need it to poll the peripheral for a PING sequence for five
seconds before jumping to the user code.  At least, I believe that's how it's
supposed to work.

# bl_main (re)design

bl_main does several things.

1. initialize peripherals
2. if there is a runnable image and directBoot is set:
3.   enable timer for n seconds
4. while timer disabled OR timer not expired
5.   poll peripherals for ping
6.   if ping detected run bootloader
7. run image

# Time for a diet

Interesting.  After re-writing bl_main(), the flash-resident bootloader now
reserves 0x0 - 0x87ff in flash, because `__etext` = 0x84b0, that is, it's now
greater than 0x8000.  So either we need to recompile kl27z-blinky-8000 to start
at 0x9000, or put the flash bootloader on a diet to make it smaller.

In production, this probably won't be an issue because we'll compile it with
optimization, but it's useful to see where the memory is going.

A big chunk is in the debug console.  It's interesting that references to it 
are under the #if DEBUG conditional, which is false, but the code is still
getting loaded.  

I see -- `assert()`, defined in `fsl_common()`, invokes `PRINTF()` which brings 
in the debug console.

So my fix was to modify `fsl_debug_console.h` as follows:

    #ifndef DEBUG
    // deactivate calls to DbgConsole functions
    #define PRINTF sizeof
    #define SCANF sizeof
    #define PUTCHAR sizeof
    #define GETCHAR sizeof
    #elif SDK_DEBUGCONSOLE 
    ...

which eliminates the DbgConsole functions and brings etext back down to 0x7e6c.
Whew.
   
# blhost

Anything sent to the board from the host via USB is echoed back to the host -- 
this is a function of the USB / RS485 interface and is not controlled by the
firmware in the board.

This is a problem, since the blhost program does not expect to hear its own
transmitted characters echoed back to it.

So I'm tracing through the blhost code.  The low-level write routine happens
in a call to serial_write() within in blhost/src/blfwk/src/serial.c:222.  It
may be possible to disable the read routines whenever serial_write() is called.

Load level read is serial_read() in blhost/src/blfwk/src/serial.c:247.

# Size of bootloader

In general, we'd like to keep the bootloader's footprint as small as possible,
since that determines the page boundary at which the user code can start.
Currently, we're using 0x8000, but that can almost certainly be lowered.

The value of _etext is the highest address used by the bootloader; rounding up
to the next page boundary indicates where the user code can start.  Here are the
values of _etext for various compiler optimization levels and flags.

_Note: I found it was necessary to delete the Debug/ and or Release/ directories
prior to rebuilding in order to reliably get an updated xxx.map file._

                             +-----------------------------------------------------+
                             |         _etext for given optimization level         |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    | Configuration | -flto ||  -O0   |  -O1   |  -O2   |  -O3   |  -Os   |  -Og   |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    |    Debug      |  no   || 0x82d4 | 0x5574 | 0x5510 | 0x6144 | 0x4f28 | 0x576c |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    |    Debug      |  yes  || 0x884c | 0x4230 | 0x4104 | 0x57d8 | 0x3c30 | 0x54f4 |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    |    Release    |  no   || 0x7194 | 0x4668 | 0x463c | 0x5294 | 0x40d8 | 0x4854 |
    +---------------+-------++--------+--------+--------+--------+--------+--------|
    |    Release    |  yes  || 0x7198 | 0x4160 | 0x4080 | note 1 | 0x3c48 | 0x47d8 |
    +---------------+-------++--------+--------+--------+--------+--------+--------|

Note that _etext for the Release configuration with -flto (link time optimizer)
and -Os (optimize for size) is under 0x4000 -- this means we can locate our user
application at 0x4000 rather than at 0x8000 which gives us substantially more
flash memory for our code.

Note 1:

    ../source/crc/src/crc32.c: In function 'crc32_finalize':
    ../source/crc/src/crc32.c:89:6: internal compiler error: Segmentation fault: 11
     void crc32_finalize(crc32_data_t *crc32Config, uint32_t *hash)
          ^
    libbacktrace could not find executable to open
    Please submit a full bug report,
    with preprocessed source if appropriate.
    See <http://gcc.gnu.org/bugs.html> for instructions.
    lto-wrapper: arm-none-eabi-g++ returned 1 exit status
    /Applications/KDS_v3.app/Contents/toolchain/bin/../lib/gcc/arm-none-eabi/4.8.4/../../../../arm-none-eabi/bin/ld: lto-wrapper failed

