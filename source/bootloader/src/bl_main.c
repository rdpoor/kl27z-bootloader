/*
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
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

#include <stdbool.h>
#include "fsl_common.h"
#include "bootloader/bl_context.h"
#include "bootloader/bl_peripheral.h"
#include "bootloader/bl_shutdown_cleanup.h"
#include "bootloader_common.h"
#include "systick/systick.h"
#include "bootloader/bootloader.h"
#include "fsl_flash.h"
#include "property/property.h"
#include "utilities/vector_table_info.h"
#include "utilities/fsl_rtos_abstraction.h"

#if BL_FEATURE_CRC_CHECK
#include "bootloader/bl_app_crc_check.h"
#endif
#if BL_FEATURE_QSPI_MODULE
#include "qspi/qspi.h"
#endif
#include "memory/memory.h"

#if BL_FEATURE_RELIABLE_UPDATE
#include "bootloader/bl_reliable_update.h"
#endif

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////

#if BL_FEATURE_POWERDOWN
#error BL_FEATURE_POWERDOWN not supported
#endif

#define NEVER_TIMEOUT 0

//! @addtogroup bl_core
//! @{

////////////////////////////////////////////////////////////////////////////////
// Local Prototypes (not exported in bl_main.h)
////////////////////////////////////////////////////////////////////////////////

//! @brief Entry point for the bootloader.
uint32_t get_timeout_ms(void);
static void bootloader_init(void);
void peripherals_init() __attribute__ ((noinline));
peripheral_descriptor_t const *scan_for_active_peripheral();
void shutdown_inactive_peripherals(peripheral_descriptor_t const *active_peripheral);
void setup_active_peripheral(peripheral_descriptor_t const *active_peripheral);
#if DEBUG && !DEBUG_PRINT_DISABLE
static const char *get_peripheral_name(uint32_t peripheralTypeMask);
#endif
bool direct_boot_is_enabled(void) __attribute__ ((noinline));
bool app_is_runnable(uint32_t *appEntry, uint32_t *appStack) __attribute__ ((noinline));
static void get_user_app_entry(uint32_t *appEntry, uint32_t *appStack);
bool app_contents_are_valid(uint32_t appEntry);
static void jump_to_app(uint32_t appEntry, uint32_t appStack);
#if BL_FEATURE_QSPI_MODULE
static void configure_quadspi_as_needed(void);
#endif
static void bootloader_flash_init(void);
static void bootloader_run(void);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

#if DEBUG && !DEBUG_PRINT_DISABLE
static const char *const kPeripheralNames[] = {
    "UART", // kPeripheralType_UART
    "I2C",  // kPeripheralType_I2CSlave
    "SPI",  // kPeripheralType_SPISlave
    "CAN",  // kPeripheralType_CAN
    "HID",  // kPeripheralType_USB_HID
    "CDC",  // kPeripheralType_USB_CDC
    "DFU",  // kPeripheralType_USB_DFU
    "MSD"   // kPeripheralType_USB_MSC
};
#endif // DEBUG

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! @brief Entry point for the bootloader.
int bl_main(void)
{
	uint32_t timeout_us = NEVER_TIMEOUT;
	uint32_t starting_us;
	uint32_t appEntry, appStack;
    peripheral_descriptor_t const *active_peripheral = NULL;

	bootloader_init();
	peripherals_init();

	if (direct_boot_is_enabled() && app_is_runnable(&appEntry, &appStack)) {
		// Run the user application unless we detect activity on one of the
		// peripherals within timeout_us.  Start the timeout timer now.
		timeout_us = get_timeout_ms() * 1000;
		starting_us = systick_get_microseconds();
	}

	// Listen for a ping sequence from the peripherals.  If detected before the
	// timeout timer expires, enter the bootloader run loop.
	while ((timeout_us == NEVER_TIMEOUT) ||
		   ((systick_get_microseconds() - starting_us) < timeout_us)) {
		active_peripheral = scan_for_active_peripheral();
		if (active_peripheral != NULL) {
			// Arrive here when a ping was detected from an active peripheral
			shutdown_inactive_peripherals(active_peripheral);
			setup_active_peripheral(active_peripheral);
			bootloader_run();  // never returns
		}
	}

	// Arrive here if and when the timeout expired.  run the user application
	jump_to_app(appEntry, appStack); // never returns

    // Should never end up here.
    debug_printf("Warning: reached end of main()\r\n");
    return 0;
}

// Get the timeout for detecting peripheral activity.  Defaults to
// BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT but may be over-ridden by
// a value in the configuration data.
uint32_t get_timeout_ms(void) {
	uint32_t timeout_ms;

    bootloader_configuration_data_t *configurationData =
        &g_bootloaderContext.propertyInterface->store->configurationData;

    timeout_ms = BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT;
    if (configurationData->peripheralDetectionTimeoutMs != 0xFFFF)
    {
    	timeout_ms = configurationData->peripheralDetectionTimeoutMs;
    }
    return timeout_ms;
}

static void bootloader_init(void)
{
    lock_init();                       // initialize global lock
    init_hardware();                   // Init pinmux and other hardware setup.
    bootloader_flash_init();           // Init flash driver.

    // Load the user configuration data so that we can configure the clocks
    g_bootloaderContext.propertyInterface->load_user_config();

#if BL_FEATURE_QSPI_MODULE
    configure_quadspi_as_needed();     // Init QSPI module if needed
#endif // BL_FEATURE_QSPI_MODULE

    configure_clocks(kClockOption_EnterBootloader);  // Configure clocks.
    systick_init();                    // Start the lifetime counter

#if BL_FEATURE_BYPASS_WATCHDOG
    g_bootloaderContext.flashDriverInterface->flash_register_callback(
    		&g_bootloaderContext.flashState,
            bootloader_watchdog_service);
    bootloader_watchdog_init();
#endif // BL_FEATURE_BYPASS_WATCHDOG

    g_bootloaderContext.memoryInterface->init();    // Init SRAM interface
    g_bootloaderContext.propertyInterface->init();  // Init the property store.

#if BL_FEATURE_RELIABLE_UPDATE
    bootloader_reliable_update_as_requested(kReliableUpdateOption_Normal, 0);
#endif // BL_FEATURE_RELIABLE_UPDATE

    // Message so python instantiated debugger can tell the
    // bootloader application is running on the target.
    debug_printf("\r\n\r\nRunning bootloader...\r\n");

#if DEBUG && !DEBUG_PRINT_DISABLE
    standard_version_t version = g_bootloaderContext.propertyInterface->store->bootloaderVersion;
    debug_printf("Bootloader version %c%d.%d.%d\r\n", version.name, version.major, version.minor, version.bugfix);
#endif
}

// =============================================================================
// dealing with peripherals.
//
// The bootloader maintains a list of available peripherals that may be used
// for communicating with the host.  When legitimate activity is detected on
// one of these peripherals (e.g. a ping packet received on the serial port),
// that peripheral becomes "active" and the other peripherals are shut down.

// Do hardware level initialization for those peripherals marked as 'enabled'.
void peripherals_init() {
    peripheral_descriptor_t const *peripheral;
    bootloader_configuration_data_t *configurationData =
        &g_bootloaderContext.propertyInterface->store->configurationData;

    // Bring up all the peripherals
    for (peripheral = g_peripherals; peripheral->typeMask != 0; ++peripheral)
    {
        // Check that the peripheral is enabled in the user configuration data
        if (configurationData->enabledPeripherals & peripheral->typeMask)
        {
            assert(peripheral->controlInterface->init);

            debug_printf("Initing %s\r\n", get_peripheral_name(peripheral->typeMask));
            peripheral->controlInterface->init(peripheral, peripheral->packetInterface->byteReceivedCallback);
        }
    }
}

// Poll each enabled peripheral, returning that peripheral if activity has been
// detected, typically when a ping sequence has been received.  Otherwise return
// NULL.
peripheral_descriptor_t const *scan_for_active_peripheral() {
    peripheral_descriptor_t const *peripheral;
    peripheral_descriptor_t const *active_peripheral = NULL;
    bootloader_configuration_data_t *configurationData =
        &g_bootloaderContext.propertyInterface->store->configurationData;

    // Traverse through all the peripherals
    for (peripheral = g_peripherals; peripheral->typeMask != 0; ++peripheral)
    {
        // Check that the peripheral is enabled in the user configuration data
        if (configurationData->enabledPeripherals & peripheral->typeMask)
        {
            assert(peripheral->controlInterface->pollForActivity);

            if (peripheral->controlInterface->pollForActivity(peripheral))
            {
                debug_printf("%s is active\r\n", get_peripheral_name(peripheral->typeMask));
                active_peripheral = peripheral;
                break;
            }
        }
    }
    return active_peripheral;
}

// One peripheral has been selected as the active_peripheral.  De-initialize all
// of the other peripherals.
void shutdown_inactive_peripherals(peripheral_descriptor_t const *active_peripheral) {
	peripheral_descriptor_t const *peripheral;
    bootloader_configuration_data_t *configurationData =
        &g_bootloaderContext.propertyInterface->store->configurationData;

	// Shut down all non active peripherals
	for (peripheral = g_peripherals; peripheral->typeMask != 0; ++peripheral)
	{
		// Check that the peripheral is enabled in the user configuration data
		if (configurationData->enabledPeripherals & peripheral->typeMask)
		{
			if (active_peripheral != peripheral)
			{
				debug_printf("Shutting down %s\r\n", get_peripheral_name(peripheral->typeMask));
				assert(peripheral->controlInterface->shutdown);
				peripheral->controlInterface->shutdown(peripheral);
			}
		}
	}
}

// Make the active_peripheral known to the bootloader, initializing the
// byte interface, packet interface and command interface.
void setup_active_peripheral(peripheral_descriptor_t const *active_peripheral) {
    g_bootloaderContext.activePeripheral = active_peripheral;

    assert(g_bootloaderContext.activePeripheral);

    // Validate required active peripheral interfaces.
    assert(g_bootloaderContext.activePeripheral->controlInterface);

    // Initialize the interfaces to active peripheral.
    if (g_bootloaderContext.activePeripheral->byteInterface &&
        g_bootloaderContext.activePeripheral->byteInterface->init)
    {
        g_bootloaderContext.activePeripheral->byteInterface->init(g_bootloaderContext.activePeripheral);
    }
    if (g_bootloaderContext.activePeripheral->packetInterface &&
        g_bootloaderContext.activePeripheral->packetInterface->init)
    {
        g_bootloaderContext.activePeripheral->packetInterface->init(g_bootloaderContext.activePeripheral);
    }

    // Initialize the command processor component.
    g_bootloaderContext.commandInterface->init();
}

#if DEBUG && !DEBUG_PRINT_DISABLE
//! @brief Returns the name of a peripheral given its type mask.
const char *get_peripheral_name(uint32_t peripheralTypeMask)
{
    uint32_t i;
    for (i = 0; i < ARRAY_SIZE(kPeripheralNames); ++i)
    {
        if (peripheralTypeMask & (1 << i))
        {
            return kPeripheralNames[i];
        }
    }

    return "Unknown peripheral";
}
#endif // DEBUG

// =============================================================================
// dealing with the user application.
//
// Before we try to start executing the user application, it must pass a number
// of tests.

// Return true if the direct boot bit is set (read from the Boot Control Area)
bool direct_boot_is_enabled(void)
{
    bootloader_configuration_data_t *configurationData =
        &g_bootloaderContext.propertyInterface->store->configurationData;

    bool direct_boot = (~configurationData->bootFlags) & kBootFlag_DirectBoot;
    return direct_boot;
}

// Return true if the application appears valid.  As a side effect, get the
// entry point and stack pointer for the app.
bool app_is_runnable(uint32_t *appEntry, uint32_t *appStack) {
    // Get the user application entry point and stack pointer.
    get_user_app_entry(appEntry, appStack);

    if (!application_location_is_valid(*appEntry)) {
    	return false;
    }
    if (!app_contents_are_valid(*appEntry)) {
    	return false;
    }
    return true;
}

//! @brief Returns the user application address and stack pointer.
//!
//! For flash-resident and rom-resident target, gets the user application address
//! and stack pointer from the APP_VECTOR_TABLE.
//! Ram-resident version does not support jumping to application address.
static void get_user_app_entry(uint32_t *appEntry, uint32_t *appStack)
{
    assert(appEntry);
    assert(appStack);

#if BL_TARGET_RAM
    *appEntry = 0;
    *appStack = 0;
#else
#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    // Check if address of SP and PC is in an execute-only region.
    if (!is_in_execute_only_region(kDefaultVectorTableAddress, 8))
    {
        *appEntry = APP_VECTOR_TABLE[kInitialPC];
        *appStack = APP_VECTOR_TABLE[kInitialSP];
    }
    else
    {
        // Set to invalid value when vector table is in execute-only region,
        // as ROM doesn't support jumping to an application in such region so far.
        // The main purpose of below operation is to prevent ROM from inifinit loop
        // between NVIC_SystemReset() and fetching SP and PC frome execute-only region.
        *appEntry = 0;
        *appStack = 0;
    }
#else
    *appEntry = APP_VECTOR_TABLE[kInitialPC];
    *appStack = APP_VECTOR_TABLE[kInitialSP];
#endif //  FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
#endif // BL_TARGET_RAM
}

//! A given jump address is considered valid if:
//! - Not 0x00000000
//! - Not 0xffffffff
//! - Not the reset handler entry point for the bootloader
//! - Is in flash or is in RAM or QuadSPI (if available)
//! @note this interface is also used by the configure_quadspi command
bool application_location_is_valid(uint32_t appEntry)
{
    const memory_map_entry_t *map;
    // Verify that the jumpLocation is non zero and then either within flash or RAM, both calculations are:
    // (jumpLocation >= startAddress) && (jumpLocation < (startAddress + size))
    if ((!appEntry) ||              // address is not null AND
        (appEntry == 0xffffffff) || // address is not blank Flash (0xff) AND
        (appEntry == (uint32_t)&Reset_Handler))
    {
        return false;
    }

    bool isValid = false;
    const uint32_t minThumb2InstructionSize = 2; // smallest thumb2 instruction size is 16-bit.
    // Check if the application address is in valid executable memory range
    status_t status = find_map_entry(appEntry, minThumb2InstructionSize, &map);
    if ((status == kStatus_Success) && (map->isExecutable))
    {
        isValid = true;
    }

    return isValid;
}

bool app_contents_are_valid(uint32_t appEntry) {
	bool isValid = true;

#if BL_FEATURE_OTFAD_MODULE
    if (isValid && is_qspi_present())
    {
        quadspi_cache_clear();
        status_t status = otfad_init_as_needed();
        if (status != kStatus_Success)
        {
            isValid = false;
        }
        update_qspi_otfad_init_status(status);
    }
#endif

#if BL_FEATURE_CRC_CHECK
    // Validate application crc only if its location is valid
    if (isValid)
    {
        isValid = is_application_crc_check_pass();
    }

#if BL_FEATURE_OTFAD_MODULE
    otfad_bypass_as_needed();
#endif // BL_FEATURE_OTFAD_MODULE

#endif

    return isValid;
}

//! @brief Exits bootloader and jumps to the user application.
static void jump_to_app(uint32_t appEntry, uint32_t appStack)
{
#if BL_FEATURE_OTFAD_MODULE
    quadspi_cache_clear();
    oftfad_resume_as_needed();
#endif

    shutdown_cleanup(kShutdownType_Shutdown);

    // Create the function call to the user application.
    // Static variables are needed since changed the stack pointer out from under the compiler
    // we need to ensure the values we are using are not stored on the previous stack
    static uint32_t s_stackPointer = 0;
    s_stackPointer = appStack;
    static void (*farewellBootloader)(void) = 0;
    farewellBootloader = (void (*)(void))appEntry;

    // Set the VTOR to the application vector table address.
    SCB->VTOR = (uint32_t)APP_VECTOR_TABLE;

    // Set stack pointers to the application stack pointer.
    __set_MSP(s_stackPointer);
    __set_PSP(s_stackPointer);

    // Jump to the application.
    farewellBootloader();

    // Should never end up here.
    debug_printf("Warning: User app returned\r\n");
    return;
}

#if BL_FEATURE_QSPI_MODULE
static void configure_quadspi_as_needed(void)
{
    // Start the lifetime counter
	systick_init();
    if (qspi_need_configure())
    {
        status_t qspiOtfadInitStatus = kStatus_QspiNotConfigured;
        // Try to configure QuadSPI module based on on qspi_config_block_pointer in BCA first,
        // If bootloader cannot get qspi config block from internal flash, try to configure QSPI
        // based on default place (start address of QuadSPI memory).
        uint32_t qspi_config_block_base =
            g_bootloaderContext.propertyInterface->store->configurationData.qspi_config_block_pointer;

        // Get the start address and flash size
        uint32_t flashStart;
        g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.flashState,
                                                                     kFLASH_PropertyPflashBlockBaseAddr, &flashStart);
        uint32_t flashSize;
        g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.flashState,
                                                                     kFLASH_PropertyPflashTotalSize, &flashSize);

        // Check if the pointer of qspi config block is valid.
        if ((qspi_config_block_base != 0xFFFFFFFF) && (qspi_config_block_base > flashStart) &&
            (qspi_config_block_base <= (flashStart + flashSize - sizeof(qspi_config_t))))
        {
#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
            if (!is_in_execute_only_region(qspi_config_block_base, sizeof(qspi_config_t)))
            {
                qspiOtfadInitStatus = quadspi_init((void *)qspi_config_block_base);
            }
#else
            qspiOtfadInitStatus = quadspi_init((void *)qspi_config_block_base);
#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
        }

        if (qspiOtfadInitStatus == kStatus_QspiNotConfigured)
        {
            qspiOtfadInitStatus = quadspi_init(NULL);
        }
        update_qspi_otfad_init_status(qspiOtfadInitStatus);
    }
    // Shutdown the lifetime counter before configuring clock.
    lock_acquire();
    systic_deinit();
    lock_release();
}
#endif

static void bootloader_flash_init(void)
{
    g_bootloaderContext.flashDriverInterface->flash_init(&g_bootloaderContext.flashState);

#if BL_TARGET_FLASH
    //! @brief A static buffer used to hold flash_run_command()
    static uint32_t s_flashRunCommand[kFLASH_ExecuteInRamFunctionMaxSizeInWords];
    //! @brief A static buffer used to hold flash_cache_clear_command()
    static uint32_t s_flashCacheClearCommand[kFLASH_ExecuteInRamFunctionMaxSizeInWords];

    static flash_execute_in_ram_function_config_t s_flashExecuteInRamFunctionInfo = {
        .activeFunctionCount = 0,
        .flashRunCommand = s_flashRunCommand,
        .flashCommonBitOperation = s_flashCacheClearCommand,
    };

    g_bootloaderContext.flashState.flashExecuteInRamFunctionInfo = &s_flashExecuteInRamFunctionInfo.activeFunctionCount;
    g_bootloaderContext.flashDriverInterface->flash_prepare_execute_in_ram_functions(&g_bootloaderContext.flashState);
#endif
}

// =============================================================================
//! @brief Bootloader main loop.
//!
//! Infinitely calls the command interface and active peripheral control interface pump routines.
static void bootloader_run(void)
{
    const peripheral_descriptor_t *activePeripheral = g_bootloaderContext.activePeripheral;

    assert(g_bootloaderContext.commandInterface->pump);

    // Read and execute commands.
    while (1)
    {
        g_bootloaderContext.commandInterface->pump();

        // Pump the active peripheral.
        if (activePeripheral->controlInterface->pump)
        {
            activePeripheral->controlInterface->pump(activePeripheral);
        }
    }
}

#if defined(__CC_ARM)
#define ITM_Port8(n) (*((volatile unsigned char *)(0xE0000000 + 4 * n)))
#define ITM_Port16(n) (*((volatile unsigned short *)(0xE0000000 + 4 * n)))
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4 * n)))

#define DEMCR (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA 0x01000000

struct __FILE
{
    int handle; /* Add whatever needed */
};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
    if (DEMCR & TRCENA)
    {
        while (ITM_Port32(0) == 0)
            ;
        ITM_Port8(0) = ch;
    }
    return (ch);
}
#endif

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
