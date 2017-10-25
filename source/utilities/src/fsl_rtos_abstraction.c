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

#include "fsl_common.h"
#include "bootloader/bl_peripheral_interface.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "fsl_device_registers.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

enum _sync_constants
{
    kSyncUnlocked = 0,
    kSyncLocked = 1
};

static lock_object_t lockObject;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void sync_init(sync_object_t *obj, bool state)
{
    *obj = state ? kSyncLocked : kSyncUnlocked;
}

bool sync_wait(sync_object_t *obj, uint32_t timeout)
{
    // Increment the object so we can tell if it changes. Because the increment is not
    // atomic (load, add, store), we must disabled interrupts during it.
    __disable_irq();
    ++(*obj);
    __enable_irq();

    // Wait for the object to be unlocked.
    while (*obj != 0)
    {
        // Spin.
    }

    return true;
}

void sync_signal(sync_object_t *obj)
{
    // Atomically unlock the object.
    __disable_irq();
    --(*obj);
    __enable_irq();
}

void sync_reset(sync_object_t *obj)
{
    __disable_irq();
    (*obj) = 0;
    __enable_irq();
}

void lock_init(void)
{
    __disable_irq();
    lockObject = 0;
    __enable_irq();
}

void lock_acquire(void)
{
    // Disable global IRQ until lock_release() is called.
    __disable_irq();
    ++lockObject;
}

void lock_release(void)
{
    // Restore previous state, enable global IRQ if all locks released.
    if (lockObject <= 1)
    {
        lockObject = 0;
        __enable_irq();
    }
    else
    {
        --lockObject;
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
