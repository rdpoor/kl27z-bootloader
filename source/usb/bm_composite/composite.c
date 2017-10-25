/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include "usb_device_config.h"
#include "usb/include/usb.h"
#include "usb/device/usb_device.h"

#include "usb/device/class/usb_device_class.h"
#include "usb/device/class/usb_device_msc.h"
#include "usb/device/class/usb_device_hid.h"
#include "usb/device/usb_device_ch9.h"
#include "usb/bm_composite/usb_descriptor.h"

#include "composite.h"

#include "fsl_device_registers.h"

#include <stdio.h>
#include <stdlib.h>

usb_device_class_config_struct_t g_composite_device[USB_COMPOSITE_INTERFACE_COUNT] = {
#if ((USB_DEVICE_CONFIG_HID) && (USB_DEVICE_CONFIG_HID > 0U))
    {
        usb_device_hid_generic_callback, (class_handle_t)NULL, &g_hid_generic_class,
    },
#endif
#if USB_DEVICE_CONFIG_MSC
    {
        usb_device_msc_callback, (class_handle_t)NULL, &g_msc_class,
    }
#endif
#if ((USB_DEVICE_CONFIG_HID == 0) && (USB_DEVICE_CONFIG_MSC == 0))
    {
        (usb_device_class_callback_t)NULL,
        (class_handle_t)NULL,
		(usb_device_class_struct_t *)NULL,
    }
#endif

};

usb_device_class_config_list_struct_t g_composite_device_config_list = {
    g_composite_device, usb_device_callback, USB_COMPOSITE_INTERFACE_COUNT,
};
