/*
 * bl_main.h
 *
 *  Created on: Sep 29, 2017
 *      Author: r
 */

#ifndef SOURCE_BOOTLOADER_BL_MAIN_H_
#define SOURCE_BOOTLOADER_BL_MAIN_H_

//! @brief Main entry point for bootloader
int bl_main(void);
bool application_location_is_valid(uint32_t applicationAddress);

#endif /* SOURCE_BOOTLOADER_BL_MAIN_H_ */
