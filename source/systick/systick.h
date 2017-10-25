/*
 * systick.h
 *
 *  Created on: Oct 3, 2017
 *      Author: r
 */

/*
 * systick implements a running tick counter based on the fsl_pit driver.
 */

#include <stdint.h>

#ifndef SYSTICK_H_
#define SYSTICK_H_

//! @brief initialize the systick system
void systick_init();

//! @brief shut down the systick counter
void systick_deinit();

//! @brief Convert ticks to microseconds
uint32_t systick_ticks_to_microseconds(uint64_t ticks);

//! @brief Convert microseconds to ticks
uint64_t systick_microseconds_to_ticks(uint32_t useconds);

//! @brief Get the running systick count in microseconds
uint32_t systick_get_microseconds();

//! @brief Get the running systick count in ticks
uint64_t systick_get_ticks();

//! @brief Busy wait for the specified number of microseconds
void systick_delay_microseconds(uint32_t microseconds);

//! @brief Busy wait for the specified number of ticks
void systick_delay_ticks(uint64_t ticks);

//! @brief Return the frequency of the systick counter
uint64_t systick_get_ticks_per_second();

#endif /* SYSTICK_H_ */
