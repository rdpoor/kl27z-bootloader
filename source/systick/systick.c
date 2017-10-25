/*
 * systick.c
 *
 *  Created on: Oct 3, 2017
 *      Author: r
 */

#include "systick.h"
#include "fsl_pit.h"
#include "fsl_clock.h"
#include <stdint.h>

/**
 * systick implements a 64-bit high-speed tick counter based on the PIT module.
 *
 * Implementation note: with a 24MHz bus clock, the 64bit compound counter will
 * roll over once every 2^64/24,000,000 = 768,614,336,405 seconds.  A 32 bit
 * counter would roll over once every 2^32 / 24,000,000 = 178.9 seconds, which
 * may be a bit too short for some uses.  So we implement the 64 bit version.
 */

#define PIT_SOURCE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)

//! @brief initialize the systick system
void systick_init() {
    pit_config_t pitConfig;
    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);
    // Chain channel 1 (high 32 bits) and channel 0 (low 32 bits) and set both
    // for maximum period.  With a 24mHz bus clock, the counter will roll over
    // once every 2^64 / 24000000 = 768,614,336,405 seconds.
    PIT_SetTimerChainMode(PIT, kPIT_Chnl_1, true);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_1, 0xffffffffU);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, 0xffffffffU);
    PIT_StartTimer(PIT, kPIT_Chnl_1);
    PIT_StartTimer(PIT, kPIT_Chnl_0);
}

//! @brief Gates the PIT clock and disables the PIT module.
void systick_deinit() {
	PIT_Deinit(PIT);
}

//! @brief Convert ticks to microseconds
uint32_t systick_ticks_to_microseconds(uint64_t ticks) {
	uint32_t ticks_per_microsecond = (PIT_SOURCE_CLOCK_FREQ / 1000000);
	return ticks / ticks_per_microsecond;
}

//! @brief Convert microseconds to ticks
uint64_t systick_microseconds_to_ticks(uint32_t microseconds) {
	uint32_t ticks_per_microsecond = (PIT_SOURCE_CLOCK_FREQ / 1000000);
    return microseconds * ticks_per_microsecond;
}

//! @brief Get the running systick count in microseconds
uint32_t systick_get_microseconds() {
	uint64_t ticks = systick_get_ticks();
	return systick_ticks_to_microseconds(ticks);
}

//! @brief Get the running systick count in ticks
uint64_t systick_get_ticks() {
	uint64_t cticks = PIT_GetLifetimeTimerCount(PIT);
	// PIT counter counts down.  Compliment bits to count up.
	uint64_t ticks = ~cticks;
	return ticks;
}

//! @brief Busy wait for the specified number of microseconds
void systick_delay_microseconds(uint32_t microseconds) {
	uint64_t ticks = systick_microseconds_to_ticks(microseconds);
	systick_delay_ticks(ticks);
}

//! @brief Busy wait for the specified number of ticks
void systick_delay_ticks(uint64_t ticks) {
    uint64_t start_tics = systick_get_ticks();
    // due to unsigned arithmetic, this works even when the counter rolls over.
    while ((systick_get_ticks() - start_tics) <= ticks) {
    	;
    }
}

//! @brief Return the frequency of the systick counter
uint64_t systick_get_ticks_per_second() {
	uint64_t ticks_per_second = PIT_SOURCE_CLOCK_FREQ;
	return ticks_per_second;
}
