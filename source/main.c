/**
 * This is template for main module created by New Kinetis SDK 2.x Project Wizard. Enjoy!
 **/

#include "board.h"
#include "bootloader/bl_main.h"

/*!
 * @brief Application entry point.
 */
int main(void) {
	BOARD_InitBoard();

#if DEBUG
	BOARD_InitDebugConsole();
#endif

  return bl_main();      // Doesn't return.  See bootloader/src/bl_main.c
}

//! Since we never exit this gets rid of the C standard functions that cause
//! extra ROM size usage.
void exit(int arg) __attribute__ ((noreturn));

void exit(int arg)
{
	while(1) {}
}

