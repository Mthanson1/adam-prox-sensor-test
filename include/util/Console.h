/*
 * Serial console used by every module that prints.
 * On the Nucleo, Serial is USART2 (PA2/PA3), bridged to USB by the ST-LINK
 * virtual COM port.
 */

#pragma once

#include <Arduino.h>

#define CONSOLE Serial

/*
 * The ST-LINK/V2-1 bridge handles up to 2 Mbaud. 921600 gives ~92 kB/s, enough
 * headroom for the CSV stream at the default sample rate.
 */
#define CONSOLE_BAUD 921600
