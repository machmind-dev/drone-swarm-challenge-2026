/**
 * gpio_platform.h
 * 
 * Arduino-like API interface to general-purpose I/O functions for the 
 * Espressif Internet-of-Things (IoT) Development Framework ESP-IDF
 * 
 * (c) 2021 by David Asher
 * https://github.com/david-asher
 * https://www.linkedin.com/in/davidasher/
 * This code is licensed under MIT license, see LICENSE.txt for details
 */

#ifndef _gpio_platform_h_
#define _gpio_platform_h_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "driver/gpio.h"

// from wiring.h, kinda
#define LOW             0x00
#define HIGH            0x01
#define INPUT           0x00
#define OUTPUT          0x01
#define INPUT_PULLUP    0x02
#define INPUT_PULLDOWN  0x04
#define OUTPUT_OPEN     0x08

void pinMode( short pin, short mode );
void digitalWrite( short pin, short value );
int digitalRead( short pin );
void delay( int ms );
uint32_t millis();

#ifdef __cplusplus
}
#endif

#endif // _gpio_platform_h_