/**
 * gpio_platform.c
 * 
 * Arduino-like API interface to general-purpose I/O functions for the 
 * Espressif Internet-of-Things (IoT) Development Framework ESP-IDF
 * 
 * (c) 2021 by David Asher
 * https://github.com/david-asher
 * https://www.linkedin.com/in/davidasher/
 * This code is licensed under MIT license, see LICENSE.txt for details
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_platform_esp.h"
#include "esp_timer.h"
/**
 * These functions emulate Arduino general pin IO,
 * leaving the original source code intact.
 * 
 */

void pinMode( short pin, short mode )
{
    // arduino definition: mode = INPUT, OUTPUT, or INPUT_PULLUP, or INPUT_PULLDOWN
    // Added OUTPUT_OPEN for open-drain output

    gpio_config_t io_conf;
    io_conf.pin_bit_mask = 1ULL << ((gpio_num_t) pin);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    switch( mode ) {
        case INPUT:
            io_conf.mode = GPIO_MODE_INPUT;
            break;
        case INPUT_PULLUP:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            break;
        case INPUT_PULLDOWN:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            break;
        case OUTPUT:
            io_conf.mode = GPIO_MODE_OUTPUT;
            break;
        case OUTPUT_OPEN:
            io_conf.mode = GPIO_MODE_OUTPUT_OD;
            break;
    }
    ESP_ERROR_CHECK( gpio_config(&io_conf) );
}

void digitalWrite( short pin, short value )
{
    gpio_set_level( (gpio_num_t) pin, (uint32_t) (value != 0) );
}

int digitalRead( short pin )
{
    return gpio_get_level( (gpio_num_t) pin );
}

void delay( int ms )
{
    vTaskDelay( ms / portTICK_PERIOD_MS );
}

uint32_t millis()
{
    return (uint32_t) ( esp_timer_get_time() / 1000 );
}