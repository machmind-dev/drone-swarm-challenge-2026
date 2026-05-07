/**
 * i2c_platform_esp.h
 *
 * I2C device interface for the
 * Espressif Internet-of-Things (IoT) Development Framework ESP-IDF
 *
 * I2C_Master is the global I2C interface shared by all devices
 *
 * (c) 2021 by David Asher
 * https://github.com/david-asher
 * https://www.linkedin.com/in/davidasher/
 * This code is licensed under MIT license, see LICENSE.txt for details
 */

#ifndef _I2C_PLATFORM_ESP_H_
#define _I2C_PLATFORM_ESP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define I2C_DEFAULT_PORT    (I2C_NUM_1)
#define I2C_DEFAULT_SDA     (GPIO_NUM_5)
#define I2C_DEFAULT_SCL     (GPIO_NUM_6)
#define I2C_DEFAULT_FREQ    (400000)

#define I2C_READ            (I2C_MASTER_READ)
#define I2C_WRITE           (I2C_MASTER_WRITE)

#define I2C_NO_DEVICE       (0xFF)

#define ACK_CHECK_EN        true

#define i2c_start_write( dev_address ) i2c_start( dev_address, I2C_WRITE )
#define i2c_start_read( dev_address ) i2c_start( dev_address, I2C_READ )

void i2c_scan();
void i2c_get_config( i2c_port_t *port, gpio_num_t *pin_sda, gpio_num_t *pin_scl, uint32_t *freq );
void i2c_init_config( i2c_port_t port, gpio_num_t pin_sda, gpio_num_t pin_scl, uint32_t freq );
void i2c_init();
void i2c_remove();
void i2c_upgrade( uint32_t upgrade_freq );
bool i2c_start( uint8_t i2c_device_address, i2c_rw_t read_write );
size_t i2c_write_byte( uint8_t data_byte_out );
size_t i2c_write( uint8_t *pByteBuffer, size_t NumByteToWrite );
uint8_t i2c_read_byte();
size_t i2c_read( uint8_t *pByteBuffer, size_t NumByteToRead );
esp_err_t i2c_transmit();

#ifdef __cplusplus
}
#endif

#endif // _I2C_PLATFORM_ESP_H_
