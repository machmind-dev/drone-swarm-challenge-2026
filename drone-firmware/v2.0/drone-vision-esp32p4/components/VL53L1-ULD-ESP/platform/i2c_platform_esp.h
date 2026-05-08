/**
 * i2c_platform_esp.h — New I2C master driver (driver_ng) shim for VL53L1X.
 *
 * Replaces the legacy i2c_driver_install API that conflicts with esp_video's
 * SCCB, which already claims the same I2C port via i2c_new_master_bus().
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>

#define I2C_DEFAULT_PORT    (I2C_NUM_1)
#define I2C_DEFAULT_SDA     (GPIO_NUM_7)
#define I2C_DEFAULT_SCL     (GPIO_NUM_8)
#define I2C_DEFAULT_FREQ    400000

/**
 * Initialise the I2C platform.
 *
 * Tries i2c_new_master_bus() first (works when the port is free — separate bus
 * from camera SCCB).  If the port is already owned by driver_ng (e.g. esp_video
 * SCCB on the shared GPIO7/GPIO8 bus), borrows the existing handle via
 * i2c_master_get_bus_handle() with up to 50 x 100 ms retries so the SCCB
 * initialisation can finish first.
 */
void      i2c_init_config(i2c_port_num_t port, gpio_num_t pin_sda,
                          gpio_num_t pin_scl, uint32_t freq);

esp_err_t i2c_write_multi(uint8_t dev_addr8, uint16_t reg,
                          uint8_t *data, uint32_t count);
esp_err_t i2c_read_multi(uint8_t dev_addr8, uint16_t reg,
                         uint8_t *data, uint32_t count);

/* Probe all 7-bit addresses 0x08–0x77 and log any that ACK. */
void i2c_scan(void);

/* Reset the bus (SCL clock pulses to free stuck SDA) and evict the cached
 * device handle for the VL53L1X default address 0x29 so the next BootState
 * attempt gets a fresh handle.  Call after i2c_scan() and after each sensor
 * slot timeout to prevent ghost state from corrupting subsequent inits. */
void i2c_bus_reset(void);

#ifdef __cplusplus
}
#endif
