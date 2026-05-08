/**
 * i2c_platform_esp.c — driver_ng (i2c_master) shim for VL53L1X on ESP-IDF 5.3+.
 *
 * ESP-IDF 5.x raises abort() if you mix the legacy i2c_driver_install() with
 * i2c_new_master_bus() on the same port.  The esp_video / esp_sccb_intf camera
 * stack uses driver_ng on the shared I2C bus (GPIO7 SDA / GPIO8 SCL, I2C_NUM_1
 * on the Waveshare ESP32-P4-WiFi6 board).  This file replaces the old shim with
 * one that uses driver_ng throughout.
 *
 * Bus ownership strategy
 * ----------------------
 *  1. Try i2c_new_master_bus() — succeeds when the port is free (future PCB
 *     with a dedicated VL53L1X I2C port).
 *  2. If the port is already taken (ESP_ERR_INVALID_STATE), borrow the handle
 *     that esp_video created via i2c_master_get_bus_handle().  Retry for up to
 *     5 s so that camera SCCB init can finish before we touch the bus.
 *
 * Device handles are cached in a small table keyed by 7-bit I2C address so
 * i2c_master_bus_add_device() is only called once per address.
 */

#include "i2c_platform_esp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "i2c_plat";

static i2c_master_bus_handle_t s_bus  = NULL;
static bool                    s_owned = false;
static uint32_t                s_freq  = I2C_DEFAULT_FREQ;

#define MAX_DEVICES 10
typedef struct { uint8_t addr7; i2c_master_dev_handle_t h; } dev_slot_t;
static dev_slot_t s_devs[MAX_DEVICES];
static int        s_ndevs = 0;

/* Look up or add a device by its 7-bit address. */
static i2c_master_dev_handle_t get_or_add(uint8_t addr7)
{
    for (int i = 0; i < s_ndevs; i++)
        if (s_devs[i].addr7 == addr7) return s_devs[i].h;

    if (s_ndevs >= MAX_DEVICES) {
        ESP_LOGE(TAG, "device table full");
        return NULL;
    }

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr7,
        .scl_speed_hz    = s_freq,
    };
    i2c_master_dev_handle_t h;
    esp_err_t ret = i2c_master_bus_add_device(s_bus, &cfg, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add_device 0x%02x failed: %s", addr7, esp_err_to_name(ret));
        return NULL;
    }
    s_devs[s_ndevs++] = (dev_slot_t){addr7, h};
    ESP_LOGD(TAG, "registered device 0x%02x", addr7);
    return h;
}

void i2c_init_config(i2c_port_num_t port, gpio_num_t sda,
                     gpio_num_t scl, uint32_t freq)
{
    if (s_bus) return;
    s_freq = freq;

    /* --- attempt to create a fresh bus (works for dedicated I2C port) --- */
    i2c_master_bus_config_t cfg = {
        .i2c_port              = port,
        .sda_io_num            = sda,
        .scl_io_num            = scl,
        .clk_source            = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt     = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&cfg, &s_bus);
    if (ret == ESP_OK) {
        s_owned = true;
        ESP_LOGI(TAG, "created I2C bus  port=%d  SDA=%d SCL=%d  %lu Hz",
                 (int)port, (int)sda, (int)scl, (unsigned long)freq);
        return;
    }

    /* --- port taken by driver_ng (camera SCCB) — borrow the handle --- */
    for (int attempt = 0; attempt < 50; attempt++) {
        ret = i2c_master_get_bus_handle(port, &s_bus);
        if (ret == ESP_OK && s_bus) {
            s_owned = false;
            /* SCCB may leave SDA stuck low (partial transaction or ISP poll).
             * Run 9-clock bus recovery before any VL53L1X transaction. */
            esp_err_t rst = i2c_master_bus_reset(s_bus);
            if (rst != ESP_OK)
                ESP_LOGW(TAG, "bus reset after borrow: %s", esp_err_to_name(rst));
            ESP_LOGI(TAG, "borrowed I2C bus from SCCB  port=%d  %lu Hz",
                     (int)port, (unsigned long)freq);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGE(TAG, "cannot get I2C bus handle on port %d: %s",
             (int)port, esp_err_to_name(ret));
}

esp_err_t i2c_write_multi(uint8_t dev_addr8, uint16_t reg,
                          uint8_t *data, uint32_t count)
{
    if (!s_bus) return ESP_ERR_INVALID_STATE;
    i2c_master_dev_handle_t h = get_or_add(dev_addr8 >> 1);
    if (!h) return ESP_ERR_NOT_FOUND;

    /* 2-byte register address + payload — static buffer avoids VLA */
    uint8_t buf[258];
    if (count > 256) return ESP_ERR_INVALID_ARG;
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] =  reg       & 0xFF;
    memcpy(buf + 2, data, count);
    return i2c_master_transmit(h, buf, 2 + count, pdMS_TO_TICKS(100));
}

esp_err_t i2c_read_multi(uint8_t dev_addr8, uint16_t reg,
                         uint8_t *data, uint32_t count)
{
    if (!s_bus) return ESP_ERR_INVALID_STATE;
    i2c_master_dev_handle_t h = get_or_add(dev_addr8 >> 1);
    if (!h) return ESP_ERR_NOT_FOUND;

    uint8_t reg_buf[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
    return i2c_master_transmit_receive(h, reg_buf, 2,
                                       data, count,
                                       pdMS_TO_TICKS(100));
}

void i2c_scan(void)
{
    if (!s_bus) {
        ESP_LOGI(TAG, "i2c scan: no bus initialised");
        return;
    }
    ESP_LOGI(TAG, "i2c scan: probing 0x08–0x77 ...");
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, pdMS_TO_TICKS(20)) == ESP_OK) {
            ESP_LOGI(TAG, "  found 0x%02X (8-bit: 0x%02X)", addr, addr << 1);
            found++;
        }
    }
    ESP_LOGI(TAG, "i2c scan: %d device(s) found", found);
}
