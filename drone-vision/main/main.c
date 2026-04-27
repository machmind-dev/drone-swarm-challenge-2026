/* main.c — Mach Mind Drone Vision Benchmark
 *
 * Minimal entry point for the ESP32S3 ArUco FPS benchmark.
 * All heavy lifting is in vision_bench.cpp.
 *
 * USB-Serial output (idf.py monitor, 115200 baud):
 *   - Per-frame detection timing and FPS while running
 *   - Final table + CSV after all resolution stages complete
 *   - Automatic restart after 10 s for repeated measurements
 *
 * Flash & monitor:
 *   docker compose run --rm esp32s3_vision idf.py -p /dev/ttyACM0 flash monitor
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "vision_bench.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Mach Mind — ESP32S3 Vision Benchmark boot");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* vision_bench_start() never returns (restarts after each run) */
    vision_bench_start();
}
