/* main.c — Mach Mind Drone Vision
 *
 * Two modes, selected at build time via Kconfig → idf.py menuconfig:
 *
 *   BENCH (default)
 *     Multi-resolution ArUco FPS sweep.  Prints timing table + CSV over
 *     USB-Serial, then restarts automatically after 10 s.
 *
 *   POSE
 *     Continuous ArUco detection using the SDC26 arena marker map.
 *     Computes per-marker distance + H/V angles and, when ≥1 known marker
 *     is visible, solves for world pose (x, y, z + quaternion).
 *     Output: USB-Serial at 115200 baud.
 *
 * Flash & monitor:
 *   docker compose exec esp32s3_vision bash
 *   idf.py -p /dev/ttyACM0 flash monitor
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "vision_bench.h"
#include "aruco_pose.h"

static const char *TAG = "main";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

#if CONFIG_VISION_MODE_POSE
    ESP_LOGI(TAG, "Mach Mind — ESP32S3 ArUco Pose Estimator boot");
    aruco_pose_start();   /* never returns */
#else
    ESP_LOGI(TAG, "Mach Mind — ESP32S3 Vision Benchmark boot");
    vision_bench_start(); /* never returns (restarts after each run) */
#endif
}
