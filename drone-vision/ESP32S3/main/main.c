/* main.c — Mach Mind Drone Vision
 *
 * Two modes, selected at build time via Kconfig → idf.py menuconfig:
 *
 *   BENCH  — multi-resolution ArUco FPS sweep, CSV output, auto-restart.
 *   POSE   — continuous ArUco detection, single-line rolling status,
 *             world pose (x/y/z) via solvePnP to USB-serial.
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

/* Pose mode needs a large stack for cv::Mat / std::vector / solvePnP. */
#define POSE_TASK_STACK_KB  32

static void pose_task(void *arg)
{
    (void)arg;
    aruco_pose_start(); /* never returns */
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

#if CONFIG_VISION_MODE_POSE
    ESP_LOGI(TAG, "Mach Mind — ESP32S3 ArUco Pose Estimator boot");
    xTaskCreatePinnedToCore(pose_task, "aruco_pose",
                            POSE_TASK_STACK_KB * 1024,
                            NULL, 5, NULL,
                            1 /* CPU1 */);
    vTaskDelete(NULL);
#else
    ESP_LOGI(TAG, "Mach Mind — ESP32S3 Vision Benchmark boot");
    vision_bench_start(); /* never returns */
#endif
}
