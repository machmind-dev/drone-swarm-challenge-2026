/* main.c — Mach Mind Drone Vision (ESP32-P4 + OV5647)
 *
 * POSE mode: continuous ArUco detection → world-pose output to USB-Serial.
 * Bench mode is not yet ported to the esp_video / V4L2 API.
 *
 * First-time build setup (inside Docker container):
 *   idf.py set-target esp32p4          ← generates sdkconfig for P4 target
 *   idf.py menuconfig                  ← optional: adjust resolution / marker size
 *   idf.py build
 *   idf.py -p /dev/ttyACM0 flash monitor
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#include "aruco_pose.h"

static const char *TAG = "main";

/* Pose task needs a large stack: cv::Mat / std::vector / solvePnP */
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

    /* ── Power management ────────────────────────────────────────────────────
     * Allow CPU to scale between 40 MHz (FreeRTOS idle / light sleep) and
     * 360 MHz (active ArUco detection).  vTaskDelay() calls become real
     * light-sleep windows, saving ~100–150 mA vs always-on 400 MHz.        */
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz       = 360,
        .min_freq_mhz       = 40,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true,
#else
        .light_sleep_enable = false,
#endif
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));
    ESP_LOGI(TAG, "PM enabled: 40–360 MHz DFS + light sleep");
#endif

    ESP_LOGI(TAG, "Mach Mind — ESP32-P4 ArUco Pose Estimator boot");

    xTaskCreatePinnedToCore(pose_task, "aruco_pose",
                            POSE_TASK_STACK_KB * 1024,
                            NULL, 5, NULL,
                            1 /* CPU1 */);
    vTaskDelete(NULL);
}
