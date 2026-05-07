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
#include "tof_task.h"

static const char *TAG = "main";

/* ── Identity ──────────────────────────────────────────────────────────── */
#define DRONE_ID  1

/* Pose task needs a large stack: cv::Mat / std::vector / solvePnP /
 * OpenCV parallel backend registry init (~5KB) + cvtColor frame init.
 * 32 KB overflows on first cvtColor call; 64 KB gives comfortable headroom. */
#define POSE_TASK_STACK_KB  64

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
     * IMPORTANT: DFS (Dynamic Frequency Scaling) is DISABLED by locking
     * min_freq == max_freq.  DFS suspends L2 cache during frequency
     * transitions; on ESP32-P4 rev 1.0 this triggers an L1→L2 auto-writeback
     * stuck issue (fixed in IDF 5.5 for later silicon revisions).  Symptom:
     * dirty cache lines containing the PSRAM heap multi_heap_t spinlock are
     * not written back to PSRAM before DMA cache-line invalidation; the next
     * CPU read of the spinlock returns stale/garbage → spinlock_acquire(0x17)
     * load-access-fault.
     * Running at a fixed 360 MHz avoids the DFS-induced L2 suspension entirely.
     * Power budget: ~60 mA extra vs DFS; acceptable for a tethered board.   */
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz       = 360,
        .min_freq_mhz       = 360,   /* no DFS: min == max */
        .light_sleep_enable = false,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));
    ESP_LOGI(TAG, "PM: fixed 360 MHz (DFS disabled — avoids L2 cache writeback errata)");
#endif

    ESP_LOGI(TAG, "Mach Mind — ESP32-P4 ArUco Pose Estimator boot");

    tof_task_start();

    xTaskCreatePinnedToCore(pose_task, "aruco_pose",
                            POSE_TASK_STACK_KB * 1024,
                            NULL, 5, NULL,
                            0 /* CPU0 — ISP ISR also on CPU0; same-core critical sections disable IRQs, no spinlock deadlock */);
    vTaskDelete(NULL);
}
