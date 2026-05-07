/* tof_task.c — VL53L1X ToF sensor task for ESP32-P4 Navigation Module
 *
 * Hardware (Mach Mind Sensors Board rev 07/2026):
 *   Shared I2C bus — SDA GPIO7, SCL GPIO8  (same physical lines as camera SCCB;
 *   camera SCCB is only active during init, bus is idle afterwards)
 *
 * Sensor slots (6 total on final PCB, 1 active for mock-up testing):
 *   Slot 0  XSHUT GPIO19  addr 0x54  ← wired on dev bench
 *   Slot 1  XSHUT GPIO??  addr 0x56  (uncomment when PCB arrives)
 *   Slot 2  XSHUT GPIO??  addr 0x58
 *   Slot 3  XSHUT GPIO??  addr 0x5A
 *   Slot 4  XSHUT GPIO??  addr 0x5C
 *   Slot 5  XSHUT GPIO??  addr 0x5E
 *
 * Each sensor is assigned a unique I2C address at boot via XSHUT sequencing
 * (VL53L1X_InitSensorArray brings sensors online one at a time).
 *
 * I2C port: I2C_NUM_1 (legacy driver) — separate from esp_video SCCB which
 * uses the new i2c_new_master_bus() API on I2C_NUM_0.
 * Poll rate: ~20 Hz (50 ms interval; VL53L1X LONG mode needs ~33 ms/measurement)
 */

#include "tof_task.h"
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
#include "i2c_platform_esp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "tof";

/* ── I2C bus config ─────────────────────────────────────────────────────── */
#define TOF_SDA_PIN     GPIO_NUM_7
#define TOF_SCL_PIN     GPIO_NUM_8
#define TOF_I2C_PORT    I2C_NUM_1
#define TOF_I2C_FREQ    400000

/* ── Sensor array — add entries as sensors are wired ────────────────────── */
static VL53L1_Dev_t s_sensors[] = {
    /* Slot 0 — mock-up test sensor */
    { .I2cDevAddr  = VL53L1_I2C_ADDRESS + 2,   /* 0x54 */
      .shutdown_pin = GPIO_NUM_19,
      .distance_mode = DISTANCE_MODE_LONG },

    /* Slot 1-5 — uncomment and set XSHUT GPIO when PCB arrives
    { .I2cDevAddr  = VL53L1_I2C_ADDRESS + 4,   // 0x56
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG },
    { .I2cDevAddr  = VL53L1_I2C_ADDRESS + 6,   // 0x58
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG },
    { .I2cDevAddr  = VL53L1_I2C_ADDRESS + 8,   // 0x5A
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG },
    { .I2cDevAddr  = VL53L1_I2C_ADDRESS + 10,  // 0x5C
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG },
    { .I2cDevAddr  = VL53L1_I2C_ADDRESS + 12,  // 0x5E
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG },
    */
};

static const uint8_t SENSOR_COUNT = sizeof(s_sensors) / sizeof(s_sensors[0]);

/* ── Latest readings (written by tof_task, read by aruco task) ──────────── */
static volatile uint16_t s_dist_mm[6]     = {0};
static volatile uint8_t  s_range_status[6] = {255, 255, 255, 255, 255, 255};

uint16_t tof_get_distance_mm(uint8_t idx)  { return (idx < SENSOR_COUNT) ? s_dist_mm[idx]     : 0;   }
uint8_t  tof_get_range_status(uint8_t idx) { return (idx < SENSOR_COUNT) ? s_range_status[idx] : 255; }
uint8_t  tof_sensor_count(void)            { return SENSOR_COUNT; }

/* ── Task ───────────────────────────────────────────────────────────────── */
static void tof_task(void *arg)
{
    (void)arg;

    i2c_init_config(TOF_I2C_PORT, TOF_SDA_PIN, TOF_SCL_PIN, TOF_I2C_FREQ);
    ESP_LOGI(TAG, "I2C_NUM_%d  SDA=GPIO%d  SCL=GPIO%d  %d Hz",
             (int)TOF_I2C_PORT, (int)TOF_SDA_PIN, (int)TOF_SCL_PIN, TOF_I2C_FREQ);

    VL53L1X_ERROR err = VL53L1X_InitSensorArray(s_sensors, SENSOR_COUNT);
    if (err != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "VL53L1X_InitSensorArray failed (%d) — check wiring", err);
        vTaskDelete(NULL);
        return;
    }

    for (int k = 0; k < SENSOR_COUNT; k++) {
        VL53L1X_StartRanging(s_sensors[k].I2cDevAddr);
        ESP_LOGI(TAG, "sensor[%d] addr=0x%02X XSHUT=GPIO%d ready",
                 k, s_sensors[k].I2cDevAddr, (int)s_sensors[k].shutdown_pin);
    }

    while (1) {
        for (int k = 0; k < SENSOR_COUNT; k++) {
            VL53L1_Dev_t *t = &s_sensors[k];
            t->range_error = VL53L1X_GetAndRestartMeasurement(
                t->I2cDevAddr, &t->range_status, &t->range_mm);

            if (t->range_error == VL53L1_ERROR_NONE) {
                s_dist_mm[k]     = t->range_mm;
                s_range_status[k] = t->range_status;
            }
        }
        /* VL53L1X LONG mode measurement time ~33 ms; poll at 50 ms → ~20 Hz */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void tof_task_start(void)
{
    xTaskCreatePinnedToCore(tof_task, "tof", 4096, NULL, 4, NULL,
                            1 /* CPU1 — aruco/camera on CPU0 */);
}
