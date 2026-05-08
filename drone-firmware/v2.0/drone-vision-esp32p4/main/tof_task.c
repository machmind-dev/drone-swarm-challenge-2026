/* tof_task.c — VL53L1X ToF sensor task for ESP32-P4 Navigation Module
 *
 * Hardware (breadboard mock-up → Mach Mind Sensors Board rev 07/2026):
 *   I2C bus — dedicated I2C_NUM_0 on GPIO5 (SDA) / GPIO6 (SCL).
 *   Separate from camera SCCB (I2C_NUM_1, GPIO7/8) so there is no bus
 *   sharing, no borrowing, and no hardware state pollution from SCCB failures.
 *   Pull-ups: provided by the VL53L1X breakout board (4.7 kΩ to 3.3 V).
 *
 *   Breadboard cluster: GPIO4 (XSHUT), GPIO5 (SDA), GPIO6 (SCL) are adjacent.
 *
 * Sensor slots (6 total on final PCB, 1 active for mock-up testing):
 *   Slot 0  XSHUT GPIO4  addr 0x54
 *   Slot 1–5  XSHUT GPIO??  addr 0x56–0x5E  (uncomment when PCB arrives)
 *
 * Poll rate: ~20 Hz (50 ms; VL53L1X LONG mode needs ~33 ms/measurement)
 */

#include "tof_task.h"
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
#include "i2c_platform_esp.h"
#include "boards.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "tof";

/* ── I2C bus config — pin constants come from boards.h ──────────────────── */
#define TOF_SDA_PIN     TOF_I2C_SDA     /* GPIO5 — dedicated I2C_NUM_0 bus   */
#define TOF_SCL_PIN     TOF_I2C_SCL     /* GPIO6 — dedicated I2C_NUM_0 bus   */
#define TOF_I2C_PORT    I2C_NUM_0
#define TOF_I2C_FREQ    400000

/* ── Sensor array — add entries as sensors are wired ────────────────────── */
static VL53L1_Dev_t s_sensors[] = {
    /* Slot 0 — mock-up test sensor
     * timing_budget: 33 ms (minimum for LONG mode)
     * inter_measurement: 40 ms (period between measurements) */
    { .I2cDevAddr      = VL53L1_I2C_ADDRESS + 2,   /* 0x54 */
      .shutdown_pin    = TOF_XSHUT_PIN,             /* GPIO4 — open-drain XSHUT              */
      .distance_mode   = DISTANCE_MODE_LONG,
      .timing_budget   = 33,
      .inter_measurement = 40 },

    /* Slot 1-5 — uncomment and set XSHUT GPIO when PCB arrives
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 4,  // 0x56
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG, .timing_budget = 33, .inter_measurement = 40 },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 6,  // 0x58
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG, .timing_budget = 33, .inter_measurement = 40 },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 8,  // 0x5A
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG, .timing_budget = 33, .inter_measurement = 40 },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 10, // 0x5C
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG, .timing_budget = 33, .inter_measurement = 40 },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 12, // 0x5E
      .shutdown_pin = GPIO_NUM_??,
      .distance_mode = DISTANCE_MODE_LONG, .timing_budget = 33, .inter_measurement = 40 },
    */
};

static const uint8_t SENSOR_COUNT = sizeof(s_sensors) / sizeof(s_sensors[0]);

/* ── Latest readings (written by tof_task, read by aruco task) ──────────── */
static volatile uint16_t s_dist_mm[6]      = {0};
static volatile uint8_t  s_range_status[6] = {255, 255, 255, 255, 255, 255};

uint16_t tof_get_distance_mm(uint8_t idx)  { return (idx < SENSOR_COUNT) ? s_dist_mm[idx]      : 0;   }
uint8_t  tof_get_range_status(uint8_t idx) { return (idx < SENSOR_COUNT) ? s_range_status[idx] : 255; }
uint8_t  tof_sensor_count(void)            { return SENSOR_COUNT; }

/* ── Task ───────────────────────────────────────────────────────────────── */
static void tof_task(void *arg)
{
    (void)arg;

    i2c_init_config(TOF_I2C_PORT, TOF_SDA_PIN, TOF_SCL_PIN, TOF_I2C_FREQ);
    ESP_LOGI(TAG, "I2C_NUM_%d  SDA=GPIO%d  SCL=GPIO%d  %d Hz",
             (int)TOF_I2C_PORT, (int)TOF_SDA_PIN, (int)TOF_SCL_PIN, TOF_I2C_FREQ);

    /* Pulse XSHUT before scanning: if the sensor held SDA low from the
     * previous boot's incomplete transaction, this hard-resets it and frees
     * the bus.  Without this, the IDF bus-reset fails and every probe times
     * out instead of NACKing.  InitSensorArray repeats this sequence for
     * address assignment, so the sensor ends up correctly initialised. */
    for (int k = 0; k < SENSOR_COUNT; k++) {
        gpio_num_t pin = s_sensors[k].shutdown_pin;
        if (pin == GPIO_NUM_NC) continue;
        gpio_config_t xshut_cfg = {
            .pin_bit_mask       = 1ULL << pin,
            .mode               = GPIO_MODE_OUTPUT_OD,
            .pull_up_en         = GPIO_PULLUP_DISABLE,
            .pull_down_en       = GPIO_PULLDOWN_DISABLE,
            .intr_type          = GPIO_INTR_DISABLE,
        };
        gpio_config(&xshut_cfg);
        gpio_set_level(pin, 0);
        ESP_LOGI(TAG, "XSHUT GPIO%d → LOW (sensor hard-reset)", (int)pin);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    for (int k = 0; k < SENSOR_COUNT; k++) {
        gpio_num_t pin = s_sensors[k].shutdown_pin;
        if (pin == GPIO_NUM_NC) continue;
        gpio_set_level(pin, 1);
        ESP_LOGI(TAG, "XSHUT GPIO%d → HIGH (sensor booting)", (int)pin);
    }
    vTaskDelay(pdMS_TO_TICKS(2));   /* VL53L1X needs ≥1.2 ms after XSHUT HIGH */

    i2c_scan();

    ESP_LOGI(TAG, "=== VL53L1X_InitSensorArray START ===");
    VL53L1X_ERROR err = VL53L1X_InitSensorArray(s_sensors, SENSOR_COUNT);
    if (err != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "=== VL53L1X_InitSensorArray FAILED err=%d — check XSHUT wiring and I2C bus ===", err);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "=== VL53L1X_InitSensorArray OK ===");

    for (int k = 0; k < SENSOR_COUNT; k++) {
        ESP_LOGI(TAG, "sensor[%d] addr=0x%02X XSHUT=GPIO%d ready",
                 k, s_sensors[k].I2cDevAddr, (int)s_sensors[k].shutdown_pin);
    }

    while (1) {
        for (int k = 0; k < SENSOR_COUNT; k++) {
            VL53L1_Dev_t *t = &s_sensors[k];
            t->range_error = VL53L1X_GetAndRestartMeasurement(
                t->I2cDevAddr, &t->range_status, &t->range_mm);

            if (t->range_error == VL53L1_ERROR_NONE) {
                s_dist_mm[k]      = t->range_mm;
                s_range_status[k] = t->range_status;
            }
        }
        /* VL53L1X LONG mode measurement time ~33 ms; poll at 50 ms -> ~20 Hz */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void tof_task_start(void)
{
    xTaskCreatePinnedToCore(tof_task, "tof", 4096, NULL, 4, NULL,
                            1 /* CPU1 — aruco/camera on CPU0 */);
}
