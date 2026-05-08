/**
 * vl53l1_platform.c
 *
 * Platform-specific implementation for ST VL53L1X on ESP-IDF 5.x.
 * I2C now uses the driver_ng (i2c_master) API via i2c_platform_esp.h.
 *
 * Original (c) 2021 by David Asher — MIT license, see LICENSE.txt.
 * ST VL53L1X portions — BSD 3-clause license.
 */

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "VL53L1X_api.h"
#include "i2c_platform_esp.h"
#include "esp_log.h"

/* Resets the I2C bus and evicts the stale 0x29 device handle after a sensor
 * timeout so the next slot starts with a clean bus state. */
extern void i2c_bus_reset(void);
static const char *TAG_PLAT = "vl53l1_plat";

static const uint8_t status_rtn[24] = {
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
    255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
    255, 255, 11, 12
};

VL53L1X_ERROR esp_to_vl53l1x_error(esp_err_t esp_code)
{
    switch (esp_code) {
    case ESP_OK:                   return VL53L1_ERROR_NONE;
    case ESP_FAIL:                 return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_NO_MEM:           return VL53L1_ERROR_BUFFER_TOO_SMALL;
    case ESP_ERR_INVALID_ARG:      return VL53L1_ERROR_INVALID_PARAMS;
    case ESP_ERR_INVALID_STATE:    return VL53L1_ERROR_INVALID_COMMAND;
    case ESP_ERR_INVALID_SIZE:     return VL53L1_ERROR_INVALID_COMMAND;
    case ESP_ERR_NOT_FOUND:        return VL53L1_ERROR_NOT_SUPPORTED;
    case ESP_ERR_NOT_SUPPORTED:    return VL53L1_ERROR_NOT_SUPPORTED;
    case ESP_ERR_TIMEOUT:          return VL53L1_ERROR_TIME_OUT;
    default:                       return VL53L1_ERROR_UNDEFINED;
    }
}

VL53L1X_ERROR VL53L1_WriteMulti(uint16_t dev, uint16_t index,
                                 uint8_t *pdata, uint32_t count)
{
    return esp_to_vl53l1x_error(i2c_write_multi((uint8_t)dev, index, pdata, count));
}

VL53L1X_ERROR VL53L1_ReadMulti(uint16_t dev, uint16_t index,
                                uint8_t *pdata, uint32_t count)
{
    return esp_to_vl53l1x_error(i2c_read_multi((uint8_t)dev, index, pdata, count));
}

VL53L1X_ERROR VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

VL53L1X_ERROR VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t buf[2] = {data >> 8, data & 0xFF};
    return VL53L1_WriteMulti(dev, index, buf, 2);
}

VL53L1X_ERROR VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t buf[4] = {
        (data >> 24) & 0xFF, (data >> 16) & 0xFF,
        (data >>  8) & 0xFF,  data        & 0xFF
    };
    return VL53L1_WriteMulti(dev, index, buf, 4);
}

VL53L1X_ERROR VL53L1_UpdateByte(uint16_t dev, uint16_t index,
                                 uint8_t AndData, uint8_t OrData)
{
    uint8_t buf = 0;
    VL53L1X_ERROR status = VL53L1_ReadMulti(dev, index, &buf, 1);
    if (status) return status;
    buf = (buf & AndData) | OrData;
    return VL53L1_WriteMulti(dev, index, &buf, 1);
}

VL53L1X_ERROR VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
    return VL53L1_ReadMulti(dev, index, data, 1) ? -1 : 0;
}

VL53L1X_ERROR VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
    uint8_t buf[2] = {0, 0};
    VL53L1X_ERROR status = VL53L1_ReadMulti(dev, index, buf, 2);
    if (status) return status;
    *data = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

VL53L1X_ERROR VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
    uint8_t buf[4] = {0};
    VL53L1X_ERROR status = VL53L1_ReadMulti(dev, index, buf, 4);
    if (status) return status;
    *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
          | ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];
    return 0;
}

VL53L1X_ERROR VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{
    *ptick_count_ms = (uint32_t)esp_timer_get_time();
    return VL53L1_ERROR_NONE;
}

VL53L1X_ERROR VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = I2C_DEFAULT_FREQ;
    return VL53L1_ERROR_NONE;
}

VL53L1X_ERROR VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    (void)dev;
    vTaskDelay(pdMS_TO_TICKS(wait_ms < 1 ? 1 : wait_ms));
    return VL53L1_ERROR_NONE;
}

VL53L1X_ERROR VL53L1_WaitUs(uint16_t dev, int32_t wait_us)
{
    (void)dev;
    int32_t ms = wait_us / 1000;
    vTaskDelay(pdMS_TO_TICKS(ms < 1 ? 1 : ms));
    return VL53L1_ERROR_NONE;
}

VL53L1X_ERROR VL53L1X_SetFastI2C(uint16_t dev)
{
    return VL53L1_WrByte(dev, VL53L1_PAD_I2C_HV__CONFIG, 0x14);
}

VL53L1X_ERROR VL53L1X_SetRangingMode(uint16_t dev, uint8_t set_ranging_mode)
{
    uint8_t mode_start;
    VL53L1_RdByte(dev, VL53L1_SYSTEM__MODE_START, &mode_start);
    mode_start = (mode_start & 0x0F) | set_ranging_mode;
    return VL53L1_WrByte(dev, VL53L1_SYSTEM__MODE_START, mode_start);
}

VL53L1X_ERROR VL53L1X_SystemStatus(uint16_t dev, uint8_t *state)
{
    return VL53L1_RdByte(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, state);
}

VL53L1X_ERROR VL53L1X_GetContinuousMeasurement(uint16_t dev,
                                                uint8_t *rangeStatus,
                                                uint16_t *distanceMM)
{
    uint8_t RgSt;
    VL53L1X_ERROR status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
    *rangeStatus = (RgSt < 24) ? status_rtn[RgSt] : (RgSt & 0x1F);
    VL53L1_RdWord(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distanceMM);
    VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
    return status;
}

VL53L1X_ERROR VL53L1X_GetAndRestartMeasurement(uint16_t dev,
                                                uint8_t *rangeStatus,
                                                uint16_t *distanceMM)
{
    uint8_t RgSt;
    VL53L1X_ERROR status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
    *rangeStatus = (RgSt < 24) ? status_rtn[RgSt] : (RgSt & 0x1F);
    VL53L1_RdWord(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distanceMM);
    VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x40);
    return status;
}

VL53L1X_ERROR VL53L1X_InitSensorArray(VL53L1_DEV sensor_array, uint8_t sensor_count)
{
    uint8_t sensorState = 0;
    uint16_t timeout_check = 0;

    for (int k = 0; k < sensor_count; k++) {
        if (sensor_array[k].shutdown_pin != GPIO_NUM_NC) {
            pinMode(sensor_array[k].shutdown_pin, OUTPUT_OPEN);
            digitalWrite(sensor_array[k].shutdown_pin, LOW);
        }
        sensor_array[k].time_stamp  = esp_timer_get_time();
        sensor_array[k].cycle_time  = 0;
        sensor_array[k].range_mm    = 0;
        sensor_array[k].range_status = 0;
        sensor_array[k].range_error = VL53L1_ERROR_NONE;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int k = 0; k < sensor_count; k++) {
        /* Reset bus while sensor is still in hardware reset — clock pulses must
         * not arrive after XSHUT HIGH or they can corrupt the sensor's I2C boot. */
        i2c_bus_reset();
        if (sensor_array[k].shutdown_pin != GPIO_NUM_NC)
            digitalWrite(sensor_array[k].shutdown_pin, HIGH);
        timeout_check = sensorState = 0;
        while (sensorState == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            VL53L1X_ERROR bst = VL53L1X_BootState(VL53L1_I2C_ADDRESS, &sensorState);
            /* A failed read can write partial data into sensorState; discard it so
             * a garbage mid-boot byte cannot cause premature loop exit. */
            if (bst) sensorState = 0;
            ESP_LOGI(TAG_PLAT, "BootState attempt %d: i2c_err=%d state=%d",
                     timeout_check, bst, sensorState);
            if (++timeout_check > 10) {
                ESP_LOGW(TAG_PLAT, "sensor[%d] addr=0x%02X not found — skipping",
                         k, sensor_array[k].I2cDevAddr);
                sensor_array[k].I2cDevAddr = 0;
                i2c_bus_reset();
                break;
            }
        }
        if (sensor_array[k].I2cDevAddr == 0) continue;
        VL53L1X_SensorInit(VL53L1_I2C_ADDRESS);
        VL53L1X_SetI2CAddress(VL53L1_I2C_ADDRESS, sensor_array[k].I2cDevAddr);
        VL53L1X_SetFastI2C(sensor_array[k].I2cDevAddr);
        VL53L1X_SetDistanceMode(sensor_array[k].I2cDevAddr, sensor_array[k].distance_mode);
        if (sensor_array[k].timing_budget)
            VL53L1X_SetTimingBudgetInMs(sensor_array[k].I2cDevAddr,
                                        sensor_array[k].timing_budget);
        if (sensor_array[k].inter_measurement)
            VL53L1X_SetInterMeasurementInMs(sensor_array[k].I2cDevAddr,
                                            sensor_array[k].inter_measurement);
        VL53L1X_SetRangingMode(sensor_array[k].I2cDevAddr, RANGING_MODE_SINGLE_SHOT);
        VL53L1X_StartRanging(sensor_array[k].I2cDevAddr);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    return VL53L1_ERROR_NONE;
}
