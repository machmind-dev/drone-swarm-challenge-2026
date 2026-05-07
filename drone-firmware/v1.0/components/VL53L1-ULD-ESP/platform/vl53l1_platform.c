/**
 * vl53l1_platform.c
 * 
 * Platform-specific interface functions for the ST VL53L1 family of
 * time-of-flight laser ranging sensors, specific implementation for the
 * Espressif Internet-of-Things (IoT) Development Framework ESP-IDF
 *
 * (c) 2021 by David Asher
 * https://github.com/david-asher
 * https://www.linkedin.com/in/davidasher/
 * This code is licensed under MIT license, see LICENSE.txt for details
 *
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
 * 
 * License terms: BSD 3-clause "New" or "Revised" License. 
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this 
 * list of conditions and the following disclaimer. 
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution. 
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 * may be used to endorse or promote products derived from this software 
 * without specific prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 * 
 */


#include "esp_timer.h"
#include "VL53L1X_api.h"

static const uint8_t status_rtn[24] = { 
	255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
	255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
	255, 255, 11, 12
};

/**
 * vl53l1 platform-specific implementation, I2C read-write device functions
 */

VL53L1X_ERROR esp_to_vl53l1x_error( esp_err_t esp_code )
{
    switch( esp_code ) {
    case ESP_OK:                    return VL53L1_ERROR_NONE;
    case ESP_FAIL:                  return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_NO_MEM:            return VL53L1_ERROR_BUFFER_TOO_SMALL;
    case ESP_ERR_INVALID_ARG:       return VL53L1_ERROR_INVALID_PARAMS;
    case ESP_ERR_INVALID_STATE:     return VL53L1_ERROR_INVALID_COMMAND;
    case ESP_ERR_INVALID_SIZE:      return VL53L1_ERROR_INVALID_COMMAND;
    case ESP_ERR_NOT_FOUND:         return VL53L1_ERROR_NOT_SUPPORTED;
    case ESP_ERR_NOT_SUPPORTED:     return VL53L1_ERROR_NOT_SUPPORTED;
    case ESP_ERR_TIMEOUT:           return VL53L1_ERROR_TIME_OUT;
    case ESP_ERR_INVALID_RESPONSE:  return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_INVALID_CRC:       return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_INVALID_VERSION:   return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_INVALID_MAC:       return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_WIFI_BASE:         return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_MESH_BASE:         return VL53L1_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_FLASH_BASE:        return VL53L1_ERROR_CONTROL_INTERFACE;
    default:                        return VL53L1_ERROR_UNDEFINED;
    }
}

VL53L1X_ERROR VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) 
{
    i2c_start_write( dev );
    i2c_write_byte( index >> 8 );
    i2c_write_byte( index & 0xFF );
    i2c_write( pdata, count );
    esp_err_t esp_code = i2c_transmit();
    return esp_to_vl53l1x_error( esp_code );
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L1X_ERROR VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) 
{
    i2c_start_write( dev );
    i2c_write_byte( index >> 8 );
    i2c_write_byte( index & 0xFF );
    i2c_start_read( dev );
    i2c_read( pdata, count );
    esp_err_t esp_code = i2c_transmit();
    return esp_to_vl53l1x_error( esp_code );
}

VL53L1X_ERROR VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) 
{
    int  status;
    uint8_t write_data = data;
    status = VL53L1_WriteMulti(dev, index, &write_data, 1);
    return status;
}

VL53L1X_ERROR VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) 
{
    int  status;
    uint8_t buffer[2];
    buffer[0] = data >> 8;
    buffer[1] = data & 0x00FF;
    status = VL53L1_WriteMulti(dev, index, (uint8_t *)buffer, 2);
    return status;
}

VL53L1X_ERROR VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) 
{
    int  status;
    uint8_t buffer[4];
    buffer[0] = (data >> 24) & 0xFF;
    buffer[1] = (data >> 16) & 0xFF;
    buffer[2] = (data >>  8) & 0xFF;
    buffer[3] = (data >>  0) & 0xFF;
    status = VL53L1_WriteMulti(dev, index, (uint8_t *)buffer, 4);
    return status;
}

VL53L1X_ERROR VL53L1_UpdateByte(uint16_t dev, uint16_t index, uint8_t AndData, uint8_t OrData) 
{
    int  status;
    uint8_t buffer = 0;

    /* read data direct onto buffer */
    status = VL53L1_ReadMulti(dev, index, &buffer, 1);
    if (status) return status;
    buffer = (buffer & AndData) | OrData;
    status = VL53L1_WriteMulti(dev, index, &buffer, (uint16_t)1);
    return status;
}

VL53L1X_ERROR VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) 
{
    int  status;
    status = VL53L1_ReadMulti(dev, index, data, 1);
    return status ? -1 : 0;
}

VL53L1X_ERROR VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) 
{
    int  status;
    uint8_t buffer[2] = {0, 0};
    status = VL53L1_ReadMulti(dev, index, buffer, 2);
    if ( status ) return status;
    *data = (buffer[0] << 8) + buffer[1];
    return status;
}

VL53L1X_ERROR VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) 
{
    int status;
    uint8_t buffer[4] = {0, 0, 0, 0};
    status = VL53L1_ReadMulti(dev, index, buffer, 4);
    if ( status ) return status;
    *data = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
    return status;
}

/**
 * vl53l1 platform-specific implementation, O/S timing functions
 */

VL53L1X_ERROR VL53L1_GetTickCount( uint32_t *ptick_count_ms )
{
    // note: _ms means microseconds, not milliseconds
    *ptick_count_ms = esp_timer_get_time();
	return VL53L1_ERROR_NONE;
}

VL53L1X_ERROR VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = I2C_DEFAULT_FREQ;
	return VL53L1_ERROR_NONE;
}

VL53L1X_ERROR VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    vTaskDelay( MAX( 1, wait_ms / portTICK_PERIOD_MS ) );
	return VL53L1_ERROR_NONE;
}

VL53L1X_ERROR VL53L1_WaitUs(uint16_t dev, int32_t wait_us)
{
    vTaskDelay( MAX( 1, wait_us / 1000 / portTICK_PERIOD_MS ) );
	return VL53L1_ERROR_NONE;
}

// other useful device functions

VL53L1X_ERROR VL53L1X_SetFastI2C(uint16_t dev)
{
    return VL53L1_WrByte(dev, VL53L1_PAD_I2C_HV__CONFIG, 0x14 );
}

/**
 * 	fields: \n
 *		- [1:0] = scheduler_mode
 *		- [3:2] = readout_mode
 *		-   [4] = mode_range__single_shot
 *		-   [5] = mode_range__back_to_back
 *		-   [6] = mode_range__timed
 *		-   [7] = mode_range__abort 
 */
VL53L1X_ERROR VL53L1X_SetRangingMode(uint16_t dev, uint8_t set_ranging_mode)
{
    uint8_t mode_start;
    VL53L1_RdByte(dev, VL53L1_SYSTEM__MODE_START, &mode_start );
    mode_start = ( mode_start & 0x0F ) | set_ranging_mode;
    return VL53L1_WrByte(dev, VL53L1_SYSTEM__MODE_START, mode_start );
}

VL53L1X_ERROR VL53L1X_SystemStatus(uint16_t dev, uint8_t *state)
{
	return VL53L1_RdByte(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, state);
}

char *VL53L1X_SystemStatusString(uint16_t dev)
{
    uint8_t system_status;
	VL53L1_RdByte(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, &system_status);
    switch( system_status ) {
    case VL53L1_STATE_POWERDOWN:        return "VL53L1_STATE_POWERDOWN";
    case VL53L1_STATE_WAIT_STATICINIT:  return "VL53L1_STATE_WAIT_STATICINIT";
    case VL53L1_STATE_STANDBY:          return "VL53L1_STATE_STANDBY";
    case VL53L1_STATE_IDLE:             return "VL53L1_STATE_IDLE";
    case VL53L1_STATE_RUNNING:          return "VL53L1_STATE_RUNNING";
    case VL53L1_STATE_RESET:            return "VL53L1_STATE_RESET";
    case VL53L1_STATE_UNKNOWN:          return "VL53L1_STATE_UNKNOWN";
    case VL53L1_STATE_ERROR:            return "VL53L1_STATE_ERROR";
    default:                            return "VL53L1_STATE_UNKNOWN";
    }
}

VL53L1X_ERROR VL53L1X_GetContinuousMeasurement(uint16_t dev, uint8_t *rangeStatus, uint16_t *distanceMM)
{
	VL53L1X_ERROR status;

	// VL53L1X_GetRangeStatus(dev, &RangeStatus)
	uint8_t RgSt;
	status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
	*rangeStatus = (RgSt < 24) ? status_rtn[RgSt] : RgSt & 0x1F;

	//	VL53L1X_GetDistance(dev, &Distance)
	VL53L1_RdWord(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distanceMM);

    //  VL53L1X_ClearInterrupt(dev)
	VL53L1_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);

	return status;
}

VL53L1X_ERROR VL53L1X_GetAndRestartMeasurement(uint16_t dev, uint8_t *rangeStatus, uint16_t *distanceMM)
{
	VL53L1X_ERROR status;

	// VL53L1X_GetRangeStatus(dev, &RangeStatus)
	uint8_t RgSt;
	status = VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &RgSt);
	*rangeStatus = (RgSt < 24) ? status_rtn[RgSt] : RgSt & 0x1F;

	//	VL53L1X_GetDistance(dev, &Distance)
	VL53L1_RdWord(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distanceMM);

    //  VL53L1X_StartRanging(dev)
    VL53L1_WrByte(dev, SYSTEM__MODE_START, 0x40);

	return status;
}

VL53L1X_ERROR VL53L1X_InitSensorArray(VL53L1_DEV sensor_array, uint8_t sensor_count)
{
    uint8_t sensorState = 0;
    uint16_t timeout_check = 0;
    int k;

    // shut off all sensors and initialize sensor structures
    for ( k = 0; k < sensor_count; k++ ) {
        pinMode( sensor_array[k].shutdown_pin, OUTPUT_OPEN );
        digitalWrite( sensor_array[k].shutdown_pin, LOW );
        sensor_array[k].time_stamp = esp_timer_get_time();
        sensor_array[k].cycle_time = 0;
        sensor_array[k].range_mm = 0;
        sensor_array[k].range_status = 0;
        sensor_array[k].range_error = VL53L1_ERROR_NONE;
    }
    vTaskDelay( 100 / portTICK_PERIOD_MS );

    for ( k = 0; k < sensor_count; k++ ) {

        // enable this device and wait for it to boot up
        digitalWrite( sensor_array[k].shutdown_pin, HIGH );
        timeout_check = sensorState = 0;
        while ( sensorState == 0 ) {
            vTaskDelay( 20 / portTICK_PERIOD_MS );
            VL53L1X_BootState( VL53L1_I2C_ADDRESS, &sensorState );
            if ( ++timeout_check > 10 ) return VL53L1_ERROR_TIME_OUT;
        }

        // initialize the device
        VL53L1X_SensorInit( VL53L1_I2C_ADDRESS );

        // change it's I2C address and use that new I2C address from now on
        VL53L1X_SetI2CAddress( VL53L1_I2C_ADDRESS, sensor_array[k].I2cDevAddr );

        // configure the device
        VL53L1X_SetFastI2C( sensor_array[k].I2cDevAddr );
        VL53L1X_SetDistanceMode( sensor_array[k].I2cDevAddr, sensor_array[k].distance_mode );
        VL53L1X_SetTimingBudgetInMs( sensor_array[k].I2cDevAddr, sensor_array[k].timing_budget );       
        VL53L1X_SetInterMeasurementInMs( sensor_array[k].I2cDevAddr, sensor_array[k].inter_measurement ); 
        VL53L1X_SetRangingMode( sensor_array[k].I2cDevAddr, RANGING_MODE_SINGLE_SHOT );

        // kick off measurement cycle
        VL53L1X_StartRanging( sensor_array[k].I2cDevAddr );   
    }
    vTaskDelay( 100 / portTICK_PERIOD_MS );

	return VL53L1_ERROR_NONE;
}
