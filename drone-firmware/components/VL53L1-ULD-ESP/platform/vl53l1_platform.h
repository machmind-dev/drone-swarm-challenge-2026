/**
 * vl53l1_platform.h
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
 * @file  vl53l1_platform.h
 * @brief Platform-specific interface functions for the ST VL53L1 ToF ranging sensors on ESP-IDF
 */
 
#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "VL53L1X_error_codes.h"
#include "VL53L1X_register_map.h"
#include "i2c_platform_esp.h"
#include "gpio_platform_esp.h"

#define VL53L1_I2C_ADDRESS          (0x52)      // default I2C address for VL53L1

#define	DISTANCE_MODE_SHORT			(1)			// up to 1 meter
#define	DISTANCE_MODE_LONG			(2)			// up to 4 meters

#define RANGING_MODE_SINGLE_SHOT	(0x10)
#define RANGING_MODE_BACK_TO_BACK	(0x20)
#define RANGING_MODE_TIMED			(0x40)
#define RANGING_MODE_ABORT			(0x80)

/** @defgroup VL53L1_define_RangeStatus_group Defines the Range Status
 *	@{
 */
#define	 VL53L1_RANGESTATUS_RANGE_VALID				0
/*!<The Range is valid. */
#define	 VL53L1_RANGESTATUS_SIGMA_FAIL				1
/*!<Sigma Fail. */
#define	 VL53L1_RANGESTATUS_SIGNAL_FAIL				2
/*!<Signal fail. */
#define	 VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED	3
/*!<Target is below minimum detection threshold. */
#define	 VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL			4
/*!<Phase out of valid limits -  different to a wrap exit. */
#define	 VL53L1_RANGESTATUS_HARDWARE_FAIL			5
/*!<Hardware fail. */
#define	 VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL	6
/*!<The Range is valid but the wraparound check has not been done. */
#define	VL53L1_RANGESTATUS_WRAP_TARGET_FAIL			7
/*!<Wrapped target - no matching phase in other VCSEL period timing. */
#define	VL53L1_RANGESTATUS_PROCESSING_FAIL			8
/*!<Internal algo underflow or overflow in lite ranging. */
#define	VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL			9
/*!<Specific to lite ranging. */
#define	VL53L1_RANGESTATUS_SYNCRONISATION_INT			10
/*!<1st interrupt when starting ranging in back to back mode. Ignore data. */
#define	VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE		11
/*!<All Range ok but object is result of multiple pulses merging together.
 * Used by RQL for merged pulse detection
 */
#define	VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL	12
/*!<Used  by RQL  as different to phase fail. */
#define	VL53L1_RANGESTATUS_MIN_RANGE_FAIL			13
/*!<User ROI input is not valid e.g. beyond SPAD Array.*/
#define	VL53L1_RANGESTATUS_RANGE_INVALID			14
/*!<lld returned valid range but negative value ! */
#define	 VL53L1_RANGESTATUS_NONE				255
/*!<No Update. */

typedef int8_t VL53L1X_ERROR;

typedef struct {

	// configuration:
	uint8_t     	I2cDevAddr;
	gpio_num_t  	shutdown_pin;
	gpio_num_t  	interrupt_pin;
	uint8_t			distance_mode;
	uint8_t			timing_budget;
	uint8_t			inter_measurement;
	char *			name;

	// measurement:
	int64_t     	time_stamp;
	uint16_t     	cycle_time;
	uint16_t    	range_mm;
	uint8_t     	range_status;
	VL53L1X_ERROR 	range_error;

} VL53L1_Dev_t;

typedef VL53L1_Dev_t *VL53L1_DEV;

#ifndef MIN
  #define MIN(v1, v2) ((v1) < (v2) ? (v1) : (v2))
#endif

#ifndef MAX
  #define MAX(v1, v2) ((v1) < (v2) ? (v2) : (v1))
#endif

#define VL53L1_STATE_POWERDOWN       (0)	// Device is in HW reset
#define VL53L1_STATE_WAIT_STATICINIT (1)	// Device is initialized and wait for static initialization
#define VL53L1_STATE_STANDBY         (2)	// Device is in Low power Standby mode
#define VL53L1_STATE_IDLE            (3)	// Device has been initialized and ready to do measurements
#define VL53L1_STATE_RUNNING         (4)	// Device is performing measurement
#define VL53L1_STATE_RESET           (5)	// Soft reset has been run on Device
#define VL53L1_STATE_UNKNOWN         (98)	// Device is in unknown state and need to be rebooted
#define VL53L1_STATE_ERROR           (99)	// Device is in error state and need to be rebooted

#define INTERRUPT_ACTIVE_LOW		 (0)	// VL53L1X_SetInterruptPolarity
#define INTERRUPT_ACTIVE_HIGH		 (1)	// default state

#define VL53_ERROR_CHECK(x) do {                                    \
			VL53L1X_ERROR __err_rc = (x);                            \
			if (__err_rc != VL53L1X_ERROR_NONE) {                    \
				printf( "\r\nVL53L1 ERROR: code=%d, file=%s, line=%d, function=%s, expression=%s", __err_rc, __FILE__, __LINE__, __ASSERT_FUNC, #x); \
			}                                                       \
	    } while(0)

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WriteMulti(
		uint16_t 			dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_ReadMulti(
		uint16_t 			dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief VL53L1_WrByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrByte(
		uint16_t dev,
		uint16_t      index,
		uint8_t       data);
/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrWord(
		uint16_t dev,
		uint16_t      index,
		uint16_t      data);
/** @brief VL53L1_WrDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WrDWord(
		uint16_t dev,
		uint16_t      index,
		uint32_t      data);
/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdByte(
		uint16_t dev,
		uint16_t      index,
		uint8_t      *pdata);
/** @brief VL53L1_RdWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdWord(
		uint16_t dev,
		uint16_t      index,
		uint16_t     *pdata);
/** @brief VL53L1_RdDWord() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_RdDWord(
		uint16_t dev,
		uint16_t      index,
		uint32_t     *pdata);
/** @brief VL53L1_WaitMs() definition.\n
 * To be implemented by the developer
 */
int8_t VL53L1_WaitMs(
		uint16_t dev,
		int32_t       wait_ms);

// other useful device functions

VL53L1X_ERROR VL53L1X_SetFastI2C(uint16_t dev);

VL53L1X_ERROR VL53L1X_SetRangingMode(uint16_t dev, uint8_t set_ranging_mode);

VL53L1X_ERROR VL53L1X_SystemStatus(uint16_t dev, uint8_t *state);

char *VL53L1X_SystemStatusString(uint16_t dev);

VL53L1X_ERROR VL53L1X_GetContinuousMeasurement(uint16_t dev, uint8_t *rangeStatus, uint16_t *distanceMM);

VL53L1X_ERROR VL53L1X_GetAndRestartMeasurement(uint16_t dev, uint8_t *rangeStatus, uint16_t *distanceMM);

VL53L1X_ERROR VL53L1X_InitSensorArray(VL53L1_DEV sensor_array, uint8_t sensor_count);


#ifdef __cplusplus
}
#endif

#endif
