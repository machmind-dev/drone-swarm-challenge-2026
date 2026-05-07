#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start VL53L1X polling task. Call once from app_main before aruco task. */
void tof_task_start(void);

/* Latest reading from sensor index 0..5.
 * Returns 0 / status=255 until the first valid measurement arrives. */
uint16_t tof_get_distance_mm(uint8_t idx);
uint8_t  tof_get_range_status(uint8_t idx);

/* Number of sensors currently active in the array. */
uint8_t  tof_sensor_count(void);

#ifdef __cplusplus
}
#endif
