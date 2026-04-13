/* main.c 
 *
 * Using multiple VL53L1 ranging sensors with ESP-IDF IoT Framework
 * 
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "VL53L1X_api.h"

#define MEASUREMENT_CYCLE_MS        (20)    // the timing budget for the VL53L1
#define TIMER_PERIODIC_MS           (25)    // periodic timer for measurements, at least 5 + MEASUREMENT_CYCLE_MS

VL53L1_Dev_t tof_array[] = {
    { // [0]
        .I2cDevAddr = VL53L1_I2C_ADDRESS + 2, 
        .shutdown_pin = GPIO_NUM_16,
        .interrupt_pin = 0,
        .distance_mode = DISTANCE_MODE_SHORT,
        .timing_budget = MEASUREMENT_CYCLE_MS,
        .inter_measurement = TIMER_PERIODIC_MS
    }, 
    { // [1]
        .I2cDevAddr = VL53L1_I2C_ADDRESS + 4, 
        .shutdown_pin = GPIO_NUM_17,
        .interrupt_pin = 0,
        .distance_mode = DISTANCE_MODE_SHORT,
        .timing_budget = MEASUREMENT_CYCLE_MS,
        .inter_measurement = TIMER_PERIODIC_MS
    }
};

uint8_t     sensor_count = sizeof(tof_array) / sizeof(VL53L1_Dev_t);

esp_timer_handle_t tof_sensor_timer;   // collects measurements from the ToF sensor

static void periodic_tof_sensor(void* arg)
{
    // mark the cycle time for this service routine
    int64_t update_measurement_time = esp_timer_get_time();

    for ( int k = 0; k < sensor_count; k++ ) {
        VL53L1_Dev_t *tof = &tof_array[k];
        // get the measurement and start a new measurement cycle
        tof->range_error = VL53L1X_GetAndRestartMeasurement(tof->I2cDevAddr, &tof->range_status, &tof->range_mm);
        tof->cycle_time = update_measurement_time - tof->time_stamp;
        tof->time_stamp = update_measurement_time;
    }
}

void app_main(void)
{
    // delay to let the serial monitor re-connect
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    printf( "\r\nDevice Starting...\r\n" );

    // startup the I2C interface and scan for the devices
    i2c_init();
    
    // initalize all of the sensors
    VL53L1X_InitSensorArray( tof_array, sensor_count );

    // scanning should now show all devices at their new I2C address
    i2c_scan();

    // create the periodic time and service routine
    const esp_timer_create_args_t tof_sensor_timer_args = {
        .callback = &periodic_tof_sensor,
        .name = "tofsensor"
    };
    esp_timer_create( &tof_sensor_timer_args, &tof_sensor_timer );
    esp_timer_start_periodic( tof_sensor_timer, TIMER_PERIODIC_MS * 1000 );

    while(1){ 
        vTaskDelay( 500 / portTICK_PERIOD_MS );
        printf( "\r range:%5d|%5d mm, cycle: %4.1f| %4.1f ms, error:%3d|%3d, status:%3d|%3d    ", 
            tof_array[0].range_mm, tof_array[1].range_mm, 
            (float)tof_array[0].cycle_time / 1000, (float)tof_array[1].cycle_time / 1000, 
            tof_array[0].range_error, tof_array[1].range_error, 
            tof_array[0].range_status, tof_array[1].range_status);
    }
}
