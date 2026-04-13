/* main.c 
 *
 * Using ESP-IDF high-resolution timed measurement cycle and VL53L1 triggered measurements
 * 
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "VL53L1X_api.h"

#define MEASUREMENT_CYCLE_MS        (50)        // the timing budget for the VL53L1
#define TIMER_PERIODIC_MS           (60)        // periodic timer for measurements

uint16_t TOF = VL53L1_I2C_ADDRESS;

int     just_a_number = 17;
int     range_mm = 0;
int32_t measurement_cycle = 0;
int64_t last_measurement = 0;

esp_timer_handle_t tof_sensor_timer;   // collects measurements from the ToF sensor

static void periodic_tof_sensor(void* arg)
{
    VL53L1X_ERROR get_status;
    int64_t update_measurement_time;
    uint8_t RangeStatus = VL53L1_RANGESTATUS_NONE;
    uint16_t Distance = -1;
    bool error;

    // mark the start time for this service routine
    update_measurement_time = esp_timer_get_time();
    measurement_cycle = update_measurement_time - last_measurement;
    last_measurement = update_measurement_time;

    // get the measurement and start a new measurement cycle
    get_status = VL53L1X_GetAndRestartMeasurement(TOF, &RangeStatus, &Distance);

    // determine if a measurement error happened
    error = get_status != VL53L1_ERROR_NONE;
    error = error || ( RangeStatus != VL53L1_RANGESTATUS_RANGE_VALID && RangeStatus != VL53L1_RANGESTATUS_WRAP_TARGET_FAIL );
    range_mm = error ? -1 : Distance;
}

void app_main(void)
{
    uint8_t model_id, module_type, sensorState = 0;
    VL53L1X_ERROR status = 0;

    // delay to let the serial monitor re-connect
    vTaskDelay( 2000 / portTICK_PERIOD_MS );
    printf( "\r\nDevice Starting...\r\n" );

    // startup the I2C interface and scan for the devices
    i2c_init();
    i2c_scan();

    // check the VL53L1 device and wait for it to boot
    status = VL53L1_RdByte(TOF, 0x010F, &model_id);
    printf("VL53L1X Model_ID: %X, status = %d\n", model_id, status );
    status = VL53L1_RdByte(TOF, 0x0110, &module_type);
    printf("VL53L1X Module_Type: %X, status = %d\n", module_type, status );
    while ( sensorState == 0 ) {
        status = VL53L1X_BootState(TOF, &sensorState);
        vTaskDelay( 20 / portTICK_PERIOD_MS );
    }
    printf("VL53L1 device booted\n");

    // initialize the ToF sensor
    VL53L1X_SensorInit( TOF );

    // 1=short (up to 1 M), 2=long (up to 4 M)
    VL53L1X_SetDistanceMode(TOF, 1);

    // in ms possible values [20, 50, 100, 200, 500]
    VL53L1X_SetTimingBudgetInMs(TOF, MEASUREMENT_CYCLE_MS);       

    // in ms, IM must be > = TB
    VL53L1X_SetInterMeasurementInMs(TOF, 5 + MEASUREMENT_CYCLE_MS);   

    // need to start the VL53L1 with a first request for measurement
    printf("VL53L1X Ultra Lite Driver Example running ...\n");
    VL53L1X_StartRanging(TOF);   
    vTaskDelay( TIMER_PERIODIC_MS / portTICK_PERIOD_MS );

    // create the periodic time and service routine
    const esp_timer_create_args_t tof_sensor_timer_args = {
        .callback = &periodic_tof_sensor,
        .name = "tofsensor"
    };
    esp_timer_create( &tof_sensor_timer_args, &tof_sensor_timer );
    esp_timer_start_periodic( tof_sensor_timer, TIMER_PERIODIC_MS * 1000 );

    while(1){ 
        vTaskDelay( 300 / portTICK_PERIOD_MS );
        printf( "\r range = %d mm, cycle = %4.1f ms  ", range_mm, (float)measurement_cycle / 1000);
    }
}
