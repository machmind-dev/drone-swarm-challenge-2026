/* main.c 
 *
 * Using ESP-IDF gpio interrupt and VL53L1 in continuous measurement mode
 * 
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "VL53L1X_api.h"

#define MEASUREMENT_CYCLE_MS        (50)            // the timing budget for the VL53L1
#define VL53L1_INTERRUPT_PIN        (GPIO_NUM_13)   // ESP32 GPIO pin connected to VL53L1 interrupt input

uint16_t TOF = VL53L1_I2C_ADDRESS;

xQueueHandle vl53_evt_queue = NULL;

int     just_a_number = 17;
int     range_mm = 0;
int32_t measurement_cycle = 0;
int64_t last_measurement = 0;

static void IRAM_ATTR vl53_isr_handler(void* arg)
{
    // This is the hardware interrupt. No processing takes place, it merely queues the
    // measurement event for the vl53_service() routine to process.
    xQueueSendFromISR(vl53_evt_queue, arg, NULL);
}

static void vl53_service(void* arg)
{
    // This is the VL53L1 measurement handler, which operates in a thread that blocks on the
    // vl53_evt_queue. 

    int some_num = 0;
    VL53L1X_ERROR get_status;
    int64_t update_measurement_time;
    uint8_t RangeStatus = VL53L1_RANGESTATUS_NONE;
    uint16_t Distance = -1;
    bool error;

    while (1) {
        if ( xQueueReceive(vl53_evt_queue, &some_num, portMAX_DELAY ) ) {

            // mark the start time for this service routine
            update_measurement_time = esp_timer_get_time();
            measurement_cycle = update_measurement_time - last_measurement;
            last_measurement = update_measurement_time;

            // get the measurement and reset the interrupt
            get_status = VL53L1X_GetContinuousMeasurement(TOF, &RangeStatus, &Distance);

            // determine if a measurement error happened
            error = get_status != VL53L1_ERROR_NONE;
            error = error || ( RangeStatus != VL53L1_RANGESTATUS_RANGE_VALID && RangeStatus != VL53L1_RANGESTATUS_WRAP_TARGET_FAIL );
            range_mm = error ? -1 : Distance;
        }
    }
}

void setup_gpio_interrupt( gpio_num_t gpio_pin, gpio_isr_t isr_handler )
{
    //create a queue to handle gpio event from isr
    vl53_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //start gpio task
    xTaskCreate(vl53_service, "vl53_service", 8192, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(gpio_pin, isr_handler, (void*) &just_a_number);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ( 1ULL << gpio_pin );
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
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

    // define the interrupt pin and ISR function
    setup_gpio_interrupt( VL53L1_INTERRUPT_PIN, vl53_isr_handler );

    // need to start the VL53L1 with a first request for measurement
    printf("VL53L1X Ultra Lite Driver Example running ...\n");
    VL53L1X_StartRanging(TOF);   

    while(1){ 
        vTaskDelay( 300 / portTICK_PERIOD_MS );
        printf( "\r range = %d mm, cycle = %4.1f ms  ", range_mm, (float)measurement_cycle / 1000);
    }
}
