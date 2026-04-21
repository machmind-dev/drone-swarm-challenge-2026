/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "esp_camera.h"

#ifdef CONFIG_CAMERA_MODULE_ESP_EYE
#define CAMERA_MODULE_NAME "ESP-EYE"

#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK    4
#define CAM_PIN_SIOD    18
#define CAM_PIN_SIOC    23

#define CAM_PIN_D7      36
#define CAM_PIN_D6      37
#define CAM_PIN_D5      38
#define CAM_PIN_D4      39
#define CAM_PIN_D3      35
#define CAM_PIN_D2      14
#define CAM_PIN_D1      13
#define CAM_PIN_D0      34
#define CAM_PIN_VSYNC   5
#define CAM_PIN_HREF    27
#define CAM_PIN_PCLK    25

#elif CONFIG_CAMERA_MODULE_ESP_S3_EYE 
#define CAMERA_MODULE_NAME "ESP-S3-EYE"

#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD    4
#define CAM_PIN_SIOC    5

#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0       11
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13

#else // 

#define CAMERA_MODULE_NAME "AI-THINKER"

#define CAM_PIN_PWDN    32 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

#endif //

#ifdef CONFIG_IDF_TARGET_ARCH_XTENSA
static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,//YUV422,PIXFORMAT_GRAYSCALE,RGB565,PIXFORMAT_JPEG
    .frame_size = FRAMESIZE_QVGA,//QQVGA-QXGA Do not use sizes above QVGA when not JPEG
    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1, //if more than one, i2s runs in continuous mode. Use only with JPEG

    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,

};
#endif // CONFIG_IDF_TARGET_ARCH_XTENSA

void cv_init_object_tracking(int pos_x, int pos_y);
int cv_object_tracking(uint8_t* data, int width, int heigth);

#ifdef CONFIG_IDF_TARGET_ARCH_XTENSA
esp_err_t camera_capture()
{
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE("CAM", "Camera Capture Failed");
        return ESP_FAIL;
    }

    // Test text area detection
    cv_object_tracking(fb->buf, fb->width, fb->height);  

    esp_camera_fb_return(fb);
    return ESP_OK;
}
#endif // 

void cv_print_info();

void app_main(void)
{
    printf("Init\n");
    cv_print_info();

    cv_init_object_tracking(320/2,200/2);
#ifdef CONFIG_IDF_TARGET_ARCH_XTENSA

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE("CAM", "Camera Init Failed");
        return;
    }
    sensor_t *s = esp_camera_sensor_get();
    ESP_LOGI("CAM", "Camera sensor %2.2x %2.2x %4.4x %2.2x", s->id.MIDH, s->id.MIDL, s->id.PID, s->id.VER);

    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_brightness(s, 1);  //up the brightness just a bit
        s->set_saturation(s, -2); //lower the saturation
    }

    while(1)
    {
        int64_t fr_start = esp_timer_get_time();

        camera_capture();
        
        int64_t fr_end = esp_timer_get_time();
        float fps = 1*1000000/(fr_end - fr_start);
        ESP_LOGW("OpenCV", "Object tracking - %2.2f FPS", fps);
    }
#else
    ESP_LOGW("OpenCV", "Camera module not defined!");
#endif
}
