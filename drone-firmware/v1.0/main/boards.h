/**
 * \file boards.h
 * \mainpage Definitions for camera pins on different boards
 * \author Tobit Flatscher (github.com/2b-t)
*/

#ifndef _MICROROS_CAMERA_BOARDS_H_
#define _MICROROS_CAMERA_BOARDS_H_

#ifdef CAMERA_MODEL_XIAO_ESP32S3
  // pins for Seeed Studio Xiao ESP32-S3 Sense
  // Taken from https://github.com/Seeed-Studio/SSCMA-Micro/blob/dev/porting/espressif/boards/seeed_xiao_esp32s3/board.h
  #define CAMERA_PIN_PWDN    -1
  #define CAMERA_PIN_RESET   -1
  #define CAMERA_PIN_XCLK    10
  #define CAMERA_PIN_SIOD    40
  #define CAMERA_PIN_SIOC    39

  #define CAMERA_PIN_D0      15
  #define CAMERA_PIN_D1      17
  #define CAMERA_PIN_D2      18
  #define CAMERA_PIN_D3      16
  #define CAMERA_PIN_D4      14
  #define CAMERA_PIN_D5      12
  #define CAMERA_PIN_D6      11
  #define CAMERA_PIN_D7      48

  #define CAMERA_PIN_VSYNC   38
  #define CAMERA_PIN_HREF    47
  #define CAMERA_PIN_PCLK    13

  #define LED_0_PIN          21
#endif

#endif // define _MICROROS_CAMERA_BOARDS_H_
