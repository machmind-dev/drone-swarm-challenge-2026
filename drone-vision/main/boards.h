/**
 * boards.h — Camera pin definitions for drone-vision benchmark
 *
 * Supported boards:
 *   CAMERA_MODEL_XIAO_ESP32S3       Seeed XIAO ESP32S3 Sense (OV2640 / OV5640)
 *   CAMERA_MODEL_OV3660_DEVKIT      M5Stack Timer Camera X / generic OV3660 breakout
 *
 * Select via Kconfig (CONFIG_VISION_BOARD_*) or by defining the macro before
 * including this header.
 */

#pragma once

/* ── XIAO ESP32S3 Sense ─────────────────────────────────────────────────── */
#ifdef CAMERA_MODEL_XIAO_ESP32S3
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

/* ── M5Stack Timer Camera X / OV3660 breakout ───────────────────────────── *
 * OV3660: 3 MP, supports up to QXGA (2048×1536) hardware, but ESP32-S3 DMA  *
 * caps usable output at VGA (640×480) before frame-buffer PSRAM fills.       *
 * Recommended test resolutions: 160×120, 320×240, 480×320, 640×480.          *
 * ─────────────────────────────────────────────────────────────────────── */
#ifdef CAMERA_MODEL_OV3660_DEVKIT
  #define CAMERA_PIN_PWDN     0
  #define CAMERA_PIN_RESET   15
  #define CAMERA_PIN_XCLK    27
  #define CAMERA_PIN_SIOD    25
  #define CAMERA_PIN_SIOC    23

  #define CAMERA_PIN_D0      32
  #define CAMERA_PIN_D1      35
  #define CAMERA_PIN_D2      34
  #define CAMERA_PIN_D3      5
  #define CAMERA_PIN_D4      39
  #define CAMERA_PIN_D5      18
  #define CAMERA_PIN_D6      36
  #define CAMERA_PIN_D7      19

  #define CAMERA_PIN_VSYNC   22
  #define CAMERA_PIN_HREF    26
  #define CAMERA_PIN_PCLK    21

  #define LED_0_PIN          2
#endif
