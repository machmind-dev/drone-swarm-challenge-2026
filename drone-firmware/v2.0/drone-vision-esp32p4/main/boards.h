/**
 * boards.h — Camera pin definitions for ESP32-P4 + OV5647 (MIPI-CSI)
 *
 * Target: Waveshare ESP32-P4 WiFi6 Dev Board with OV5647 5MP camera module.
 *
 * MIPI-CSI data and clock lanes are on fixed hardware pins — they are wired
 * directly to the MIPI-CSI controller and require no GPIO configuration.
 * Only the SCCB (I2C camera control bus) and optional power-control GPIOs
 * are user-configurable.
 *
 * ── Waveshare ESP32-P4-WiFi6-DEV camera connector (J1) ─────────────────────
 *   VERIFY these GPIO numbers against your board's schematic before flashing.
 *   Waveshare publishes schematics at their product wiki page.
 *
 *   Signal   GPIO   Notes
 *   ───────  ─────  ───────────────────────────────────────────────────────
 *   SCCB SDA   7    I2C data (camera sensor register control)
 *   SCCB SCL   8    I2C clock
 *   PWDN      -1    Not routed — sensor stays always powered
 *   RESET     -1    Not routed — tied to 3.3 V via pull-up on module
 *   MIPI CSI  HW    Fixed silicon pins, no user config required
 */

#pragma once
#include "driver/gpio.h"

/* ── Waveshare ESP32-P4 WiFi6 + OV5647 ───────────────────────────────────── */
#define CAMERA_SCCB_SDA   GPIO_NUM_7
#define CAMERA_SCCB_SCL   GPIO_NUM_8
#define CAMERA_RESET_PIN  GPIO_NUM_NC   /* not connected / tied high on board */
#define CAMERA_PWDN_PIN   GPIO_NUM_NC   /* not connected / active-low unused  */
