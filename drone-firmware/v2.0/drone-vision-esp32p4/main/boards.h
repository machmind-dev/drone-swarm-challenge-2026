/**
 * boards.h — GPIO pin map for Waveshare ESP32-P4-WiFi6 + OV5647 + VL53L1X
 *
 * ── GPIO allocation ─────────────────────────────────────────────────────────
 *   GPIO   Function          Notes
 *   ─────  ────────────────  ────────────────────────────────────────────────
 *     0    BOOT button       Reserved — hold LOW during reset to enter DL mode
 *     2    VL53L1X SDA      I2C_NUM_0 — dedicated TOF bus, breakout pull-ups
 *     3    VL53L1X SCL      I2C_NUM_0 — dedicated TOF bus, breakout pull-ups
 *     4    VL53L1X XSHUT   Slot 0 open-drain XSHUT
 *     5    VL53L1X XSHUT   Slot 5 open-drain XSHUT (upward-facing)
 *     7    Camera SCCB SDA  I2C_NUM_1 — camera sensor control (OV5647)
 *     8    Camera SCCB SCL  I2C_NUM_1 — camera sensor control (OV5647)
 *    28    VL53L1X XSHUT   Slot 4 open-drain XSHUT
 *    29    VL53L1X XSHUT   Slot 3 open-drain XSHUT
 *    30    VL53L1X XSHUT   Slot 2 open-drain XSHUT
 *    31    VL53L1X XSHUT   Slot 1 open-drain XSHUT
 *    37    UART0 TX         Console via CH343 USB-UART bridge → /dev/ttyACM0
 *    38    UART0 RX         Console via CH343 USB-UART bridge → /dev/ttyACM0
 *    51    (reserved)       WiFi6 (ESP32-C6) / camera PWDN — do not use
 *   HW     MIPI-CSI lanes   Fixed silicon differential pairs, no GPIO config
 *
 * ── I2C bus topology ────────────────────────────────────────────────────────
 *   I2C_NUM_0  GPIO2/3   VL53L1X ToF sensors (dedicated, 4.7 kΩ on breakout)
 *   I2C_NUM_1  GPIO7/8   OV5647 SCCB (pull-ups on camera module)
 */

#pragma once
#include "driver/gpio.h"

/* ── Camera (OV5647 via MIPI-CSI, SCCB control on I2C_NUM_1) ────────────── */
#define CAMERA_SCCB_SDA   GPIO_NUM_7
#define CAMERA_SCCB_SCL   GPIO_NUM_8
#define CAMERA_RESET_PIN  GPIO_NUM_NC   /* not connected / tied high on board */
#define CAMERA_PWDN_PIN   GPIO_NUM_NC   /* not connected / active-low unused  */

/* ── VL53L1X ToF sensors (I2C_NUM_0, dedicated, see tof_task.c) ─────────── */
#define TOF_I2C_SDA         GPIO_NUM_2    /* dedicated I2C_NUM_0 bus             */
#define TOF_I2C_SCL         GPIO_NUM_3    /* dedicated I2C_NUM_0 bus             */
#define TOF_XSHUT_PIN_0     GPIO_NUM_4    /* slot 0 XSHUT — open-drain          */
#define TOF_XSHUT_PIN_1     GPIO_NUM_31   /* slot 1 XSHUT — open-drain          */
#define TOF_XSHUT_PIN_2     GPIO_NUM_30   /* slot 2 XSHUT — open-drain          */
#define TOF_XSHUT_PIN_3     GPIO_NUM_29   /* slot 3 XSHUT — open-drain          */
#define TOF_XSHUT_PIN_4     GPIO_NUM_28   /* slot 4 XSHUT — open-drain          */
#define TOF_XSHUT_PIN_5     GPIO_NUM_5    /* slot 5 XSHUT — upward-facing       */
