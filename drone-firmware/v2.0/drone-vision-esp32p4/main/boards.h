/**
 * boards.h — GPIO pin map for Waveshare ESP32-P4-WiFi6 + OV5647 + VL53L1X
 *
 * ── GPIO allocation ─────────────────────────────────────────────────────────
 *   GPIO   Function          Notes
 *   ─────  ────────────────  ────────────────────────────────────────────────
 *     0    BOOT button       Reserved — hold LOW during reset to enter DL mode
 *     2    VL53L1X SDA      I2C_NUM_0 — dedicated TOF bus, breakout pull-ups
 *     3    VL53L1X SCL      I2C_NUM_0 — dedicated TOF bus, breakout pull-ups
 *     4    (free)
 *     7    Camera SCCB SDA  I2C_NUM_1 — camera sensor control (OV5647)
 *     8    Camera SCCB SCL  I2C_NUM_1 — camera sensor control (OV5647)
 *    37    UART0 TX         Console via CH343 USB-UART bridge → /dev/ttyACM0
 *    38    UART0 RX         Console via CH343 USB-UART bridge → /dev/ttyACM0
 *    51    VL53L1X XSHUT   Open-drain output; driven LOW = sensor shutdown
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
#define TOF_XSHUT_PIN     GPIO_NUM_51   /* slot 0 XSHUT — open-drain          */
#define TOF_I2C_SDA       GPIO_NUM_2    /* dedicated I2C_NUM_0 bus             */
#define TOF_I2C_SCL       GPIO_NUM_3    /* dedicated I2C_NUM_0 bus             */
