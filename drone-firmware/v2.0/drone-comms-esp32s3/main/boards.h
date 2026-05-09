/**
 * boards.h — GPIO pin map for XIAO ESP32S3 Sense (Mach Mind comms node)
 *
 * ── GPIO allocation ─────────────────────────────────────────────────────────
 *   GPIO   Function          Notes
 *   ─────  ────────────────  ────────────────────────────────────────────────
 *     1    Drone ID LED       Blink pattern = DRONE_ID pulses
 *     2    P4 link TX (D1)    UART2 → ESP32-P4 GPIO23 (RX)
 *     3    P4 link RX (D2)    UART2 ← ESP32-P4 GPIO22 (TX)
 *    43    PX4 UART1 TX       UART1 → PX4 flight controller
 *    44    PX4 UART1 RX       UART1 ← PX4 flight controller
 *
 * ── UART topology ───────────────────────────────────────────────────────────
 *   UART_NUM_0  GPIO43/44  Console (USB-SERIAL-JTAG or UART0 bridge)
 *   UART_NUM_1  GPIO43/44  MAVLink to PX4 (57600 baud)
 *   UART_NUM_2  GPIO2/3    Binary link to ESP32-P4 (115200 baud)
 *
 * Note: on XIAO ESP32S3, GPIO43/44 are the default console UART0 pins.
 * CONFIG_ESP_CONSOLE_UART_DEFAULT selects UART0 for console monitor.
 * UART_NUM_1 is reassigned here to PX4 link — make sure to set
 * CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y in sdkconfig.defaults if monitoring
 * without a USB-UART bridge.
 */

#pragma once
#include "driver/gpio.h"
#include "driver/uart.h"

/* ── Drone identity ──────────────────────────────────────────────────────── */
#define DRONE_ID_LED_PIN    GPIO_NUM_1

/* ── MAVLink / PX4 link (UART1) ──────────────────────────────────────────── */
#define PX4_UART_PORT       UART_NUM_1
#define PX4_UART_TX         GPIO_NUM_43    /* → PX4 RX */
#define PX4_UART_RX         GPIO_NUM_44    /* ← PX4 TX */
#define PX4_UART_BAUD       57600

/* ── P4 sensor link (UART2) ──────────────────────────────────────────────── */
#define P4_UART_PORT        UART_NUM_2
#define P4_UART_TX          GPIO_NUM_2     /* D1 → P4 GPIO23 (RX) */
#define P4_UART_RX          GPIO_NUM_3     /* D2 ← P4 GPIO22 (TX) */
#define P4_UART_BAUD        115200
