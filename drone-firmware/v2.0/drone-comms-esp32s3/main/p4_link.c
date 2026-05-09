/* p4_link.c — ESP32-S3 UART2 receiver for binary frames from ESP32-P4.
 *
 * Board wiring (XIAO ESP32S3 / Mach Mind Sensors Board rev 07/2026):
 *   S3 GPIO3 (D2) RX ← P4 GPIO22 (UART1 TX)
 *   S3 GPIO2 (D1) TX → P4 GPIO23 (UART1 RX)
 *   115200 8N1
 *
 * Parser state machine (byte-by-byte):
 *   WAIT_SOF → GOT_LEN → GOT_TYPE → READING_PAYLOAD → VALIDATE
 *
 * On valid CRC the payload is copied into s_tof / s_pose under a critical
 * section (portENTER_CRITICAL).  The main task reads with portENTER_CRITICAL
 * too, guaranteeing a consistent snapshot even if interrupted mid-copy.
 */

#include "p4_link.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <string.h>

/* Path to shared protocol header */
#include "../../shared/p4_link_protocol.h"

static const char *TAG = "p4_rx";

/* ── Hardware config (S3 side) ────────────────────────────────────────── */
#define P4_RX_UART_PORT   UART_NUM_2
#define P4_RX_GPIO_RX     GPIO_NUM_3   /* D2 ← P4 GPIO22 TX */
#define P4_RX_GPIO_TX     GPIO_NUM_2   /* D1 → P4 GPIO23 RX */
#define P4_RX_BAUD        115200
#define P4_RX_BUF         256

/* ── Shared state ─────────────────────────────────────────────────────── */
static portMUX_TYPE   s_mux       = portMUX_INITIALIZER_UNLOCKED;
static p4_tof_data_t  s_tof       = {0};
static p4_pose_data_t s_pose      = {0};
static bool           s_received  = false;
static int64_t        s_last_rx_us = 0;

/* ── Parser ───────────────────────────────────────────────────────────── */
typedef enum { WAIT_SOF, GOT_LEN, GOT_TYPE, READING_PAYLOAD } rx_state_t;

static void rx_task(void *arg)
{
    (void)arg;
    uint8_t      buf[P4_LINK_FRAME_LEN + 4];   /* generous scratch */
    rx_state_t   state    = WAIT_SOF;
    uint8_t      len      = 0;
    uint8_t      type     = 0;
    uint8_t      p_idx    = 0;
    uint8_t      payload[P4_LINK_PAYLOAD_LEN + 2];

    ESP_LOGI(TAG, "RX task started — UART%d GPIO%d/GPIO%d %d baud",
             (int)P4_RX_UART_PORT, (int)P4_RX_GPIO_RX,
             (int)P4_RX_GPIO_TX, P4_RX_BAUD);

    while (1) {
        uint8_t byte;
        int n = uart_read_bytes(P4_RX_UART_PORT, &byte, 1, pdMS_TO_TICKS(200));
        if (n <= 0) continue;

        switch (state) {
        case WAIT_SOF:
            if (byte == P4_LINK_SOF) state = GOT_LEN;
            break;
        case GOT_LEN:
            if (byte == P4_LINK_PAYLOAD_LEN) {
                len   = byte;
                state = GOT_TYPE;
            } else {
                state = WAIT_SOF;   /* wrong length — discard */
            }
            break;
        case GOT_TYPE:
            if (byte == P4_LINK_TYPE_COMBINED) {
                type  = byte;
                p_idx = 0;
                state = READING_PAYLOAD;
            } else {
                state = WAIT_SOF;   /* unknown type — discard */
            }
            break;
        case READING_PAYLOAD:
            payload[p_idx++] = byte;
            if (p_idx == len + 1 /* +1 for CRC byte */) {
                uint8_t rx_crc   = payload[len];
                uint8_t calc_crc = p4_link_crc8(&type, 1);
                /* CRC covers TYPE (already consumed) + PAYLOAD */
                uint8_t crc_step = calc_crc;
                (void)crc_step;
                /* Re-compute CRC over [TYPE, payload[0..len-1]] */
                uint8_t crc_buf[1 + P4_LINK_PAYLOAD_LEN];
                crc_buf[0] = type;
                memcpy(&crc_buf[1], payload, len);
                uint8_t expected = p4_link_crc8(crc_buf, 1 + len);

                if (rx_crc == expected) {
                    const p4_combined_t *c = (const p4_combined_t *)payload;
                    portENTER_CRITICAL(&s_mux);
                    for (int i = 0; i < P4_LINK_SENSORS; i++) {
                        s_tof.dist_mm[i] = c->tof.dist_mm[i];
                        s_tof.status[i]  = c->tof.status[i];
                    }
                    s_pose.valid = (c->pose.valid != 0);
                    s_pose.x  = c->pose.x;  s_pose.y  = c->pose.y;  s_pose.z  = c->pose.z;
                    s_pose.qx = c->pose.qx; s_pose.qy = c->pose.qy;
                    s_pose.qz = c->pose.qz; s_pose.qw = c->pose.qw;
                    s_received   = true;
                    s_last_rx_us = esp_timer_get_time();
                    portEXIT_CRITICAL(&s_mux);
                } else {
                    ESP_LOGD(TAG, "CRC mismatch: got 0x%02x expected 0x%02x", rx_crc, expected);
                }
                state = WAIT_SOF;
            }
            /* Safety: if payload overflows (corrupt LEN slipped through), reset */
            if (p_idx > P4_LINK_PAYLOAD_LEN + 1) {
                state = WAIT_SOF;
                p_idx = 0;
            }
            break;
        }
        (void)buf;
    }
}

/* ── Public API ───────────────────────────────────────────────────────── */

void p4_link_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = P4_RX_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(P4_RX_UART_PORT, P4_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(P4_RX_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(P4_RX_UART_PORT,
                                 P4_RX_GPIO_TX, P4_RX_GPIO_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(rx_task, "p4_rx", 3072, NULL, 5, NULL);
    ESP_LOGI(TAG, "P4 link initialised — UART%d  RX=GPIO%d  TX=GPIO%d  %d baud",
             (int)P4_RX_UART_PORT, (int)P4_RX_GPIO_RX, (int)P4_RX_GPIO_TX, P4_RX_BAUD);
}

bool p4_link_get_tof(p4_tof_data_t *out)
{
    bool ok;
    portENTER_CRITICAL(&s_mux);
    ok = s_received;
    if (ok) *out = s_tof;
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

bool p4_link_get_pose(p4_pose_data_t *out)
{
    bool ok;
    portENTER_CRITICAL(&s_mux);
    ok = s_received;
    if (ok) *out = s_pose;
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

int64_t p4_link_age_ms(void)
{
    if (!s_received) return INT64_MAX;
    return (esp_timer_get_time() - s_last_rx_us) / 1000;
}
