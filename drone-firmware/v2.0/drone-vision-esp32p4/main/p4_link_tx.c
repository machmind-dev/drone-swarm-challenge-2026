/* p4_link_tx.c — P4->S3 binary frame transmitter.
 *
 * Sends P4_LINK_TYPE_COMBINED frames at the ToF poll rate (20 Hz).
 * Frame: SOF(1) + LEN(1) + TYPE(1) + p4_combined_t(127) + CRC8(1) = 131 bytes.
 * At 115200 baud: 131 x 10 bits / 115200 = 11.4 ms  ->  well within the 50 ms budget.
 */

#include "p4_link_tx.h"
#include "boards.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

/* boards.h provides: S3_UART_PORT, S3_UART_TX, S3_UART_RX, S3_UART_BAUD */

#include "../shared/p4_link_protocol.h"

static const char *TAG = "p4_tx";

void p4_link_tx_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = S3_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(S3_UART_PORT, 512, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(S3_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(S3_UART_PORT,
                                 S3_UART_TX, S3_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART%d  TX=GPIO%d  RX=GPIO%d  %d baud",
             (int)S3_UART_PORT, (int)S3_UART_TX, (int)S3_UART_RX, S3_UART_BAUD);
}

void p4_link_send_combined(const uint16_t *dist_mm, const uint8_t *status,
                            bool pose_valid,
                            float x, float y, float z,
                            float qx, float qy, float qz, float qw,
                            uint8_t trigger_id,
                            const p4_boxes_t *boxes)
{
    p4_combined_t payload;

    for (int i = 0; i < P4_LINK_SENSORS; i++) {
        payload.tof.dist_mm[i] = dist_mm[i];
        payload.tof.status[i]  = status[i];
    }
    payload.pose.valid      = pose_valid ? 1 : 0;
    payload.pose.trigger_id = trigger_id;
    payload.pose.x  = x;  payload.pose.y  = y;  payload.pose.z  = z;
    payload.pose.qx = qx; payload.pose.qy = qy; payload.pose.qz = qz;
    payload.pose.qw = qw;

    if (boxes) {
        payload.boxes = *boxes;
    } else {
        memset(&payload.boxes, 0, sizeof(payload.boxes));
    }

    /* Build frame: SOF | LEN | TYPE | PAYLOAD | CRC */
    uint8_t frame[P4_LINK_FRAME_LEN];
    frame[0] = P4_LINK_SOF;
    frame[1] = P4_LINK_PAYLOAD_LEN;
    frame[2] = P4_LINK_TYPE_COMBINED;
    memcpy(&frame[3], &payload, P4_LINK_PAYLOAD_LEN);
    /* CRC over TYPE + PAYLOAD */
    frame[3 + P4_LINK_PAYLOAD_LEN] = p4_link_crc8(&frame[2], 1 + P4_LINK_PAYLOAD_LEN);

    uart_write_bytes(S3_UART_PORT, (const char *)frame, sizeof(frame));
}
