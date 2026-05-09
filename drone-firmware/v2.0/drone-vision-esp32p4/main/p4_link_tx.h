/* p4_link_tx.h — ESP32-P4 UART transmitter for the P4→S3 binary link. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Initialise UART1 (GPIO22 TX → S3, GPIO23 RX ← S3) at 115200 baud. */
void p4_link_tx_init(void);

/* Pack and send one COMBINED frame over UART1.
 * dist_mm / status: arrays of P4_LINK_SENSORS elements from tof_task.
 * pose_valid: set true when ArUco has a fresh world-pose estimate.
 * x,y,z,qx,qy,qz,qw: pose in arena frame (ignored when pose_valid=false). */
void p4_link_send_combined(const uint16_t *dist_mm, const uint8_t *status,
                            bool pose_valid,
                            float x, float y, float z,
                            float qx, float qy, float qz, float qw);
