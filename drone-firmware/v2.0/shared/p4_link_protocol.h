/* p4_link_protocol.h — Binary frame protocol between ESP32-P4 and ESP32-S3.
 *
 * Frame layout:
 *   | SOF (0xAB, 1B) | LEN (1B, payload bytes) | TYPE (1B) | PAYLOAD | CRC8 (1B) |
 *
 * CRC-8 (poly 0x07, init 0x00) covers TYPE + PAYLOAD bytes.
 *
 * This header is shared between firmware projects — keep it pure C, no
 * ESP-IDF types.  Include in both drone-vision-esp32p4 and drone-comms-esp32s3.
 */

#pragma once
#include <stdint.h>

#define P4_LINK_SOF           0xAB
#define P4_LINK_TYPE_COMBINED 0x03   /* only type currently used */

#define P4_LINK_SENSORS       6

/* ── Payload structures (packed, little-endian) ───────────────────────── */

typedef struct __attribute__((packed)) {
    uint16_t dist_mm[P4_LINK_SENSORS];   /* 12 B — VL53L1X raw readings    */
    uint8_t  status[P4_LINK_SENSORS];    /*  6 B — range_status (0 = valid) */
} p4_tof_t;   /* 18 bytes */

typedef struct __attribute__((packed)) {
    uint8_t  valid;         /* 1 = pose data valid, 0 = no ArUco fix */
    uint8_t  trigger_id;    /* TEST: first detected marker ID, 0xFF = none */
    float    x, y, z;      /* position metres, arena frame           */
    float    qx, qy, qz, qw; /* orientation quaternion               */
} p4_pose_t;   /* 30 bytes */

/* ── Box marker entries (detected capture-zone boxes) */
#define P4_LINK_BOX_MAX  6   /* max box markers per frame (blue 31-36, red 41-46) */

typedef struct __attribute__((packed)) {
    uint8_t id;       /* ArUco ID: 31-36 = blue team, 41-46 = red team */
    float   x, y, z;  /* world position, metres (arena frame) */
} p4_box_entry_t;   /* 13 bytes */

typedef struct __attribute__((packed)) {
    uint8_t        count;                    /* 0..P4_LINK_BOX_MAX valid entries */
    p4_box_entry_t entries[P4_LINK_BOX_MAX]; /* only [0..count-1] are valid */
} p4_boxes_t;   /* 1 + 6x13 = 79 bytes */

typedef struct __attribute__((packed)) {
    p4_tof_t   tof;    /* 18 B */
    p4_pose_t  pose;   /* 30 B */
    p4_boxes_t boxes;  /* 79 B */
} p4_combined_t;   /* 127 bytes */

/* ── Frame sizes ──────────────────────────────────────────────────────── */
#define P4_LINK_PAYLOAD_LEN  ((uint8_t)sizeof(p4_combined_t))  /* 127 */
#define P4_LINK_FRAME_LEN    (1 + 1 + 1 + P4_LINK_PAYLOAD_LEN + 1)  /* 131 */

/* ── CRC-8 (poly 0x07) — computed over [TYPE, PAYLOAD...] ────────────── */
static inline uint8_t p4_link_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? ((uint8_t)(crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}
