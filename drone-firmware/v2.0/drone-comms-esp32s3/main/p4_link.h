/* p4_link.h — ESP32-S3 UART receiver for the P4→S3 binary link. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Must match p4_link_protocol.h */
#define P4_LINK_SENSORS 6

typedef struct {
    uint16_t dist_mm[P4_LINK_SENSORS];
    uint8_t  status[P4_LINK_SENSORS];
} p4_tof_data_t;

typedef struct {
    bool  valid;
    float x, y, z;
    float qx, qy, qz, qw;
} p4_pose_data_t;

/* Start the UART2 receiver task.  Must be called once before app_main spins. */
void p4_link_init(void);

/* Copy the latest received ToF frame into *out.
 * Returns true if at least one frame has been received since boot. */
bool p4_link_get_tof(p4_tof_data_t *out);

/* Copy the latest received pose frame into *out.
 * Returns true if at least one frame has been received since boot. */
bool p4_link_get_pose(p4_pose_data_t *out);

/* Milliseconds since the last successfully parsed frame was received.
 * Returns INT64_MAX if no frame has arrived yet. */
int64_t p4_link_age_ms(void);
