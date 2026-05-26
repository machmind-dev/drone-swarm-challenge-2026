#pragma once
/* aruco_pose.h — on-board ArUco world-pose estimator (ESP32-P4 / MIPI-CSI) */

#include <stdbool.h>
#include "../shared/p4_link_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void aruco_pose_start(void);

/* Copy the most recent world-pose estimate.  Returns true when a valid pose
 * was computed this frame; trigger_id is 0xFF if no trigger marker visible. */
bool aruco_pose_get_latest(float *x, float *y, float *z,
                            float *qx, float *qy, float *qz, float *qw,
                            uint8_t *trigger_id, float *reproj_err);

/* Copy the most recent box marker world positions (IDs 31-46, count >= 0). */
void aruco_boxes_get_latest(p4_boxes_t *out);

#ifdef __cplusplus
}
#endif
