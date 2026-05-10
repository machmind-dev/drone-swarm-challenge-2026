#pragma once
/* aruco_pose.h — on-board ArUco world-pose estimator (ESP32-P4 / MIPI-CSI) */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void aruco_pose_start(void);

/**
 * aruco_pose_get_latest() — copy the most recent world-pose estimate.
 * Returns true if a valid pose was computed this frame, false if no known
 * marker was visible.  Safe to call from any task/core.
 */
bool aruco_pose_get_latest(float *x, float *y, float *z,
                            float *qx, float *qy, float *qz, float *qw);

#ifdef __cplusplus
}
#endif
