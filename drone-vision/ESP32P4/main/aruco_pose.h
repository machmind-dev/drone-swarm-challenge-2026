#pragma once
/* aruco_pose.h — on-board ArUco world-pose estimator (ESP32-P4 / MIPI-CSI) */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * aruco_pose_start() — initialises MIPI-CSI camera via V4L2, enters
 * continuous ArUco detection loop.  Never returns.
 */
void aruco_pose_start(void);

#ifdef __cplusplus
}
#endif
