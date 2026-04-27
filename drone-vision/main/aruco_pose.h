#pragma once
/* aruco_pose.h — on-board ArUco world-pose estimator (no ROS2, serial output) */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * aruco_pose_start() — initialises camera, enters continuous detection loop.
 * Never returns.  Call from app_main() when CONFIG_VISION_MODE_POSE=y.
 */
void aruco_pose_start(void);

#ifdef __cplusplus
}
#endif
