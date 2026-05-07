#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Shared volatile state owned by main.c
extern volatile float   vp_x, vp_y, vp_z;
extern volatile float   vp_qx, vp_qy, vp_qz, vp_qw;
extern volatile bool    vision_pose_valid;
extern volatile int64_t last_vision_pose_ms;
extern volatile bool    vision_enabled;

// MAVLink sender owned by main.c
void mav_send_vision_estimate(float x, float y, float z,
                               float roll, float pitch, float yaw);
// ArUco task entry point
void aruco_task_start(void);

#ifdef __cplusplus
}
#endif
