/* aruco_task.c
 * Onboard ArUco detection for XIAO ESP32S3 Sense.
 * Runs on Core 1. Reads camera frames via a FreeRTOS queue,
 * detects markers using espressif/opencv, computes pose via
 * solvePnP, and writes results into the shared volatile vp_* vars.
 *
 * Add to CMakeLists.txt:
 *   idf_component_register(SRCS "main.c" "aruco_task.c" ...)
 *
 * Add to idf_component.yml:
 *   espressif/opencv: "^4.10.0~3"
 */

#include "aruco_task.h"

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef EPS
#undef EPS
#endif

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect/aruco_detector.hpp"
#include "opencv2/objdetect/aruco_dictionary.hpp"
#include "opencv2/calib3d.hpp"

static const char *TAG = "aruco";

/* ── Tuning ────────────────────────────────────────────────────────────── */
#define ARUCO_DICT        cv::aruco::DICT_4X4_50
#define MARKER_SIZE_M     0.15f          /* physical marker side length, metres */
#define FRAME_W           160
#define FRAME_H           120
#define QUEUE_LEN         1              /* drop frames, never block camera */

/* ── Camera intrinsics for XIAO OV2640 at 160×120 ─────────────────────
 * These are approximate — calibrate with a checkerboard for best accuracy.
 * fx = fy ≈ (sensor_fx / full_res_w) * capture_w
 * OV2640 full-res focal ≈ 2.8 mm, pixel pitch ≈ 2.2 µm → ~1273 px at 1600
 * Scaled to 160px: 1273 * (160/1600) = 127.3
 * cx/cy = half frame size
 * ─────────────────────────────────────────────────────────────────────── */
static const double CAM_FX = 127.3, CAM_FY = 127.3;
static const double CAM_CX = 80.0,  CAM_CY = 60.0;
/* Distortion: OV2640 has mild barrel — treat as zero for QQVGA ArUco use */
static const double DIST_COEFFS[5] = {0, 0, 0, 0, 0};


/* ── Internal queue: camera task → aruco task ───────────────────────────── */
static QueueHandle_t s_frame_queue = NULL;

/* We pass the raw buffer pointer + length, not the fb_t, to avoid holding
 * the camera DMA buffer across tasks. aruco_task owns a static copy buffer. */
typedef struct {
    uint8_t *buf;   /* points into the static copy buffer below */
    size_t   len;
} frame_msg_t;

static uint8_t s_frame_copy[FRAME_W * FRAME_H];  /* single static copy buffer */

/* ── Rotation vector → quaternion ──────────────────────────────────────── */
static void rvec_to_quat(const cv::Vec3d &rvec,
                          float *qx, float *qy, float *qz, float *qw)
{
    double angle = cv::norm(rvec);
    if (angle < 1e-9) { *qx=0; *qy=0; *qz=0; *qw=1; return; }
    double s = sin(angle * 0.5) / angle;
    *qx = (float)(rvec[0] * s);
    *qy = (float)(rvec[1] * s);
    *qz = (float)(rvec[2] * s);
    *qw = (float)cos(angle * 0.5);
}

/* ── ArUco detection task (Core 1) ────────────────────────────────────── */
static void aruco_task_fn(void *arg)
{
    /* Camera matrix and dist coeffs */
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3)
        << CAM_FX, 0, CAM_CX,
           0, CAM_FY, CAM_CY,
           0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat(1, 5, CV_64F, (void *)DIST_COEFFS);

    /* ArUco detector */
    auto dictionary = cv::aruco::getPredefinedDictionary(ARUCO_DICT);
    cv::aruco::DetectorParameters params;
    /* Loosen defaults slightly for low-res 160×120 */
    params.minMarkerPerimeterRate  = 0.03;
    params.maxMarkerPerimeterRate  = 4.0;
    params.polygonalApproxAccuracyRate = 0.08;
    params.minCornerDistanceRate   = 0.02;
    params.adaptiveThreshWinSizeMin  = 5;
    params.adaptiveThreshWinSizeMax  = 21;
    params.adaptiveThreshWinSizeStep = 4;
    cv::aruco::ArucoDetector detector(dictionary, params);

    /* 3D object points for solvePnP (marker corners in marker-local frame) */
    float h = MARKER_SIZE_M / 2.0f;
    std::vector<cv::Point3f> obj_pts = {
        {-h,  h, 0}, { h,  h, 0},
        { h, -h, 0}, {-h, -h, 0},
    };

    frame_msg_t msg;
    uint32_t frame_count = 0, detect_count = 0;

    ESP_LOGI(TAG, "ArUco task started on core %d", xPortGetCoreID());

    while (true) {
        /* Block until a frame arrives (camera task sends at ~10 Hz) */
        if (xQueueReceive(s_frame_queue, &msg, pdMS_TO_TICKS(200)) != pdTRUE) {
            /* Timeout — invalidate vision if no frames */
            vision_pose_valid = false;
            continue;
        }

        frame_count++;

        /* Wrap raw buffer in cv::Mat — no copy, zero heap alloc */
        cv::Mat frame(FRAME_H, FRAME_W, CV_8UC1, msg.buf);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        detector.detectMarkers(frame, corners, ids, rejected);

        if (ids.empty()) {
            /* No markers in this frame */
            continue;
        }

        detect_count++;

        /* Use the first detected marker (extend to multi-marker EKF later) */
        std::vector<cv::Vec3d> rvecs, tvecs;
        for (size_t i = 0; i < ids.size(); i++) {
            cv::Vec3d rvec, tvec;
            cv::solvePnP(obj_pts, corners[i], camera_matrix, dist_coeffs,
                         rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

            /* tvec is in camera frame (X right, Y down, Z forward).
             * Convert to NED: X_ned=Z_cam, Y_ned=X_cam, Z_ned=-Y_cam */
            float px = (float) tvec[2];   /* forward  → NED North */
            float py = (float) tvec[0];   /* right    → NED East  */
            float pz = (float)-tvec[1];   /* up       → NED -Down */

            float qx, qy, qz, qw;
            rvec_to_quat(rvec, &qx, &qy, &qz, &qw);

            /* Write shared volatile state — same vars vision_pose_callback uses */
            vp_x = px; vp_y = py; vp_z = pz;
            vp_qx = qx; vp_qy = qy; vp_qz = qz; vp_qw = qw;
            vision_pose_valid   = true;
            last_vision_pose_ms = esp_timer_get_time() / 1000;

            /* Forward to PX4 EKF2 directly over MAVLink UART — no ROS round-trip */
            if (vision_enabled) {
                float roll  = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy));
                float pitch = asinf( 2.0f*(qw*qy - qz*qx));
                float yaw   = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
                mav_send_vision_estimate(px, py, pz, roll, pitch, yaw);
            }

            ESP_LOGD(TAG, "ID:%d  pos=(%.3f, %.3f, %.3f)  frames:%lu det:%lu",
                     ids[i], px, py, pz, frame_count, detect_count);

            /* Only use the first marker per frame for now */
            break;
        }
    }
}

/* ── Camera feed task (Core 0, alongside existing main loop) ───────────── */
static void aruco_cam_task_fn(void *arg)
{
    ESP_LOGI(TAG, "ArUco cam task started on core %d", xPortGetCoreID());

    while (true) {
        /* Only grab frames when vision detection is active */
        if (!vision_enabled) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (fb->len == FRAME_W * FRAME_H) {
            /* Copy frame into static buffer, return DMA buffer immediately */
            memcpy(s_frame_copy, fb->buf, fb->len);
            esp_camera_fb_return(fb);

            frame_msg_t msg = { .buf = s_frame_copy, .len = fb->len };
            /* Overwrite any stale frame — QUEUE_LEN=1, don't block */
            xQueueOverwrite(s_frame_queue, &msg);
        } else {
            esp_camera_fb_return(fb);
        }

        /* ~10 FPS feed to detection — camera timer callback still runs at 10 Hz
         * for streaming, so this doesn't starve it */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ── Public init ───────────────────────────────────────────────────────── */
void aruco_task_start(void)
{
    s_frame_queue = xQueueCreate(QUEUE_LEN, sizeof(frame_msg_t));
    configASSERT(s_frame_queue);

    /* Cam feeder on Core 0, same core as the existing main loop */
    xTaskCreatePinnedToCore(aruco_cam_task_fn, "aruco_cam",
                            4096, NULL, 4, NULL, 0);

    /* Detector on Core 1, separate from micro-ROS task */
    xTaskCreatePinnedToCore(aruco_task_fn, "aruco_det",
                            8192, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "ArUco tasks launched");
}
