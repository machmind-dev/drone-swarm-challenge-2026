/* aruco_pose.cpp — Mach Mind ESP32S3 on-board ArUco world-pose estimator
 *
 * Adapted from ground-station-software/vision/aruco_node.py (ROS 2 Jazzy).
 * Runs on the drone itself — no ROS2, no image overlay, no debug stream.
 * Output goes to USB-Serial (idf.py monitor / any 115200 terminal).
 *
 * Per-frame output:
 *   [aruco]  M7  [known]    dist=2.34m  H=+5.2deg  V=-8.1deg
 *   [aruco]  M9  [known]    dist=3.10m  H=-12.4deg  V=-6.2deg
 *   [aruco]  M25 [UNKNOWN]  dist=1.80m  H=+0.1deg  V=-4.5deg
 *   [pose]   x=5.231  y=3.142  z=1.503  qw=0.9971 qx=0.0104 qy=-0.0198 qz=0.0502  (2 matched)
 *
 * Marker map: SDC26 competition arena, 25 markers, DICT_4X4_50, 0.5 m side.
 * Source: ground-station-software/vision/aruco_params.yaml
 *
 * Camera intrinsics: analytically derived from OV3660 assumed FOV.
 * For best accuracy calibrate with cv::calibrateCamera and update
 * CONFIG_VISION_POSE_FX / FY (not yet wired — leave as TODO).
 */

#include "aruco_pose.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "driver/gpio.h"

#ifdef EPS
#undef EPS
#endif
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect/aruco_detector.hpp"
#include "opencv2/calib3d.hpp"

/* Board pin selection — must precede boards.h */
#include "sdkconfig.h"
#if CONFIG_VISION_BOARD_OV3660_DEVKIT
  #define CAMERA_MODEL_OV3660_DEVKIT
#else
  #define CAMERA_MODEL_XIAO_ESP32S3
#endif
#include "boards.h"

static const char *TAG = "aruco_pose";

/* ── SDC26 arena marker map ─────────────────────────────────────────────────
 * Source: aruco_params.yaml, regulations Figure 3, Section 2.1.
 * Format: id, x(m), y(m), z(m), yaw_deg
 *   yaw = direction the marker face points INTO the arena:
 *     y=0  wall → 180° (faces +Y)   y=10 wall → 0°   (faces -Y)
 *     x=0  wall →  90° (faces +X)   x=20 wall → 270° (faces -X)
 * Arena dimensions: x=[0,20] y=[0,10] z=height above floor
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct { int id; float x, y, z, yaw_deg; } world_marker_t;

static const world_marker_t MARKER_MAP[] = {
    /* ID 0 — back wall centre, world origin */
    {  0, 10.0f,  5.0f, 2.0f,   0.0f },

    /* y=0 wall (faces +Y, yaw=180) */
    { 12,  4.0f,  0.0f, 4.0f, 180.0f },   /* upper */
    { 11,  4.0f,  0.0f, 2.0f, 180.0f },   /* lower */
    { 10,  8.0f,  0.0f, 4.0f, 180.0f },
    {  9,  8.0f,  0.0f, 2.0f, 180.0f },
    {  8, 12.0f,  0.0f, 4.0f, 180.0f },
    {  7, 12.0f,  0.0f, 2.0f, 180.0f },
    {  6, 16.0f,  0.0f, 4.0f, 180.0f },
    {  5, 16.0f,  0.0f, 2.0f, 180.0f },

    /* y=10 wall (faces -Y, yaw=0) */
    { 18,  4.0f, 10.0f, 4.0f,   0.0f },
    { 17,  4.0f, 10.0f, 2.0f,   0.0f },
    { 20,  8.0f, 10.0f, 4.0f,   0.0f },
    { 19,  8.0f, 10.0f, 2.0f,   0.0f },
    { 22, 12.0f, 10.0f, 4.0f,   0.0f },
    { 21, 12.0f, 10.0f, 2.0f,   0.0f },
    { 24, 16.0f, 10.0f, 4.0f,   0.0f },
    { 23, 16.0f, 10.0f, 2.0f,   0.0f },

    /* x=0 wall (faces +X, yaw=90) */
    { 16,  0.0f,  6.66f, 4.0f,  90.0f },
    { 15,  0.0f,  6.66f, 2.0f,  90.0f },
    { 14,  0.0f,  3.33f, 4.0f,  90.0f },
    { 13,  0.0f,  3.33f, 2.0f,  90.0f },

    /* x=20 wall (faces -X, yaw=270) */
    {  2, 20.0f,  6.66f, 4.0f, 270.0f },
    {  1, 20.0f,  6.66f, 2.0f, 270.0f },
    {  4, 20.0f,  3.33f, 4.0f, 270.0f },
    {  3, 20.0f,  3.33f, 2.0f, 270.0f },
};
#define NUM_MAP_MARKERS  (int)(sizeof(MARKER_MAP) / sizeof(MARKER_MAP[0]))

/* ── Marker map lookup ──────────────────────────────────────────────────── */
static const world_marker_t *find_marker(int id)
{
    for (int i = 0; i < NUM_MAP_MARKERS; i++)
        if (MARKER_MAP[i].id == id) return &MARKER_MAP[i];
    return NULL;
}

/* ── World-frame corners for a known marker ─────────────────────────────
 * Replicates aruco_node.py _corners_world(): rotate local TL/TR/BR/BL
 * by marker yaw around world Z, then translate to world position.
 * out[4] is filled in ArUco corner order: TL, TR, BR, BL.
 * ───────────────────────────────────────────────────────────────────────── */
static void marker_world_corners(const world_marker_t *m, float s,
                                  cv::Point3f out[4])
{
    float yaw_rad = m->yaw_deg * (float)M_PI / 180.0f;
    float cy = cosf(yaw_rad), sy = sinf(yaw_rad);

    /* Local corners before yaw rotation (x-right, y-up, z=0 plane) */
    const float local[4][2] = { {-s, s}, {s, s}, {s, -s}, {-s, -s} };

    for (int i = 0; i < 4; i++) {
        float lx = local[i][0], ly = local[i][1];
        out[i].x = cy * lx - sy * ly + m->x;
        out[i].y = sy * lx + cy * ly + m->y;
        out[i].z = m->z;   /* all 4 corners at same height — GCS convention */
    }
}

/* ── Rotation matrix → quaternion (qx, qy, qz, qw) ─────────────────────
 * Shepperd method, ported from aruco_node.py matrix_to_quaternion().
 * ───────────────────────────────────────────────────────────────────────── */
static void rot_to_quat(const cv::Mat &R,
                         float *qx, float *qy, float *qz, float *qw)
{
    double r00 = R.at<double>(0,0), r01 = R.at<double>(0,1), r02 = R.at<double>(0,2);
    double r10 = R.at<double>(1,0), r11 = R.at<double>(1,1), r12 = R.at<double>(1,2);
    double r20 = R.at<double>(2,0), r21 = R.at<double>(2,1), r22 = R.at<double>(2,2);
    double trace = r00 + r11 + r22;
    double x, y, z, w;

    if (trace > 0.0) {
        double s = 0.5 / sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (r21 - r12) * s;
        y = (r02 - r20) * s;
        z = (r10 - r01) * s;
    } else if (r00 > r11 && r00 > r22) {
        double s = 2.0 * sqrt(1.0 + r00 - r11 - r22);
        w = (r21 - r12) / s;
        x = 0.25 * s;
        y = (r01 + r10) / s;
        z = (r02 + r20) / s;
    } else if (r11 > r22) {
        double s = 2.0 * sqrt(1.0 + r11 - r00 - r22);
        w = (r02 - r20) / s;
        x = (r01 + r10) / s;
        y = 0.25 * s;
        z = (r12 + r21) / s;
    } else {
        double s = 2.0 * sqrt(1.0 + r22 - r00 - r11);
        w = (r10 - r01) / s;
        x = (r02 + r20) / s;
        y = (r12 + r21) / s;
        z = 0.25 * s;
    }

    *qx = (float)x; *qy = (float)y; *qz = (float)z; *qw = (float)w;
}

/* ── Resolution selection ───────────────────────────────────────────────── */
#if   CONFIG_VISION_POSE_RES_QQVGA
  #define POSE_W          160
  #define POSE_H          120
  #define POSE_FRAMESIZE  FRAMESIZE_QQVGA
#elif CONFIG_VISION_POSE_RES_HVGA
  #define POSE_W          480
  #define POSE_H          320
  #define POSE_FRAMESIZE  FRAMESIZE_HVGA
#else  /* default: QVGA */
  #define POSE_W          320
  #define POSE_H          240
  #define POSE_FRAMESIZE  FRAMESIZE_QVGA
#endif

#ifndef CONFIG_VISION_POSE_MARKER_SIZE_CM
#define CONFIG_VISION_POSE_MARKER_SIZE_CM 50   /* default if built in bench mode */
#endif
#define POSE_MARKER_SIZE_M  (CONFIG_VISION_POSE_MARKER_SIZE_CM / 100.0f)

/* ── Public entry point ─────────────────────────────────────────────────── */
void aruco_pose_start(void)
{
    ESP_LOGI(TAG, "=== Mach Mind ArUco Pose Estimator ===");
    ESP_LOGI(TAG, "resolution=%dx%d  marker=%.2fm  map=%d markers  dict=%d",
             POSE_W, POSE_H, POSE_MARKER_SIZE_M,
             NUM_MAP_MARKERS, CONFIG_VISION_ARUCO_DICT);

    /* ── Camera init ──────────────────────────────────────────────────────── */
    camera_config_t cfg = {
        .pin_pwdn     = CAMERA_PIN_PWDN,
        .pin_reset    = CAMERA_PIN_RESET,
        .pin_xclk     = CAMERA_PIN_XCLK,
        .pin_sccb_sda = CAMERA_PIN_SIOD,
        .pin_sccb_scl = CAMERA_PIN_SIOC,
        .pin_d7 = CAMERA_PIN_D7, .pin_d6 = CAMERA_PIN_D6,
        .pin_d5 = CAMERA_PIN_D5, .pin_d4 = CAMERA_PIN_D4,
        .pin_d3 = CAMERA_PIN_D3, .pin_d2 = CAMERA_PIN_D2,
        .pin_d1 = CAMERA_PIN_D1, .pin_d0 = CAMERA_PIN_D0,
        .pin_vsync    = CAMERA_PIN_VSYNC,
        .pin_href     = CAMERA_PIN_HREF,
        .pin_pclk     = CAMERA_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size   = POSE_FRAMESIZE,
        .jpeg_quality = 12,
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    ESP_ERROR_CHECK(esp_camera_init(&cfg));

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        sensor->set_sharpness(sensor, 2);
        sensor->set_contrast(sensor, 2);
        sensor->set_saturation(sensor, 0);
        if (sensor->id.PID == OV3660_PID) {
            /* OV3660 ships with horizontal mirror on by default in esp32-camera.
             * A mirrored ArUco marker has its bit pattern reversed — no rotation
             * can recover it, so detectMarkers returns nothing.
             * vflip=1 corrects the vertical orientation on M5Stack-style mounts. */
            sensor->set_hmirror(sensor, 0);
            sensor->set_vflip(sensor, 1);
            sensor->set_denoise(sensor, 0);
            ESP_LOGI(TAG, "OV3660 — hmirror=off vflip=on DNR=off sharpness=2");
        }
    }

    /* ── Camera intrinsics ────────────────────────────────────────────────
     * Base: fx=fy=140 px at QQVGA 160×120 (GCS aruco_node.py fallback).
     * Scaled to current resolution.  Replace with calibrated values for
     * production; error here translates directly to pose error.
     * ───────────────────────────────────────────────────────────────────── */
    double fx = (double)POSE_W  * (140.0 / 160.0);
    double fy = (double)POSE_H  * (140.0 / 120.0);
    double cx = (double)POSE_W  / 2.0;
    double cy = (double)POSE_H  / 2.0;
    cv::Mat K = (cv::Mat_<double>(3, 3)
                 << fx,  0, cx,
                     0, fy, cy,
                     0,  0,  1);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);

    ESP_LOGI(TAG, "Intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx, fy, cx, cy);

    /* ── Single-marker local object points (TL, TR, BR, BL) ────────────── */
    float s = POSE_MARKER_SIZE_M / 2.0f;
    std::vector<cv::Point3f> single_obj = {
        {-s,  s, 0.0f}, { s,  s, 0.0f},
        { s, -s, 0.0f}, {-s, -s, 0.0f},
    };

    /* ── ArUco detector — tuned for speed on QQVGA/QVGA ────────────────────
     * adaptiveThreshWinSizeMax=7 (2 window sizes instead of 6) halves the
     * adaptive threshold work, which dominates detection time at low res.
     * minMarkerPerimeterRate=0.10 skips blobs smaller than 10% of image
     * perimeter, cutting contour work on noise.
     * ─────────────────────────────────────────────────────────────────── */
    auto dict = cv::aruco::getPredefinedDictionary(
        (cv::aruco::PredefinedDictionaryType)CONFIG_VISION_ARUCO_DICT);
    cv::aruco::DetectorParameters params;
    /* minMarkerPerimeterRate=0.20: rejects blobs with perimeter < 32px
     * (side < 8px), covering detection out to ~8m for 50cm markers at QQVGA.
     * 0.35 was too tight — at 4m the margin against focal-length uncertainty
     * was too small and markers were dropped. */
    params.minMarkerPerimeterRate      = 0.20f;
    params.maxMarkerPerimeterRate      = 4.0f;
    params.polygonalApproxAccuracyRate = 0.08f;
    params.minCornerDistanceRate       = 0.02f;
    /* Single adaptive threshold window: size-3 adds nothing for 50cm markers
     * at QQVGA — their minimum useful side is ~14px. One pass ~halves the
     * most expensive step in the detector. */
    params.adaptiveThreshWinSizeMin    = 7;
    params.adaptiveThreshWinSizeMax    = 7;
    params.adaptiveThreshWinSizeStep   = 1;
    params.cornerRefinementMethod      = cv::aruco::CORNER_REFINE_NONE;
    params.errorCorrectionRate         = 0.6f;
    cv::aruco::ArucoDetector detector(dict, params);

    /* ── DRAM fast-path buffer ─────────────────────────────────────────── */
    size_t frame_bytes = (size_t)(POSE_W * POSE_H);
    uint8_t *fast_buf = (uint8_t *)heap_caps_malloc(
                            frame_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (fast_buf)
        ESP_LOGI(TAG, "DRAM fast_buf: %zu B", frame_bytes);
    else
        ESP_LOGW(TAG, "DRAM fast_buf failed — PSRAM slow path (~3× slower)");

    /* ── Detection loop ─────────────────────────────────────────────────── */
    uint64_t frame_n = 0;
    uint64_t t_fps_start = esp_timer_get_time(); (void)t_fps_start;

    /* Pre-allocate vectors outside loop to avoid per-frame heap alloc/free */
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    ids.reserve(8);
    corners.reserve(8);
    rejected.reserve(64);

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* Copy to DRAM fast-path, return frame buffer immediately */
        uint8_t *pixels = fast_buf ? fast_buf : fb->buf;
        if (fast_buf)
            memcpy(fast_buf, fb->buf, frame_bytes);
        esp_camera_fb_return(fb);
        frame_n++;

        uint64_t t0 = esp_timer_get_time();
        cv::Mat frame(POSE_H, POSE_W, CV_8UC1, pixels);

        /* ── Detect ─────────────────────────────────────────────────────── */
        ids.clear();
        corners.clear();
        rejected.clear();
        detector.detectMarkers(frame, corners, ids, rejected);

        /* ── Rolling single-line status ─────────────────────────────────────
         * \r returns to column 0; \033[K erases to end of line.
         * No \n so every frame overwrites the previous line in the terminal.
         * A static char buf avoids per-frame heap allocation.
         * ─────────────────────────────────────────────────────────────── */

        if (ids.empty()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* ── Per-marker: distance via single-marker solvePnP ─────────────
         * tvec in camera frame: X=right, Y=down, Z=forward.
         * dist = Euclidean distance from camera to marker centre.
         * ─────────────────────────────────────────────────────────────── */
        static char mbuf[160];
        int mpos = 0;

        for (int i = 0; i < (int)ids.size(); i++) {
            int mid = ids[i];
            std::vector<cv::Point2f> &c = corners[i];

            cv::Mat rvec_s, tvec_s;
            cv::solvePnP(single_obj, c, K, D,
                         rvec_s, tvec_s, false, cv::SOLVEPNP_IPPE_SQUARE);

            float tx = (float)tvec_s.at<double>(0);
            float ty = (float)tvec_s.at<double>(1);
            float tz = (float)tvec_s.at<double>(2);
            float dist = sqrtf(tx*tx + ty*ty + tz*tz);

            mpos += snprintf(mbuf + mpos, sizeof(mbuf) - mpos,
                             "M%d:%.2fm ", mid, dist);
        }

        /* Trim trailing space */
        if (mpos > 0 && mbuf[mpos - 1] == ' ') mbuf[--mpos] = '\0';

        printf("%s\n", mbuf);
        fflush(stdout);

        /* Yield so IDLE1 can reset the task watchdog */
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Unreachable — but tidy up if ever exited */
    if (fast_buf) heap_caps_free(fast_buf);
    esp_camera_deinit();
}
