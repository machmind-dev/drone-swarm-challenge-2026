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
#include "esp_heap_caps.h"
#include "esp_attr.h"
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

/* ── Resolution constants ──────────────────────────────────────────────────
 * CAM_W/H: what the OV2640 produces (QQVGA = 160×120).
 * DET_W/H: what the detector receives (2× downsampled = 80×60).
 *
 * Downsampling 4× fewer pixels makes adaptiveThreshold and warpPerspective
 * ~4× faster, keeping detectMarkers well under the 5-second task-WDT window
 * even when all pixel data lives in PSRAM.
 * ─────────────────────────────────────────────────────────────────────── */
#define CAM_W   160
#define CAM_H   120
#define DET_W    80   /* detection resolution (2× downsampled from 160×120) */
#define DET_H    60

#define ARUCO_DICT        cv::aruco::DICT_4X4_50
#define MARKER_SIZE_M     0.50f          /* physical marker side length, metres */
#define QUEUE_LEN         1              /* drop frames, never block camera */

/* ── Camera intrinsics for XIAO OV2640 at 80×60 ───────────────────────────
 * Halved from 160×120 values: fx/fy scale linearly with resolution,
 * cx/cy are the new image half-size.
 * ─────────────────────────────────────────────────────────────────────── */
static const double CAM_FX = 63.65, CAM_FY = 63.65;
static const double CAM_CX = 40.0,  CAM_CY = 30.0;
/* Distortion: OV2640 has mild barrel — treat as zero for this resolution */
static const double DIST_COEFFS[5] = {0, 0, 0, 0, 0};


/* ── Internal queue: camera task → aruco task ───────────────────────────── */
static QueueHandle_t s_frame_queue = NULL;

/* Static PSRAM stacks for both ArUco tasks.
 * At boot, only ~22 KB of internal DRAM is free — even a 4 KB cam task stack
 * is too expensive.  EXT_RAM_BSS_ATTR forces these into .ext_ram.bss (PSRAM)
 * at link time; the linker.lf fragment only covers OpenCV libs, not libmain.a.
 * xTaskCreateStaticPinnedToCore accepts PSRAM stacks when
 * CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y. */
EXT_RAM_BSS_ATTR static StackType_t  s_cam_stack[4096 / sizeof(StackType_t)];
static StaticTask_t s_cam_tcb;   /* TCB must be in internal DRAM (xPortCheckValidTCBMem) */
EXT_RAM_BSS_ATTR static StackType_t  s_det_stack[32768 / sizeof(StackType_t)];
static StaticTask_t s_det_tcb;   /* TCB must be in internal DRAM (xPortCheckValidTCBMem) */

/* Frame copy buffer (PSRAM): cam task writes 80×60 downsampled pixels here.
 * Only 4800 bytes — small enough to copy quickly to the fast internal buffer
 * before each detectMarkers call. */
typedef struct {
    uint8_t *buf;   /* points into s_frame_copy */
    size_t   len;
} frame_msg_t;

EXT_RAM_BSS_ATTR static uint8_t s_frame_copy[DET_W * DET_H];

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
    ESP_LOGI(TAG, "ArUco det task entry, core=%d stack=%u",
             xPortGetCoreID(), (unsigned)uxTaskGetStackHighWaterMark(NULL));

    /* Fast frame buffer in internal DRAM.
     * OpenCV's adaptiveThreshold/warpPerspective loop over every pixel with
     * random access — each PSRAM access has ~10× the latency of internal SRAM.
     * Copying the 4800-byte frame to internal DRAM before calling detectMarkers
     * reduces per-call time from ~5 s to ~0.5–1 s. */
    uint8_t *fast_frame = (uint8_t *)heap_caps_malloc(DET_W * DET_H,
                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!fast_frame)
        ESP_LOGW(TAG, "fast_frame alloc failed — using PSRAM (slow)");

    /* Camera matrix and dist coeffs */
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3)
        << CAM_FX, 0, CAM_CX,
           0, CAM_FY, CAM_CY,
           0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat(1, 5, CV_64F, (void *)DIST_COEFFS);

    /* ArUco detector — base params tuned for 80×60 */
    auto dictionary = cv::aruco::getPredefinedDictionary(ARUCO_DICT);
    cv::aruco::DetectorParameters params;
    params.minMarkerPerimeterRate      = 0.10;  /* ≥8 px perimeter — rejects noise clusters */
    params.maxMarkerPerimeterRate      = 4.0;
    params.polygonalApproxAccuracyRate = 0.08;
    params.minCornerDistanceRate       = 0.02;
    params.adaptiveThreshWinSizeMin    = 3;
    params.adaptiveThreshWinSizeMax    = 15;
    params.adaptiveThreshWinSizeStep   = 4;

    /* Per-sensor tuning.
     * OV3660: 2048×1536 → 80×60 aggressive downscaling blurs marker edges and
     * adds noise.  Tighter detection params alone hurt real detections more than
     * they help — root fix is a 3×3 Gaussian blur applied before detectMarkers.
     * Blur smooths noise so adaptive threshold finds real edges, not noise spikes.
     * Detector params kept identical to OV2640 so the full correction budget is
     * available for markers blurred by downscaling.
     * OV2640: no blur needed — sensor is clean at QQVGA. */
    sensor_t *cam_sensor = esp_camera_sensor_get();
    const bool is_ov3660 = (cam_sensor && cam_sensor->id.PID == OV3660_PID);
    params.errorCorrectionRate = 0.6f;
    ESP_LOGI(TAG, "ArUco: %s params (errCorr=%.1f blur=%s)",
             is_ov3660 ? "OV3660" : "OV2640",
             params.errorCorrectionRate,
             is_ov3660 ? "3x3" : "off");

    cv::aruco::ArucoDetector detector(dictionary, params);

    /* 3D object points for solvePnP (marker corners in marker-local frame) */
    float h = MARKER_SIZE_M / 2.0f;
    std::vector<cv::Point3f> obj_pts = {
        {-h,  h, 0}, { h,  h, 0},
        { h, -h, 0}, {-h, -h, 0},
    };

    frame_msg_t msg;
    uint32_t frame_count = 0, detect_count = 0;

    ESP_LOGI(TAG, "ArUco det task started on core %d", xPortGetCoreID());

    while (true) {
        /* Block until a frame arrives */
        if (xQueueReceive(s_frame_queue, &msg, pdMS_TO_TICKS(200)) != pdTRUE) {
            vision_pose_valid = false;
            continue;
        }

        frame_count++;

        /* Copy PSRAM frame → fast internal DRAM, then wrap in cv::Mat.
         * If allocation failed, fall back to the PSRAM buffer directly. */
        uint8_t *pixel_buf = fast_frame ? fast_frame : msg.buf;
        if (fast_frame)
            memcpy(fast_frame, msg.buf, DET_W * DET_H);

        cv::Mat frame(DET_H, DET_W, CV_8UC1, pixel_buf);

        /* OV3660: 3×3 Gaussian blur before detection.
         * Removes high-frequency noise introduced by 2048×1536 → 80×60 downscaling
         * so adaptive threshold finds real marker edges, not noise spikes.
         * In-place on the same Mat — no extra allocation needed. */
        if (is_ov3660)
            cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        detector.detectMarkers(frame, corners, ids, rejected);

        /* Brief yield after heavy computation so IDLE1 can reset its watchdog.
         * detectMarkers at 80×60 in internal DRAM takes ~300–600 ms.
         * 1 ms yield every frame keeps IDLE1 well within the 30 s WDT window. */
        vTaskDelay(pdMS_TO_TICKS(1));

        if (ids.empty()) {
            if (frame_count % 5 == 0)
                ESP_LOGI(TAG, "No marker — frame %lu", frame_count);
            continue;
        }

        detect_count++;

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

            vp_x = px; vp_y = py; vp_z = pz;
            vp_qx = qx; vp_qy = qy; vp_qz = qz; vp_qw = qw;
            vision_pose_valid   = true;
            last_vision_pose_ms = esp_timer_get_time() / 1000;

            if (vision_enabled) {
                float roll  = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy));
                float pitch = asinf( 2.0f*(qw*qy - qz*qx));
                float yaw   = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
                mav_send_vision_estimate(px, py, pz, roll, pitch, yaw);
            }

            /* Viewing angles: where the marker appears in the camera frame.
             * tvec is in camera frame (X right, Y down, Z forward). */
            float dist    = sqrtf(px*px + py*py + pz*pz);
            float v_yaw   = atan2f((float)tvec[0], (float)tvec[2]) * (180.0f / (float)M_PI);
            float v_pitch = atan2f(-(float)tvec[1], (float)tvec[2]) * (180.0f / (float)M_PI);

            ESP_LOGI(TAG, "ID:%d  dist=%.2fm  yaw=%+.1f°  pitch=%+.1f°  det=%lu/%lu",
                     ids[i], dist, v_yaw, v_pitch, detect_count, frame_count);

            break;  /* use first marker only */
        }
    }
}

/* ── Camera feed task (Core 0) ─────────────────────────────────────────── */
static void aruco_cam_task_fn(void *arg)
{
    ESP_LOGI(TAG, "ArUco cam task started on core %d", xPortGetCoreID());

    uint32_t drain_count = 0;
    ESP_LOGI(TAG, "ArUco cam drain loop starting");

    while (true) {
        /* Always drain at full camera rate.  fb_get blocks until a frame is
         * ready, so this loop naturally matches the sensor frame rate (~25 FPS).
         * Draining slower causes DMA OVF and corrupted half-frames. */
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        drain_count++;
        if (drain_count % 100 == 0)
            ESP_LOGI(TAG, "Cam drain: count=%lu len=%u vision=%d",
                     drain_count, (unsigned)fb->len, (int)vision_enabled);

        /* Feed detector at ~8 FPS (every 3rd frame at ~25 FPS) when enabled.
         * Downsample 160×120 → 80×60 by taking every other pixel in x and y.
         * This reduces detectMarkers cost by 4× and fits in 4800 bytes. */
        if (vision_enabled && fb->len == (size_t)(CAM_W * CAM_H) && (drain_count % 3) == 0) {
            const uint8_t *src = (const uint8_t *)fb->buf;
            uint8_t *dst = s_frame_copy;
            for (int y = 0; y < DET_H; y++)
                for (int x = 0; x < DET_W; x++)
                    dst[y * DET_W + x] = src[(y * 2) * CAM_W + (x * 2)];
            esp_camera_fb_return(fb);

            frame_msg_t msg = { .buf = s_frame_copy, .len = (size_t)(DET_W * DET_H) };
            xQueueOverwrite(s_frame_queue, &msg);
        } else {
            esp_camera_fb_return(fb);
        }
        /* No vTaskDelay — let fb_get block naturally to prevent OVF */
    }
}

/* ── Public init ───────────────────────────────────────────────────────── */
void aruco_task_start(void)
{
    s_frame_queue = xQueueCreate(QUEUE_LEN, sizeof(frame_msg_t));
    configASSERT(s_frame_queue);

    ESP_LOGI(TAG, "Free internal DRAM before task create: %u B",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    /* Cam feeder on Core 0 — priority 4 (below micro_ros prio 5).
     * fb_count=8 gives 320 ms OVF tolerance without needing elevated priority. */
    TaskHandle_t cam_handle = xTaskCreateStaticPinnedToCore(
        aruco_cam_task_fn, "aruco_cam",
        sizeof(s_cam_stack) / sizeof(StackType_t),
        NULL, 4,
        s_cam_stack, &s_cam_tcb, 0);
    if (!cam_handle)
        ESP_LOGE(TAG, "aruco_cam task creation FAILED");

    /* Detector on Core 1 — PSRAM stack, internal DRAM TCB. */
    TaskHandle_t det_handle = xTaskCreateStaticPinnedToCore(
        aruco_task_fn, "aruco_det",
        sizeof(s_det_stack) / sizeof(StackType_t),
        NULL, 4,
        s_det_stack, &s_det_tcb, 1);
    if (!det_handle)
        ESP_LOGE(TAG, "aruco_det task creation FAILED");

    ESP_LOGI(TAG, "ArUco tasks launched (cam=%s det=%s)",
             cam_handle ? "OK" : "FAIL",
             det_handle ? "OK" : "FAIL");
}
