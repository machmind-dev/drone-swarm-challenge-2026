/* aruco_pose.cpp — Mach Mind ESP32-P4 on-board ArUco world-pose estimator
 *
 * Target : Waveshare ESP32-P4 WiFi6 Dev Board + OV5647 (MIPI-CSI 2-lane)
 * IDF    : 5.3+   components: espressif/esp_cam_sensor, esp_driver_cam, esp_driver_isp
 *
 * Camera API: esp_cam_ctlr_csi (low-level, matches official IDF mipi_isp_dsi example).
 *   Sensor init → ISP → CSI controller → esp_cam_ctlr_receive() per frame.
 *   No esp_video / V4L2 layer — avoids esp_video_buffer_element PSRAM init bug.
 *
 * Capture: OV5647 RAW8 800×800@50fps → ISP demosaick → RGB565 in PSRAM frame buffer.
 * Detection: center-crop 800×800→800×600, resize uniformly→320×240, ArUco detect.
 *
 * Per-frame output (USB-Serial):
 *   M7:2.34m M9:3.10m
 *
 * Intrinsics (OV5647, 800×800@50fps mode, 2×2 binned):
 *   Sensor: 2592×1944, 1.4µm pixels. Lens: 2.8mm nominal, FOV(H)=72°.
 *   800×800 mode: X_start=500..2623, 2×2 subsampled → effective pitch=2.8µm.
 *   f_eff from FOV(H)=72°: ~2497µm → fx_800 ≈ 892 px (at 800×800 capture).
 *   Capture is center-cropped 800×800→800×600 then scaled uniformly →320×240.
 *   fx = fy = 892 × (320/800) ≈ 357 px. Calibrate with cv::calibrateCamera
 *   for production accuracy.
 */

#include "aruco_pose.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"

/* Camera sensor init — SCCB bus + OV5647 detect/format/stream */
#include "driver/i2c_master.h"
#include "esp_sccb_intf.h"
extern "C" {
#include "esp_sccb_i2c.h"  /* no extern C in header */
}
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"

/* CSI controller and ISP */
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"

#ifdef EPS
#undef EPS
#endif
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect/aruco_detector.hpp"
#include "opencv2/calib3d.hpp"

#include "sdkconfig.h"
#include "boards.h"

static const char *TAG = "aruco_pose";

/* ── SDC26 arena marker map ─────────────────────────────────────────────────
 * Source: aruco_params.yaml, regulations Figure 3, Section 2.1.
 * Arena: x=[0,20] y=[0,10] z=height above floor (all in metres).
 * yaw = direction the marker face points INTO the arena.
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct { int id; float x, y, z, yaw_deg; } world_marker_t;

static const world_marker_t MARKER_MAP[] = {
    {  0, 10.0f,  5.0f, 2.0f,   0.0f },  /* back wall centre, world origin */

    /* y=0 wall (faces +Y, yaw=180) */
    { 12,  4.0f,  0.0f, 4.0f, 180.0f }, { 11,  4.0f,  0.0f, 2.0f, 180.0f },
    { 10,  8.0f,  0.0f, 4.0f, 180.0f }, {  9,  8.0f,  0.0f, 2.0f, 180.0f },
    {  8, 12.0f,  0.0f, 4.0f, 180.0f }, {  7, 12.0f,  0.0f, 2.0f, 180.0f },
    {  6, 16.0f,  0.0f, 4.0f, 180.0f }, {  5, 16.0f,  0.0f, 2.0f, 180.0f },

    /* y=10 wall (faces -Y, yaw=0) */
    { 18,  4.0f, 10.0f, 4.0f,   0.0f }, { 17,  4.0f, 10.0f, 2.0f,   0.0f },
    { 20,  8.0f, 10.0f, 4.0f,   0.0f }, { 19,  8.0f, 10.0f, 2.0f,   0.0f },
    { 22, 12.0f, 10.0f, 4.0f,   0.0f }, { 21, 12.0f, 10.0f, 2.0f,   0.0f },
    { 24, 16.0f, 10.0f, 4.0f,   0.0f }, { 23, 16.0f, 10.0f, 2.0f,   0.0f },

    /* x=0 wall (faces +X, yaw=90) */
    { 16,  0.0f,  6.66f, 4.0f,  90.0f }, { 15,  0.0f,  6.66f, 2.0f,  90.0f },
    { 14,  0.0f,  3.33f, 4.0f,  90.0f }, { 13,  0.0f,  3.33f, 2.0f,  90.0f },

    /* x=20 wall (faces -X, yaw=270) */
    {  2, 20.0f,  6.66f, 4.0f, 270.0f }, {  1, 20.0f,  6.66f, 2.0f, 270.0f },
    {  4, 20.0f,  3.33f, 4.0f, 270.0f }, {  3, 20.0f,  3.33f, 2.0f, 270.0f },
};
#define NUM_MAP_MARKERS  (int)(sizeof(MARKER_MAP) / sizeof(MARKER_MAP[0]))

static const world_marker_t *find_marker(int id)
{
    for (int i = 0; i < NUM_MAP_MARKERS; i++)
        if (MARKER_MAP[i].id == id) return &MARKER_MAP[i];
    return NULL;
}

/* ── World-frame corners for a known marker ─────────────────────────────────
 * Replicates aruco_node.py _corners_world(): rotate TL/TR/BR/BL by marker
 * yaw around world Z, then translate to world position.
 * ─────────────────────────────────────────────────────────────────────────── */
static void marker_world_corners(const world_marker_t *m, float s,
                                  cv::Point3f out[4])
{
    float yaw_rad = m->yaw_deg * (float)M_PI / 180.0f;
    float cy = cosf(yaw_rad), sy = sinf(yaw_rad);
    const float local[4][2] = { {-s, s}, {s, s}, {s, -s}, {-s, -s} };
    for (int i = 0; i < 4; i++) {
        float lx = local[i][0], ly = local[i][1];
        out[i].x = cy * lx - sy * ly + m->x;
        out[i].y = sy * lx + cy * ly + m->y;
        out[i].z = m->z;
    }
}

/* ── Rotation matrix → quaternion (Shepperd method) ────────────────────── */
static void rot_to_quat(const cv::Mat &R,
                         float *qx, float *qy, float *qz, float *qw)
{
    double r00=R.at<double>(0,0), r01=R.at<double>(0,1), r02=R.at<double>(0,2);
    double r10=R.at<double>(1,0), r11=R.at<double>(1,1), r12=R.at<double>(1,2);
    double r20=R.at<double>(2,0), r21=R.at<double>(2,1), r22=R.at<double>(2,2);
    double trace = r00 + r11 + r22;
    double x, y, z, w;
    if (trace > 0.0) {
        double s = 0.5 / sqrt(trace + 1.0);
        w = 0.25/s; x=(r21-r12)*s; y=(r02-r20)*s; z=(r10-r01)*s;
    } else if (r00 > r11 && r00 > r22) {
        double s = 2.0 * sqrt(1.0 + r00 - r11 - r22);
        w=(r21-r12)/s; x=0.25*s; y=(r01+r10)/s; z=(r02+r20)/s;
    } else if (r11 > r22) {
        double s = 2.0 * sqrt(1.0 + r11 - r00 - r22);
        w=(r02-r20)/s; x=(r01+r10)/s; y=0.25*s; z=(r12+r21)/s;
    } else {
        double s = 2.0 * sqrt(1.0 + r22 - r00 - r11);
        w=(r10-r01)/s; x=(r02+r20)/s; y=(r12+r21)/s; z=0.25*s;
    }
    *qx=(float)x; *qy=(float)y; *qz=(float)z; *qw=(float)w;
}

/* ── Capture resolution — OV5647 MIPI RAW8 800x800@50fps ────────────────── */
#define POSE_W   800
#define POSE_H   800
#define POSE_BPP 2          /* ISP outputs RGB565 = 2 bytes/pixel */

/* ── Requested detection resolution ─────────────────────────────────────── */
#if   CONFIG_VISION_POSE_RES_QQVGA
  #define POSE_REQ_W  160
  #define POSE_REQ_H  120
#elif CONFIG_VISION_POSE_RES_HVGA
  #define POSE_REQ_W  480
  #define POSE_REQ_H  320
#else  /* default: QVGA */
  #define POSE_REQ_W  320
  #define POSE_REQ_H  240
#endif

#ifndef CONFIG_VISION_POSE_MARKER_SIZE_CM
#define CONFIG_VISION_POSE_MARKER_SIZE_CM 50
#endif
#define POSE_MARKER_SIZE_M  (CONFIG_VISION_POSE_MARKER_SIZE_CM / 100.0f)

/* OV5647 format string — must match esp_cam_sensor format name exactly */
#define OV5647_FORMAT_NAME  "MIPI_2lane_24Minput_RAW8_800x800_50fps"

/* Lane bit rate for 800x800@50fps */
#define CSI_LANE_BITRATE_MBPS  200

/* ── ISR-safe callback: give the fixed PSRAM frame buffer to every DMA trans  */
static bool IRAM_ATTR s_on_get_new_vb(esp_cam_ctlr_handle_t handle,
                                       esp_cam_ctlr_trans_t *trans,
                                       void *user_data)
{
    const esp_cam_ctlr_trans_t *fixed = (const esp_cam_ctlr_trans_t *)user_data;
    trans->buffer = fixed->buffer;
    trans->buflen = fixed->buflen;
    return false;
}

/* ── Public entry point ──────────────────────────────────────────────────── */
void aruco_pose_start(void)
{
    ESP_LOGI(TAG, "=== Mach Mind ArUco Pose Estimator -- ESP32-P4 + OV5647 ===");
    ESP_LOGI(TAG, "capture=%dx%d  detect=%dx%d  marker=%.2fm  map=%d  dict=%d",
             POSE_W, POSE_H, POSE_REQ_W, POSE_REQ_H,
             POSE_MARKER_SIZE_M, NUM_MAP_MARKERS, CONFIG_VISION_ARUCO_DICT);

    /* ── I2C master bus for SCCB ─────────────────────────────────────────── */
    i2c_master_bus_config_t i2c_bus_cfg = {};
    i2c_bus_cfg.i2c_port               = I2C_NUM_0;
    i2c_bus_cfg.sda_io_num             = (gpio_num_t)CAMERA_SCCB_SDA;
    i2c_bus_cfg.scl_io_num             = (gpio_num_t)CAMERA_SCCB_SCL;
    i2c_bus_cfg.clk_source             = I2C_CLK_SRC_DEFAULT;
    i2c_bus_cfg.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t i2c_bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));

    /* ── Detect OV5647 via sensor auto-detect array ─────────────────────── *
     * Iterates over all ESP_CAM_SENSOR_DETECT_FN entries placed in the
     * .esp_cam_sensor_detect_fn linker section by esp_cam_sensor component.
     * CONFIG_CAMERA_OV5647_AUTO_DETECT_MIPI_INTERFACE_SENSOR=y forces the
     * linker to include the ov5647_detect symbol from the component archive. */
    esp_cam_sensor_config_t sensor_cfg = {};
    sensor_cfg.xclk_pin  = (gpio_num_t)(-1);
    sensor_cfg.reset_pin = (gpio_num_t)CAMERA_RESET_PIN;
    sensor_cfg.pwdn_pin  = (gpio_num_t)CAMERA_PWDN_PIN;
    esp_cam_sensor_device_t *cam = NULL;

    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; ++p) {

        sccb_i2c_config_t sccb_cfg = {};
        sccb_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        sccb_cfg.device_address  = p->sccb_addr;
        sccb_cfg.scl_speed_hz    = 200000;
        ESP_ERROR_CHECK(sccb_new_i2c_io(i2c_bus, &sccb_cfg, &sensor_cfg.sccb_handle));
        sensor_cfg.sensor_port = p->port;
        cam = (*(p->detect))(&sensor_cfg);
        if (cam) {
            ESP_LOGI(TAG, "OV5647 detected (sccb_addr=0x%02x)", p->sccb_addr);
            break;
        }
        /* Not found on this address -- release and try next */
        esp_sccb_del_i2c_io(sensor_cfg.sccb_handle);
    }

    if (!cam) {
        ESP_LOGE(TAG, "No camera sensor found -- check SCCB wiring (SDA=%d SCL=%d)",
                 CAMERA_SCCB_SDA, CAMERA_SCCB_SCL);
        return;
    }

    /* Log available formats, then select 800x800 RAW8 50fps */
    esp_cam_sensor_format_array_t fmt_array = {};
    esp_cam_sensor_query_format(cam, &fmt_array);
    ESP_LOGI(TAG, "Sensor formats (%" PRIu32 "):", fmt_array.count);
    const esp_cam_sensor_format_t *target_fmt = NULL;
    for (int i = 0; i < fmt_array.count; i++) {
        ESP_LOGI(TAG, "  [%d] %s", i, fmt_array.format_array[i].name);
        if (!strcmp(fmt_array.format_array[i].name, OV5647_FORMAT_NAME))
            target_fmt = &fmt_array.format_array[i];
    }
    if (!target_fmt) {
        ESP_LOGE(TAG, "Format '%s' not found -- check sdkconfig", OV5647_FORMAT_NAME);
        return;
    }
    ESP_ERROR_CHECK(esp_cam_sensor_set_format(cam, target_fmt));
    ESP_LOGI(TAG, "Sensor format set: %s", OV5647_FORMAT_NAME);

    int stream_on = 1;
    ESP_ERROR_CHECK(esp_cam_sensor_ioctl(cam, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_on));

    /* ── ISP: demosaick RAW8 → RGB565 ───────────────────────────────────── */
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz                 = 80 * 1000 * 1000,
        .input_data_source      = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type  = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet  = true,   /* OV5647 with LINESYNC_ENABLE sends LS packets */
        .has_line_end_packet    = false,
        .h_res                  = POSE_W,
        .v_res                  = POSE_H,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_cfg, &isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    /* ── CSI controller ──────────────────────────────────────────────────── */
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id                = 0,
        .h_res                  = POSE_W,
        .v_res                  = POSE_H,
        .data_lane_num          = 2,
        .lane_bit_rate_mbps     = CSI_LANE_BITRATE_MBPS,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .queue_items            = 1,
        .byte_swap_en           = false,
    };
    esp_cam_ctlr_handle_t cam_handle = NULL;
    ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_cfg, &cam_handle));

    /* Allocate PSRAM frame buffer via CSI controller allocator (correct alignment) */
    const size_t frame_bytes = (size_t)POSE_W * POSE_H * POSE_BPP;
    void *frame_buf = esp_cam_ctlr_alloc_buffer(cam_handle, frame_bytes,
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!frame_buf) {
        ESP_LOGE(TAG, "Frame buffer alloc failed (%zu bytes PSRAM)", frame_bytes);
        return;
    }
    ESP_LOGI(TAG, "Frame buffer: %p  size=%zu", frame_buf, frame_bytes);

    /* Fixed transaction -- same buffer reused every frame */
    esp_cam_ctlr_trans_t fixed_trans = {
        .buffer = frame_buf,
        .buflen = frame_bytes,
    };

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans  = s_on_get_new_vb,
        .on_trans_finished = NULL,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, &fixed_trans));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam_handle));
    ESP_LOGI(TAG, "CSI streaming started");

    /* ── Camera intrinsics at detection resolution ────────────────────────── *
     * OV5647 800x800@50fps: 2x2 binned, FOV(H)=72° -> fx_800 approx 892 px.
     * Center-crop 800x800->800x600, then uniform resize ->POSE_REQ (320x240).
     * Scale = POSE_REQ_W / POSE_W = 320/800 = 0.4 -> fx=fy approx 357 px.  */
    const double scale = (double)POSE_REQ_W / (double)POSE_W;
    const double fx = 892.0 * scale;
    const double fy = fx;
    const double cx = (double)POSE_REQ_W / 2.0;
    const double cy = (double)POSE_REQ_H / 2.0;
    cv::Mat K = (cv::Mat_<double>(3, 3)
                 << fx,  0, cx,
                     0, fy, cy,
                     0,  0,  1);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
    ESP_LOGI(TAG, "Intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx, fy, cx, cy);

    /* ── Single-marker local object points (TL, TR, BR, BL) ─────────────── */
    float s = POSE_MARKER_SIZE_M / 2.0f;
    std::vector<cv::Point3f> single_obj = {
        {-s,  s, 0.0f}, { s,  s, 0.0f},
        { s, -s, 0.0f}, {-s, -s, 0.0f},
    };

    /* ── ArUco detector ──────────────────────────────────────────────────── */
    auto dict = cv::aruco::getPredefinedDictionary(
        (cv::aruco::PredefinedDictionaryType)CONFIG_VISION_ARUCO_DICT);
    cv::aruco::DetectorParameters params;
    params.minMarkerPerimeterRate      = 0.20f;
    params.maxMarkerPerimeterRate      = 4.0f;
    params.polygonalApproxAccuracyRate = 0.08f;
    params.minCornerDistanceRate       = 0.02f;
    params.adaptiveThreshWinSizeMin    = 7;
    params.adaptiveThreshWinSizeMax    = 7;
    params.adaptiveThreshWinSizeStep   = 1;
    params.cornerRefinementMethod      = cv::aruco::CORNER_REFINE_NONE;
    params.errorCorrectionRate         = 0.6f;
    cv::aruco::ArucoDetector detector(dict, params);

    /* ── Pre-allocated conversion + detection buffers ────────────────────── */
    cv::Mat gray_cap(POSE_H, POSE_W, CV_8UC1);
    cv::Mat det_frame(POSE_REQ_H, POSE_REQ_W, CV_8UC1);
    if (gray_cap.empty() || det_frame.empty()) {
        ESP_LOGE(TAG, "OpenCV buffer alloc failed");
        return;
    }

    /* Center-crop geometry: 800x800 -> 800x600 -> 320x240 */
    const int crop_h = POSE_W * POSE_REQ_H / POSE_REQ_W;   /* 800*240/320 = 600 */
    const int crop_y = (POSE_H - crop_h) / 2;               /* (800-600)/2 = 100 */

    /* ── Pre-allocated detection vectors ─────────────────────────────────── */
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    ids.reserve(8);
    corners.reserve(8);
    rejected.reserve(64);

    /* ── Detection loop ──────────────────────────────────────────────────── */
    while (1) {
        /* Block until the CSI DMA fills the frame buffer.
         * On return, frame_buf contains one complete RGB565 frame from ISP. */
        esp_cam_ctlr_trans_t recv_trans = {
            .buffer = frame_buf,
            .buflen = frame_bytes,
        };
        if (esp_cam_ctlr_receive(cam_handle, &recv_trans, ESP_CAM_CTLR_MAX_DELAY)
                != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* RGB565 -> grayscale */
        cv::Mat rgb(POSE_H, POSE_W, CV_8UC2, frame_buf);
        cv::cvtColor(rgb, gray_cap, cv::COLOR_BGR5652GRAY);

        /* Center-crop 800x800->800x600, then uniform resize ->320x240 */
        cv::Mat gray_crop = gray_cap(cv::Rect(0, crop_y, POSE_W, crop_h));
        cv::resize(gray_crop, det_frame, cv::Size(POSE_REQ_W, POSE_REQ_H));

        /* Detect ArUco markers */
        ids.clear(); corners.clear(); rejected.clear();
        detector.detectMarkers(det_frame, corners, ids, rejected);

        if (ids.empty()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Per-marker: distance via single-marker solvePnP */
        static char mbuf[160];
        int mpos = 0;

        for (int i = 0; i < (int)ids.size(); i++) {
            std::vector<cv::Point2f> &c = corners[i];
            cv::Mat rvec_s, tvec_s;
            cv::solvePnP(single_obj, c, K, D,
                         rvec_s, tvec_s, false, cv::SOLVEPNP_IPPE_SQUARE);
            float tx = (float)tvec_s.at<double>(0);
            float ty = (float)tvec_s.at<double>(1);
            float tz = (float)tvec_s.at<double>(2);
            float dist = sqrtf(tx*tx + ty*ty + tz*tz);
            mpos += snprintf(mbuf + mpos, sizeof(mbuf) - mpos,
                             "M%d:%.2fm ", ids[i], dist);
        }
        if (mpos > 0 && mbuf[mpos - 1] == ' ') mbuf[--mpos] = '\0';

        printf("%s\n", mbuf);
        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
