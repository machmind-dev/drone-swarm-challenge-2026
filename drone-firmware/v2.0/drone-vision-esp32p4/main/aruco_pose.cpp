/* aruco_pose.cpp — Mach Mind ESP32-P4 on-board ArUco world-pose estimator
 *
 * Target : Waveshare ESP32-P4 WiFi6 Dev Board + OV5647 (MIPI-CSI 2-lane)
 * IDF    : 5.3+   component: espressif/esp_video (V4L2 layer)
 *
 * Camera API: esp_video V4L2 (Waveshare-recommended approach).
 *   esp_video_init() handles sensor SCCB, CSI controller, and ISP internally.
 *   Frame capture: open() → REQBUFS → mmap → STREAMON → DQBUF/QBUF loop.
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
#include "tof_task.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

/* POSIX / Linux V4L2 */
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/errno.h>
#include "linux/videodev2.h"

/* esp_video init + device name */
#include "esp_video_init.h"
#include "esp_video_device.h"

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* IDF */
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"

/* MIPI CSI PHY LDO — ch3 must be powered before CSI activity */
#include "esp_ldo_regulator.h"

#ifdef EPS
#undef EPS
#endif
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect/aruco_detector.hpp"
#include "opencv2/calib3d.hpp"

#include "sdkconfig.h"
#include "boards.h"

/* ── Vision resolution ─────────────────────────────────────────────────────
 * VISION_RES_QVGA : 320×240 — ~8 m reliable detection range  (default)
 * VISION_RES_HVGA : 480×320 — ~12 m reliable detection range, slower
 * Uncomment exactly one. */
#define VISION_RES_QVGA
/* #define VISION_RES_HVGA */

/* Pose-range gate — markers detected farther than this contribute neither
 * to the averaged drone pose nor to best_R_wc. Reduced 8→5 m (2026-05-30) to
 * keep the opposite wall's marker out of view during the 12↔14 flip tests, so
 * each flip can't pull in a second marker. Note: at the arena midline (e.g.
 * y=5) markers 12 and 14 are both exactly 5 m away — position closer to one
 * wall for the gate to actually exclude the other. */
#define POSE_MAX_RANGE_M  5.0f

/* Viewing-incidence gate — reject a marker viewed too obliquely. θ is the angle
 * between the camera line-of-sight and the marker face normal (0° = square-on,
 * 90° = edge-on). At steep incidence the two IPPE solutions are ~180° apart in
 * yaw yet both reproject well, so a flipped/ambiguous pose slips through; this
 * keeps only near-head-on views where the pose is unambiguous. 45° = a ±45°
 * half-angle cone (90° total). Tighter → cleaner poses but more dropouts. */
#define MAX_VIEW_ANGLE_DEG  45.0f
#define MIN_VIEW_COS        0.70710678f  /* cosf(45°); accept if |n·los| >= this */

/* Box drop-zone gate — boxes only ever sit in the two team areas at the arena
 * ends.  A box whose computed world position falls outside BOTH rectangles is a
 * spurious/mislocated detection (bad solvePnP, stale anchor, wrong marker) and
 * is NOT published over the P4→S3 link / micro-ROS.
 *   Red  team area: x ∈ [0, 7],   y ∈ [0, 10]
 *   Blue team area: x ∈ [13, 20], y ∈ [0, 10]
 * BOX_AREA_MARGIN_M widens each rectangle to tolerate pose noise (0 = strict). */
#define BOX_RED_X_MIN     0.0f
#define BOX_RED_X_MAX     7.0f
#define BOX_BLUE_X_MIN    13.0f
#define BOX_BLUE_X_MAX    20.0f
#define BOX_AREA_Y_MIN    0.0f
#define BOX_AREA_Y_MAX    10.0f
#define BOX_AREA_MARGIN_M 0.0f

/* Arena envelope (metres) — used to reject the wrong IPPE planar-ambiguity
 * solution, which reflects the recovered drone position across the marker's
 * wall and lands outside these bounds. Margin absorbs detection noise. */
#define ARENA_X_M       20.0f
#define ARENA_Y_M       10.0f
#define ARENA_MARGIN_M   1.0f

static const char *TAG = "aruco_pose";

/* OV5647 AEC brightness target passed to V4L2_CID_EXPOSURE_ABSOLUTE.
 * Range 0-47: sensor_target = value × 4.92; 47 ≈ 91% of full scale (maximum).
 * 47: dark indoor venue.  15-20: daylit venue with windows. */
#ifndef CONFIG_VISION_AEC_TARGET
#define CONFIG_VISION_AEC_TARGET 10
#endif

/* ── Shared pose state — written by aruco task (CPU0), read by tof task (CPU1) */
static portMUX_TYPE s_pose_mux  = portMUX_INITIALIZER_UNLOCKED;
static volatile bool    s_pose_valid = false;
static volatile float   s_px, s_py, s_pz;
static volatile float   s_pqx, s_pqy, s_pqz, s_pqw;
static volatile float   s_reproj_err = 0.0f; /* mean corner reprojection error (pixels) */
static volatile uint8_t s_trigger_id = 0xFF; /* TEST: 0xFF=none, else first detected marker ID */
static p4_boxes_t       s_boxes = {};  /* protected by s_pose_mux */

extern "C" bool aruco_pose_get_latest(float *x, float *y, float *z,
                                       float *qx, float *qy, float *qz, float *qw,
                                       uint8_t *trigger_id, float *reproj_err)
{
    taskENTER_CRITICAL(&s_pose_mux);
    bool v = s_pose_valid;
    *x = s_px; *y = s_py; *z = s_pz;
    *qx = s_pqx; *qy = s_pqy; *qz = s_pqz; *qw = s_pqw;
    *trigger_id = s_trigger_id;
    *reproj_err = s_reproj_err;
    taskEXIT_CRITICAL(&s_pose_mux);
    return v;
}

extern "C" void aruco_boxes_get_latest(p4_boxes_t *out)
{
    taskENTER_CRITICAL(&s_pose_mux);
    *out = s_boxes;
    taskEXIT_CRITICAL(&s_pose_mux);
}

/* ── __register_exitproc stub ────────────────────────────────────────────────
 * cv::aruco::getPredefinedDictionary() holds a function-local static Dictionary.
 * The espressif__opencv RISC-V build emits a direct call to __register_exitproc
 * (bypassing __cxa_atexit entirely) to register the static's destructor.
 * In ESP-IDF embedded newlib _GLOBAL_REENT->_atexit is NULL, so
 * __register_exitproc dereferences a garbage pointer and panics.
 * This firmware never calls exit(), so destructors never need to run.
 * Providing this definition here preempts the newlib version at link time
 * (component libs are linked before -lc under --whole-archive).
 * ─────────────────────────────────────────────────────────────────────────── */
extern "C" int __register_exitproc(int type, void (*fn)(void), void *arg, void *d)
{
    (void)type; (void)fn; (void)arg; (void)d;
    return 0;
}

/* ── SDC26 arena marker map ─────────────────────────────────────────────────
 * Arena: x=[0,20] y=[0,10] z=height above floor (all in metres).
 * Pole layout (confirmed 2026-05-20):
 *   8 poles, each with TOP marker (z=4 m) and BOTTOM marker (z=2 m).
 *   yaw = direction the marker face points INTO the arena.
 *
 *  y=10  [6/14 x=5]  [7/15 x=10]  [8/16 x=15]   face -Y  yaw=0°
 *  x=0   [5/13 y=5]                              face +X  yaw=90°
 *  x=20  [1/9  y=5]                              face -X  yaw=270°
 *  y=0   [4/12 x=5]  [3/11 x=10]  [2/10 x=15]   face +Y  yaw=180°
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct { int id; float x, y, z, yaw_deg; } world_marker_t;

static const world_marker_t MARKER_MAP[] = {
    /* x=20 end pole — faces -X (yaw=270°) */
    {  1, 20.0f,  5.0f, 4.0f, 270.0f }, {  9, 20.0f,  5.0f, 2.0f, 270.0f },

    /* x=0 end pole — faces +X (yaw=90°) */
    {  5,  0.0f,  5.0f, 4.0f,  90.0f }, { 13,  0.0f,  5.0f, 2.0f,  90.0f },

    /* y=10 wall poles — face -Y (yaw=0°), x=5/10/15 */
    {  6,  5.0f, 10.0f, 4.0f,   0.0f }, { 14,  5.0f, 10.0f, 2.0f,   0.0f },
    {  7, 10.0f, 10.0f, 4.0f,   0.0f }, { 15, 10.0f, 10.0f, 2.0f,   0.0f },
    {  8, 15.0f, 10.0f, 4.0f,   0.0f }, { 16, 15.0f, 10.0f, 2.0f,   0.0f },

    /* y=0 wall poles — face +Y (yaw=180°), x=5/10/15 */
    {  4,  5.0f,  0.0f, 4.0f, 180.0f }, { 12,  5.0f,  0.0f, 2.0f, 180.0f },
    {  3, 10.0f,  0.0f, 4.0f, 180.0f }, { 11, 10.0f,  0.0f, 2.0f, 180.0f },
    {  2, 15.0f,  0.0f, 4.0f, 180.0f }, { 10, 15.0f,  0.0f, 2.0f, 180.0f },

    /* TEST — place physical marker 22 at arena centre (10,5) z=2 m.
     * Remove after approach test. */
    { 22, 10.0f,  5.0f, 2.0f,   0.0f },
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
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
#pragma GCC diagnostic pop

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

#define POSE_BPP 2          /* ISP outputs RGB565 = 2 bytes/pixel */

/* ── Number of V4L2 frame buffers ────────────────────────────────────────── *
 * 2 buffers required: the CSI DMA descriptor chain is circular (ping-pong).  *
 * With fewer than 2 buffers queued, csi_video_on_get_new_trans() returns     *
 * NULL on the first "get next buffer" ISR call → the DMA chain never         *
 * completes its current transfer → DQBUF blocks forever.                    *
 *                                                                             *
 * To prevent PSRAM cache corruption during OpenCV processing (vtable          *
 * pointers in heap objects near the frame buffer read while DMA writes to    *
 * buf[1] → garbage PC crash), we issue VIDIOC_STREAMOFF immediately after    *
 * DQBUF and before any OpenCV call, then re-queue + VIDIOC_STREAMON after.  */
#define CAM_BUF_COUNT  2

/* ── Detection resolution and stream preview dimensions ──────────────────── */
#if defined(VISION_RES_HVGA)
  #define POSE_REQ_W  480
  #define POSE_REQ_H  320
  #define VIEW_W       80
  #define VIEW_H       53   /* ≈3:2 — 800-wide 10× downscale of 533-row crop */
#else                         /* default: QVGA */
  #define POSE_REQ_W  320
  #define POSE_REQ_H  240
  #define VIEW_W       80
  #define VIEW_H       60   /* exact 4:3 — 10× downscale of 800×600 crop */
#endif

#ifndef CONFIG_VISION_POSE_MARKER_SIZE_CM
#define CONFIG_VISION_POSE_MARKER_SIZE_CM 50
#endif
#define POSE_MARKER_SIZE_M  (CONFIG_VISION_POSE_MARKER_SIZE_CM / 100.0f)

/* ══════════════════════════════════════════════════════════════════════════
 * CAMERA_VIEW_MODE — binary live viewer at 921600 baud, NO OpenCV.
 *
 * Enable: -DCAMERA_VIEW_MODE in CMakeLists.txt.
 * Baud:   CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600 in sdkconfig.defaults.
 *
 * Binary protocol per frame (stream_view.py compatible):
 *   8 bytes  magic   [0xAA][0x55][0xA5][0x5A][0xF0][0x0F][0x50][0x3C]
 *   4 bytes  header  W (uint16 LE), H (uint16 LE)
 *   W×H×2 bytes      raw RGB565 — detection crop region, 10× downscaled
 *
 * QVGA: VIEW_W=80, VIEW_H=60 → 9612 bytes/frame → ~9.6 fps @ 921600 baud
 * HVGA: VIEW_W=80, VIEW_H=53 → 8492 bytes/frame → ~10.9 fps @ 921600 baud
 *
 * Camera stays STREAMON always (no per-frame stop — no overheating).
 * Host viewer: python3 tools/stream_view.py /dev/ttyACM0 921600
 * ══════════════════════════════════════════════════════════════════════════ */
#ifdef CAMERA_VIEW_MODE

static void camera_view_mode(int video_fd,
                              uint8_t **cam_bufs,
                              uint32_t cap_w,
                              uint32_t cap_h)
{
    /* Same center-crop as ArUco detection so the viewer shows exactly what
     * the detector will process: 800×800 → 800×crop_h → VIEW_W×VIEW_H. */
    const int crop_h = (int)cap_w * POSE_REQ_H / POSE_REQ_W;
    const int crop_y = ((int)cap_h - crop_h) / 2;

    static uint16_t s_frame[VIEW_W * VIEW_H];
    static const uint8_t MAGIC[8] = {0xAA, 0x55, 0xA5, 0x5A, 0xF0, 0x0F, 0x50, 0x3C};
    /* Dimension header — stream_view.py reads W and H from these 4 bytes */
    const uint8_t dim_hdr[4] = {
        (uint8_t)(VIEW_W & 0xFF), (uint8_t)(VIEW_W >> 8),
        (uint8_t)(VIEW_H & 0xFF), (uint8_t)(VIEW_H >> 8),
    };

    ESP_LOGI(TAG, "=== CAMERA VIEW MODE %dx%d crop -> %dx%d stream ===",
             (int)cap_w, (int)cap_h, VIEW_W, VIEW_H);
    ESP_LOGI(TAG, "Run: python3 tools/stream_view.py /dev/ttyACM0 921600");
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_set_vprintf([](const char *, va_list) -> int { return 0; });
    vTaskDelay(pdMS_TO_TICKS(5000));

    for (;;) {
        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(video_fd, VIDIOC_DQBUF, &buf) != 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Crop detection region, downscale → VIEW_W×VIEW_H (nearest-neighbor) */
        const uint16_t *src = (const uint16_t *)(void *)cam_bufs[buf.index];
        for (int dy = 0; dy < VIEW_H; dy++) {
            int sy = crop_y + dy * crop_h / VIEW_H;
            const uint16_t *row = src + (size_t)sy * cap_w;
            for (int dx = 0; dx < VIEW_W; dx++) {
                s_frame[dy * VIEW_W + dx] = row[dx * (int)cap_w / VIEW_W];
            }
        }

        /* Return buffer immediately — camera keeps streaming, no overheating */
        struct v4l2_buffer qbuf = {};
        qbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.index  = buf.index;
        ioctl(video_fd, VIDIOC_QBUF, &qbuf);

        /* Binary frame: 8-byte magic + 4-byte W×H header + VIEW_W×VIEW_H×2 RGB565 */
        fwrite(MAGIC,   1, sizeof(MAGIC),   stdout);
        fwrite(dim_hdr, 1, sizeof(dim_hdr), stdout);
        fwrite(s_frame, 1, sizeof(s_frame), stdout);
        fflush(stdout);
    }
}
#endif /* CAMERA_VIEW_MODE */


/* ── Public entry point ──────────────────────────────────────────────────── */
void aruco_pose_start(void)
{
    /* ── MIPI CSI PHY LDO — MUST be powered before any CSI/ISP activity ───
     * ESP32-P4 MIPI RX PHY requires LDO ch3 at 2500 mV. Without this the
     * PHY is unbiased and no frames arrive. esp_video_init() may also acquire
     * this internally; the channel is reference-counted so acquiring twice is safe. */
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_cfg = {};
    ldo_cfg.chan_id    = 3;
    ldo_cfg.voltage_mv = 2500;
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi_phy));
    ESP_LOGI(TAG, "MIPI CSI PHY LDO ch3 @ 2500 mV acquired");

    ESP_LOGI(TAG, "=== Mach Mind ArUco Pose Estimator -- ESP32-P4 + OV5647 ===");
    ESP_LOGI(TAG, "detect=%dx%d  marker=%.2fm  map=%d  dict=%d",
             POSE_REQ_W, POSE_REQ_H,
             POSE_MARKER_SIZE_M, NUM_MAP_MARKERS, CONFIG_VISION_ARUCO_DICT);

    /* ── esp_video init — sensor SCCB, CSI controller, ISP all handled internally */
    esp_video_init_csi_config_t csi_config = {};
    csi_config.sccb_config.init_sccb         = true;
    csi_config.sccb_config.i2c_config.port   = 1; /* I2C_NUM_1 — TOF task borrows this handle at t+3s */
    csi_config.sccb_config.i2c_config.scl_pin = CAMERA_SCCB_SCL;
    csi_config.sccb_config.i2c_config.sda_pin = CAMERA_SCCB_SDA;
    csi_config.sccb_config.freq              = 100000;
    csi_config.reset_pin                     = CAMERA_RESET_PIN;
    csi_config.pwdn_pin                      = CAMERA_PWDN_PIN;

    esp_video_init_config_t video_cfg = {};
    video_cfg.csi = &csi_config;

    ESP_ERROR_CHECK(esp_video_init(&video_cfg));
    ESP_LOGI(TAG, "esp_video init OK");

    /* ── Open V4L2 device ────────────────────────────────────────────────── */
    int video_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (video_fd < 0) {
        ESP_LOGE(TAG, "Failed to open %s (errno=%d) — camera unavailable, running ToF-only",
                 ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno);
        while (true) {
            uint8_t n = tof_sensor_count();
            printf("TOF:");
            for (uint8_t k = 0; k < n; k++) {
                if (k) printf(",");
                uint16_t d = tof_get_distance_mm(k);
                if (tof_get_range_status(k) == 0 && d > 0)
                    printf("%u", d);
                else
                    printf("---");
            }
            printf("mm\n");
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    ESP_LOGI(TAG, "Opened %s fd=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, video_fd);

    /* ── Query capabilities ──────────────────────────────────────────────── */
    struct v4l2_capability cap = {};
    if (ioctl(video_fd, VIDIOC_QUERYCAP, &cap) == 0) {
        ESP_LOGI(TAG, "driver=%s card=%s version=%u.%u.%u",
                 cap.driver, cap.card,
                 (unsigned)((cap.version >> 16) & 0xff),
                 (unsigned)((cap.version >>  8) & 0xff),
                 (unsigned)((cap.version      ) & 0xff));
    }

    /* ── Get current format (sensor resolution), then request RGB565 output ─ */
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed");
        close(video_fd);
        return;
    }
    ESP_LOGI(TAG, "Sensor format: %" PRIu32 "x%" PRIu32 " pixfmt=0x%08" PRIx32,
             fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat);

    /* Request RGB565 output — only if ISP hasn't already configured it.
     * esp_video pre-configures the pipeline from sdkconfig (OV5647→ISP→RGB565),
     * so VIDIOC_G_FMT often already returns RGB565 and S_FMT would be rejected. */
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        if (ioctl(video_fd, VIDIOC_S_FMT, &fmt) != 0) {
            ESP_LOGE(TAG, "VIDIOC_S_FMT RGB565 failed");
            close(video_fd);
            return;
        }
        ESP_LOGI(TAG, "Format changed to RGB565");
    } else {
        ESP_LOGI(TAG, "Format already RGB565 — no S_FMT needed");
    }

    const uint32_t cap_w = fmt.fmt.pix.width;
    const uint32_t cap_h = fmt.fmt.pix.height;
    ESP_LOGI(TAG, "Capture: %" PRIu32 "x%" PRIu32 " RGB565", cap_w, cap_h);

    /* ── Request 2 mmap buffers ──────────────────────────────────────────── */
    struct v4l2_requestbuffers req = {};
    req.count  = CAM_BUF_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed");
        close(video_fd);
        return;
    }
    ESP_LOGI(TAG, "Buffers allocated: %" PRIu32, req.count);

    uint8_t *cam_bufs[CAM_BUF_COUNT] = {};

    for (int i = 0; i < CAM_BUF_COUNT; i++) {
        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF[%d] failed", i);
            close(video_fd);
            return;
        }
        cam_bufs[i] = (uint8_t *)mmap(NULL, buf.length,
                                       PROT_READ | PROT_WRITE, MAP_SHARED,
                                       video_fd, buf.m.offset);
        if (!cam_bufs[i]) {
            ESP_LOGE(TAG, "mmap[%d] failed", i);
            close(video_fd);
            return;
        }
        ESP_LOGI(TAG, "  buf[%d]: %p  len=%" PRIu32, i, (void*)cam_bufs[i], (uint32_t)buf.length);

        /* Queue buffer into driver */
        if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF[%d] failed", i);
            close(video_fd);
            return;
        }
    }

#ifdef CAMERA_VIEW_MODE
    /* View mode: skip OpenCV warmup entirely — start streaming immediately */
    {
        int st = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(video_fd, VIDIOC_STREAMON, &st) != 0) {
            ESP_LOGE(TAG, "VIDIOC_STREAMON failed for view mode");
            close(video_fd);
            return;
        }
        ESP_LOGI(TAG, "STREAMON OK — entering camera_view_mode");
    }
    camera_view_mode(video_fd, cam_bufs, cap_w, cap_h);
    close(video_fd);
    return;
#endif /* CAMERA_VIEW_MODE */

    /* ── Camera intrinsics at detection resolution ────────────────────────── *
     * OV5647 800x800@50fps: 2x2 binned, FOV(H)=72° -> fx_800 approx 892 px.
     * Center-crop 800x800->800x600, then uniform resize ->POSE_REQ (320x240).
     * Scale = POSE_REQ_W / cap_w = 320/800 = 0.4 -> fx=fy approx 357 px.  */
    const double scale = (double)POSE_REQ_W / (double)cap_w;
    /* Focal length corrected from ground-truth measurement:
     * marker 50x50cm placed at 1.500m reported 1.220m → scale = 1.500/1.220 */
    const double fx = 892.0 * scale * (1.500 / 1.220);
    const double fy = fx;
    /* Optical centre: confirmed on three measurements (two cameras, two X_ADDR
     * configs: 978 and 500) → always x≈68.7/80 stream = 687 in 800-wide output.
     * ISP crop is anchored at X_ADDR_START=500; X_ADDR changes do not shift the
     * output position of the principal point. Lens is physically offset right. */
    const double cam_cx = 687.0;
    const double cx = cam_cx * scale;
    const double cy_c = (double)POSE_REQ_H / 2.0;
    cv::Mat K = (cv::Mat_<double>(3, 3)
                 << fx,  0, cx,
                     0, fy, cy_c,
                     0,  0,  1);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
    ESP_LOGI(TAG, "Intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx, fy, cx, cy_c);

    /* ── Single-marker local object points (TL, TR, BR, BL) ─────────────── */
    float s = POSE_MARKER_SIZE_M / 2.0f;
    std::vector<cv::Point3f> single_obj = {
        {-s,  s, 0.0f}, { s,  s, 0.0f},
        { s, -s, 0.0f}, {-s, -s, 0.0f},
    };
    /* Box markers are 25 cm × 25 cm */
    constexpr float BOX_MARKER_HALF = 0.125f;
    std::vector<cv::Point3f> box_obj = {
        {-BOX_MARKER_HALF,  BOX_MARKER_HALF, 0.0f},
        { BOX_MARKER_HALF,  BOX_MARKER_HALF, 0.0f},
        { BOX_MARKER_HALF, -BOX_MARKER_HALF, 0.0f},
        {-BOX_MARKER_HALF, -BOX_MARKER_HALF, 0.0f},
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
    /* adaptiveThreshConstant left at default 7. The 98th-percentile norm_max
     * stretch ensures proper dynamic range before this threshold runs. */
    params.cornerRefinementMethod      = cv::aruco::CORNER_REFINE_NONE;
    params.errorCorrectionRate         = 0.6f;
    cv::aruco::ArucoDetector detector(dict, params);

    /* ── Fast SRAM frame buffer (mirrors the S3 aruco_task approach) ────────── *
     * OpenCV's detectMarkers does random-access reads over every pixel; each  *
     * PSRAM access costs ~10× more than SRAM.  More critically, HEX PSRAM DMA *
     * (camera) creates MSPI bus pressure that corrupts adjacent PSRAM cache   *
     * lines → crashes when detectMarkers reads a corrupted pixel or BSS value. *
     * Solution: copy the frame to internal SRAM before detectMarkers, exactly  *
     * as the S3 aruco_task.cpp does (fast_frame in MALLOC_CAP_INTERNAL).       *
     * STREAMOFF window is shortened to just the C-loop conversion, then DMA   *
     * restarts while detectMarkers runs safely in SRAM.                       */
    uint8_t *fast_frame = (uint8_t *)heap_caps_malloc(
        POSE_REQ_W * POSE_REQ_H, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!fast_frame) {
        ESP_LOGE(TAG, "fast_frame SRAM alloc failed (need %u B)",
                 (unsigned)(POSE_REQ_W * POSE_REQ_H));
        return;
    }
    ESP_LOGI(TAG, "fast_frame %ux%u = %u B in SRAM @ %p",
             POSE_REQ_W, POSE_REQ_H, (unsigned)(POSE_REQ_W * POSE_REQ_H),
             (void *)fast_frame);

    /* Center-crop geometry: 800x800 -> 800x600 -> 320x240 */
    const int crop_h = (int)cap_w * POSE_REQ_H / POSE_REQ_W;   /* 800*240/320 = 600 */
    const int crop_y = ((int)cap_h - crop_h) / 2;               /* (800-600)/2 = 100 */

#ifdef DETECTION_STREAM
    /* det_snap: VIEW_W×VIEW_H uint16 RGB565 — filled from the DMA buffer before
     * STREAMOFF (same technique as camera_view_mode) so ISP lens-shading and AE
     * corrections are active.  Marker outlines drawn white at stream time. */
    static uint16_t det_snap[VIEW_W * VIEW_H];
#endif

    /* ── Pre-allocated detection vectors ─────────────────────────────────── */
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    ids.reserve(8);
    corners.reserve(8);
    rejected.reserve(64);

    /* ── Pre-warm ALL OpenCV code paths BEFORE VIDIOC_STREAMON ───────────── *
     * All OpenCV function-local statics (cv::TLSData<CoreTLSData>,           *
     * icvFetchContourEx<schar> at contours_new.cpp:41, etc.) live in         *
     * libopencv_imgproc.a BSS, which sbss_psram.lf routes to PSRAM.  They   *
     * are lazily initialised on the first call that exercises each path.     *
     *                                                                         *
     * If the first call happens AFTER VIDIOC_STREAMON, camera DMA is already *
     * writing to PSRAM frame buffers; MSPI cache-line pressure corrupts these*
     * statics before/during their initialisation → spinlock_acquire          *
     * load-access-fault (MTVAL = frame-pixel data, MEPC = icvFetchContourEx  *
     * at contours_new.cpp:41 or TLSDataContainer::getData).                  *
     *                                                                         *
     * Fix: force every code path here while SRAM has maximum headroom and    *
     * DMA is NOT running.  SPIRAM_MALLOC_ALWAYSINTERNAL=32768 causes these   *
     * small allocations (spinlocks, TLS structs) to land in SRAM.            *
     *                                                                         *
     * The warm image MUST contain real edges/contours so that                *
     * adaptiveThreshold + contour tracing → icvFetchContourEx<schar> is      *
     * actually reached.  A 32×32 image with a centred white square suffices. */
    {
        /* 32×32 grayscale: centred 16×16 white square on black — real contours */
        cv::Mat warm(32, 32, CV_8UC1, cv::Scalar(0));
        cv::rectangle(warm, cv::Point(8, 8), cv::Point(23, 23),
                      cv::Scalar(255), cv::FILLED);
        cv::Mat warm_small(32, 32, CV_8UC1);
        cv::resize(warm, warm_small, cv::Size(32, 32));
        std::vector<int> w_ids;
        std::vector<std::vector<cv::Point2f>> w_corners, w_rej;
        detector.detectMarkers(warm_small, w_corners, w_ids, w_rej);

        /* Also warm the BGR5652GRAY cvtColor path used in the main loop */
        uint8_t px565[8] = {};
        cv::Mat warm_rgb(2, 2, CV_8UC2, px565);
        cv::Mat warm_g(2, 2, CV_8UC1);
        cv::cvtColor(warm_rgb, warm_g, cv::COLOR_BGR5652GRAY);

        ESP_LOGI(TAG, "OpenCV fully pre-warmed before STREAMON (sram free=%u B)",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }

    /* ── Start streaming AFTER all OpenCV statics are initialised ─────────── */
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed");
        close(video_fd);
        return;
    }
    ESP_LOGI(TAG, "V4L2 streaming started");

    /* ── Log the confirmed pixel format so we can read it from serial ────── */
    {
        struct v4l2_format chk = {};
        chk.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(video_fd, VIDIOC_G_FMT, &chk);
        uint32_t pf = chk.fmt.pix.pixelformat;
        char fcc[5] = { (char)(pf&0xff), (char)((pf>>8)&0xff),
                        (char)((pf>>16)&0xff), (char)((pf>>24)&0xff), 0 };
        ESP_LOGI(TAG, "CONFIRMED ISP output: %"PRIu32"x%"PRIu32
                 "  pixfmt=0x%08"PRIx32" ('%s')",
                 chk.fmt.pix.width, chk.fmt.pix.height, pf, fcc);
    }

    /* ── Boost OV5647 AEC target via VIDIOC_S_EXT_CTRLS ────────────────────
     * Standard V4L2 VIDIOC_S_CTRL returns EINVAL on /dev/video0 for sensor
     * parameters — the CSI device only handles VIDIOC_S_EXT_CTRLS.
     *
     * V4L2_CID_EXPOSURE_ABSOLUTE maps to ESP_CAM_SENSOR_EXPOSURE_VAL which
     * calls ov5647_set_AE_target().  The OV5647 AEC uses this 0-235 value
     * as its luminance convergence target.
     * At 800x800@50fps (VTS=984, fps=50):
     *   sensor_target = V4L2_value x fps x VTS / 10000
     *                 = V4L2_value x 4.92
     * V4L2_value=47 => sensor_target=231 (~91% of 255) — push AEC bright.
     *
     * Root cause of dark image: ov5647_settings.h had gain ceiling
     * 0x3a18:0x3a19=0x00f8 (~15.5x max) for 800x800 mode; 1080p mode uses
     * 0x03ff (~64x).  Fixed in ov5647_settings.h before this build.
     */
    {
        struct v4l2_ext_controls ectrls = {};
        struct v4l2_ext_control  ectrl  = {};
        ectrls.ctrl_class = V4L2_CID_CAMERA_CLASS;
        ectrls.count      = 1;
        ectrls.controls   = &ectrl;

        /* Set OV5647 AEC brightness target.  Override CONFIG_VISION_AEC_TARGET
         * in CMakeLists (add_compile_definitions) to tune per venue:
         *   47 = dark indoor arena (~91% full scale — original value)
         *   20 = daylit venue with windows (~38% full scale — default now)
         *    0 = minimum (fully automatic, no floor) */
        ectrl.id    = V4L2_CID_EXPOSURE_ABSOLUTE;
        ectrl.value = CONFIG_VISION_AEC_TARGET;
        if (ioctl(video_fd, VIDIOC_S_EXT_CTRLS, &ectrls) == 0)
            ESP_LOGI(TAG, "OV5647 AEC target=%d (~%.0f%% full scale)",
                     CONFIG_VISION_AEC_TARGET,
                     (float)CONFIG_VISION_AEC_TARGET * 4.92f / 255.0f * 100.0f);
        else
            ESP_LOGW(TAG, "VIDIOC_S_EXT_CTRLS EXPOSURE_ABSOLUTE failed (errno=%d)", errno);
    }

    /* Give the OV5647 and ISP pipeline 5 s to stabilise before the first
     * DQBUF.  The ISP lens-shading correction and AE need several frames to
     * converge; the per-frame STREAMOFF in the detection loop pauses DMA but
     * does not reset the ISP algorithm state, so once converged the corrections
     * remain active.  500 ms was too short — lens vignette circle visible. */
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Sensor stabilisation delay done — entering capture loop");

#ifdef CAMERA_VIEW_MODE
    /* View mode: skip OpenCV/ArUco -- stream color+gray frames over UART */
    camera_view_mode(video_fd, cam_bufs, cap_w, cap_h);
    close(video_fd);
    return;
#endif /* CAMERA_VIEW_MODE */

        /* ── Detection loop — V4L2 DQBUF / STREAMOFF / process / STREAMON ──── */
    int diag_frame = 0;
    int fps_count  = 0;
    int64_t fps_ts = esp_timer_get_time();

    /* Color thumbnail buffers — filled in STREAMOFF window, read in dump */
    static uint8_t s_color_thumb[80 * 60 * 3];
    static bool    s_color_thumb_ready = false;

    while (1) {
        /* Block until the driver has a completed frame */
        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (diag_frame == 0) {
            ESP_LOGI(TAG, "Calling VIDIOC_DQBUF (frame 0) ...");
        }
        if (ioctl(video_fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGW(TAG, "VIDIOC_DQBUF failed (errno=%d), retrying", errno);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

#ifdef DETECTION_STREAM
        /* Color snapshot BEFORE STREAMOFF — DMA still running on the other buffer.
         * This mirrors camera_view_mode exactly: buf.index is exclusively ours
         * after DQBUF, so reading cam_bufs[buf.index] is safe while DMA writes
         * to the other buffer.  ISP lens-shading and AE corrections are active,
         * producing a proper scene image (no raw lens vignette circle). */
        {
            const uint16_t *src = (const uint16_t *)(void *)cam_bufs[buf.index];
            for (int dy = 0; dy < VIEW_H; dy++) {
                int sy = crop_y + (int)((uint32_t)dy * (uint32_t)crop_h / VIEW_H);
                const uint16_t *row = src + (uint32_t)sy * cap_w;
                for (int dx = 0; dx < VIEW_W; dx++) {
                    int sx = (int)((uint32_t)dx * cap_w / VIEW_W);
                    det_snap[dy * VIEW_W + dx] = row[sx];
                }
            }
        }
#endif

        /* NO STREAMOFF — camera streams continuously so ISP AE/AWB/LSC converges.
         * buf.index is exclusively ours after DQBUF; DMA is writing to the
         * other buffer.  PSRAM reads of cam_bufs[buf.index] are safe here,
         * same as camera_view_mode.  We QBUF this buffer below, AFTER we have
         * copied all needed data into SRAM fast_frame. */
        uint8_t *frame_ptr = cam_bufs[buf.index];

        /* FPS counter — log every 5 seconds */
        fps_count++;
        int64_t now = esp_timer_get_time();
        if (now - fps_ts >= 5000000LL) {
            float fps_val = fps_count * 1e6f / (float)(now - fps_ts);
            ESP_LOGI(TAG, "fps=%.1f", fps_val);
            fps_count = 0;
            fps_ts    = now;
        }

        /* One-pass RGB565→grayscale + center-crop + nearest-neighbour resize
         * PSRAM→SRAM.  DMA is streaming on the other buffer — reading this
         * buffer is safe because DQBUF gives us exclusive ownership of it. */
        {
            /* 98th-percentile normalisation: bright point sources saturate
             * but do NOT collapse ambient scene to black. */
            static uint8_t norm_max = 16;
            const uint32_t ns = (255u * 256u) /
                                (norm_max < 8u ? 8u : (uint32_t)norm_max);
            const uint16_t *src = (const uint16_t *)(void *)frame_ptr;
            uint8_t        *dst = fast_frame;
            static uint16_t lhist[256];
            memset(lhist, 0, sizeof(lhist));
            for (int y = 0; y < POSE_REQ_H; y++) {
                int sy = crop_y + (int)((uint32_t)y * (uint32_t)crop_h / POSE_REQ_H);
                const uint16_t *row = src + (uint32_t)sy * cap_w;
                for (int x = 0; x < POSE_REQ_W; x++) {
                    uint32_t sx = (uint32_t)x * cap_w / POSE_REQ_W;
                    uint32_t px = row[sx];
                    /* BGR565: R=[15:11] G=[10:5] B=[4:0]
                     * Expand to 8-bit (multiply-shift, avoids lookup table) */
                    uint32_t r8 = ((px >> 11) & 0x1Fu) * 527u >> 6;
                    uint32_t g8 = ((px >>  5) & 0x3Fu) * 259u >> 6;
                    uint32_t b8 = ( px        & 0x1Fu) * 527u >> 6;
                    /* BT.601 luma, normalise 0..norm_max -> 0..255 */
                    uint32_t luma = (77u*r8 + 150u*g8 + 29u*b8) >> 8;
                    lhist[luma & 0xFFu]++;
                    uint32_t out = (luma * ns) >> 8;
                    *dst++ = (uint8_t)(out > 255u ? 255u : out);
                }
            }
            /* 98th percentile: walk from top, stop when cumsum >= 2% of pixels */
            {
                const uint32_t tail = (uint32_t)(POSE_REQ_W * POSE_REQ_H) * 2u / 100u;
                uint32_t cum = 0; uint8_t p98 = 8u;
                for (int b = 255; b >= 0; b--) {
                    cum += lhist[b];
                    if (cum >= tail) { p98 = (uint8_t)b; break; }
                }
                norm_max = p98 > 8u ? p98 : 8u;
            }

            /* Color thumbnail: 80x60 RGB888, normalized same as luma */
            {
                uint8_t *ctdst = s_color_thumb;
                const uint16_t *csrc = (const uint16_t *)(void *)frame_ptr;
                for (int cty = 0; cty < 60; cty++) {
                    int csy = crop_y + (int)((uint32_t)cty * (uint32_t)crop_h / 60);
                    const uint16_t *crow = csrc + (uint32_t)csy * cap_w;
                    for (int ctx = 0; ctx < 80; ctx++) {
                        int csx = (int)((uint32_t)ctx * cap_w / 80);
                        uint32_t cpx = crow[csx];
                        uint32_t cr = ((cpx >> 11) & 0x1Fu) * 527u >> 6;
                        uint32_t cg = ((cpx >>  5) & 0x3Fu) * 259u >> 6;
                        uint32_t cb = (cpx & 0x1Fu) * 527u >> 6;
                        cr = (cr * ns) >> 8; if (cr > 255u) cr = 255u;
                        cg = (cg * ns) >> 8; if (cg > 255u) cg = 255u;
                        cb = (cb * ns) >> 8; if (cb > 255u) cb = 255u;
                        *ctdst++ = (uint8_t)cr;
                        *ctdst++ = (uint8_t)cg;
                        *ctdst++ = (uint8_t)cb;
                    }
                }
            }

        }

        /* det_snap already captured from DMA buffer above (before frame fill). */

        /* Return DMA buffer to driver now — all data copied to SRAM fast_frame.
         * Camera keeps streaming continuously; ISP AE/AWB converges normally. */
        {
            struct v4l2_buffer qbuf = {};
            qbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            qbuf.memory = V4L2_MEMORY_MMAP;
            qbuf.index  = buf.index;
            ioctl(video_fd, VIDIOC_QBUF, &qbuf);
        }

        /* Detect ArUco markers in fast_frame (SRAM — no PSRAM access).
         * detectMarkers() calls malloc(76800) internally; with
         * SPIRAM_MALLOC_ALWAYSINTERNAL=131072 this lands in SRAM, so there is
         * no PSRAM heap spinlock conflict while DMA is active. */
        cv::Mat det_mat(POSE_REQ_H, POSE_REQ_W, CV_8UC1, fast_frame);

        ids.clear(); corners.clear(); rejected.clear();
        detector.detectMarkers(det_mat, corners, ids, rejected);

        /* Diagnostic: every 50 frames log pixel stats + detection counts */
        if (++diag_frame % 50 == 0) {
            uint8_t *p = fast_frame;
            uint32_t sum = 0;
            uint8_t pmin = p[0], pmax = p[0];
            for (int pi = 0; pi < POSE_REQ_W * POSE_REQ_H; pi += 64) {
                sum += p[pi];
                if (p[pi] < pmin) pmin = p[pi];
                if (p[pi] > pmax) pmax = p[pi];
            }
            float mean_val = (float)sum / (float)(POSE_REQ_W * POSE_REQ_H / 64);
            ESP_LOGI(TAG, "diag frame=%d mean=%.1f min=%u max=%u ids=%zu rejected=%zu",
                     diag_frame, mean_val, pmin, pmax,
                     ids.size(), rejected.size());
        }

        /* ── Serial frame dump — Option 3 visual debug ───────────────────────
         * Every 200 frames: downsample fast_frame (POSE_REQ_W x POSE_REQ_H)
         * to 80x60 and hex-dump it over USB-Serial.  A host Python script
         * reassembles the pixels and shows them with matplotlib.
         * Protocol:  FSTART:80x60
  <9600 hex chars>
  FEND

         * At 115200 baud this takes ~1 s and does not affect detection.    */
        if (diag_frame % 200 == 0) {
            /* Grayscale: what OpenCV processes */
            static const int DW = 80, DH = 60;
            static uint8_t dbuf[80 * 60];
            for (int dy = 0; dy < DH; dy++) {
                int sy2 = (int)((uint32_t)dy * POSE_REQ_H / DH);
                for (int dx = 0; dx < DW; dx++) {
                    int sx2 = (int)((uint32_t)dx * POSE_REQ_W / DW);
                    dbuf[dy * DW + dx] = fast_frame[sy2 * POSE_REQ_W + sx2];
                }
            }
            printf("FSTART:80x60\n");
            for (int i = 0; i < DW * DH; i++) { printf("%02x", dbuf[i]); }
            printf("\nFEND\n");

            /* Color RGB: camera actual output (normalized) */
            printf("FSTART:80x60:RGB\n");
            for (int i = 0; i < 80 * 60 * 3; i++) { printf("%02x", s_color_thumb[i]); }
            printf("\nFEND\n");
            fflush(stdout);
        }

        /* Per-marker: distance + world pose (skipped when no markers found) */
        if (!ids.empty()) {
            static char mbuf[160];
            int mpos = 0;

            double px_sum = 0, py_sum = 0, pz_sum = 0;
            float  qx_out = 0, qy_out = 0, qz_out = 0, qw_out = 1;
            int    pose_n = 0;
            float  best_dist = 1e9f;
            float  best_reproj = 0.0f;
            /* World-from-camera transform + drone world pose of the closest arena
             * marker chosen THIS frame.  best_dist (reset to 1e9f each frame) is
             * the freshness flag: only valid when an arena marker was chosen this
             * frame.  Boxes are positioned from these and MUST NOT reuse a stale
             * (previous-frame) value — see box loop below. */
            cv::Mat best_R_wc;
            float   best_pw_x = 0.0f, best_pw_y = 0.0f, best_pw_z = 0.0f;

            /* solvePnPGeneric(IPPE) returns both ambiguous planar solutions.
             * For a vertical wall marker the ambiguity is a ~180° rotation
             * about the marker's vertical axis, which reflects the recovered
             * camera position across the wall plane: the wrong solution lands
             * *behind* the wall (outside the arena) with a 180°-flipped yaw —
             * and once fused into EKF2 it sends the drone flying the wrong way.
             *
             * Disambiguate absolutely, with no temporal state, so it is correct
             * on the very first frame (the earlier s_prev_yaw continuity
             * heuristic was order-dependent and latched onto whatever frame 0
             * guessed — see flight tests 2026-05-30). Keep only solutions whose
             * recovered drone position is on the arena-facing side of the marker
             * AND inside the arena envelope, then pick the lowest reprojection
             * error among the survivors. */

            for (int i = 0; i < (int)ids.size(); i++) {
                std::vector<cv::Point2f> &c = corners[i];

                std::vector<cv::Mat> rvecs_s, tvecs_s;
                cv::solvePnPGeneric(single_obj, c, K, D,
                                    rvecs_s, tvecs_s, false, cv::SOLVEPNP_IPPE);
                float tx   = (float)tvecs_s[0].at<double>(0);
                float ty   = (float)tvecs_s[0].at<double>(1);
                float tz   = (float)tvecs_s[0].at<double>(2);
                float dist = sqrtf(tx*tx + ty*ty + tz*tz);
                mpos += snprintf(mbuf + mpos, sizeof(mbuf) - mpos,
                                 "M%d:%.2fm ", ids[i], dist);

                const world_marker_t *m = find_marker(ids[i]);
                if (!m || dist > POSE_MAX_RANGE_M) continue;

                float yr = m->yaw_deg * (float)M_PI / 180.0f;
                cv::Mat R_lw = (cv::Mat_<double>(3,3) <<
                     cos(yr),  0,  sin(yr),
                     sin(yr),  0, -cos(yr),
                     0,        1,  0      );
                cv::Mat t_mw = (cv::Mat_<double>(3,1) <<
                    (double)m->x, (double)m->y, (double)m->z);

                /* Marker face normal in world = 3rd column of R_lw (points into
                 * the arena). The true drone pose is on the +normal side. */
                double nx = R_lw.at<double>(0, 2);
                double ny = R_lw.at<double>(1, 2);

                int     chosen        = -1;
                float   chosen_reproj = 0.0f;
                float   min_reproj    = 1e9f;
                cv::Mat chosen_R_wc, chosen_p_world;

                for (int sol = 0; sol < (int)rvecs_s.size(); sol++) {
                    cv::Mat R_l2c;
                    cv::Rodrigues(rvecs_s[sol], R_l2c);
                    /* Marker-frame upright pre-filter: cheap reject of gross
                     * 90°/180° face-normal flips. */
                    if (R_l2c.at<double>(1, 1) > -0.8) continue;

                    /* Viewing-incidence gate: reject markers seen too obliquely
                     * (> MAX_VIEW_ANGLE_DEG off the face normal), where the IPPE
                     * yaw is ambiguous. n = marker normal in camera frame (3rd col
                     * of R_l2c); los = unit line-of-sight to the marker (tvec dir).
                     * cos(incidence) = |n·los|; 1 = square-on, 0 = edge-on. */
                    {
                        double nx_c = R_l2c.at<double>(0, 2);
                        double ny_c = R_l2c.at<double>(1, 2);
                        double nz_c = R_l2c.at<double>(2, 2);
                        double sx = tvecs_s[sol].at<double>(0);
                        double sy = tvecs_s[sol].at<double>(1);
                        double sz = tvecs_s[sol].at<double>(2);
                        double sl = sqrt(sx*sx + sy*sy + sz*sz);
                        double cosang = (sl > 1e-6)
                            ? fabs((nx_c*sx + ny_c*sy + nz_c*sz) / sl) : 0.0;
                        if (cosang < MIN_VIEW_COS) {
                            if (diag_frame % 30 == 0)
                                printf("VIEW M%d sol=%d incidence=%.0f° > %.0f° rejected\n",
                                       ids[i], sol,
                                       acos(cosang) * 180.0 / M_PI,
                                       (double)MAX_VIEW_ANGLE_DEG);
                            continue;
                        }
                    }

                    cv::Mat R_wc  = R_lw * R_l2c.t();
                    cv::Mat p_loc = -R_l2c.t() * tvecs_s[sol];
                    cv::Mat p_w   = R_lw * p_loc + t_mw;
                    double pwx = p_w.at<double>(0);
                    double pwy = p_w.at<double>(1);

                    /* Wall-side gate: drone must be in front of the marker face. */
                    if ((pwx - (double)m->x) * nx +
                        (pwy - (double)m->y) * ny <= 0.0) continue;

                    /* Arena-envelope gate (margin absorbs detection noise). */
                    if (pwx < -ARENA_MARGIN_M || pwx > ARENA_X_M + ARENA_MARGIN_M ||
                        pwy < -ARENA_MARGIN_M || pwy > ARENA_Y_M + ARENA_MARGIN_M)
                        continue;

                    /* Reprojection error: mean pixel distance, detected vs projected. */
                    std::vector<cv::Point2f> proj_pts;
                    cv::projectPoints(single_obj, rvecs_s[sol], tvecs_s[sol],
                                      K, D, proj_pts);
                    float reproj_sum = 0.0f;
                    for (int j = 0; j < 4; j++) {
                        float ex = proj_pts[j].x - c[j].x;
                        float ey = proj_pts[j].y - c[j].y;
                        reproj_sum += sqrtf(ex*ex + ey*ey);
                    }
                    float reproj = reproj_sum / 4.0f;

                    if (reproj < min_reproj) {
                        min_reproj     = reproj;
                        chosen         = sol;
                        chosen_reproj  = reproj;
                        chosen_R_wc    = R_wc;
                        chosen_p_world = p_w;
                    }
                }

                /* Disambiguation trace (rate-limited) — confirms in the flight
                 * log which solution survived the gates and where it placed the
                 * drone. Remove once the fix is verified. */
                if (diag_frame % 30 == 0) {
                    printf("AMB M%d sols=%d chosen=%d reproj=%.2f pw=(%.2f,%.2f)\n",
                           ids[i], (int)rvecs_s.size(), chosen, (double)chosen_reproj,
                           chosen >= 0 ? chosen_p_world.at<double>(0) : 0.0,
                           chosen >= 0 ? chosen_p_world.at<double>(1) : 0.0);
                }

                if (chosen < 0) continue;

                px_sum += chosen_p_world.at<double>(0);
                py_sum += chosen_p_world.at<double>(1);
                pz_sum += chosen_p_world.at<double>(2);

                if (dist < best_dist) {
                    best_dist   = dist;
                    best_reproj = chosen_reproj;
                    best_R_wc   = chosen_R_wc;
                    best_pw_x   = (float)chosen_p_world.at<double>(0);
                    best_pw_y   = (float)chosen_p_world.at<double>(1);
                    best_pw_z   = (float)chosen_p_world.at<double>(2);
                    rot_to_quat(best_R_wc, &qx_out, &qy_out, &qz_out, &qw_out);
                }
                pose_n++;
            }
            if (mpos > 0 && mbuf[mpos - 1] == ' ') mbuf[--mpos] = '\0';

            /* TEST: scan ids for trigger marker 22; report 0xFF otherwise */
            uint8_t new_tid = 0xFF;
            for (int i = 0; i < (int)ids.size(); i++) {
                if (ids[i] == 22) { new_tid = 22; break; }
            }

            /* Compute world positions for detected box markers (31-36, 41-46).
             * Uses world-from-camera transform from the closest arena map marker.
             *
             * REQUIRE a fresh same-frame anchor (best_dist < 1e9f means an arena
             * marker was chosen THIS frame).  tvec_b is the box measured in the
             * current camera frame; best_pw_*/best_R_wc must describe the camera's
             * pose in that SAME frame or the box is projected from a stale drone
             * pose and lands in front of the wrong marker.  The earlier
             * !best_R_wc.empty() gate reused the last-ever anchor across frames,
             * which mislocated boxes whenever no arena marker was co-visible. */
            p4_boxes_t new_boxes = {};
            if (best_dist < 1e9f) {
                for (int i = 0; i < (int)ids.size(); i++) {
                    if (new_boxes.count >= P4_LINK_BOX_MAX) break;
                    int bid = ids[i];
                    if ((bid >= 31 && bid <= 36) || (bid >= 41 && bid <= 46)) {
                        cv::Mat rvec_b, tvec_b;
                        cv::solvePnP(box_obj, corners[i], K, D,
                                     rvec_b, tvec_b, false, cv::SOLVEPNP_IPPE_SQUARE);
                        /* box_world = drone_world + R_world_cam * tvec_box */
                        cv::Mat p_box = (cv::Mat_<double>(3,1) <<
                                         (double)best_pw_x,
                                         (double)best_pw_y,
                                         (double)best_pw_z)
                                        + best_R_wc * tvec_b;
                        float bx = (float)p_box.at<double>(0);
                        float by = (float)p_box.at<double>(1);
                        /* Drop-zone gate: only publish boxes inside a team area. */
                        bool in_red  = bx >= BOX_RED_X_MIN  - BOX_AREA_MARGIN_M &&
                                       bx <= BOX_RED_X_MAX  + BOX_AREA_MARGIN_M &&
                                       by >= BOX_AREA_Y_MIN - BOX_AREA_MARGIN_M &&
                                       by <= BOX_AREA_Y_MAX + BOX_AREA_MARGIN_M;
                        bool in_blue = bx >= BOX_BLUE_X_MIN - BOX_AREA_MARGIN_M &&
                                       bx <= BOX_BLUE_X_MAX + BOX_AREA_MARGIN_M &&
                                       by >= BOX_AREA_Y_MIN - BOX_AREA_MARGIN_M &&
                                       by <= BOX_AREA_Y_MAX + BOX_AREA_MARGIN_M;
                        if (!in_red && !in_blue) {
                            printf("BOX id=%d dropped: world=(%.2f,%.2f) outside team areas\n",
                                   bid, bx, by);
                            continue;
                        }
                        p4_box_entry_t &e = new_boxes.entries[new_boxes.count++];
                        e.id = (uint8_t)bid;
                        e.x  = bx;
                        e.y  = by;
                        e.z  = 0.0f;   /* boxes are on the ground plane */
                    }
                }
            }

            if (new_boxes.count > 0)
                printf("BOXES:%d id=%d\n", (int)new_boxes.count, (int)new_boxes.entries[0].id);

            if (pose_n > 0) {
                printf("%s POSE:%d:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f\n",
                       mbuf, pose_n,
                       px_sum / pose_n, py_sum / pose_n, pz_sum / pose_n,
                       qx_out, qy_out, qz_out, qw_out);
                taskENTER_CRITICAL(&s_pose_mux);
                s_pose_valid = true;
                s_px = (float)(px_sum / pose_n);
                s_py = (float)(py_sum / pose_n);
                s_pz = (float)(pz_sum / pose_n);
                s_pqx = qx_out; s_pqy = qy_out; s_pqz = qz_out; s_pqw = qw_out;
                s_reproj_err = best_reproj;
                s_trigger_id = new_tid;
                s_boxes = new_boxes;
                taskEXIT_CRITICAL(&s_pose_mux);
            } else {
                printf("%s\n", mbuf);
                taskENTER_CRITICAL(&s_pose_mux);
                s_pose_valid = false;
                s_trigger_id = new_tid;
                s_boxes = new_boxes;   /* still commit box positions even without arena ref */
                taskEXIT_CRITICAL(&s_pose_mux);
            }
            fflush(stdout);
        } else {
            /* No markers detected — clear pose so tof_task stops sending valid pose */
            taskENTER_CRITICAL(&s_pose_mux);
            s_pose_valid = false;
            s_trigger_id = 0xFF;
            s_boxes.count = 0;
            taskEXIT_CRITICAL(&s_pose_mux);
        }

        /* ── ToF distance output — every frame ──────────────────────────────── */
        {
            uint8_t n = tof_sensor_count();
            printf("TOF:");
            for (uint8_t k = 0; k < n; k++) {
                if (k) printf(",");
                uint16_t d = tof_get_distance_mm(k);
                if (tof_get_range_status(k) == 0 && d > 0)
                    printf("%u", d);
                else
                    printf("---");
            }
            printf("mm\n");
            fflush(stdout);
        }

#ifdef DETECTION_STREAM
        /* Stream det_snap (ISP color RGB565, VIEW_W×VIEW_H) with detected marker
         * outlines drawn white.  det_snap was captured from DMA buffer before
         * STREAMOFF so ISP lens-shading and AE corrections are fully applied. */
        {
            /* Draw detected marker outlines white */
            const float vsx = (float)VIEW_W / POSE_REQ_W;
            const float vsy = (float)VIEW_H / POSE_REQ_H;
            for (int i = 0; i < (int)ids.size(); i++) {
                for (int j = 0; j < 4; j++) {
                    int x0 = (int)(corners[i][j].x        * vsx);
                    int y0 = (int)(corners[i][j].y        * vsy);
                    int x1 = (int)(corners[i][(j+1)%4].x * vsx);
                    int y1 = (int)(corners[i][(j+1)%4].y * vsy);
                    int steps = std::max(std::abs(x1-x0), std::abs(y1-y0));
                    if (steps < 1) steps = 1;
                    for (int s = 0; s <= steps; s++) {
                        int px = x0 + (x1-x0)*s/steps;
                        int py = y0 + (y1-y0)*s/steps;
                        if (px >= 0 && px < VIEW_W && py >= 0 && py < VIEW_H)
                            det_snap[py * VIEW_W + px] = 0xFFFF;
                    }
                }
            }
            static const uint8_t DET_MAGIC[8] =
                {0xAA, 0x55, 0xA5, 0x5A, 0xF0, 0x0F, 0x50, 0x3C};
            const uint8_t dim_hdr[4] = {
                (uint8_t)(VIEW_W & 0xFF), (uint8_t)(VIEW_W >> 8),
                (uint8_t)(VIEW_H & 0xFF), (uint8_t)(VIEW_H >> 8),
            };
            fwrite(DET_MAGIC, 1, sizeof(DET_MAGIC), stdout);
            fwrite(dim_hdr,   1, sizeof(dim_hdr),   stdout);
            fwrite(det_snap,  2, VIEW_W * VIEW_H,   stdout);
            fflush(stdout);
        }
#endif /* DETECTION_STREAM */

        /* Yield 5 ms per loop so FreeRTOS IDLE task can run and reset the task
         * watchdog.  Without this, DQBUF never blocks (camera at 50 fps
         * outpaces detection at ~3 fps → always a frame queued) and IDLE
         * never gets CPU time → WDT triggers every 5 s. */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
