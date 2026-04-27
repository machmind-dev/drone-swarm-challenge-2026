/* vision_bench.cpp — Mach Mind ESP32S3 ArUco FPS Benchmark
 *
 * Measures ArUco detection throughput at multiple OV3660/OV2640 resolutions.
 * Results are printed as a human-readable + CSV table over USB-Serial.
 *
 * Methodology
 * ───────────
 * For each resolution stage the camera is (re-)initialised, then:
 *   1. Cam task (Core 0): drains DMA at full sensor rate and, optionally,
 *      point-downsamples to DETECT_SCALE of the captured size before
 *      forwarding via a 1-deep queue (older frames are dropped).
 *   2. Det task (Core 1): receives frames, copies from PSRAM to a fast
 *      internal-DRAM buffer, runs detectMarkers + solvePnP, records timing.
 *
 * After CONFIG_VISION_BENCH_FRAMES detection calls the tasks signal the
 * main task via an event group; the camera is deinit'd and the next stage
 * begins.
 *
 * Metric definitions
 * ──────────────────
 *   cap_fps    Frames captured per second (hardware sensor rate).
 *   det_fps    Completed detectMarkers calls per second.
 *   det_ms     Mean detectMarkers wall-clock time per call (ms).
 *   hit_pct    Percentage of frames where ≥1 marker was found.
 *   dist_m     Mean metric distance of detected markers (solvePnP).
 *
 * Detection resolution vs. capture resolution
 * ────────────────────────────────────────────
 * DETECT_SCALE = 1   → detect at full capture resolution (most accurate).
 * DETECT_SCALE = 2   → detect at half linear (quarter pixel count).
 * Only scale=1 is used here because the benchmark goal is to find the
 * *optimal capture resolution*; applying a fixed secondary scale would
 * conflate two variables.
 */

#include "vision_bench.h"

#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

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
#include "opencv2/objdetect/aruco_dictionary.hpp"
#include "opencv2/calib3d.hpp"

/* Board selection must happen before boards.h so the pin macros are defined */
#include "sdkconfig.h"
#if CONFIG_VISION_BOARD_OV3660_DEVKIT
  #define CAMERA_MODEL_OV3660_DEVKIT
#else
  #define CAMERA_MODEL_XIAO_ESP32S3
#endif
#include "boards.h"

static const char *TAG = "bench";

/* ── Benchmark parameters ───────────────────────────────────────────────── */
#ifndef CONFIG_VISION_BENCH_FRAMES
#define CONFIG_VISION_BENCH_FRAMES  200   /* default if built in pose mode */
#endif
#ifndef CONFIG_VISION_MARKER_SIZE_CM
#define CONFIG_VISION_MARKER_SIZE_CM 15   /* default if built in pose mode */
#endif
#define BENCH_FRAMES    CONFIG_VISION_BENCH_FRAMES
#define ARUCO_DICT      ((cv::aruco::PredefinedDictionaryType)CONFIG_VISION_ARUCO_DICT)
#define MARKER_SIZE_M   (CONFIG_VISION_MARKER_SIZE_CM / 100.0f)

/* ── Resolution table ───────────────────────────────────────────────────── */
typedef struct {
    const char        *name;
    framesize_t        framesize;
    int                w, h;
} res_stage_t;

static const res_stage_t STAGES[] = {
    { "QQVGA  160x120",  FRAMESIZE_QQVGA,  160,  120 },
    { "QVGA   320x240",  FRAMESIZE_QVGA,   320,  240 },
    { "HVGA   480x320",  FRAMESIZE_HVGA,   480,  320 },
    { "VGA    640x480",  FRAMESIZE_VGA,    640,  480 },
};
#define NUM_STAGES  (sizeof(STAGES) / sizeof(STAGES[0]))

/* ── Result storage ─────────────────────────────────────────────────────── */
typedef struct {
    float cap_fps;
    float det_fps;
    float det_ms_mean;
    float det_ms_min;
    float det_ms_max;
    float hit_pct;
    float dist_m_mean;
    int   total_frames;
    int   detected_frames;
} bench_result_t;

static bench_result_t s_results[NUM_STAGES];

/* ── Inter-task communication ───────────────────────────────────────────── */
#define EVT_BENCH_DONE  (1 << 0)
#define EVT_CAM_STOP    (1 << 1)

static EventGroupHandle_t s_evt   = NULL;
static QueueHandle_t      s_queue = NULL;

typedef struct {
    uint8_t *buf;
    int      w, h;
} frame_msg_t;

/* Shared stage index written by main task, read by cam/det tasks */
static volatile int s_stage = 0;

/* PSRAM frame copy buffer — sized for largest stage (VGA = 640×480) */
EXT_RAM_BSS_ATTR static uint8_t s_frame_psram[640 * 480];

/* ── Camera init/deinit ─────────────────────────────────────────────────── */
static esp_err_t camera_init_stage(int stage_idx)
{
    const res_stage_t *s = &STAGES[stage_idx];

    camera_config_t cfg = {
        .pin_pwdn   = CAMERA_PIN_PWDN,
        .pin_reset  = CAMERA_PIN_RESET,
        .pin_xclk   = CAMERA_PIN_XCLK,
        .pin_sccb_sda = CAMERA_PIN_SIOD,
        .pin_sccb_scl = CAMERA_PIN_SIOC,
        .pin_d7     = CAMERA_PIN_D7,
        .pin_d6     = CAMERA_PIN_D6,
        .pin_d5     = CAMERA_PIN_D5,
        .pin_d4     = CAMERA_PIN_D4,
        .pin_d3     = CAMERA_PIN_D3,
        .pin_d2     = CAMERA_PIN_D2,
        .pin_d1     = CAMERA_PIN_D1,
        .pin_d0     = CAMERA_PIN_D0,
        .pin_vsync  = CAMERA_PIN_VSYNC,
        .pin_href   = CAMERA_PIN_HREF,
        .pin_pclk   = CAMERA_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size   = s->framesize,
        .jpeg_quality = 12,
        .fb_count     = 4,              /* enough to prevent DMA OVF */
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t ret = esp_camera_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED for %s: 0x%x", s->name, ret);
        return ret;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        /* Maximise edge sharpness for ArUco corner detection */
        sensor->set_sharpness(sensor, 2);
        sensor->set_contrast(sensor, 2);
        sensor->set_saturation(sensor, 0);
        /* OV3660-specific: disable DNR so marker edges stay crisp */
        if (sensor->id.PID == OV3660_PID) {
            sensor->set_denoise(sensor, 0);
            ESP_LOGI(TAG, "OV3660 detected — DNR disabled, sharpness=2");
        }
    }

    ESP_LOGI(TAG, "Camera ready: %s", s->name);
    return ESP_OK;
}

/* ── Camera feeder task (Core 0) ────────────────────────────────────────── */
static EXT_RAM_BSS_ATTR StackType_t s_cam_stack[4096 / sizeof(StackType_t)];
static StaticTask_t s_cam_tcb;

static void cam_task_fn(void *arg)
{
    uint32_t count = 0;
    int64_t  t0    = esp_timer_get_time();

    ESP_LOGI(TAG, "cam_task started, core=%d", xPortGetCoreID());

    while (!(xEventGroupGetBits(s_evt) & EVT_CAM_STOP)) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        count++;
        const res_stage_t *s = &STAGES[s_stage];
        size_t expected = (size_t)(s->w * s->h);

        if (fb->len == expected) {
            memcpy(s_frame_psram, fb->buf, expected);
            esp_camera_fb_return(fb);

            frame_msg_t msg = { .buf = s_frame_psram, .w = s->w, .h = s->h };
            xQueueOverwrite(s_queue, &msg);
        } else {
            esp_camera_fb_return(fb);
        }

        /* Log capture FPS every 50 frames */
        if (count % 50 == 0) {
            int64_t dt = esp_timer_get_time() - t0;
            float fps = (float)count / ((float)dt / 1e6f);
            ESP_LOGI(TAG, "[cam] %s  count=%" PRIu32 "  cap_fps=%.1f",
                     STAGES[s_stage].name, count, fps);
        }
    }

    /* Store capture FPS into result */
    int64_t dt = esp_timer_get_time() - t0;
    if (dt > 0 && s_stage < (int)NUM_STAGES)
        s_results[s_stage].cap_fps = (float)count / ((float)dt / 1e6f);

    ESP_LOGI(TAG, "cam_task done stage=%d  cap_fps=%.1f",
             s_stage, s_results[s_stage].cap_fps);
    vTaskDelete(NULL);
}

/* ── ArUco detector task (Core 1) ───────────────────────────────────────── */
static EXT_RAM_BSS_ATTR StackType_t s_det_stack[32768 / sizeof(StackType_t)];
static StaticTask_t s_det_tcb;

static void det_task_fn(void *arg)
{
    int stage_idx = s_stage;
    ESP_LOGI(TAG, "det_task started, core=%d  stage=%d  frames=%d",
             xPortGetCoreID(), stage_idx, BENCH_FRAMES);

    /* Fast internal-DRAM frame buffer for the detector.
     * PSRAM random access is ~10× slower than internal SRAM; copying the
     * frame before each detectMarkers call roughly 10× reduces latency.
     * Sized for this stage only — VGA (300 KB) exceeds DRAM headroom and
     * falls back to the PSRAM slow path automatically. */
    size_t frame_pixels = (size_t)(STAGES[stage_idx].w * STAGES[stage_idx].h);
    uint8_t *fast_buf = (uint8_t *)heap_caps_malloc(frame_pixels,
                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!fast_buf)
        ESP_LOGW(TAG, "fast_buf alloc failed for %s — using PSRAM (slow path)", STAGES[stage_idx].name);

    /* Camera intrinsics — computed per stage in the detection loop */
    const res_stage_t *st = &STAGES[stage_idx];
    double fx = (st->w / 160.0) * 63.65;
    double fy = (st->h / 120.0) * 63.65;
    double cx = st->w / 2.0;
    double cy = st->h / 2.0;

    cv::Mat K = (cv::Mat_<double>(3, 3)
                 << fx, 0, cx,
                    0, fy, cy,
                    0,  0,  1);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);

    /* ArUco detector — tuned for the smallest expected marker pixel size */
    auto dict   = cv::aruco::getPredefinedDictionary(ARUCO_DICT);
    cv::aruco::DetectorParameters params;
    params.minMarkerPerimeterRate      = 0.04;  /* allow small markers */
    params.maxMarkerPerimeterRate      = 4.0;
    params.polygonalApproxAccuracyRate = 0.08;
    params.minCornerDistanceRate       = 0.02;
    params.adaptiveThreshWinSizeMin    = 3;
    params.adaptiveThreshWinSizeMax    = 23;
    params.adaptiveThreshWinSizeStep   = 4;
    params.errorCorrectionRate         = 0.6f;
    cv::aruco::ArucoDetector detector(dict, params);

    float h = MARKER_SIZE_M / 2.0f;
    std::vector<cv::Point3f> obj_pts = {
        {-h,  h, 0}, { h,  h, 0}, { h, -h, 0}, {-h, -h, 0},
    };

    bench_result_t *res = &s_results[stage_idx];
    res->det_ms_min = 1e9f;
    res->det_ms_max = 0.0f;
    float det_ms_sum  = 0.0f;
    float dist_sum    = 0.0f;

    int64_t t0 = esp_timer_get_time();
    frame_msg_t msg;

    for (int i = 0; i < BENCH_FRAMES; i++) {
        /* Wait up to 2 s for a frame */
        if (xQueueReceive(s_queue, &msg, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "frame timeout at i=%d", i);
            continue;
        }

        size_t len = (size_t)(msg.w * msg.h);
        uint8_t *pixels = fast_buf ? fast_buf : msg.buf;
        if (fast_buf)
            memcpy(fast_buf, msg.buf, len);

        cv::Mat frame(msg.h, msg.w, CV_8UC1, pixels);

        int64_t t_det = esp_timer_get_time();
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        detector.detectMarkers(frame, corners, ids, rejected);
        float det_ms = (float)(esp_timer_get_time() - t_det) / 1000.0f;

        det_ms_sum       += det_ms;
        if (det_ms < res->det_ms_min) res->det_ms_min = det_ms;
        if (det_ms > res->det_ms_max) res->det_ms_max = det_ms;
        res->total_frames++;

        if (!ids.empty()) {
            res->detected_frames++;
            cv::Vec3d rvec, tvec;
            cv::solvePnP(obj_pts, corners[0], K, D,
                         rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
            float dist = (float)cv::norm(tvec);
            dist_sum += dist;

#if CONFIG_VISION_SHOW_POSE
            ESP_LOGI(TAG, "[det] ID:%d  dist=%.2fm  det_ms=%.1f",
                     ids[0], dist, det_ms);
#endif
        }

        /* Yield 1 ms so IDLE1 can reset its watchdog */
        vTaskDelay(pdMS_TO_TICKS(1));

        /* Progress log every 50 frames */
        if ((i + 1) % 50 == 0) {
            float elapsed = (float)(esp_timer_get_time() - t0) / 1e6f;
            ESP_LOGI(TAG, "[det] stage=%d  frame=%d/%d  det_fps=%.2f  avg_ms=%.1f",
                     stage_idx, i + 1, BENCH_FRAMES,
                     res->total_frames / elapsed,
                     det_ms_sum / res->total_frames);
        }
    }

    /* Finalise results */
    int64_t elapsed_us = esp_timer_get_time() - t0;
    float elapsed_s    = (float)elapsed_us / 1e6f;
    res->det_fps      = res->total_frames / elapsed_s;
    res->det_ms_mean  = res->total_frames ? det_ms_sum / res->total_frames : 0.0f;
    res->hit_pct      = res->total_frames
                        ? 100.0f * res->detected_frames / res->total_frames
                        : 0.0f;
    res->dist_m_mean  = res->detected_frames
                        ? dist_sum / res->detected_frames
                        : 0.0f;

    if (fast_buf) heap_caps_free(fast_buf);

    ESP_LOGI(TAG, "det_task done stage=%d  det_fps=%.2f  hit=%.0f%%",
             stage_idx, res->det_fps, res->hit_pct);

    xEventGroupSetBits(s_evt, EVT_BENCH_DONE);
    vTaskDelete(NULL);
}

/* ── Print results table ────────────────────────────────────────────────── */
static void print_results(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════════╗\n");
    printf("║          Mach Mind — ESP32S3 ArUco FPS Benchmark Results            ║\n");
    printf("╠═══════════════╦═════════╦═════════╦══════════════╦═════════╦════════╣\n");
    printf("║ Resolution    ║ cap_fps ║ det_fps ║ det_ms       ║ hit%%    ║ dist_m ║\n");
    printf("║               ║         ║         ║ mean/min/max ║         ║ (mean) ║\n");
    printf("╠═══════════════╬═════════╬═════════╬══════════════╬═════════╬════════╣\n");

    for (int i = 0; i < (int)NUM_STAGES; i++) {
        bench_result_t *r = &s_results[i];
        printf("║ %-13s ║ %7.1f ║ %7.2f ║ %4.0f/%3.0f/%3.0f ║ %6.1f%% ║ %6.2f ║\n",
               STAGES[i].name,
               r->cap_fps,
               r->det_fps,
               r->det_ms_mean, r->det_ms_min, r->det_ms_max,
               r->hit_pct,
               r->dist_m_mean);
    }

    printf("╚═══════════════╩═════════╩═════════╩══════════════╩═════════╩════════╝\n");
    printf("\n");

    /* CSV for easy import into spreadsheet */
    printf("CSV:\n");
    printf("resolution,cap_fps,det_fps,det_ms_mean,det_ms_min,det_ms_max,hit_pct,dist_m_mean\n");
    for (int i = 0; i < (int)NUM_STAGES; i++) {
        bench_result_t *r = &s_results[i];
        printf("%s,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f\n",
               STAGES[i].name,
               r->cap_fps, r->det_fps,
               r->det_ms_mean, r->det_ms_min, r->det_ms_max,
               r->hit_pct, r->dist_m_mean);
    }
    printf("\n");

    /* Recommendation */
    float best_score = 0.0f;
    int   best_idx   = 0;
    for (int i = 0; i < (int)NUM_STAGES; i++) {
        bench_result_t *r = &s_results[i];
        /* Score = det_fps × hit_pct  (balance throughput and accuracy) */
        if (r->det_fps > 0.5f) {
            float score = r->det_fps * r->hit_pct;
            if (score > best_score) {
                best_score = score;
                best_idx   = i;
            }
        }
    }
    printf(">>> Recommended resolution: %s  (det_fps=%.2f  hit=%.1f%%)\n",
           STAGES[best_idx].name,
           s_results[best_idx].det_fps,
           s_results[best_idx].hit_pct);
    printf("\n");
}

/* ── Public entry point ─────────────────────────────────────────────────── */
void vision_bench_start(void)
{
    s_evt   = xEventGroupCreate();
    s_queue = xQueueCreate(1, sizeof(frame_msg_t));
    configASSERT(s_evt && s_queue);

    memset(s_results, 0, sizeof(s_results));

    ESP_LOGI(TAG, "=== Mach Mind ESP32S3 ArUco Benchmark ===");
    ESP_LOGI(TAG, "dict=%d  marker=%.2fm  frames_per_stage=%d",
             (int)ARUCO_DICT, MARKER_SIZE_M, BENCH_FRAMES);

    for (int stage = 0; stage < (int)NUM_STAGES; stage++) {
        s_stage = stage;
        xEventGroupClearBits(s_evt, EVT_BENCH_DONE | EVT_CAM_STOP);

        ESP_LOGI(TAG, "--- Stage %d: %s ---", stage, STAGES[stage].name);

        if (camera_init_stage(stage) != ESP_OK) {
            ESP_LOGE(TAG, "Skipping stage %d", stage);
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(200)); /* sensor stabilisation */

        /* Launch cam feeder */
        TaskHandle_t cam_h = xTaskCreateStaticPinnedToCore(
            cam_task_fn, "bench_cam",
            sizeof(s_cam_stack) / sizeof(StackType_t),
            NULL, 4, s_cam_stack, &s_cam_tcb, 0);

        /* Launch detector */
        TaskHandle_t det_h = xTaskCreateStaticPinnedToCore(
            det_task_fn, "bench_det",
            sizeof(s_det_stack) / sizeof(StackType_t),
            NULL, 4, s_det_stack, &s_det_tcb, 1);

        if (!cam_h || !det_h) {
            ESP_LOGE(TAG, "Task creation failed for stage %d", stage);
            esp_camera_deinit();
            continue;
        }

        /* Wait for detector to finish */
        xEventGroupWaitBits(s_evt, EVT_BENCH_DONE, pdTRUE, pdTRUE, portMAX_DELAY);

        /* Signal cam task to stop, wait a moment for it to exit */
        xEventGroupSetBits(s_evt, EVT_CAM_STOP);
        vTaskDelay(pdMS_TO_TICKS(300));

        esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    print_results();
    ESP_LOGI(TAG, "Benchmark complete. Connect a marker to repeat (auto-restart in 10 s).");

    /* Restart after 10 s so the user can run multiple measurements */
    vTaskDelay(pdMS_TO_TICKS(10000));
    esp_restart();
}
