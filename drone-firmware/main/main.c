/* main.c — Mach Mind Drone ESP32S3 Firmware
 *
 * ROS topics (subscribed):
 *   /gcs/drone_{ID}/command    std_msgs/String  — flight commands
 *   /gcs/drone_{ID}/config     std_msgs/String  — feature config
 *   /gcs/drone_{ID}/control    PoseStamped      — position setpoints (m, positive-up Z)
 *   /drone_{ID}/vision_pose    PoseStamped      — ArUco EKF pose
 *
 * ROS topics (published):
 *   /drone_{ID}/camera/image_raw   sensor_msgs/Image       — mono8 160×120
 *   /drone_{ID}/state              std_msgs/String         — state machine
 *   /drone_{ID}/role               std_msgs/String         — role (idle)
 *   /visualization_marker          visualization_msgs/Marker
 *
 * MAVLink → PX4 (UART1 57600):
 *   Heartbeat, OBSTACLE_DISTANCE,
 *   COMPONENT_ARM_DISARM, SET_MODE,
 *   SET_POSITION_TARGET_LOCAL_NED,
 *   NAV_LAND, VISION_POSITION_ESTIMATE
 *
 * Takeoff altitude options:
 *   A: 1.0 m  — tight formations only, >1.2 m H spacing required
 *   B: 1.5 m  — recommended, clears ground effect, absorbs baro drift  ← default
 *   C: 2.0 m  — close proximity swarm (<0.5 m H), best downwash margin
 *
 * VISION_POSITION_ESTIMATE frame: MAV_FRAME_LOCAL_NED (best PX4 EKF2 compat)
 * Other options: MAV_FRAME_VISION_NED (16), MAV_FRAME_LOCAL_FRD (20)
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include <sys/param.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_camera.h"
#include "nvs_flash.h"

#include "VL53L1X_api.h"

#include <c_library_v2/common/mavlink.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "esp_netif.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <sensor_msgs/msg/image.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <visualization_msgs/msg/marker.h>
#include <uros_network_interfaces.h>

#define CAMERA_MODEL_XIAO_ESP32S3 1
#include "boards.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

static const char *TAG = "drone";

/* ── Identity ──────────────────────────────────────────────────────────── */
#define DRONE_ID          2
#define DRONE_ID_LED_PIN  GPIO_NUM_1

/* ── RViz marker dimensions ────────────────────────────────────────────── */
#define DRONE_DISC_DIAMETER_M  0.18f
#define DRONE_DISC_THICKNESS_M 0.02f
#define DRONE_DEFAULT_Z_M      0.5f
#define VISION_TIMEOUT_MS      1500

#define MARKER_ID_DISC(id)     ((id)*100)
#define MARKER_ID_TEXT(id)     ((id)*100+1)
#define MARKER_ID_RIGHT(id)    ((id)*100+10)
#define MARKER_ID_TOP(id)      ((id)*100+11)
#define MARKER_ID_LEFT(id)     ((id)*100+12)
#define MARKER_ID_FRONT(id)    ((id)*100+13)
#define MARKER_ID_WP_ARROW(id) ((id)*100+20)
#define MARKER_ID_WP_TEXT(id)  ((id)*100+21)

/* ── Network ───────────────────────────────────────────────────────────── */
#define DRONE_IP_BASE_OCTET 100
#define DRONE_IP_NETMASK    "255.255.255.0"
#define DRONE_IP_GATEWAY    "192.168.178.1"
#define DRONE_IP_PREFIX     "192.168.178."

/* ── MAVLink / PX4 ─────────────────────────────────────────────────────── */
#define UART_TX     GPIO_NUM_43
#define UART_RX     GPIO_NUM_44
#define GCS_SYSID   42
#define GCS_COMPID  200
#define PX4_SYSID   1
#define PX4_COMPID  1

/* PX4 custom flight modes (main_mode << 16) */
#define PX4_MODE_STABILIZED  0x00070000UL   /* RC takes control */
#define PX4_MODE_OFFBOARD    0x00060000UL   /* onboard computer control */

/* ── C2 watchdog ───────────────────────────────────────────────────────── */
#define C2_PING_TIMEOUT_MS   500   /* per ping attempt */
#define C2_PING_ATTEMPTS     2     /* attempts per check */
#define C2_CHECK_INTERVAL_MS 1000  /* interval between checks */
#define C2_FAIL_THRESHOLD    2     /* consecutive failures before ELAND */

/* ── Mission parameters ────────────────────────────────────────────────── */
#define MISSION_TAKEOFF_ALT_M      1.5f   /* Option B — change to 2.0 for tight swarm */
#define MISSION_TAKEOFF_WAIT_MS    5000
#define MISSION_SETPOINT_DWELL_MS  3000
#define MISSION_LAND_DESCEND_MS    5000
#define OFFBOARD_STREAM_PERIOD_MS  50     /* 20 Hz setpoint stream */

/* ── micro-ROS macros ──────────────────────────────────────────────────── */
#define RCCHECK(fn) \
    { rcl_ret_t _rc = (fn); \
      if (_rc != RCL_RET_OK) { \
          printf("Failed line %d: %d. Aborting.\n", __LINE__, (int)_rc); \
          vTaskDelete(NULL); } }
#define RCSOFTCHECK(fn) \
    { rcl_ret_t _rc = (fn); \
      if (_rc != RCL_RET_OK) \
          printf("Soft fail line %d: %d.\n", __LINE__, (int)_rc); }

/* ══════════════════════════════════════════════════════════════════════════
 * State machine
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    DRONE_DISARMED = 0,
    DRONE_ARMED,
    DRONE_MISSION,
    DRONE_RETURNING_HOME,
    DRONE_LANDING,
    DRONE_KILLED,
} drone_state_t;

static volatile drone_state_t drone_state = DRONE_DISARMED;
static volatile bool          state_dirty = true;

static const char * const state_names[] = {
    "disarmed", "armed", "mission", "returning_home", "landing", "killed"
};

/* Home position (NED, captured on ARM when vision is valid) */
static volatile float home_x = 0.0f, home_y = 0.0f, home_z = 0.0f;

/* Mission setpoint from /gcs/drone_{ID}/control (positive-up Z) */
static volatile float setpoint_x = 0.0f, setpoint_y = 0.0f, setpoint_z = 1.5f;
static volatile bool  setpoint_received = false;

/* Feature flags */
static volatile bool camera_streaming    = false;
static volatile bool vision_enabled      = false;
static volatile bool gcs_control_active  = false;

/* Vision pose cache (written by vision_pose_callback, read by mission/home tasks) */
static volatile float vp_x = 0.0f, vp_y = 0.0f, vp_z = 0.0f;
static volatile float vp_qx = 0.0f, vp_qy = 0.0f, vp_qz = 0.0f, vp_qw = 1.0f;
static volatile bool    vision_pose_valid   = false;
static volatile int64_t last_vision_pose_ms = 0;

/* FreeRTOS task handles */
static TaskHandle_t mission_task_handle     = NULL;
static TaskHandle_t return_home_task_handle = NULL;
static TaskHandle_t eland_task_handle       = NULL;

/* ══════════════════════════════════════════════════════════════════════════
 * UART / MAVLink
 * ══════════════════════════════════════════════════════════════════════════ */

static void uart_mavlink_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = 57600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_TX, UART_RX,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void mav_send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(UART_NUM_1, (const char *)buf, len);
}

/* Heartbeat — identifies this device as onboard controller to PX4 */
static void send_heartbeat_once(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(GCS_SYSID, GCS_COMPID, &msg,
        MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    mav_send(&msg);
}

/* ── MAVLink flight helpers ────────────────────────────────────────────── */

static void mav_arm(bool arm)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID, PX4_COMPID,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        arm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: %s", arm ? "ARM" : "DISARM");
}

/* Force-disarm regardless of flight state (KILL) */
static void mav_kill(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID, PX4_COMPID,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0.0f, 21196.0f, 0, 0, 0, 0, 0);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: KILL (force disarm)");
}

/* Emergency land — NAV_LAND at current horizontal position */
static void mav_eland(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID, PX4_COMPID,
        MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, NAN, NAN, NAN);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: EMERGENCY LAND");
}

/* SET_POSITION_TARGET_LOCAL_NED — position-only, NED frame.
 * z_ned: negative = upward (e.g. -1.5 for 1.5 m altitude). */
static void mav_set_position_ned(float x, float y, float z_ned)
{
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        GCS_SYSID, GCS_COMPID, &msg,
        (uint32_t)(esp_timer_get_time() / 1000),
        PX4_SYSID, PX4_COMPID,
        MAV_FRAME_LOCAL_NED,
        0b0000111111111000,   /* type_mask: position only */
        x, y, z_ned,
        0, 0, 0,
        0, 0, 0,
        0, 0);
    mav_send(&msg);
}

/* Set PX4 flight mode via SET_MODE */
static void mav_set_mode(uint32_t custom_mode)
{
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: SET_MODE 0x%08lX", (unsigned long)custom_mode);
}

/* VISION_POSITION_ESTIMATE — MAV_FRAME_LOCAL_NED.
 * covariance=NULL signals unknown covariance to PX4 EKF2. */
static void mav_send_vision_estimate(float x, float y, float z,
                                      float roll, float pitch, float yaw)
{
    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_pack(
        GCS_SYSID, GCS_COMPID, &msg,
        (uint64_t)esp_timer_get_time(),
        x, y, z, roll, pitch, yaw,
        NULL, 0);
    mav_send(&msg);
}

/* ══════════════════════════════════════════════════════════════════════════
 * ToF sensors
 * ══════════════════════════════════════════════════════════════════════════ */

#define MEASUREMENT_CYCLE_MS 20
#define TIMER_PERIODIC_MS    25

VL53L1_Dev_t tof_array[] = {
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 2, .shutdown_pin = GPIO_NUM_2,
      .distance_mode = DISTANCE_MODE_SHORT,
      .timing_budget = MEASUREMENT_CYCLE_MS, .inter_measurement = TIMER_PERIODIC_MS },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 4, .shutdown_pin = GPIO_NUM_3,
      .distance_mode = DISTANCE_MODE_SHORT,
      .timing_budget = MEASUREMENT_CYCLE_MS, .inter_measurement = TIMER_PERIODIC_MS },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 6, .shutdown_pin = GPIO_NUM_4,
      .distance_mode = DISTANCE_MODE_SHORT,
      .timing_budget = MEASUREMENT_CYCLE_MS, .inter_measurement = TIMER_PERIODIC_MS },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 8, .shutdown_pin = GPIO_NUM_41,
      .distance_mode = DISTANCE_MODE_SHORT,
      .timing_budget = MEASUREMENT_CYCLE_MS, .inter_measurement = TIMER_PERIODIC_MS },
};

uint8_t sensor_count = sizeof(tof_array) / sizeof(VL53L1_Dev_t);
esp_timer_handle_t tof_sensor_timer;

static void periodic_tof_sensor(void *arg)
{
    int64_t now = esp_timer_get_time();
    for (int k = 0; k < sensor_count; k++) {
        VL53L1_Dev_t *tof = &tof_array[k];
        tof->range_error = VL53L1X_GetAndRestartMeasurement(
            tof->I2cDevAddr, &tof->range_status, &tof->range_mm);
        tof->cycle_time = now - tof->time_stamp;
        tof->time_stamp = now;
    }
}

static void send_obstacle_distance_all4_debug(void)
{
    mavlink_message_t msg;
    uint16_t distances[72];
    for (int i = 0; i < 72; i++) distances[i] = UINT16_MAX;

    uint16_t rc = tof_array[0].range_mm / 10;
    uint16_t tc = tof_array[1].range_mm / 10;
    uint16_t lc = tof_array[2].range_mm / 10;
    uint16_t fc = tof_array[3].range_mm / 10;

    if (rc < 4)   rc = 4; else if (rc > 400) rc = 400;
    if (tc < 4)   tc = 4; else if (tc > 400) tc = 400;
    if (lc < 4)   lc = 4; else if (lc > 400) lc = 400;
    if (fc < 4)   fc = 4; else if (fc > 400) fc = 400;

    distances[71] = distances[0]  = distances[1]  = fc;
    distances[17] = distances[18] = distances[19] = rc;
    distances[35] = distances[36] = distances[37] = tc;
    distances[53] = distances[54] = distances[55] = lc;

    mavlink_msg_obstacle_distance_pack(GCS_SYSID, GCS_COMPID, &msg,
        esp_timer_get_time(), MAV_DISTANCE_SENSOR_LASER,
        distances, 5, 4, 400, 5.0f, 0.0f, MAV_FRAME_BODY_FRD);
    mav_send(&msg);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Drone ID LED burst
 * ══════════════════════════════════════════════════════════════════════════ */

static void drone_id_led_update(void)
{
    static TickType_t last_tick = 0;
    static int state = 0, blink_index = 0;
    const TickType_t on_time     = pdMS_TO_TICKS(120);
    const TickType_t off_time    = pdMS_TO_TICKS(180);
    const TickType_t burst_pause = pdMS_TO_TICKS(1800);
    TickType_t now = xTaskGetTickCount();
    switch (state) {
        case 0:
            if ((now - last_tick) >= burst_pause) {
                blink_index = 0; gpio_set_level(DRONE_ID_LED_PIN, 1);
                last_tick = now; state = 1; }
            break;
        case 1:
            if ((now - last_tick) >= on_time) {
                gpio_set_level(DRONE_ID_LED_PIN, 0);
                last_tick = now; blink_index++; state = 2; }
            break;
        case 2:
            if (blink_index >= DRONE_ID) { state = 0; last_tick = now; }
            else if ((now - last_tick) >= off_time) {
                gpio_set_level(DRONE_ID_LED_PIN, 1); last_tick = now; state = 1; }
            break;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Camera config
 * ══════════════════════════════════════════════════════════════════════════ */

static const camera_config_t camera_config = {
    .pin_pwdn = CAMERA_PIN_PWDN, .pin_reset = CAMERA_PIN_RESET,
    .pin_xclk = CAMERA_PIN_XCLK,
    .pin_sccb_sda = CAMERA_PIN_SIOD, .pin_sccb_scl = CAMERA_PIN_SIOC,
    .pin_d7 = CAMERA_PIN_D7, .pin_d6 = CAMERA_PIN_D6,
    .pin_d5 = CAMERA_PIN_D5, .pin_d4 = CAMERA_PIN_D4,
    .pin_d3 = CAMERA_PIN_D3, .pin_d2 = CAMERA_PIN_D2,
    .pin_d1 = CAMERA_PIN_D1, .pin_d0 = CAMERA_PIN_D0,
    .pin_vsync = CAMERA_PIN_VSYNC, .pin_href = CAMERA_PIN_HREF,
    .pin_pclk  = CAMERA_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size   = FRAMESIZE_QQVGA,
    .jpeg_quality = 63, .fb_count = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
};

/* ══════════════════════════════════════════════════════════════════════════
 * Black-frame generator with "D{ID}" text
 *
 * 3×5 pixel bitmap font (3 bits per row, MSB = leftmost pixel).
 * Indices: 0=D  1='1'  2='2'  3='3'  4='4'  5='5'
 * ══════════════════════════════════════════════════════════════════════════ */

static const uint8_t BF_FONT[6][5] = {
    { 0b110, 0b101, 0b101, 0b101, 0b110 },  /* D */
    { 0b010, 0b110, 0b010, 0b010, 0b111 },  /* 1 */
    { 0b110, 0b001, 0b010, 0b100, 0b111 },  /* 2 */
    { 0b110, 0b001, 0b110, 0b001, 0b110 },  /* 3 */
    { 0b101, 0b101, 0b111, 0b001, 0b001 },  /* 4 */
    { 0b111, 0b100, 0b110, 0b001, 0b110 },  /* 5 */
};

static void bf_draw_char(uint8_t *buf, int cx, int cy, int fi, int scale)
{
    for (int row = 0; row < 5; row++) {
        for (int col = 0; col < 3; col++) {
            if (!((BF_FONT[fi][row] >> (2 - col)) & 1)) continue;
            for (int sy = 0; sy < scale; sy++) {
                for (int sx = 0; sx < scale; sx++) {
                    int px = cx + col * scale + sx;
                    int py = cy + row * scale + sy;
                    if (px >= 0 && px < 160 && py >= 0 && py < 120)
                        buf[py * 160 + px] = 0xFF;
                }
            }
        }
    }
}

/* Fill 160×120 mono8 buffer with black frame + centred "D{id}" */
static void generate_black_frame(uint8_t *buf, int id)
{
    memset(buf, 0, 160 * 120);
    const int scale  = 4;
    const int char_w = 3 * scale + scale;   /* char width + 1 px gap */
    int x0 = 80 - char_w;                   /* centre two chars */
    int y0 = 60 - (5 * scale) / 2;
    int fi = (id >= 1 && id <= 5) ? id : 1;
    bf_draw_char(buf, x0,          y0, 0,  scale);   /* D      */
    bf_draw_char(buf, x0 + char_w, y0, fi, scale);   /* digit  */
}

/* ══════════════════════════════════════════════════════════════════════════
 * micro-ROS resources
 * ══════════════════════════════════════════════════════════════════════════ */

static rcl_publisher_t    publisher_image;
static rcl_publisher_t    publisher_marker;
static rcl_publisher_t    publisher_state;
static rcl_publisher_t    publisher_role;

static visualization_msgs__msg__Marker   waypoint_arrow_msg;
static visualization_msgs__msg__Marker   waypoint_label_msg;
static geometry_msgs__msg__Point         wp_arrow_points[2];

static rcl_subscription_t command_sub;
static rcl_subscription_t config_sub;
static rcl_subscription_t control_sub;
static rcl_subscription_t vision_pose_sub;

static sensor_msgs__msg__Image           img_msg;
static visualization_msgs__msg__Marker   drone_disc_msg;
static visualization_msgs__msg__Marker   text_msg;
static visualization_msgs__msg__Marker   obstacle_labels[4];
static std_msgs__msg__String             command_msg;
static std_msgs__msg__String             config_msg;
static geometry_msgs__msg__PoseStamped   control_msg;
static geometry_msgs__msg__PoseStamped   vision_pose_msg;
static std_msgs__msg__String             state_pub_msg;
static std_msgs__msg__String             role_pub_msg;

static struct timespec ts;

static char topic_image[64];
static char topic_gcs_command[64];
static char topic_gcs_config[64];
static char topic_gcs_control[64];
static char topic_vision_pose[64];
static char topic_camera_frame[64];
static char topic_state[64];
static char topic_role[64];
static char drone_ns[16];

/* ══════════════════════════════════════════════════════════════════════════
 * Helpers
 * ══════════════════════════════════════════════════════════════════════════ */

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static float clamp_m(float x, float lo, float hi)
{
    return x < lo ? lo : x > hi ? hi : x;
}

static float get_initial_x_from_drone_id(void) { return 1.0f; }
static float get_initial_y_from_drone_id(void) { return (float)DRONE_ID; }
static float get_initial_z_from_drone_id(void) { return DRONE_DEFAULT_Z_M; }

/* Quaternion → Euler (ZYX, radians) — for VISION_POSITION_ESTIMATE */
static void quat_to_euler(float qx, float qy, float qz, float qw,
                           float *roll, float *pitch, float *yaw)
{
    *roll  = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy));
    *pitch = asinf( 2.0f*(qw*qy - qz*qx));
    *yaw   = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
}

static void set_label_color_by_distance(visualization_msgs__msg__Marker *m, float d)
{
    if (d < 0.30f)      { m->color.r=1.0f; m->color.g=0.0f; m->color.b=0.0f; }
    else if (d < 0.80f) { m->color.r=1.0f; m->color.g=0.6f; m->color.b=0.0f; }
    else                { m->color.r=0.0f; m->color.g=1.0f; m->color.b=0.0f; }
    m->color.a = 1.0f;
}

static void init_obstacle_label(visualization_msgs__msg__Marker *m,
                                 int id, const char *ns)
{
    visualization_msgs__msg__Marker__init(m);
    rosidl_runtime_c__String__assign(&m->header.frame_id, "map");
    rosidl_runtime_c__String__assign(&m->ns, ns);
    m->id     = id;
    m->type   = visualization_msgs__msg__Marker__TEXT_VIEW_FACING;
    m->action = visualization_msgs__msg__Marker__ADD;
    m->pose.orientation.w = 1.0f;
    m->scale.z = 0.12f;
    m->color.r = m->color.g = m->color.b = m->color.a = 1.0f;
    m->text.data     = (char *)malloc(16);
    m->text.size     = 0;
    m->text.capacity = 16;
}

static void apply_pose_to_drone_markers(float px, float py, float pz,
                                         float qx, float qy, float qz, float qw)
{
    drone_disc_msg.pose.position.x    = px;
    drone_disc_msg.pose.position.y    = py;
    drone_disc_msg.pose.position.z    = pz;
    drone_disc_msg.pose.orientation.x = qx;
    drone_disc_msg.pose.orientation.y = qy;
    drone_disc_msg.pose.orientation.z = qz;
    drone_disc_msg.pose.orientation.w = qw;

    text_msg.pose.position.x = px;
    text_msg.pose.position.y = py;
    text_msg.pose.position.z = pz + 0.22f;
    text_msg.pose.orientation.x = 0.0f;
    text_msg.pose.orientation.y = 0.0f;
    text_msg.pose.orientation.z = 0.0f;
    text_msg.pose.orientation.w = 1.0f;
}

static void update_obstacle_markers(void)
{
    float dx = drone_disc_msg.pose.position.x;
    float dy = drone_disc_msg.pose.position.y;
    float dz = drone_disc_msg.pose.position.z;
    const float offset = 0.20f;

    float rm = clamp_m(tof_array[0].range_mm / 1000.0f, 0.05f, 4.0f);
    float tm = clamp_m(tof_array[1].range_mm / 1000.0f, 0.05f, 4.0f);
    float lm = clamp_m(tof_array[2].range_mm / 1000.0f, 0.05f, 4.0f);
    float fm = clamp_m(tof_array[3].range_mm / 1000.0f, 0.05f, 4.0f);

    obstacle_labels[0].pose.position.x = dx;
    obstacle_labels[0].pose.position.y = dy - offset;
    obstacle_labels[0].pose.position.z = dz;

    obstacle_labels[1].pose.position.x = dx;
    obstacle_labels[1].pose.position.y = dy;
    obstacle_labels[1].pose.position.z = dz + offset;

    obstacle_labels[2].pose.position.x = dx;
    obstacle_labels[2].pose.position.y = dy + offset;
    obstacle_labels[2].pose.position.z = dz;

    obstacle_labels[3].pose.position.x = dx + offset;
    obstacle_labels[3].pose.position.y = dy;
    obstacle_labels[3].pose.position.z = dz;

    snprintf(obstacle_labels[0].text.data, 16, "%d", tof_array[0].range_mm / 10);
    snprintf(obstacle_labels[1].text.data, 16, "%d", tof_array[1].range_mm / 10);
    snprintf(obstacle_labels[2].text.data, 16, "%d", tof_array[2].range_mm / 10);
    snprintf(obstacle_labels[3].text.data, 16, "%d", tof_array[3].range_mm / 10);

    for (int i = 0; i < 4; i++)
        obstacle_labels[i].text.size = strlen(obstacle_labels[i].text.data);

    set_label_color_by_distance(&obstacle_labels[0], rm);
    set_label_color_by_distance(&obstacle_labels[1], tm);
    set_label_color_by_distance(&obstacle_labels[2], lm);
    set_label_color_by_distance(&obstacle_labels[3], fm);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Mission FreeRTOS task
 *
 * Sequence:
 *   1. Stream idle setpoints 1 s  → PX4 accepts OFFBOARD switch
 *   2. ARM + switch to OFFBOARD
 *   3. Climb to MISSION_TAKEOFF_ALT_M (5 s)
 *   4. Fly to setpoint, dwell (3 s)
 *   5. Return above home, send NAV_LAND, wait, disarm
 * ══════════════════════════════════════════════════════════════════════════ */

static void mission_task_fn(void *arg)
{
    TickType_t t0;
    uint32_t elapsed;

    /* Phase 1: pre-arm stream (PX4 requires >2 Hz setpoints before OFFBOARD) */
    t0 = xTaskGetTickCount();
    do {
        if (drone_state != DRONE_MISSION) goto mission_abort;
        mav_set_position_ned(home_x, home_y, -0.1f);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < 1000);

    /* Phase 2: arm + OFFBOARD */
    mav_arm(true);
    vTaskDelay(pdMS_TO_TICKS(500));
    mav_set_mode(PX4_MODE_OFFBOARD);

    /* Phase 3: climb */
    t0 = xTaskGetTickCount();
    do {
        if (drone_state != DRONE_MISSION) goto mission_abort;
        mav_set_position_ned(home_x, home_y, -MISSION_TAKEOFF_ALT_M);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < MISSION_TAKEOFF_WAIT_MS);

    /* Phase 4: fly to setpoint and dwell */
    float sp_x = setpoint_received ? setpoint_x : home_x;
    float sp_y = setpoint_received ? setpoint_y : home_y;
    float sp_z = setpoint_received ? -setpoint_z : -MISSION_TAKEOFF_ALT_M;

    t0 = xTaskGetTickCount();
    do {
        if (drone_state != DRONE_MISSION) goto mission_abort;
        mav_set_position_ned(sp_x, sp_y, sp_z);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < MISSION_SETPOINT_DWELL_MS);

    /* Phase 5: land on spot */
    drone_state = DRONE_LANDING;
    state_dirty = true;

    mav_eland();
    vTaskDelay(pdMS_TO_TICKS(MISSION_LAND_DESCEND_MS));

    mav_arm(false);
    drone_state = DRONE_DISARMED;
    state_dirty = true;
    goto mission_done;

mission_abort:
    ESP_LOGW(TAG, "Mission aborted (state override)");
mission_done:
    mission_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Return-home FreeRTOS task
 * ══════════════════════════════════════════════════════════════════════════ */

static void return_home_task_fn(void *arg)
{
    TickType_t t0 = xTaskGetTickCount();
    uint32_t elapsed;

    /* Stream home position above home XY at cruise altitude */
    do {
        if (drone_state != DRONE_RETURNING_HOME) goto rh_done;
        mav_set_position_ned(home_x, home_y, -MISSION_TAKEOFF_ALT_M);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < 5000);

    /* Land */
    drone_state = DRONE_LANDING;
    state_dirty = true;
    mav_eland();
    vTaskDelay(pdMS_TO_TICKS(MISSION_LAND_DESCEND_MS));

    mav_arm(false);
    drone_state = DRONE_DISARMED;
    state_dirty = true;

rh_done:
    return_home_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Emergency-land FreeRTOS task
 * ══════════════════════════════════════════════════════════════════════════ */

static void eland_task_fn(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(MISSION_LAND_DESCEND_MS));
    mav_arm(false);
    drone_state = DRONE_DISARMED;
    state_dirty = true;
    eland_task_handle = NULL;
    vTaskDelete(NULL);
}

static void trigger_eland(void)
{
    if (mission_task_handle)     { vTaskDelete(mission_task_handle);     mission_task_handle = NULL; }
    if (return_home_task_handle) { vTaskDelete(return_home_task_handle); return_home_task_handle = NULL; }
    if (eland_task_handle)       { vTaskDelete(eland_task_handle);       eland_task_handle = NULL; }
    mav_eland();
    drone_state = DRONE_LANDING;
    state_dirty = true;
    xTaskCreate(eland_task_fn, "eland", 2048, NULL, 5, &eland_task_handle);
}


/* ══════════════════════════════════════════════════════════════════════════
 * GCS command callback  (/gcs/drone_{ID}/command)
 * ══════════════════════════════════════════════════════════════════════════ */

static void command_callback(const void *msg_in)
{
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msg_in;
    if (!m || !m->data.data || m->data.size == 0) return;

    char buf[64] = {0};
    size_t n = m->data.size < 63 ? m->data.size : 63;
    memcpy(buf, m->data.data, n);
    for (int i = (int)n - 1; i >= 0; i--) {
        if (buf[i]==' '||buf[i]=='\n'||buf[i]=='\r'||buf[i]=='\t') buf[i]='\0'; else break;
    }
    ESP_LOGI(TAG, "CMD: %s", buf);

    if (strcmp(buf, "COMMAND_ARM") == 0) {
        if (vision_pose_valid) {
            home_x = vp_x; home_y = vp_y; home_z = vp_z;
            ESP_LOGI(TAG, "Home captured: (%.2f, %.2f, %.2f)", home_x, home_y, home_z);
        }
        mav_arm(true);
        drone_state = DRONE_ARMED;
        state_dirty = true;

    } else if (strcmp(buf, "COMMAND_DISARM") == 0) {
        mav_arm(false);
        drone_state = DRONE_DISARMED;
        state_dirty = true;

    } else if (strcmp(buf, "COMMAND_MISSION_START") == 0) {
        if (drone_state != DRONE_ARMED && drone_state != DRONE_RETURNING_HOME) {
            ESP_LOGW(TAG, "Mission blocked — state: %s", state_names[drone_state]);
            return;
        }
        if (return_home_task_handle) {
            vTaskDelete(return_home_task_handle);
            return_home_task_handle = NULL;
        }
        drone_state = DRONE_MISSION;
        state_dirty = true;
        xTaskCreate(mission_task_fn, "mission", 4096, NULL, 5, &mission_task_handle);

    } else if (strcmp(buf, "COMMAND_RETURN_HOME") == 0) {
        if (drone_state != DRONE_MISSION && drone_state != DRONE_RETURNING_HOME) {
            ESP_LOGW(TAG, "RTH blocked — not in flight");
            return;
        }
        if (mission_task_handle) {
            vTaskDelete(mission_task_handle);
            mission_task_handle = NULL;
        }
        drone_state = DRONE_RETURNING_HOME;
        state_dirty = true;
        mav_set_mode(PX4_MODE_OFFBOARD);
        xTaskCreate(return_home_task_fn, "return_home", 4096, NULL, 5, &return_home_task_handle);

    } else if (strcmp(buf, "COMMAND_ELAND") == 0) {
        trigger_eland();

    } else if (strcmp(buf, "COMMAND_KILL") == 0) {
        if (mission_task_handle)     { vTaskDelete(mission_task_handle);     mission_task_handle = NULL; }
        if (return_home_task_handle) { vTaskDelete(return_home_task_handle); return_home_task_handle = NULL; }
        mav_kill();
        drone_state = DRONE_KILLED;
        state_dirty = true;

    } else {
        ESP_LOGW(TAG, "CMD: unknown '%s'", buf);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * GCS config callback  (/gcs/drone_{ID}/config)
 * ══════════════════════════════════════════════════════════════════════════ */

static void config_callback(const void *msg_in)
{
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msg_in;
    if (!m || !m->data.data || m->data.size == 0) return;

    char buf[64] = {0};
    size_t n = m->data.size < 63 ? m->data.size : 63;
    memcpy(buf, m->data.data, n);
    for (int i = (int)n - 1; i >= 0; i--) {
        if (buf[i]==' '||buf[i]=='\n'||buf[i]=='\r'||buf[i]=='\t') buf[i]='\0'; else break;
    }
    ESP_LOGI(TAG, "CFG: %s", buf);

    if (strcmp(buf, "CONFIG_CAMERA_ENABLE") == 0) {
        camera_streaming = true;

    } else if (strcmp(buf, "CONFIG_CAMERA_DISABLE") == 0) {
        camera_streaming = false;

    } else if (strcmp(buf, "CONFIG_VISION_ENABLE") == 0) {
        vision_enabled = true;
        ESP_LOGI(TAG, "ArUco EKF: ON → VISION_POSITION_ESTIMATE to PX4");

    } else if (strcmp(buf, "CONFIG_VISION_DISABLE") == 0) {
        vision_enabled = false;
        ESP_LOGI(TAG, "ArUco EKF: OFF");

    } else if (strcmp(buf, "CONFIG_SOURCE_RC") == 0) {
        gcs_control_active = false;
        mav_set_mode(PX4_MODE_STABILIZED);   /* RC takes over */

    } else if (strcmp(buf, "CONFIG_SOURCE_GCS") == 0) {
        gcs_control_active = true;
        mav_set_mode(PX4_MODE_OFFBOARD);     /* onboard controller */

    } else {
        ESP_LOGW(TAG, "CFG: unknown '%s'", buf);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * GCS control callback  (/gcs/drone_{ID}/control)
 *
 * Receives position setpoints as PoseStamped:
 *   position.x/y  — horizontal NED metres
 *   position.z    — altitude in metres, positive = up
 *                   converted to NED z_ned = -position.z before sending
 * ══════════════════════════════════════════════════════════════════════════ */

static void control_callback(const void *msg_in)
{
    const geometry_msgs__msg__PoseStamped *msg =
        (const geometry_msgs__msg__PoseStamped *)msg_in;
    if (!msg) return;

    setpoint_x = (float)msg->pose.position.x;
    setpoint_y = (float)msg->pose.position.y;
    setpoint_z = (float)msg->pose.position.z;   /* positive up */
    setpoint_received = true;

    /* Update waypoint arrow tip to new setpoint */
    wp_arrow_points[1].x = setpoint_x;
    wp_arrow_points[1].y = setpoint_y;
    wp_arrow_points[1].z = setpoint_z;

    ESP_LOGI(TAG, "Setpoint: (%.2f, %.2f, %.2f up)", setpoint_x, setpoint_y, setpoint_z);

    /* In GCS OFFBOARD mode, forward immediately to PX4 */
    if (gcs_control_active && drone_state == DRONE_ARMED) {
        mav_set_position_ned(setpoint_x, setpoint_y, -setpoint_z);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Vision pose callback  (/drone_{ID}/vision_pose)
 * ══════════════════════════════════════════════════════════════════════════ */

static void vision_pose_callback(const void *msg_in)
{
    const geometry_msgs__msg__PoseStamped *msg =
        (const geometry_msgs__msg__PoseStamped *)msg_in;
    if (!msg) { ESP_LOGW(TAG, "Vision pose: null"); return; }

    float px = (float)msg->pose.position.x;
    float py = (float)msg->pose.position.y;
    float pz = (float)msg->pose.position.z;
    float qx = (float)msg->pose.orientation.x;
    float qy = (float)msg->pose.orientation.y;
    float qz = (float)msg->pose.orientation.z;
    float qw = (float)msg->pose.orientation.w;

    apply_pose_to_drone_markers(px, py, pz, qx, qy, qz, qw);

    vp_x = px; vp_y = py; vp_z = pz;
    vp_qx = qx; vp_qy = qy; vp_qz = qz; vp_qw = qw;
    vision_pose_valid   = true;
    last_vision_pose_ms = now_ms();

    /* Forward to PX4 EKF2 when vision is enabled */
    if (vision_enabled) {
        float roll, pitch, yaw;
        quat_to_euler(qx, qy, qz, qw, &roll, &pitch, &yaw);
        mav_send_vision_estimate(px, py, pz, roll, pitch, yaw);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Timer callback — 100 ms  (camera, RViz markers, state publish)
 * ══════════════════════════════════════════════════════════════════════════ */

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (!timer) return;

    /* Vision timeout */
    if (vision_pose_valid && (now_ms() - last_vision_pose_ms) > VISION_TIMEOUT_MS) {
        vision_pose_valid = false;
        ESP_LOGW(TAG, "Vision pose timeout");
    }

    /* Publish state on change */
    if (state_dirty) {
        state_dirty = false;
        const char *s = state_names[(int)drone_state];
        rosidl_runtime_c__String__assign(&state_pub_msg.data, s);
        RCSOFTCHECK(rcl_publish(&publisher_state, &state_pub_msg, NULL));
        ESP_LOGI(TAG, "State → %s", s);
    }

    /* Camera / black frame */
    if (camera_streaming) {
        camera_fb_t *pic = esp_camera_fb_get();
        if (pic) {
            if (pic->len <= img_msg.data.capacity) {
                clock_gettime(CLOCK_REALTIME, &ts);
                img_msg.header.stamp.sec     = ts.tv_sec;
                img_msg.header.stamp.nanosec = ts.tv_nsec;
                img_msg.header.frame_id =
                    micro_ros_string_utilities_set(img_msg.header.frame_id, topic_camera_frame);
                img_msg.width    = 160; img_msg.height = 120; img_msg.step = 160;
                img_msg.encoding = micro_ros_string_utilities_set(img_msg.encoding, "mono8");
                img_msg.data.size = pic->len;
                memcpy(img_msg.data.data, pic->buf, pic->len);
                RCSOFTCHECK(rcl_publish(&publisher_image, &img_msg, NULL));
            }
            esp_camera_fb_return(pic);
        } else {
            ESP_LOGW(TAG, "Camera capture failed");
        }
    } else {
        /* Publish synthetic black frame with drone ID */
        if (img_msg.data.capacity >= 160 * 120) {
            clock_gettime(CLOCK_REALTIME, &ts);
            img_msg.header.stamp.sec     = ts.tv_sec;
            img_msg.header.stamp.nanosec = ts.tv_nsec;
            img_msg.header.frame_id =
                micro_ros_string_utilities_set(img_msg.header.frame_id, topic_camera_frame);
            img_msg.width    = 160; img_msg.height = 120; img_msg.step = 160;
            img_msg.encoding = micro_ros_string_utilities_set(img_msg.encoding, "mono8");
            img_msg.data.size = 160 * 120;
            generate_black_frame(img_msg.data.data, DRONE_ID);
            RCSOFTCHECK(rcl_publish(&publisher_image, &img_msg, NULL));
        }
    }

    /* RViz markers */
    update_obstacle_markers();
    RCSOFTCHECK(rcl_publish(&publisher_marker, &drone_disc_msg, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_marker, &text_msg, NULL));
    for (int i = 0; i < 4; i++)
        RCSOFTCHECK(rcl_publish(&publisher_marker, &obstacle_labels[i], NULL));

    /* Waypoint arrow — tail tracks live drone position, tip fixed at setpoint */
    if (setpoint_received) {
        wp_arrow_points[0].x = vp_x;
        wp_arrow_points[0].y = vp_y;
        wp_arrow_points[0].z = vp_z;
        waypoint_label_msg.pose.position.x = (vp_x + setpoint_x) * 0.5f;
        waypoint_label_msg.pose.position.y = (vp_y + setpoint_y) * 0.5f;
        waypoint_label_msg.pose.position.z = (vp_z + setpoint_z) * 0.5f + (DRONE_ID * 0.15f);
        RCSOFTCHECK(rcl_publish(&publisher_marker, &waypoint_arrow_msg, NULL));
        RCSOFTCHECK(rcl_publish(&publisher_marker, &waypoint_label_msg, NULL));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * micro-ROS task
 * ══════════════════════════════════════════════════════════════════════════ */

static void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
    while (rclc_support_init_with_options(&support, 0, NULL,
                                           &init_options, &allocator) != RCL_RET_OK) {
        ESP_LOGW(TAG, "Agent not reachable, retrying in 2 s...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGI(TAG, "micro-ROS agent connected!");

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_drone_brain", "", &support));

    /* ── Publishers ──────────────────────────────────────────────────────── */
    RCCHECK(rclc_publisher_init_default(&publisher_image, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), topic_image));

    RCCHECK(rclc_publisher_init_default(&publisher_marker, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, Marker),
        "/visualization_marker"));

    RCCHECK(rclc_publisher_init_default(&publisher_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_state));

    RCCHECK(rclc_publisher_init_default(&publisher_role, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_role));

    /* ── Subscribers ─────────────────────────────────────────────────────── */
    RCCHECK(rclc_subscription_init_best_effort(&command_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_gcs_command));

    RCCHECK(rclc_subscription_init_best_effort(&config_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_gcs_config));

    RCCHECK(rclc_subscription_init_best_effort(&control_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        topic_gcs_control));

    RCCHECK(rclc_subscription_init_best_effort(&vision_pose_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        topic_vision_pose));

    /* ── Message buffers ─────────────────────────────────────────────────── */
    command_msg.data.data = (char *)malloc(64);
    command_msg.data.size = 0; command_msg.data.capacity = 64;

    config_msg.data.data = (char *)malloc(64);
    config_msg.data.size = 0; config_msg.data.capacity = 64;

    geometry_msgs__msg__PoseStamped__init(&control_msg);
    geometry_msgs__msg__PoseStamped__init(&vision_pose_msg);

    state_pub_msg.data.data = (char *)malloc(32);
    state_pub_msg.data.size = 0; state_pub_msg.data.capacity = 32;

    role_pub_msg.data.data = (char *)malloc(32);
    role_pub_msg.data.size = 0; role_pub_msg.data.capacity = 32;

    static micro_ros_utilities_memory_conf_t conf = {};
    conf.max_string_capacity             = 50;
    conf.max_ros2_type_sequence_capacity = 5;
    conf.max_basic_type_sequence_capacity = 5;
    micro_ros_utilities_memory_rule_t const rules[] = {
        {"header.frame_id", 40},
        {"format",           5},
        {"encoding",        10},
        {"data",         19200},
    };
    conf.rules   = rules;
    conf.n_rules = sizeof(rules) / sizeof(rules[0]);
    micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), &img_msg, conf);

    /* Publish initial role — not yet used, placeholder */
    rosidl_runtime_c__String__assign(&role_pub_msg.data, "idle");
    RCSOFTCHECK(rcl_publish(&publisher_role, &role_pub_msg, NULL));

    /* ── Timer + executor (1 timer + 4 subscriptions = 5 handles) ─────────── */
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &command_sub,
                &command_msg, &command_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &config_sub,
                &config_msg, &config_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &control_sub,
                &control_msg, &control_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &vision_pose_sub,
                &vision_pose_msg, &vision_pose_callback, ON_NEW_DATA));

    /* C2 watchdog state — ping runs in this task to avoid transport race conditions */
    int64_t last_c2_check_ms = now_ms();
    int     c2_failures       = 0;

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));

        int64_t t = now_ms();
        if (t - last_c2_check_ms >= C2_CHECK_INTERVAL_MS) {
            last_c2_check_ms = t;
            if (rmw_uros_ping_agent(C2_PING_TIMEOUT_MS, C2_PING_ATTEMPTS) == RMW_RET_OK) {
                c2_failures = 0;
            } else {
                c2_failures++;
                ESP_LOGW(TAG, "C2 ping failed (%d/%d)", c2_failures, C2_FAIL_THRESHOLD);
                if (c2_failures >= C2_FAIL_THRESHOLD) {
                    c2_failures = 0;
                    if (drone_state != DRONE_LANDING &&
                        drone_state != DRONE_DISARMED &&
                        drone_state != DRONE_KILLED) {
                        ESP_LOGE(TAG, "C2 link lost — initiating emergency landing");
                        trigger_eland();
                    }
                }
            }
        }

        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&publisher_image,  &node));
    RCCHECK(rcl_publisher_fini(&publisher_marker, &node));
    RCCHECK(rcl_publisher_fini(&publisher_state,  &node));
    RCCHECK(rcl_publisher_fini(&publisher_role,   &node));
    RCCHECK(rcl_subscription_fini(&command_sub,      &node));
    RCCHECK(rcl_subscription_fini(&config_sub,        &node));
    RCCHECK(rcl_subscription_fini(&control_sub,       &node));
    RCCHECK(rcl_subscription_fini(&vision_pose_sub,   &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Device hostname / static IP
 * ══════════════════════════════════════════════════════════════════════════ */

static void set_device_hostname_from_drone_id(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) { ESP_LOGW(TAG, "netif not found"); return; }
    char hostname[32];
    snprintf(hostname, sizeof(hostname), "mach-mind-drone-%d", DRONE_ID);
    if (esp_netif_set_hostname(netif, hostname) == ESP_OK)
        ESP_LOGI(TAG, "Hostname: %s", hostname);
}

static esp_err_t set_preferred_ip_from_drone_id(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) { ESP_LOGE(TAG, "netif not found"); return ESP_FAIL; }
    esp_err_t err = esp_netif_dhcpc_stop(netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) return err;
    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));
    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), DRONE_IP_PREFIX "%d", DRONE_IP_BASE_OCTET + DRONE_ID);
    ip4addr_aton(ip_str,           (ip4_addr_t *)&ip_info.ip);
    ip4addr_aton(DRONE_IP_GATEWAY, (ip4_addr_t *)&ip_info.gw);
    ip4addr_aton(DRONE_IP_NETMASK, (ip4_addr_t *)&ip_info.netmask);
    err = esp_netif_set_ip_info(netif, &ip_info);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "Static IP: %s  GW: %s", ip_str, DRONE_IP_GATEWAY);
    return err;
}

static void print_current_ip_info(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return;
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(netif, &ip) == ESP_OK)
        ESP_LOGI(TAG, "IP: " IPSTR "  GW: " IPSTR "  MASK: " IPSTR,
                 IP2STR(&ip.ip), IP2STR(&ip.gw), IP2STR(&ip.netmask));
}

/* ══════════════════════════════════════════════════════════════════════════
 * app_main
 * ══════════════════════════════════════════════════════════════════════════ */

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    set_device_hostname_from_drone_id();
    ESP_ERROR_CHECK(set_preferred_ip_from_drone_id());
    print_current_ip_info();
#endif

    /* Build per-drone topic strings */
    snprintf(drone_ns,           sizeof(drone_ns),           "drone_%d",                   DRONE_ID);
    snprintf(topic_image,        sizeof(topic_image),         "/drone_%d/camera/image_raw", DRONE_ID);
    snprintf(topic_vision_pose,  sizeof(topic_vision_pose),   "/drone_%d/vision_pose",      DRONE_ID);
    snprintf(topic_camera_frame, sizeof(topic_camera_frame),  "drone_%d/camera",            DRONE_ID);
    snprintf(topic_gcs_command,  sizeof(topic_gcs_command),   "/gcs/drone_%d/command",      DRONE_ID);
    snprintf(topic_gcs_config,   sizeof(topic_gcs_config),    "/gcs/drone_%d/config",       DRONE_ID);
    snprintf(topic_gcs_control,  sizeof(topic_gcs_control),   "/gcs/drone_%d/control",      DRONE_ID);
    snprintf(topic_state,        sizeof(topic_state),          "/drone_%d/state",            DRONE_ID);
    snprintf(topic_role,         sizeof(topic_role),           "/drone_%d/role",             DRONE_ID);

    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "DRONE ID   : %d",           DRONE_ID);
    ESP_LOGI(TAG, "Namespace  : %s",           drone_ns);
    ESP_LOGI(TAG, "CMD topic  : %s",           topic_gcs_command);
    ESP_LOGI(TAG, "CFG topic  : %s",           topic_gcs_config);
    ESP_LOGI(TAG, "CTL topic  : %s",           topic_gcs_control);
    ESP_LOGI(TAG, "State topic: %s",           topic_state);
    ESP_LOGI(TAG, "Takeoff alt: %.1f m",       MISSION_TAKEOFF_ALT_M);
    ESP_LOGI(TAG, "Hostname   : mach-mind-drone-%d", DRONE_ID);
    ESP_LOGI(TAG, "Expected IP: 192.168.178.%d", 100 + DRONE_ID);
    ESP_LOGI(TAG, "==============================");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DRONE_ID_LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = 0, .pull_down_en = 0,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\r\nDevice Starting — Drone ID %d\r\n", DRONE_ID);

    i2c_init();
    uart_mavlink_init();
    VL53L1X_InitSensorArray(tof_array, sensor_count);

    const esp_timer_create_args_t tof_timer_args = {
        .callback = &periodic_tof_sensor, .name = "tofsensor"
    };
    ESP_ERROR_CHECK(esp_timer_create(&tof_timer_args, &tof_sensor_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tof_sensor_timer, TIMER_PERIODIC_MS * 1000));

    if (esp_camera_init(&camera_config) != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed — black-frame mode only");
        camera_streaming = false;
    } else {
        ESP_LOGI(TAG, "Camera OK");
        sensor_t *s = esp_camera_sensor_get();
        if (s) { s->set_vflip(s, 1); s->set_hmirror(s, 1); }
    }

    const float ix = get_initial_x_from_drone_id();
    const float iy = get_initial_y_from_drone_id();
    const float iz = get_initial_z_from_drone_id();

    visualization_msgs__msg__Marker__init(&drone_disc_msg);
    rosidl_runtime_c__String__assign(&drone_disc_msg.header.frame_id, "map");
    rosidl_runtime_c__String__assign(&drone_disc_msg.ns, drone_ns);
    drone_disc_msg.id     = MARKER_ID_DISC(DRONE_ID);
    drone_disc_msg.type   = visualization_msgs__msg__Marker__CYLINDER;
    drone_disc_msg.action = visualization_msgs__msg__Marker__ADD;
    drone_disc_msg.pose.position.x    = ix;
    drone_disc_msg.pose.position.y    = iy;
    drone_disc_msg.pose.position.z    = iz;
    drone_disc_msg.pose.orientation.w = 1.0f;
    drone_disc_msg.scale.x = DRONE_DISC_DIAMETER_M;
    drone_disc_msg.scale.y = DRONE_DISC_DIAMETER_M;
    drone_disc_msg.scale.z = DRONE_DISC_THICKNESS_M;
    drone_disc_msg.color.r = 0.2f;
    drone_disc_msg.color.g = 0.6f;
    drone_disc_msg.color.b = 1.0f;
    drone_disc_msg.color.a = 0.95f;

    visualization_msgs__msg__Marker__init(&text_msg);
    rosidl_runtime_c__String__assign(&text_msg.header.frame_id, "map");
    rosidl_runtime_c__String__assign(&text_msg.ns, drone_ns);
    text_msg.id     = MARKER_ID_TEXT(DRONE_ID);
    text_msg.type   = visualization_msgs__msg__Marker__TEXT_VIEW_FACING;
    text_msg.action = visualization_msgs__msg__Marker__ADD;
    text_msg.pose.position.x    = ix;
    text_msg.pose.position.y    = iy;
    text_msg.pose.position.z    = iz + 0.22f;
    text_msg.pose.orientation.w = 1.0f;
    text_msg.scale.z = 0.16f;
    text_msg.color.r = text_msg.color.g = text_msg.color.b = text_msg.color.a = 1.0f;
    char drone_label[8];
    snprintf(drone_label, sizeof(drone_label), "D%d", DRONE_ID);
    rosidl_runtime_c__String__assign(&text_msg.text, drone_label);

    char obs_ns[4][32];
    snprintf(obs_ns[0], sizeof(obs_ns[0]), "%s_obs_right", drone_ns);
    snprintf(obs_ns[1], sizeof(obs_ns[1]), "%s_obs_top",   drone_ns);
    snprintf(obs_ns[2], sizeof(obs_ns[2]), "%s_obs_left",  drone_ns);
    snprintf(obs_ns[3], sizeof(obs_ns[3]), "%s_obs_front", drone_ns);
    init_obstacle_label(&obstacle_labels[0], MARKER_ID_RIGHT(DRONE_ID), obs_ns[0]);
    init_obstacle_label(&obstacle_labels[1], MARKER_ID_TOP(DRONE_ID),   obs_ns[1]);
    init_obstacle_label(&obstacle_labels[2], MARKER_ID_LEFT(DRONE_ID),  obs_ns[2]);
    init_obstacle_label(&obstacle_labels[3], MARKER_ID_FRONT(DRONE_ID), obs_ns[3]);

    apply_pose_to_drone_markers(ix, iy, iz, 0.0f, 0.0f, 0.0f, 1.0f);
    update_obstacle_markers();

    /* Waypoint arrow marker (2-point ARROW: tail = drone, tip = setpoint) */
    visualization_msgs__msg__Marker__init(&waypoint_arrow_msg);
    rosidl_runtime_c__String__assign(&waypoint_arrow_msg.header.frame_id, "map");
    rosidl_runtime_c__String__assign(&waypoint_arrow_msg.ns, drone_ns);
    waypoint_arrow_msg.id                = MARKER_ID_WP_ARROW(DRONE_ID);
    waypoint_arrow_msg.type              = visualization_msgs__msg__Marker__ARROW;
    waypoint_arrow_msg.action            = visualization_msgs__msg__Marker__ADD;
    waypoint_arrow_msg.pose.orientation.w = 1.0f;
    waypoint_arrow_msg.scale.x           = 0.02f;   /* shaft diameter */
    waypoint_arrow_msg.scale.y           = 0.05f;   /* arrowhead diameter */
    waypoint_arrow_msg.scale.z           = 0.0f;    /* auto arrowhead length */
    waypoint_arrow_msg.color.r           = 1.0f;
    waypoint_arrow_msg.color.g           = 0.8f;
    waypoint_arrow_msg.color.b           = 0.0f;
    waypoint_arrow_msg.color.a           = 0.9f;
    waypoint_arrow_msg.points.data       = wp_arrow_points;
    waypoint_arrow_msg.points.size       = 2;
    waypoint_arrow_msg.points.capacity   = 2;

    /* Waypoint label marker (midpoint + per-drone Z stagger) */
    visualization_msgs__msg__Marker__init(&waypoint_label_msg);
    rosidl_runtime_c__String__assign(&waypoint_label_msg.header.frame_id, "map");
    rosidl_runtime_c__String__assign(&waypoint_label_msg.ns, drone_ns);
    waypoint_label_msg.id                = MARKER_ID_WP_TEXT(DRONE_ID);
    waypoint_label_msg.type              = visualization_msgs__msg__Marker__TEXT_VIEW_FACING;
    waypoint_label_msg.action            = visualization_msgs__msg__Marker__ADD;
    waypoint_label_msg.pose.orientation.w = 1.0f;
    waypoint_label_msg.scale.z           = 0.12f;
    waypoint_label_msg.color.r           = 1.0f;
    waypoint_label_msg.color.g           = 0.8f;
    waypoint_label_msg.color.b           = 0.0f;
    waypoint_label_msg.color.a           = 1.0f;
    char wp_label[16];
    snprintf(wp_label, sizeof(wp_label), "D%d WP", DRONE_ID);
    rosidl_runtime_c__String__assign(&waypoint_label_msg.text, wp_label);

    xTaskCreate(micro_ros_task, "uros_task",
                CONFIG_MICRO_ROS_APP_STACK, NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);

    while (1) {
        drone_id_led_update();
        send_heartbeat_once();
        send_obstacle_distance_all4_debug();

        printf("\r RIGHT: %4d cm | TOP: %4d cm | LEFT: %4d cm | "
               "FRONT: %4d cm | STATE: %-15s | VISION: %s    ",
               tof_array[0].range_mm / 10,
               tof_array[1].range_mm / 10,
               tof_array[2].range_mm / 10,
               tof_array[3].range_mm / 10,
               state_names[(int)drone_state],
               vision_pose_valid ? "OK" : "NO");

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
