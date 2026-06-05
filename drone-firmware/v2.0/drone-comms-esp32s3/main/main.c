/* main.c — Mach Mind Drone ESP32S3 Comms Firmware (v2.0)
 *
 * Role: communication bridge between ESP32-P4 sensor board and PX4/GCS.
 *
 * Data sources:
 *   ESP32-P4 → UART2 (115200 baud, binary framed):
 *     - 6× VL53L1X ToF distances (slots 0–5, see sensor map below)
 *     - ArUco world-pose estimate (x/y/z/quaternion, valid flag)
 *
 * Sensor map (body-frame angles, FRD convention):
 *   Slot 0  Left     −90°   MAV_SENSOR_ROTATION_YAW_270
 *   Slot 1  L-front  −45°   MAV_SENSOR_ROTATION_YAW_315
 *   Slot 2  Front      0°   MAV_SENSOR_ROTATION_NONE
 *   Slot 3  R-front  +45°   MAV_SENSOR_ROTATION_YAW_45
 *   Slot 4  Right    +90°   MAV_SENSOR_ROTATION_YAW_90
 *   Slot 5  Up        —     MAV_SENSOR_ROTATION_PITCH_90
 *
 * MAVLink → PX4 (UART1, 57600 baud):
 *   HEARTBEAT, OBSTACLE_DISTANCE (5 horizontal sensors),
 *   DISTANCE_SENSOR (slot 5, upward), VISION_POSITION_ESTIMATE,
 *   COMPONENT_ARM_DISARM, SET_MODE,
 *   SET_POSITION_TARGET_LOCAL_NED, NAV_LAND
 *
 * ROS topics (subscribed, via micro-ROS):
 *   /gcs/drone_{ID}/command    std_msgs/String  — flight commands
 *   /gcs/drone_{ID}/config     std_msgs/String  — CONFIG_VISION_ENABLE/DISABLE
 *   /gcs/drone_{ID}/control    PoseStamped      — position setpoints
 *   /gcs/system/team_color     std_msgs/String  — "red" (LH) or "blue" (RH) — positions RViz disc
 *
 * ROS topics (published, via micro-ROS):
 *   /drone_{ID}/state          std_msgs/String  — state machine
 *   /drone_{ID}/role           std_msgs/String  — role
 *   /drone_{ID}/battery        std_msgs/Int8    — battery % (0–100, -1=unknown)
 *   /visualization_marker      visualization_msgs/Marker — RViz drone disc
 *
 * OBSTACLE_DISTANCE bin map (5°/bin, 72 bins, bin 0 = forward, CW):
 *   Slot 2 Front    bins 69,70,71,0,1,2
 *   Slot 3 R-front  bins 6,7,8,9,10,11
 *   Slot 4 Right    bins 15,16,17,18,19,20
 *   Slot 0 Left     bins 51,52,53,54,55,56
 *   Slot 1 L-front  bins 60,61,62,63,64,65
 *
 * Takeoff altitude options:
 *   A: 1.0 m  — tight formations only  ← default
 *   B: 1.5 m  — recommended
 *   C: 2.0 m  — close proximity swarm
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
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
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/ip4_addr.h"

#include <c_library_v2/common/mavlink.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rosidl_runtime_c/string_functions.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int8.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <visualization_msgs/msg/marker.h>

#include "uros_network_interfaces.h"
#include "boards.h"
#include "p4_link.h"

static const char *TAG = "drone";

/* ── Identity ──────────────────────────────────────────────────────────── */
#define DRONE_ID          2

/* ── RViz marker IDs ────────────────────────────────────────────────────── */
#define DRONE_DISC_DIAMETER_M  0.18f
#define DRONE_DISC_THICKNESS_M 0.02f
#define DRONE_DEFAULT_Z_M      0.5f
#define VISION_TIMEOUT_MS      1500
#define VISION_FADE_MS         3000   /* send last pose with rising covariance after ArUco loss */
#define MAX_POSE_JUMP_M        1.0f   /* reject single-frame pose jumps larger than this (m) */
#define MAX_YAW_JUMP_RAD       1.5708f /* 90° — reject single-frame ArUco yaw flips. With no
                                        * magnetometer, EKF2 has no independent heading reference
                                        * to veto a flipped ArUco yaw, so we gate it here. */
/* Heading seed (no magnetometer): the drone's launch heading, in the SAME convention
 * as vision_yaw — arena frame, 0°=+X, 90°=+Y, 180°=−X, −90°=−Y. Seeded into EKF2 at
 * arm so the first ArUco fix agrees instead of snapping 180°. Scene-dependent and set
 * from the team_color message: LH (red, x=1) faces +X; RH (blue, x=19) faces −X. */
#define START_YAW_LH_DEG       0.0f   /* LH / red  — launch faces +X */
#define START_YAW_RH_DEG       180.0f /* RH / blue — launch faces −X */
#define SEED_YAW_ENABLE        1      /* 0 = disable heading seeding */
#define SEED_YAW_MS            4000   /* seed only this long after arm (drone still on the
                                        * ground at start heading), then stop so it does not
                                        * fight later yaw maneuvers; gyro carries it until vision */
#define COLLISION_MARGIN_M     0.3f   /* keep this distance from any detected obstacle */
#define REPROJ_REJECT_PX       10.0f  /* reject ArUco frame if reprojection error > this */
#define PX4_RESET_DETECT_M     1.0f   /* a single-frame jump in PX4's reported local
                                        * position larger than this is an EKF position
                                        * RESET, not real motion (no GPS anchors NED; a
                                        * vision re-acquisition after a dropout can force
                                        * one). Absorb the jump into ned_offset so setpoints
                                        * and EV stay aligned to the arena frame — otherwise
                                        * the drone flies the reset delta into a wall.
                                        * See docs/flight-tests/2026-05-31 (log_24). */
#define REACQ_RAMP_MS          2000   /* after a vision dropout (> VISION_TIMEOUT_MS), ramp
                                        * the EV position covariance loose→tight over this
                                        * window so EKF2 converges to the re-acquired pose
                                        * gradually instead of hard-resetting (teleporting). */
#define REACQ_VAR_MAX          4.0f   /* initial (loose) EV position variance on re-acquire
                                        * (m²; ~2 m std), decays to the reproj-based var */

#define MARKER_ID_DISC(id)  ((id)*100)
#define MARKER_ID_TEXT(id)  ((id)*100+1)

/* ── Network ───────────────────────────────────────────────────────────── */
#define DRONE_IP_BASE_OCTET 100
#define DRONE_IP_NETMASK    "255.255.255.0"
#define DRONE_IP_GATEWAY    "192.168.178.1"
#define DRONE_IP_PREFIX     "192.168.178."

/* ── MAVLink / PX4 ─────────────────────────────────────────────────────── */
#define GCS_SYSID   42
#define GCS_COMPID  200
#define PX4_SYSID   1
#define PX4_COMPID  1

#define PX4_MODE_STABILIZED  0x00070000UL
#define PX4_MODE_OFFBOARD    0x00060000UL

/* ── C2 watchdog ───────────────────────────────────────────────────────── */
#define C2_PING_TIMEOUT_MS   200   /* 200 ms: tolerates WiFi RTT, limits executor block */
#define C2_PING_ATTEMPTS     2
#define C2_CHECK_INTERVAL_MS 2000  /* check every 2 s — reduces executor block frequency */
#define C2_FAIL_THRESHOLD    3     /* 3 × 2 s = 6 s — raised from 2 for WiFi RTT tolerance.
                                    * TODO: lower to 1 (2 s) before official finals flight. */

/* ── Mission parameters ────────────────────────────────────────────────── */
#define MISSION_TAKEOFF_ALT_M      1.0f
#define MISSION_TAKEOFF_WAIT_MS    5000
#define MISSION_MAX_HOVER_MS       (15UL * 60UL * 1000UL)
#define MISSION_LAND_DESCEND_MS    5000
#define OFFBOARD_STREAM_PERIOD_MS  50

/* ── Manhattan waypoint navigation ─────────────────────────────────────── */
#define MANHATTAN_STEP_M        1.0f   /* distance between intermediate waypoints */
#define MANHATTAN_ARRIVAL_M     1.0f   /* waypoint reached when closer than this */
#define MANHATTAN_PASSTHROUGH_M 1.0f   /* setpoints ≤ this skip Manhattan (keyboard) */
#define MANHATTAN_OBSTACLE_MM   1000   /* ToF threshold (mm) — stop and await new GCS command.
                                        * Detection window is 1..MANHATTAN_OBSTACLE_MM with a
                                        * valid (status==0, non-zero) reading; ≥ this, 0 mm, or
                                        * an error status all read as "clear". 500→1000→1500→1000.
                                        * TODO: add 1 s hover/debounce on detection (net-hole flicker). */
#define MANHATTAN_MAX_WPS       40     /* 20 m + 10 m at 1 m steps + margin */

/* ════════════════════════════════════════════════════════════════════════════
 * TEST — Yaw spin on marker 22.
 * Place physical marker 22 at arena position (10.0, 5.0) z=2 m.
 * When detected during MISSION: hold position and spin 360° in place,
 * then hover until next GCS command.
 * ════════════════════════════════════════════════════════════════════════════ */
#define TEST_ARUCO_APPROACH
#define TEST_TRIGGER_ID       22
#define TEST_YAW_RATE_DEG_S   45.0f   /* degrees per second — 360° in 8 s */

/* ── OBSTACLE_DISTANCE — VL53L1X 30° FOV → 6 bins per sensor ───────────
 * Bins filled: centre ± 3 bins (±15°).  Slots 0–4 (horizontal only).
 * Slot 5 (up) sent separately as DISTANCE_SENSOR. */
#define OD_BINS  72
/* [slot][bin_index 0..5] */
static const uint8_t OD_BIN_MAP[5][6] = {
    { 51, 52, 53, 54, 55, 56 },   /* slot 0  Left  −90°  (270°) */
    { 60, 61, 62, 63, 64, 65 },   /* slot 1  L-front −45° (315°) */
    { 69, 70, 71,  0,  1,  2 },   /* slot 2  Front   0°          */
    {  6,  7,  8,  9, 10, 11 },   /* slot 3  R-front +45°        */
    { 15, 16, 17, 18, 19, 20 },   /* slot 4  Right  +90°         */
};

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

static volatile float home_x = 0.0f, home_y = 0.0f, home_z = 0.0f;
static volatile int8_t battery_remaining_pct = -1;
static volatile float px4_pos_x = 0.0f, px4_pos_y = 0.0f, px4_pos_z = 0.0f;
static volatile float px4_yaw   = 0.0f;   /* radians, from ATTITUDE msg — debug log only */
static volatile bool  px4_pos_valid = false;
static volatile float px4_home_x = 0.0f, px4_home_y = 0.0f, px4_home_z = 0.0f;
static float map_home_x = 0.0f, map_home_y = 0.0f, map_home_z = 0.0f;

static volatile float setpoint_x = 0.0f, setpoint_y = 0.0f, setpoint_z = 1.5f;
static volatile float setpoint_yaw = 0.0f;
static volatile bool  setpoint_received = false;

/* ── Manhattan navigation state ─────────────────────────────────────────── */
static float          nav_wps_x[MANHATTAN_MAX_WPS];
static float          nav_wps_y[MANHATTAN_MAX_WPS];
static int            nav_wp_count  = 0;
static int            nav_wp_idx    = 0;
static volatile float nav_dest_x    = 0.0f;
static volatile float nav_dest_y    = 0.0f;
static volatile float nav_dest_z    = MISSION_TAKEOFF_ALT_M;
static volatile bool  nav_new_dest  = false;
static volatile bool  nav_active    = false;
static volatile bool  nav_obs_hold  = false;

volatile bool vision_enabled = false;

/* Vision pose — written by main loop from p4_link data */
volatile float vp_x = 0.0f, vp_y = 0.0f, vp_z = 0.0f;
volatile float vp_qx = 0.0f, vp_qy = 0.0f, vp_qz = 0.0f, vp_qw = 1.0f;
volatile bool    vision_pose_valid   = false;
volatile int64_t last_vision_pose_ms = 0;

/* Last accepted vision pose — used for fade-out and jump-filter baseline */
static float vp_last_x = 0.0f, vp_last_y = 0.0f, vp_last_z = 0.0f;
/* Yaw derived from ArUco quaternion; camera +Z = body forward, so yaw ≠ standard formula */
static volatile float vision_yaw = 0.0f;

/* Heading seed for the mag-less EKF — default LH (+X); overridden by team_color for RH. */
static volatile float seed_yaw_rad   = START_YAW_LH_DEG * (float)M_PI / 180.0f;
static volatile bool  seed_yaw_valid = true;

/* True once map_home has been set from a real vision pose.  Guards the
 * inertial fallback so the GCS disc is never placed at arena (0,0) just
 * because PX4 NED starts at 0 before ArUco is first acquired. */
static volatile bool inertial_anchor_valid = false;

/* Arena → NED offset: drone's known starting position in arena frame.
 * Subtracted from ArUco positions before sending to PX4 so the EKF
 * always sees positions relative to the drone's physical start (NED 0,0).
 * Set by team_color_callback when LH/RH Scene is pressed on GCS, and
 * adjusted on the fly when PX4 resets its local position (see the
 * LOCAL_POSITION_NED handler) so the arena↔NED mapping survives EKF resets. */
static float ned_offset_x = 0.0f, ned_offset_y = 0.0f;
/* Arena↔NED frame sign. With no magnetometer/vision, EKF "North" = the drone's
 * physical facing at arm. Red (LH) faces +X → North=+X, East=−Y (the baseline
 * convention). Blue (RH) faces −X → North=−X, East=+Y, i.e. red's frame rotated
 * 180°. So blue negates BOTH axes in every arena↔NED conversion: +1 = red/LH,
 * −1 = blue/RH. Set by team_color_callback. */
static volatile float frame_sign = 1.0f;

/* Last values sent to PX4 via VISION_POSITION_ESTIMATE — for debug log */
static float last_vis_sent_x = 0.0f, last_vis_sent_y = 0.0f;


static volatile bool  gcs_control_active = false;

static TaskHandle_t mission_task_handle     = NULL;
static TaskHandle_t return_home_task_handle = NULL;
static TaskHandle_t eland_task_handle       = NULL;
static TaskHandle_t prearm_stream_handle    = NULL;

#ifdef TEST_ARUCO_APPROACH
static volatile bool s_test_aruco_triggered = false;
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * UART / MAVLink
 * ══════════════════════════════════════════════════════════════════════════ */

static void uart_mavlink_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = PX4_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PX4_UART_PORT, 2048, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PX4_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PX4_UART_PORT, PX4_UART_TX, PX4_UART_RX,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void mav_send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(PX4_UART_PORT, (const char *)buf, len);
}

static void send_heartbeat_once(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(GCS_SYSID, GCS_COMPID, &msg,
        MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    mav_send(&msg);
}

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

static void mav_kill(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID, PX4_COMPID,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0.0f, 21196.0f, 0, 0, 0, 0, 0);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: KILL");
}

static void mav_eland(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(GCS_SYSID, GCS_COMPID, &msg,
        PX4_SYSID, PX4_COMPID,
        MAV_CMD_NAV_LAND, 0,
        0, 0, 0, NAN, NAN, NAN, NAN);
    mav_send(&msg);
    ESP_LOGI(TAG, "MAV: EMERGENCY LAND");
}

static void mav_set_position_ned(float x, float y, float z_ned)
{
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        GCS_SYSID, GCS_COMPID, &msg,
        (uint32_t)(esp_timer_get_time() / 1000),
        PX4_SYSID, PX4_COMPID,
        MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        x, y, z_ned, 0, 0, 0, 0, 0, 0, 0, 0);
    mav_send(&msg);
}

static void mav_set_position_yaw_ned(float x, float y, float z_ned, float yaw)
{
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        GCS_SYSID, GCS_COMPID, &msg,
        (uint32_t)(esp_timer_get_time() / 1000),
        PX4_SYSID, PX4_COMPID,
        MAV_FRAME_LOCAL_NED,
        0b0000101111111000,
        x, y, z_ned, 0, 0, 0, 0, 0, 0, yaw, 0);
    mav_send(&msg);
}

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

void mav_send_vision_estimate(float x, float y, float z, float yaw_rad, float pos_variance)
{
    /* Upper triangle of 6×6 pose covariance (position then attitude).
     * Diagonal indices: [0]=xx [6]=yy [11]=zz [15]=rr [18]=pp [20]=yy_att
     * pos_variance: 0.01 = good fix; rises toward 0.50 during fade-out after ArUco loss.
     * Roll/pitch/yaw: NaN → EKF2 skips all attitude fusion; gyro owns heading. */
    static float cov[21];
    static bool cov_init = false;
    if (!cov_init) {
        cov_init = true;
        for (int i = 0; i < 21; i++) cov[i] = 0.0f;
        cov[15] = __builtin_nanf("");  /* roll  — no attitude fusion */
        cov[18] = __builtin_nanf("");  /* pitch — no attitude fusion */
        cov[20] = __builtin_nanf("");  /* yaw   — no fusion; gyro owns heading */
    }
    cov[0]  = pos_variance;
    cov[6]  = pos_variance;
    cov[11] = pos_variance;

    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_pack(
        GCS_SYSID, GCS_COMPID, &msg,
        (uint64_t)esp_timer_get_time(),
        x, y, z,
        0.0f, 0.0f, yaw_rad,
        cov, 0);
    mav_send(&msg);
}

/* ── Obstacle distance — 5 horizontal sensors in one OBSTACLE_DISTANCE msg ── */
static void send_obstacle_distance(const p4_tof_data_t *tof)
{
    mavlink_message_t msg;
    uint16_t distances[OD_BINS];
    for (int i = 0; i < OD_BINS; i++) distances[i] = UINT16_MAX;

    /* Fill bins for each horizontal sensor (slots 0–4) */
    for (int s = 0; s < 5; s++) {
        uint16_t d_cm = tof->dist_mm[s] / 10;
        /* Clamp: sensor min ~4 cm, max ~400 cm (LONG mode) */
        if (d_cm < 4)   d_cm = 4;
        if (d_cm > 400) d_cm = 400;
        for (int b = 0; b < 6; b++)
            distances[OD_BIN_MAP[s][b]] = d_cm;
    }

    mavlink_msg_obstacle_distance_pack(
        GCS_SYSID, GCS_COMPID, &msg,
        (uint64_t)esp_timer_get_time(),
        MAV_DISTANCE_SENSOR_LASER,
        distances,
        5,       /* increment (legacy uint8_t, 5°/bin) */
        4,       /* min_distance_cm */
        400,     /* max_distance_cm */
        5.0f,    /* increment_f: 5°/bin */
        0.0f,    /* angle_offset: bin 0 = forward (0°) */
        MAV_FRAME_BODY_FRD);
    mav_send(&msg);
}

/* ── Upward sensor — dedicated DISTANCE_SENSOR message ──────────────────── */
static void send_upward_distance_sensor(const p4_tof_data_t *tof)
{
    mavlink_message_t msg;
    uint16_t d_cm = tof->dist_mm[5] / 10;
    if (d_cm < 4)   d_cm = 4;
    if (d_cm > 400) d_cm = 400;

    mavlink_msg_distance_sensor_pack(
        GCS_SYSID, GCS_COMPID, &msg,
        (uint32_t)(esp_timer_get_time() / 1000),
        4,    /* min_distance_cm */
        400,  /* max_distance_cm */
        d_cm,
        MAV_DISTANCE_SENSOR_LASER,
        5,    /* id — unique sensor ID */
        MAV_SENSOR_ROTATION_PITCH_90,   /* pointing up */
        255,  /* covariance unknown */
        0, 0, NULL, 0);
    mav_send(&msg);
}

/* ── Console output ──────────────────────────────────────────────────────── */
static void tof_console_print(const p4_tof_data_t *tof)
{
    int64_t age = p4_link_age_ms();
    if (age == INT64_MAX || age > 2000) {
        printf("\r[ToF] waiting for P4 data...                                    ");
    } else {
        if (vision_pose_valid) {
            printf("\r[ToF] L90:%4umm L45:%4umm FWD:%4umm R45:%4umm R90:%4umm UP:%4umm | STATE:%-14s VIS:OK x=%.2f y=%.2f z=%.2f",
                   tof->dist_mm[0], tof->dist_mm[1], tof->dist_mm[2],
                   tof->dist_mm[3], tof->dist_mm[4], tof->dist_mm[5],
                   state_names[(int)drone_state],
                   (double)vp_x, (double)vp_y, (double)vp_z);
        } else {
            printf("\r[ToF] L90:%4umm L45:%4umm FWD:%4umm R45:%4umm R90:%4umm UP:%4umm | STATE:%-14s VIS:NO              ",
                   tof->dist_mm[0], tof->dist_mm[1], tof->dist_mm[2],
                   tof->dist_mm[3], tof->dist_mm[4], tof->dist_mm[5],
                   state_names[(int)drone_state]);
        }
    }
    fflush(stdout);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Helpers
 * ══════════════════════════════════════════════════════════════════════════ */

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static float get_initial_x_from_drone_id(void) { return 1.0f; }
static float get_initial_y_from_drone_id(void) { return (float)DRONE_ID; }
static float get_initial_z_from_drone_id(void) { return DRONE_DEFAULT_Z_M; }


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
 * micro-ROS resources
 * ══════════════════════════════════════════════════════════════════════════ */

static rcl_publisher_t    publisher_marker;
static rcl_publisher_t    publisher_pose;
static rcl_publisher_t    publisher_state;
static rcl_publisher_t    publisher_role;
static rcl_publisher_t    publisher_battery;

/* ── Box markers — published on existing publisher_marker (/visualization_marker) ── */
/* IDs 31-36, 41-46 don't conflict with drone disc (100) or text label (101). */
#define BOX_COUNT      12
#define BOX_TIMEOUT_MS 3000

static const uint8_t BOX_IDS[BOX_COUNT] = {
    31, 32, 33, 34, 35, 36,   /* blue team */
    41, 42, 43, 44, 45, 46,   /* red team  */
};
static visualization_msgs__msg__Marker box_markers_storage[BOX_COUNT];
/* Floating coordinate labels above each box cube: "ArUco No. ZZ (X, Y)". */
static visualization_msgs__msg__Marker box_text_storage[BOX_COUNT];

static rcl_subscription_t command_sub;
static rcl_subscription_t config_sub;
static rcl_subscription_t control_sub;
static rcl_subscription_t team_color_sub;

static visualization_msgs__msg__Marker   drone_disc_msg;
static visualization_msgs__msg__Marker   text_msg;
static geometry_msgs__msg__PoseStamped   vision_pose_msg;
static std_msgs__msg__String             command_msg;
static std_msgs__msg__String             config_msg;
static geometry_msgs__msg__PoseStamped   control_msg;
static std_msgs__msg__String             team_color_msg;
static std_msgs__msg__String             state_pub_msg;
static std_msgs__msg__String             role_pub_msg;
static std_msgs__msg__Int8               battery_pub_msg;

static char topic_gcs_command[64];
static char topic_gcs_config[64];
static char topic_gcs_control[64];
static char topic_state[64];
static char topic_role[64];
static char topic_battery[64];
static char topic_pose[64];
static char drone_ns[16];

/* ── Apply pose to RViz drone disc ───────────────────────────────────────── */
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

/* ══════════════════════════════════════════════════════════════════════════
 * Pre-arm setpoint stream task
 * ══════════════════════════════════════════════════════════════════════════ */
static void prearm_stream_task_fn(void *arg)
{
    ESP_LOGI("mission", "Prearm stream started");
    while (drone_state == DRONE_ARMED) {
        mav_set_position_ned(home_x, home_y, -0.1f);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
    }
    ESP_LOGI("mission", "Prearm stream stopped (state→%d)", (int)drone_state);
    prearm_stream_handle = NULL;
    vTaskDelete(NULL);
}

/* ── Obstacle clearance — returns ToF reading (mm) for the sensor nearest
 * to bearing_body_rad (body-frame angle, 0=forward, +ve=right, radians).
 * Sensor slots: 0=Left(−90°) 1=L-front(−45°) 2=Front(0°) 3=R-front(+45°) 4=Right(+90°)
 * Returns UINT16_MAX when bearing is rearward (>±112.5°) or data unavailable. */
static uint16_t tof_clearance_for_bearing(float bearing_body_rad)
{
    float deg = bearing_body_rad * 180.0f / (float)M_PI;
    /* Normalise to (−180, 180] */
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;

    int slot;
    if      (fabsf(deg)        <= 22.5f) slot = 2;  /* Front    0° */
    else if (fabsf(deg - 45.f) <= 22.5f) slot = 3;  /* R-front +45° */
    else if (fabsf(deg + 45.f) <= 22.5f) slot = 1;  /* L-front −45° */
    else if (fabsf(deg - 90.f) <= 22.5f) slot = 4;  /* Right   +90° */
    else if (fabsf(deg + 90.f) <= 22.5f) slot = 0;  /* Left    −90° */
    else return UINT16_MAX;  /* rearward — no sensor covers this bearing */

    p4_tof_data_t tof;
    if (!p4_link_get_tof(&tof)) return UINT16_MAX;
    if (tof.status[slot] != 0 || tof.dist_mm[slot] == 0) return UINT16_MAX;
    return tof.dist_mm[slot];
}

/* ── Smallest valid clearance (mm) across the 5 horizontal ToF sensors
 * (slots 0-4: LH −90°, 45° LH, FWD, 45° RH, RH +90°).  Skips sensors with a
 * non-zero status or a zero reading.  Returns UINT16_MAX when no horizontal
 * sensor currently has a valid reading. Used to stop on an obstacle in ANY
 * horizontal direction while stepping along a leg. */
static uint16_t tof_min_horizontal_clearance(void)
{
    p4_tof_data_t tof;
    if (!p4_link_get_tof(&tof)) return UINT16_MAX;
    uint16_t min_mm = UINT16_MAX;
    for (int s = 0; s < 5; s++) {
        if (tof.status[s] != 0 || tof.dist_mm[s] == 0) continue;  /* invalid → skip */
        if (tof.dist_mm[s] < min_mm) min_mm = tof.dist_mm[s];
    }
    return min_mm;
}

/* ── Clamp setpoint so the drone stops COLLISION_MARGIN_M short of any obstacle
 * in the direction of travel.  Modifies *sp_x / *sp_y in place. */
static void clamp_setpoint_for_obstacles(float cur_x, float cur_y,
                                          float *sp_x,  float *sp_y)
{
    float dx = *sp_x - cur_x;
    float dy = *sp_y - cur_y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < 0.05f) return;  /* already at target, nothing to clamp */

    /* The drone flies facing its travel direction, so the obstacle in the
     * approach direction is on the forward sensor (body 0°). */
    uint16_t clearance_mm = tof_clearance_for_bearing(0.0f);
    if (clearance_mm == UINT16_MAX) return;  /* no sensor / rearward */

    float safe_m = (clearance_mm / 1000.0f) - COLLISION_MARGIN_M;
    if (safe_m < 0.0f) safe_m = 0.0f;

    if (dist > safe_m) {
        /* Clamp along the approach vector */
        *sp_x = cur_x + (dx / dist) * safe_m;
        *sp_y = cur_y + (dy / dist) * safe_m;
        ESP_LOGD(TAG, "Obstacle clamp: clearance=%.2fm safe=%.2fm → sp(%.2f,%.2f)",
                 (double)(clearance_mm / 1000.0f), (double)safe_m,
                 (double)*sp_x, (double)*sp_y);
    }
}

/* ── Build Manhattan waypoint list: X-leg first, then Y-leg, in STEP_M steps.
 * Returns the number of waypoints written into wps_x / wps_y. */
static int nav_build_waypoints(float fx, float fy, float tx, float ty,
                                float step, float *wps_x, float *wps_y, int max_wps)
{
    int   n      = 0;
    float cx     = fx;
    float cy     = fy;
    float dx     = tx - fx;
    float dy     = ty - fy;
    float sign_x = dx >= 0.0f ? 1.0f : -1.0f;
    float sign_y = dy >= 0.0f ? 1.0f : -1.0f;

    /* X-leg */
    int nx = (int)(fabsf(dx) / step);
    for (int i = 0; i < nx && n < max_wps; i++) {
        cx += sign_x * step;
        wps_x[n] = cx; wps_y[n] = cy; n++;
    }
    if (fabsf(tx - cx) > 0.05f && n < max_wps) {
        wps_x[n] = tx; wps_y[n] = cy; n++;  /* X remainder */
    }

    /* Y-leg */
    int ny = (int)(fabsf(dy) / step);
    for (int i = 0; i < ny && n < max_wps; i++) {
        cy += sign_y * step;
        wps_x[n] = tx; wps_y[n] = cy; n++;
    }
    if (fabsf(ty - cy) > 0.05f && n < max_wps) {
        wps_x[n] = tx; wps_y[n] = ty; n++;  /* Y remainder */
    }

    return n;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Mission FreeRTOS task
 * ══════════════════════════════════════════════════════════════════════════ */
static void mission_task_fn(void *arg)
{
    TickType_t t0;
    uint32_t elapsed;

    t0 = xTaskGetTickCount();
    do {
        if (drone_state != DRONE_MISSION) goto mission_abort;
        mav_set_position_ned(home_x, home_y, -0.1f);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < 1000);

    mav_set_mode(PX4_MODE_OFFBOARD);
    {
        TickType_t t_ob = xTaskGetTickCount();
        while ((uint32_t)((xTaskGetTickCount() - t_ob) * portTICK_PERIOD_MS) < 500) {
            if (drone_state != DRONE_MISSION) goto mission_abort;
            mav_set_position_ned(home_x, home_y, -0.1f);
            vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        }
    }
    mav_arm(true);

    t0 = xTaskGetTickCount();
    do {
        if (drone_state != DRONE_MISSION) goto mission_abort;
        mav_set_position_ned(home_x, home_y, -MISSION_TAKEOFF_ALT_M);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < MISSION_TAKEOFF_WAIT_MS);

    t0 = xTaskGetTickCount();
    do {
        if (drone_state != DRONE_MISSION) goto mission_abort;
        {
            float sp_x, sp_y, sp_z, sp_yaw;
            float cur_x = px4_pos_valid ? px4_pos_x : home_x;
            float cur_y = px4_pos_valid ? px4_pos_y : home_y;

            /* ── 1. New GCS destination → build waypoint list ── */
            if (nav_new_dest) {
                nav_wp_count = nav_build_waypoints(cur_x, cur_y,
                                                   nav_dest_x, nav_dest_y,
                                                   MANHATTAN_STEP_M,
                                                   nav_wps_x, nav_wps_y,
                                                   MANHATTAN_MAX_WPS);
                nav_wp_idx   = 0;
                nav_active   = (nav_wp_count > 0);
                nav_obs_hold = false;
                nav_new_dest = false;
                ESP_LOGI(TAG, "Manhattan: %d wp → NED(%.2f,%.2f)",
                         nav_wp_count, (double)nav_dest_x, (double)nav_dest_y);
            }

            /* ── 2. Manhattan active — advance through waypoints ── */
            if (nav_active) {
                float wp_x = nav_wps_x[nav_wp_idx];
                float wp_y = nav_wps_y[nav_wp_idx];
                float dx   = wp_x - cur_x;
                float dy   = wp_y - cur_y;
                float wp_yaw = atan2f(dy, dx);

                /* Obstacle check each step: scan ALL 5 horizontal ToF sensors
                 * (LH, 45° LH, FWD, 45° RH, RH) and take the nearest. If anything
                 * is within MANHATTAN_OBSTACLE_MM on any side, stop and hover on
                 * the current position. No heading estimate is used (it previously
                 * picked the wrong sensor and flew through the obstacle). */
                uint16_t clearance = tof_min_horizontal_clearance();

                if (clearance != UINT16_MAX && clearance < MANHATTAN_OBSTACLE_MM) {
                    /* Obstacle — stop and wait for new GCS command */
                    nav_active   = false;
                    nav_obs_hold = true;
                    sp_x   = cur_x;
                    sp_y   = cur_y;
                    sp_z   = -nav_dest_z;
                    sp_yaw = vision_pose_valid ? vision_yaw : 0.0f;
                    ESP_LOGW(TAG, "Manhattan: obstacle %u mm — awaiting new GCS command",
                             clearance);
                } else {
                    sp_x   = wp_x;
                    sp_y   = wp_y;
                    sp_z   = -nav_dest_z;
                    sp_yaw = wp_yaw;

                    /* Arrival check */
                    float dist = sqrtf(dx * dx + dy * dy);
                    if (dist < MANHATTAN_ARRIVAL_M) {
                        nav_wp_idx++;
                        if (nav_wp_idx >= nav_wp_count) {
                            nav_active = false;
                            /* Hover at the arrived position. Without latching a
                             * hold, the next cycle finds nav_active/nav_obs_hold
                             * false and setpoint_received false (the Manhattan
                             * path never sets it) → falls through to the
                             * "hover at home" branch and the drone flies all the
                             * way back to launch. Latch the destination into the
                             * pass-through hold so branch 4 parks it here. */
                            setpoint_x        = nav_dest_x;
                            setpoint_y        = nav_dest_y;
                            setpoint_z        = nav_dest_z;
                            setpoint_yaw      = sp_yaw;
                            setpoint_received = true;
                            ESP_LOGI(TAG, "Manhattan: arrived at destination — holding");
                        } else {
                            ESP_LOGI(TAG, "Manhattan: wp %d/%d reached",
                                     nav_wp_idx, nav_wp_count);
                        }
                    }
                }

            /* ── 3. Obstacle hold — hover until new GCS command ── */
            } else if (nav_obs_hold) {
                sp_x   = cur_x;
                sp_y   = cur_y;
                sp_z   = -nav_dest_z;
                sp_yaw = vision_pose_valid ? vision_yaw : 0.0f;

            /* ── 4. Pass-through (keyboard / small steps) ── */
            } else if (setpoint_received) {
                sp_x   = setpoint_x;
                sp_y   = setpoint_y;
                sp_z   = -setpoint_z;
                sp_yaw = setpoint_yaw;
                if (px4_pos_valid)
                    clamp_setpoint_for_obstacles(cur_x, cur_y, &sp_x, &sp_y);

            /* ── 5. No setpoint — hover at home ── */
            } else {
                sp_x   = home_x;
                sp_y   = home_y;
                sp_z   = -MISSION_TAKEOFF_ALT_M;
                sp_yaw = vision_pose_valid ? vision_yaw : 0.0f;
            }

            mav_set_position_yaw_ned(sp_x, sp_y, sp_z, sp_yaw);
        }
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < MISSION_MAX_HOVER_MS);
    ESP_LOGW(TAG, "Mission 15 min timeout — auto-landing");

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
    do {
        if (drone_state != DRONE_RETURNING_HOME) goto rh_done;
        mav_set_position_ned(home_x, home_y, -MISSION_TAKEOFF_ALT_M);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
        elapsed = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
    } while (elapsed < 5000);
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

#ifdef TEST_ARUCO_APPROACH
static void aruco_approach_task_fn(void *arg)
{
    (void)arg;
    /* Kill mission task — no setpoint conflict during spin */
    if (mission_task_handle) {
        vTaskDelete(mission_task_handle);
        mission_task_handle = NULL;
    }

    /* Capture hold position: vision if valid, else PX4 inertial */
    float hold_x = vision_pose_valid ? vp_x : (px4_pos_valid ? px4_pos_x : 0.0f);
    float hold_y = vision_pose_valid ? vp_y : (px4_pos_valid ? px4_pos_y : 0.0f);
    float hold_z = -MISSION_TAKEOFF_ALT_M;  /* NED: negative = above ground */

    /* Initial yaw from ArUco pose quaternion */
    float yaw = atan2f(2.0f * (vp_qw * vp_qz + vp_qx * vp_qy),
                       1.0f - 2.0f * (vp_qy * vp_qy + vp_qz * vp_qz));

    /* Yaw increment per control tick */
    const float yaw_step = TEST_YAW_RATE_DEG_S * ((float)M_PI / 180.0f)
                           * (OFFBOARD_STREAM_PERIOD_MS / 1000.0f);
    float rotated = 0.0f;

    ESP_LOGI(TAG, "[TEST] M22 spin: hold(%.2f,%.2f) yaw0=%.1f° rate=%.0f°/s",
             (double)hold_x, (double)hold_y,
             (double)(yaw * 180.0f / (float)M_PI),
             (double)TEST_YAW_RATE_DEG_S);

    /* Spin one full 360° in place */
    while (rotated < 2.0f * (float)M_PI && drone_state == DRONE_MISSION) {
        yaw += yaw_step;
        rotated += fabsf(yaw_step);
        /* Wrap yaw to [-π, π] */
        while (yaw >  (float)M_PI) yaw -= 2.0f * (float)M_PI;
        while (yaw < -(float)M_PI) yaw += 2.0f * (float)M_PI;
        mav_set_position_yaw_ned(hold_x, hold_y, hold_z, yaw);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
    }

    /* Hold position after spin until next GCS command */
    ESP_LOGI(TAG, "[TEST] M22 spin done — hovering at (%.2f,%.2f)", (double)hold_x, (double)hold_y);
    while (drone_state == DRONE_MISSION) {
        mav_set_position_yaw_ned(hold_x, hold_y, hold_z, yaw);
        vTaskDelay(pdMS_TO_TICKS(OFFBOARD_STREAM_PERIOD_MS));
    }
    vTaskDelete(NULL);
}
#endif /* TEST_ARUCO_APPROACH */

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

static void publish_state_now(void)
{
    state_dirty = false;
    const char *s = state_names[(int)drone_state];
    rosidl_runtime_c__String__assign(&state_pub_msg.data, s);
    RCSOFTCHECK(rcl_publish(&publisher_state, &state_pub_msg, NULL));
    ESP_LOGI(TAG, "State → %s", s);
}

/* ══════════════════════════════════════════════════════════════════════════
 * GCS command callback
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
            /* vp_x/vp_y are arena coords; home_x/y must be NED (drone start = NED 0,0).
             * Subtracting ned_offset converts arena → NED so all mav_set_position_ned
             * calls using home_x/y hold the drone at its physical start position. */
            home_x = frame_sign * (vp_x - ned_offset_x);
            home_y = frame_sign * (ned_offset_y - vp_y);   /* frame_sign flips both axes for blue/RH */
            home_z = vp_z;
            ESP_LOGI(TAG, "Home captured from vision: arena(%.2f,%.2f) → NED(%.2f,%.2f)",
                     (double)vp_x, (double)vp_y, (double)home_x, (double)home_y);
        } else if (px4_pos_valid) {
            /* No ArUco fix — use PX4 inertial position so prearm setpoints match reality */
            home_x = px4_pos_x; home_y = px4_pos_y; home_z = px4_pos_z;
            ESP_LOGW(TAG, "Home captured from PX4 inertial: (%.2f, %.2f, %.2f)", home_x, home_y, home_z);
        }
        if (px4_pos_valid) {
            px4_home_x = px4_pos_x;
            px4_home_y = px4_pos_y;
            px4_home_z = px4_pos_z;
            inertial_anchor_valid = true;
            ESP_LOGI(TAG, "Inertial anchor set at ARM: map_home=(%.2f,%.2f) px4_home=(%.2f,%.2f)",
                     (double)map_home_x, (double)map_home_y,
                     (double)px4_home_x, (double)px4_home_y);
        }
        drone_state = DRONE_ARMED;
        publish_state_now();
        if (!prearm_stream_handle)
            xTaskCreate(prearm_stream_task_fn, "prearm", 4096, NULL, 4, &prearm_stream_handle);

    } else if (strcmp(buf, "COMMAND_DISARM") == 0) {
        if (prearm_stream_handle) {
            vTaskDelete(prearm_stream_handle);
            prearm_stream_handle = NULL;
        }
        mav_arm(false);
        drone_state = DRONE_DISARMED;
        publish_state_now();

    } else if (strcmp(buf, "COMMAND_MISSION_START") == 0) {
        if (drone_state != DRONE_ARMED && drone_state != DRONE_RETURNING_HOME) {
            ESP_LOGW(TAG, "Mission blocked — state: %s", state_names[drone_state]);
            return;
        }
        if (return_home_task_handle) {
            vTaskDelete(return_home_task_handle);
            return_home_task_handle = NULL;
        }
        setpoint_received = false;
        setpoint_yaw = 0.0f;
#ifdef TEST_ARUCO_APPROACH
        s_test_aruco_triggered = false;
#endif
        drone_state = DRONE_MISSION;
        publish_state_now();
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
        publish_state_now();
        mav_set_mode(PX4_MODE_OFFBOARD);
        xTaskCreate(return_home_task_fn, "return_home", 4096, NULL, 5, &return_home_task_handle);

    } else if (strcmp(buf, "COMMAND_ELAND") == 0) {
        trigger_eland();

    } else if (strcmp(buf, "COMMAND_KILL") == 0) {
        if (mission_task_handle)     { vTaskDelete(mission_task_handle);     mission_task_handle = NULL; }
        if (return_home_task_handle) { vTaskDelete(return_home_task_handle); return_home_task_handle = NULL; }
        mav_kill();
        drone_state = DRONE_KILLED;
        publish_state_now();

    } else {
        ESP_LOGW(TAG, "CMD: unknown '%s'", buf);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * GCS config callback
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

    if (strcmp(buf, "CONFIG_VISION_ENABLE") == 0) {
        vision_enabled = true;
        ESP_LOGI(TAG, "Vision EKF: ON — relaying P4 pose to PX4");

    } else if (strcmp(buf, "CONFIG_VISION_DISABLE") == 0) {
        vision_enabled = false;
        ESP_LOGI(TAG, "Vision EKF: OFF");

    } else if (strcmp(buf, "CONFIG_SOURCE_RC") == 0) {
        gcs_control_active = false;
        mav_set_mode(PX4_MODE_STABILIZED);

    } else if (strcmp(buf, "CONFIG_SOURCE_GCS") == 0) {
        gcs_control_active = true;
        mav_set_mode(PX4_MODE_OFFBOARD);

    } else {
        ESP_LOGW(TAG, "CFG: unknown '%s'", buf);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * GCS control callback
 * ══════════════════════════════════════════════════════════════════════════ */
static void control_callback(const void *msg_in)
{
    const geometry_msgs__msg__PoseStamped *msg =
        (const geometry_msgs__msg__PoseStamped *)msg_in;
    if (!msg) return;

    float ned_x = frame_sign * ((float)msg->pose.position.x - ned_offset_x);
    /* East = arena −Y: the arena frame is Z-up/CCW but PX4 NED is Z-down/CW, so a
     * right-handed NED with North=arena+X forces East to run along arena −Y. Negate
     * Y here (and at every arena↔NED boundary) or commands mirror about the start.
     * Confirmed 2026-06-04: drone2 cmd y=2 → phys y=6 = 2*sy − cmd. */
    float ned_y = frame_sign * (ned_offset_y - (float)msg->pose.position.y);
    float ned_z = (float)msg->pose.position.z;
    float qx = (float)msg->pose.orientation.x;
    float qy = (float)msg->pose.orientation.y;
    float qz = (float)msg->pose.orientation.z;
    float qw = (float)msg->pose.orientation.w;
    float yaw = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));

    /* Manhattan distance from current position to new setpoint */
    float cur_x = px4_pos_valid ? px4_pos_x : 0.0f;
    float cur_y = px4_pos_valid ? px4_pos_y : 0.0f;
    float man_dist = fabsf(ned_x - cur_x) + fabsf(ned_y - cur_y);

    if (man_dist <= MANHATTAN_PASSTHROUGH_M) {
        /* Small step (keyboard / fly mode) — pass through directly */
        setpoint_x    = ned_x;
        setpoint_y    = ned_y;
        setpoint_z    = ned_z;
        setpoint_yaw  = yaw;
        setpoint_received = true;
        ESP_LOGI(TAG, "Setpoint pass-through NED=(%.2f,%.2f) z=%.2f dist=%.2f",
                 (double)ned_x, (double)ned_y, (double)ned_z, (double)man_dist);
        if (gcs_control_active && drone_state == DRONE_ARMED)
            mav_set_position_ned(ned_x, ned_y, -ned_z);
    } else {
        /* Large step — hand to Manhattan sequencer */
        nav_dest_x   = ned_x;
        nav_dest_y   = ned_y;
        nav_dest_z   = ned_z;
        nav_new_dest = true;
        nav_obs_hold = false;
        ESP_LOGI(TAG, "Setpoint → Manhattan NED=(%.2f,%.2f) z=%.2f dist=%.2f",
                 (double)ned_x, (double)ned_y, (double)ned_z, (double)man_dist);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Team colour callback — seeds EKF with known starting position
 * ══════════════════════════════════════════════════════════════════════════ */
static void team_color_callback(const void *msg_in)
{
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msg_in;
    if (!m || !m->data.data || m->data.size == 0) return;

    char buf[16] = {0};
    size_t n = m->data.size < 15 ? m->data.size : 15;
    memcpy(buf, m->data.data, n);
    for (int i = (int)n - 1; i >= 0; i--) {
        if (buf[i]==' '||buf[i]=='\n'||buf[i]=='\r'||buf[i]=='\t') buf[i]='\0'; else break;
    }

    float sx, sy;
    if (strcmp(buf, "red") == 0) {
        /* LH side — D1→(1,5)  D2→(1,4)  D3→(1,3)  D4→(1,2)  D5→(1,1) */
        sx = 1.0f;
        sy = 6.0f - (float)DRONE_ID;
        seed_yaw_rad   = START_YAW_LH_DEG * (float)M_PI / 180.0f;  /* faces +X */
        seed_yaw_valid = true;
        frame_sign     = 1.0f;   /* North=+X, East=−Y */
    } else if (strcmp(buf, "blue") == 0) {
        /* RH side — D1→(19,5) D2→(19,6) D3→(19,7) D4→(19,8) D5→(19,9) */
        sx = 19.0f;
        sy = (float)DRONE_ID + 4.0f;
        seed_yaw_rad   = START_YAW_RH_DEG * (float)M_PI / 180.0f;  /* faces −X */
        seed_yaw_valid = true;
        frame_sign     = -1.0f;  /* RH faces −X → North=−X, East=+Y: negate both axes */
    } else {
        ESP_LOGW(TAG, "team_color: unknown '%s'", buf);
        return;
    }

    /* Store arena→NED offset for coordinate translation */
    ned_offset_x = sx;
    ned_offset_y = sy;

    /* Move RViz disc to correct starting position immediately */
    map_home_x = sx;
    map_home_y = sy;
    map_home_z = 0.0f;
    apply_pose_to_drone_markers(sx, sy, DRONE_DEFAULT_Z_M, 0.0f, 0.0f, 0.0f, 1.0f);

    ESP_LOGI(TAG, "Team: %s → start (%.0f,%.0f) NED offset=(%.0f,%.0f)",
             buf, (double)sx, (double)sy, (double)ned_offset_x, (double)ned_offset_y);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Timer callback — 100 ms (state publish, vision timeout, RViz markers)
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

    /* State publish — every 10 × 100 ms = 1 s (was 2 s); keeps OFFLINE margin at 2 s */
    static uint32_t state_tick = 0;
    if (state_dirty || (++state_tick >= 10)) {
        state_dirty = false;
        state_tick  = 0;
        const char *s = state_names[(int)drone_state];
        rosidl_runtime_c__String__assign(&state_pub_msg.data, s);
        RCSOFTCHECK(rcl_publish(&publisher_state, &state_pub_msg, NULL));
    }

    /* Battery publish every 5 s */
    static uint32_t bat_tick = 0;
    if (++bat_tick >= 50) {
        bat_tick = 0;
        battery_pub_msg.data = battery_remaining_pct;
        RCSOFTCHECK(rcl_publish(&publisher_battery, &battery_pub_msg, NULL));
    }

    /* Three-tier disc position:
     * 1. ArUco valid  → direct vision pose (accurate)
     * 2. NED anchor + PX4 valid → dead-reckon from last ArUco anchor via NED delta (live)
     * 3. Pre-flight   → frozen at team_color home (set before arm) */
    /* Heading arrow/disc orientation: the P4 quaternion is camera-in-world
     * (camera +Z = forward), but RViz draws the pose arrow along local +X, so
     * the raw quaternion renders 90° off (arrow follows camera +X = drone
     * right). Rebuild a yaw-only quaternion from the camera forward vector
     * (same extraction that feeds vision_yaw) so the arrow points where the
     * drone looks, flat, in both LH and RH scenes. */
    float vp_fwd_x  = 2.0f * (vp_qx * vp_qz + vp_qy * vp_qw);
    float vp_fwd_y  = 2.0f * (vp_qy * vp_qz - vp_qx * vp_qw);
    float vp_yaw    = atan2f(vp_fwd_y, vp_fwd_x);
    float vp_yaw_qz = sinf(vp_yaw * 0.5f);
    float vp_yaw_qw = cosf(vp_yaw * 0.5f);

    if (vision_pose_valid) {
        apply_pose_to_drone_markers(vp_x, vp_y, vp_z, 0.0f, 0.0f, vp_yaw_qz, vp_yaw_qw);
    } else if (inertial_anchor_valid && px4_pos_valid) {
        float live_x = map_home_x + frame_sign * (px4_pos_x - px4_home_x);
        /* frame_sign maps the NED delta back to arena for both LH (+1) and RH (−1)
         * starts; the −Y on the East term is the red baseline (East = arena −Y). */
        float live_y = map_home_y - frame_sign * (px4_pos_y - px4_home_y);
        float live_z = map_home_z + (px4_pos_z - px4_home_z);
        apply_pose_to_drone_markers(live_x, live_y, live_z, 0.0f, 0.0f, 0.0f, 1.0f);
    }

    /* RViz markers at 2 Hz (every 5th tick) */
    static uint32_t marker_tick = 0;
    if (++marker_tick >= 5) {
        marker_tick = 0;
        RCSOFTCHECK(rcl_publish(&publisher_marker, &drone_disc_msg, NULL));
        RCSOFTCHECK(rcl_publish(&publisher_marker, &text_msg, NULL));
        if (vision_pose_valid) {
            int64_t ts = esp_timer_get_time();
            vision_pose_msg.header.stamp.sec     = (int32_t)(ts / 1000000LL);
            vision_pose_msg.header.stamp.nanosec = (uint32_t)((ts % 1000000LL) * 1000UL);
            vision_pose_msg.pose.position.x    = vp_x;
            vision_pose_msg.pose.position.y    = vp_y;
            vision_pose_msg.pose.position.z    = vp_z;
            vision_pose_msg.pose.orientation.x = 0.0f;
            vision_pose_msg.pose.orientation.y = 0.0f;
            vision_pose_msg.pose.orientation.z = vp_yaw_qz;
            vision_pose_msg.pose.orientation.w = vp_yaw_qw;
            RCSOFTCHECK(rcl_publish(&publisher_pose, &vision_pose_msg, NULL));
        }
    }

    /* Box marker publishing — every tick (100 ms).
     * ADD when detected within BOX_TIMEOUT_MS; DELETE once after timeout. */
    {
        static int64_t s_box_last_ms[BOX_COUNT];
        static float   s_box_x[BOX_COUNT];
        static float   s_box_y[BOX_COUNT];
        static float   s_box_z[BOX_COUNT];
        static bool    s_box_add_sent[BOX_COUNT];

        p4_boxes_t boxes;
        p4_link_get_boxes(&boxes);
        int64_t t = now_ms();

        if (boxes.count > 0)
            ESP_LOGI(TAG, "boxes rx: count=%d id[0]=%d", boxes.count, boxes.entries[0].id);

        for (int j = 0; j < (int)boxes.count; j++) {
            uint8_t bid = boxes.entries[j].id;
            for (int bi = 0; bi < BOX_COUNT; bi++) {
                if (BOX_IDS[bi] == bid) {
                    s_box_last_ms[bi] = t;
                    s_box_x[bi] = boxes.entries[j].x;
                    s_box_y[bi] = boxes.entries[j].y;
                    s_box_z[bi] = boxes.entries[j].z;
                    /* Label "ArUco No. ZZ (X, Y)" floating above the cube.
                     * Assigned only on update to limit String__assign churn. */
                    char lbl[48];
                    snprintf(lbl, sizeof(lbl), "ArUco No. %u (%.1f, %.1f)",
                             (unsigned)bid, (double)s_box_x[bi], (double)s_box_y[bi]);
                    rosidl_runtime_c__String__assign(&box_text_storage[bi].text, lbl);
                    box_text_storage[bi].pose.position.x = s_box_x[bi];
                    box_text_storage[bi].pose.position.y = s_box_y[bi];
                    box_text_storage[bi].pose.position.z = s_box_z[bi] + 0.7f; /* above cube top */
                    break;
                }
            }
        }

        /* Publish each active box as an individual Marker on /visualization_marker.
         * Boxes are fixed arena objects — once ADD is sent, never DELETE.
         * Re-publish confirmed boxes every 2 s to prevent RViz decay timeout. */
        static uint32_t box_refresh_tick = 0;
        bool do_refresh = (++box_refresh_tick >= 20);
        if (do_refresh) box_refresh_tick = 0;

        for (int bi = 0; bi < BOX_COUNT; bi++) {
            bool seen = (s_box_last_ms[bi] > 0 &&
                         (t - s_box_last_ms[bi]) < BOX_TIMEOUT_MS);
            if (seen) {
                box_markers_storage[bi].action = visualization_msgs__msg__Marker__ADD;
                box_markers_storage[bi].pose.position.x = s_box_x[bi];
                box_markers_storage[bi].pose.position.y = s_box_y[bi];
                box_markers_storage[bi].pose.position.z = s_box_z[bi] + 0.25f; /* centre at half-height */
                RCSOFTCHECK(rcl_publish(&publisher_marker, &box_markers_storage[bi], NULL));
                RCSOFTCHECK(rcl_publish(&publisher_marker, &box_text_storage[bi], NULL));
                s_box_add_sent[bi] = true;
            } else if (do_refresh && s_box_add_sent[bi]) {
                RCSOFTCHECK(rcl_publish(&publisher_marker, &box_markers_storage[bi], NULL));
                RCSOFTCHECK(rcl_publish(&publisher_marker, &box_text_storage[bi], NULL));
            }
            /* No DELETE — fixed boxes persist in RViz once detected */
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * MAVLink UART RX task
 * ══════════════════════════════════════════════════════════════════════════ */
static void mavlink_rx_task_fn(void *arg)
{
    mavlink_message_t rx_msg;
    mavlink_status_t  rx_status;
    uint8_t           byte;

    ESP_LOGI(TAG, "MAVLink RX task started");
    while (true) {
        int n = uart_read_bytes(PX4_UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg, &rx_status)) {
            if (rx_msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
                mavlink_battery_status_t bat;
                mavlink_msg_battery_status_decode(&rx_msg, &bat);
                battery_remaining_pct = bat.battery_remaining;
            } else if (rx_msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                mavlink_local_position_ned_t lpos;
                mavlink_msg_local_position_ned_decode(&rx_msg, &lpos);
                /* EKF position-reset detection. A jump > PX4_RESET_DETECT_M is an
                 * EKF frame reset, not real motion. Absorb it into ned_offset ONLY
                 * when vision was live recently (< 2 s ago) — meaning it is a genuine
                 * mid-flight reset, not a re-acquisition after flow drift. If vision
                 * has been gone > 2 s the large jump is likely legitimate optical-flow
                 * correction; absorbing it would cancel a real position fix. */
                if (px4_pos_valid &&
                        last_vision_pose_ms > 0 &&
                        (now_ms() - last_vision_pose_ms) < 2000) {
                    float dxr = lpos.x - px4_pos_x;
                    float dyr = lpos.y - px4_pos_y;
                    if (sqrtf(dxr * dxr + dyr * dyr) > PX4_RESET_DETECT_M) {
                        ned_offset_x -= frame_sign * dxr;
                        ned_offset_y += frame_sign * dyr;   /* frame_sign generalizes to blue/RH */
                        ESP_LOGW(TAG, "PX4 pos reset Δ=(%.2f,%.2f) absorbed into ned_offset",
                                 (double)dxr, (double)dyr);
                    }
                }
                px4_pos_x = lpos.x;
                px4_pos_y = lpos.y;
                px4_pos_z = -lpos.z;   /* NED z negated to up-positive */
                px4_pos_valid = true;
            } else if (rx_msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&rx_msg, &att);
                px4_yaw = att.yaw;     /* radians, NED convention */
            }
        }
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
    char agent_port[8];
    snprintf(agent_port, sizeof(agent_port), "%d", 8880 + DRONE_ID);
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, agent_port, rmw_options));
    RCCHECK(rmw_uros_options_set_client_key((uint32_t)DRONE_ID, rmw_options));
#endif

    ESP_LOGI(TAG, "uros_task: agent=%s:%s",
             CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
    while (rclc_support_init_with_options(&support, 0, NULL,
                                           &init_options, &allocator) != RCL_RET_OK) {
        ESP_LOGW(TAG, "Agent not reachable, retrying...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGI(TAG, "micro-ROS agent connected!");

    rcl_node_t node;
    char node_name[32];
    snprintf(node_name, sizeof(node_name), "esp32_drone_brain_%d", DRONE_ID);
    RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

    /* ── Publishers ──────────────────────────────────────────────────────── */
    RCCHECK(rclc_publisher_init_default(&publisher_marker, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, Marker),
        "/visualization_marker"));

    RCCHECK(rclc_publisher_init_default(&publisher_pose, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        topic_pose));

    RCCHECK(rclc_publisher_init_default(&publisher_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_state));

    RCCHECK(rclc_publisher_init_default(&publisher_role, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_role));

    RCCHECK(rclc_publisher_init_default(&publisher_battery, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), topic_battery));

    /* Box markers reuse publisher_marker — no extra publisher slot needed */

    /* ── Subscribers ─────────────────────────────────────────────────────── */
    RCCHECK(rclc_subscription_init_best_effort(&command_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_gcs_command));

    RCCHECK(rclc_subscription_init_best_effort(&config_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_gcs_config));

    RCCHECK(rclc_subscription_init_best_effort(&control_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        topic_gcs_control));

    {
        rmw_qos_profile_t tc_qos = rmw_qos_profile_default;
        tc_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        rcl_subscription_options_t tc_opts = rcl_subscription_get_default_options();
        tc_opts.qos = tc_qos;
        RCCHECK(rcl_subscription_init(&team_color_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/gcs/system/team_color", &tc_opts));
    }

    /* ── Message buffers ─────────────────────────────────────────────────── */
    command_msg.data.data = (char *)malloc(64);
    command_msg.data.size = 0; command_msg.data.capacity = 64;

    config_msg.data.data = (char *)malloc(64);
    config_msg.data.size = 0; config_msg.data.capacity = 64;

    geometry_msgs__msg__PoseStamped__init(&control_msg);

    team_color_msg.data.data = (char *)malloc(16);
    team_color_msg.data.size = 0; team_color_msg.data.capacity = 16;

    state_pub_msg.data.data = (char *)malloc(32);
    state_pub_msg.data.size = 0; state_pub_msg.data.capacity = 32;

    role_pub_msg.data.data = (char *)malloc(32);
    role_pub_msg.data.size = 0; role_pub_msg.data.capacity = 32;

    rosidl_runtime_c__String__assign(&role_pub_msg.data, "idle");
    RCSOFTCHECK(rcl_publish(&publisher_role, &role_pub_msg, NULL));

    /* ── Timer + executor (1 timer + 4 subscriptions = 5 handles) ───────── */
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
    RCCHECK(rclc_executor_add_subscription(&executor, &team_color_sub,
                &team_color_msg, &team_color_callback, ON_NEW_DATA));

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
                    if (drone_state == DRONE_DISARMED || drone_state == DRONE_KILLED) {
                        ESP_LOGE(TAG, "C2 link lost while disarmed — restarting");
                        vTaskDelay(pdMS_TO_TICKS(500));
                        esp_restart();
                    } else if (drone_state != DRONE_LANDING) {
                        ESP_LOGE(TAG, "C2 link lost — initiating emergency landing");
                        trigger_eland();
                    }
                }
            }
        }
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&publisher_marker,  &node));
    RCCHECK(rcl_publisher_fini(&publisher_pose,    &node));
    RCCHECK(rcl_publisher_fini(&publisher_state,   &node));
    RCCHECK(rcl_publisher_fini(&publisher_role,    &node));
    RCCHECK(rcl_publisher_fini(&publisher_battery, &node));
    RCCHECK(rcl_subscription_fini(&command_sub,    &node));
    RCCHECK(rcl_subscription_fini(&config_sub,     &node));
    RCCHECK(rcl_subscription_fini(&control_sub,    &node));
    RCCHECK(rcl_subscription_fini(&team_color_sub, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Network helpers
 * ══════════════════════════════════════════════════════════════════════════ */
static void set_device_hostname_from_drone_id(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return;
    char hostname[32];
    snprintf(hostname, sizeof(hostname), "mach-mind-drone-%d", DRONE_ID);
    if (esp_netif_set_hostname(netif, hostname) == ESP_OK)
        ESP_LOGI(TAG, "Hostname: %s", hostname);
}

static esp_err_t set_preferred_ip_from_drone_id(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return ESP_FAIL;
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
        ESP_LOGI(TAG, "Static IP: %s", ip_str);
    return err;
}

/* ══════════════════════════════════════════════════════════════════════════
 * app_main
 * ══════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    printf("app_main started\r\n"); fflush(stdout);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    set_device_hostname_from_drone_id();
    ESP_ERROR_CHECK(set_preferred_ip_from_drone_id());
#endif

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Build per-drone topic strings */
    snprintf(drone_ns,           sizeof(drone_ns),           "drone_%d",                   DRONE_ID);
    snprintf(topic_gcs_command,  sizeof(topic_gcs_command),   "/gcs/drone_%d/command",      DRONE_ID);
    snprintf(topic_gcs_config,   sizeof(topic_gcs_config),    "/gcs/drone_%d/config",       DRONE_ID);
    snprintf(topic_gcs_control,  sizeof(topic_gcs_control),   "/gcs/drone_%d/control",      DRONE_ID);
    snprintf(topic_state,        sizeof(topic_state),          "/drone_%d/state",            DRONE_ID);
    snprintf(topic_role,         sizeof(topic_role),           "/drone_%d/role",             DRONE_ID);
    snprintf(topic_battery,      sizeof(topic_battery),        "/drone_%d/battery",          DRONE_ID);
    snprintf(topic_pose,         sizeof(topic_pose),           "/drone_%d/vision_pose",      DRONE_ID);

    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "DRONE ID   : %d",     DRONE_ID);
    ESP_LOGI(TAG, "CMD topic  : %s",     topic_gcs_command);
    ESP_LOGI(TAG, "CFG topic  : %s",     topic_gcs_config);
    ESP_LOGI(TAG, "Takeoff alt: %.1f m", MISSION_TAKEOFF_ALT_M);
    ESP_LOGI(TAG, "==============================");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DRONE_ID_LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = 0, .pull_down_en = 0,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* Initialise RViz drone disc at start position */
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
    drone_disc_msg.lifetime.sec     = 1;
    drone_disc_msg.lifetime.nanosec = 500000000;  /* 1.5 s — fades when we stop publishing */

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

    map_home_x = ix;
    map_home_y = iy;
    map_home_z = iz;
    apply_pose_to_drone_markers(ix, iy, iz, 0.0f, 0.0f, 0.0f, 1.0f);

    geometry_msgs__msg__PoseStamped__init(&vision_pose_msg);
    rosidl_runtime_c__String__assign(&vision_pose_msg.header.frame_id, "map");

    /* ── Box marker messages ────────────────────────────────────────────────── */
    for (int bi = 0; bi < BOX_COUNT; bi++) {
        visualization_msgs__msg__Marker__init(&box_markers_storage[bi]);
        rosidl_runtime_c__String__assign(&box_markers_storage[bi].header.frame_id, "map");
        const char *ns = (BOX_IDS[bi] <= 36) ? "blue" : "red";
        rosidl_runtime_c__String__assign(&box_markers_storage[bi].ns, ns);
        box_markers_storage[bi].id     = BOX_IDS[bi];
        box_markers_storage[bi].type   = visualization_msgs__msg__Marker__CUBE;
        box_markers_storage[bi].action = visualization_msgs__msg__Marker__ADD;
        box_markers_storage[bi].pose.orientation.w = 1.0f;
        box_markers_storage[bi].scale.x = 0.5f;
        box_markers_storage[bi].scale.y = 0.5f;
        box_markers_storage[bi].scale.z = 0.5f;
        if (BOX_IDS[bi] <= 36) {
            box_markers_storage[bi].color.r = 0.1f;
            box_markers_storage[bi].color.g = 0.3f;
            box_markers_storage[bi].color.b = 0.9f;
        } else {
            box_markers_storage[bi].color.r = 0.9f;
            box_markers_storage[bi].color.g = 0.1f;
            box_markers_storage[bi].color.b = 0.1f;
        }
        box_markers_storage[bi].color.a = 0.75f;

        /* Coordinate label — distinct ns ("box_label") so it does not overwrite
         * the cube. Text + position are filled in when box data arrives. */
        visualization_msgs__msg__Marker__init(&box_text_storage[bi]);
        rosidl_runtime_c__String__assign(&box_text_storage[bi].header.frame_id, "map");
        rosidl_runtime_c__String__assign(&box_text_storage[bi].ns, "box_label");
        box_text_storage[bi].id     = BOX_IDS[bi];
        box_text_storage[bi].type   = visualization_msgs__msg__Marker__TEXT_VIEW_FACING;
        box_text_storage[bi].action = visualization_msgs__msg__Marker__ADD;
        box_text_storage[bi].pose.orientation.w = 1.0f;
        box_text_storage[bi].scale.z = 0.18f;   /* text height (m) */
        box_text_storage[bi].color.r = box_text_storage[bi].color.g =
            box_text_storage[bi].color.b = box_text_storage[bi].color.a = 1.0f;
    }

    uart_mavlink_init();
    xTaskCreate(mavlink_rx_task_fn, "mav_rx", 4096, NULL, 4, NULL);

    p4_link_init();   /* start P4 UART2 receiver task */

    ESP_LOGI(TAG, "Free heap: %u B internal",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    static StaticTask_t uros_tcb;
    static StackType_t  uros_stack[CONFIG_MICRO_ROS_APP_STACK];
    TaskHandle_t uros_handle = xTaskCreateStatic(
                micro_ros_task, "uros_task",
                CONFIG_MICRO_ROS_APP_STACK, NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                uros_stack, &uros_tcb);
    if (!uros_handle)
        ESP_LOGE(TAG, "uros_task creation FAILED");
    else
        ESP_LOGI(TAG, "uros_task created OK");

    /* ── Main loop — 20 Hz ────────────────────────────────────────────── */
    p4_tof_data_t  tof  = {0};
    p4_pose_data_t pose = {0};
    uint32_t hb_tick = 0;

    while (1) {
        drone_id_led_update();

        /* Read latest P4 sensor data */
        p4_link_get_tof(&tof);
        p4_link_get_pose(&pose);

        /* Update vision pose globals from UART data */
        if (pose.valid) {
            /* Issue 3: reject implausibly large single-frame jumps (ArUco flip at
             * steep angles produces position discontinuities > 1 m in one 50 ms frame). */
            if (pose.reproj_err > REPROJ_REJECT_PX) {
                ESP_LOGW(TAG, "Vision rejected: reproj=%.1fpx > %.0fpx",
                         (double)pose.reproj_err, (double)REPROJ_REJECT_PX);
            } else {

            float dxv  = pose.x - vp_x, dyv = pose.y - vp_y, dzv = pose.z - vp_z;
            float jump = sqrtf(dxv * dxv + dyv * dyv + dzv * dzv);
            /* Candidate heading from the incoming quaternion (same convention as
             * vision_yaw below) for the yaw-continuity gate. */
            float cand_fwd_x = 2.0f * (pose.qx * pose.qz + pose.qy * pose.qw);
            float cand_fwd_y = 2.0f * (pose.qy * pose.qz - pose.qx * pose.qw);
            float cand_yaw   = atan2f(cand_fwd_y, cand_fwd_x);
            float dyaw       = cand_yaw - vision_yaw;
            while (dyaw >  (float)M_PI) dyaw -= 2.0f * (float)M_PI;
            while (dyaw < -(float)M_PI) dyaw += 2.0f * (float)M_PI;
            /* Yaw-continuity gate: reject the ~180° IPPE same-position yaw flip
             * (which the position-jump gate cannot see). Only active once vision
             * is established — the first fix after arm/dropout is taken as the
             * baseline. Yaw is not fused into EKF2 so flips only affect vision_yaw
             * used for the continuity gate itself, not PX4 heading state. */
            if (vision_pose_valid && jump > MAX_POSE_JUMP_M) {
                ESP_LOGW(TAG, "Vision jump %.2fm rejected (ArUco flip?)", (double)jump);
            } else if (vision_pose_valid && fabsf(dyaw) > MAX_YAW_JUMP_RAD) {
                ESP_LOGW(TAG, "Vision yaw jump %.0f° rejected (ArUco yaw flip?)",
                         (double)(dyaw * 180.0f / (float)M_PI));
            } else {
                /* Detect re-acquisition after a vision dropout (or the very first
                 * fix) so we can ramp EV covariance loose→tight below. */
                static int64_t reacq_ms = 0;
                int64_t prev_vis_ms = last_vision_pose_ms;

                vp_x = pose.x; vp_y = pose.y; vp_z = pose.z;
                vp_qx = pose.qx; vp_qy = pose.qy; vp_qz = pose.qz; vp_qw = pose.qw;
                vp_last_x = vp_x; vp_last_y = vp_y; vp_last_z = vp_z;
                vision_pose_valid   = true;
                last_vision_pose_ms = now_ms();

                if (prev_vis_ms == 0 ||
                        (last_vision_pose_ms - prev_vis_ms) > (int64_t)VISION_TIMEOUT_MS)
                    reacq_ms = last_vision_pose_ms;   /* start the re-acquire ramp */

                /* Keep inertial anchor in sync with current vision position so the
                 * RViz disc doesn't teleport when vision times out. */
                if (px4_pos_valid) {
                    map_home_x = pose.x; map_home_y = pose.y; map_home_z = pose.z;
                    px4_home_x = px4_pos_x; px4_home_y = px4_pos_y; px4_home_z = px4_pos_z;
                    inertial_anchor_valid = true;
                }

                /* Camera +Z = body forward; project onto world XY to get heading.
                 * Used only for the yaw-continuity gate — not fused into EKF2. */
                vision_yaw  = cand_yaw;

                if (vision_enabled) {
                    float tx = frame_sign * (vp_x - ned_offset_x);
                    float ty = frame_sign * (ned_offset_y - vp_y);
                    last_vis_sent_x = tx;
                    last_vis_sent_y = ty;
                    /* Dynamic covariance: var = 0.01 + (reproj/10)² × 0.49
                     * <2px→~0.01 (excellent), 5px→~0.13, 8px→~0.32, 10px=rejected */
                    float r = pose.reproj_err / REPROJ_REJECT_PX;
                    float pos_var = 0.01f + r * r * 0.49f;
                    /* Re-acquisition ramp: after a dropout the re-acquired pose may
                     * disagree with the flow-propagated EKF position by metres. Start
                     * loose and tighten over REACQ_RAMP_MS so EKF2 slews to it instead
                     * of hard-resetting (the log_24 teleport-into-wall). */
                    int64_t since = last_vision_pose_ms - reacq_ms;
                    if (reacq_ms != 0 && since < (int64_t)REACQ_RAMP_MS) {
                        float ramp = REACQ_VAR_MAX *
                                     (1.0f - (float)since / (float)REACQ_RAMP_MS);
                        if (ramp > pos_var) pos_var = ramp;
                    }
                    mav_send_vision_estimate(tx, ty, -vp_z, vision_yaw, pos_var); /* arena Z up→NED Z down */
                }
            }
            } /* end reproj_err gate */
        } else if (vision_enabled && last_vision_pose_ms > 0) {
            /* Issue 2: vision fade-out — after ArUco is lost keep sending the last
             * known pose with rising covariance for VISION_FADE_MS.  PX4 EKF2
             * gradually down-weights the source instead of hard-resetting to NED
             * origin (which caused the "teleport to start" crash). */
            int64_t age = now_ms() - last_vision_pose_ms;
            if (age < (int64_t)VISION_FADE_MS) {
                float cov = 0.01f + (float)age / (float)VISION_FADE_MS * 0.49f;
                float tx = frame_sign * (vp_last_x - ned_offset_x);
                float ty = frame_sign * (ned_offset_y - vp_last_y);
                last_vis_sent_x = tx;
                last_vis_sent_y = ty;
                mav_send_vision_estimate(tx, ty, -vp_last_z, vision_yaw, cov); /* arena Z up→NED Z down */
            }
        }

#if SEED_YAW_ENABLE
        /* ── Heading seed (no magnetometer) ─────────────────────────────────────
         * Before the first ArUco fix, EKF2 has no absolute heading (mag off), so the
         * first vision yaw snaps it ~180° (the "first-marker flip"). For a short
         * window after arm — while the drone is still on the ground at the known
         * start heading — send START_YAW as a yaw-only vision estimate (position
         * echoes the EKF's own estimate at loose covariance, so only yaw is fused).
         * EKF2 yaw converges to START_YAW; then we stop so the seed can't fight
         * later yaw maneuvers, and gyro (low drift) carries it until real vision.
         * Stops early if real vision arrives (last_vision_pose_ms != 0). */
        {
            static int64_t seed_start_ms = 0;
            static uint8_t seed_prev_state = 255;
            if (drone_state != seed_prev_state) {
                if (drone_state == DRONE_ARMED) seed_start_ms = now_ms();  /* (re)start on arm */
                seed_prev_state = drone_state;
            }
            static int64_t last_seed_ms = 0;
            if (vision_enabled && seed_yaw_valid && last_vision_pose_ms == 0 && px4_pos_valid &&
                seed_start_ms != 0 && (now_ms() - seed_start_ms) < SEED_YAW_MS &&
                (drone_state == DRONE_ARMED || drone_state == DRONE_MISSION) &&
                (now_ms() - last_seed_ms) >= 100) {                        /* ~10 Hz */
                last_seed_ms = now_ms();
                mav_send_vision_estimate(px4_pos_x, px4_pos_y, -px4_pos_z,
                                         seed_yaw_rad, 9.0f);
            }
        }
#endif

#ifdef TEST_ARUCO_APPROACH
        if (pose.valid && pose.trigger_id == TEST_TRIGGER_ID &&
                (drone_state == DRONE_MISSION || drone_state == DRONE_ARMED) &&
                !s_test_aruco_triggered) {
            s_test_aruco_triggered = true;
            xTaskCreate(aruco_approach_task_fn, "aruco_trig", 3072, NULL, 5, NULL);
            ESP_LOGI(TAG, "[TEST] M%d detected — approach triggered", TEST_TRIGGER_ID);
        }
#endif

        /* MAVLink: heartbeat at 1 Hz, obstacle data at 20 Hz */
        if (++hb_tick >= 20) { hb_tick = 0; send_heartbeat_once(); }
        send_obstacle_distance(&tof);
        send_upward_distance_sensor(&tof);

        /* Console — overwrite line at 20 Hz */
        tof_console_print(&tof);

        /* Position debug log — every 2 s (40 ticks × 50 ms) */
        static uint32_t pos_log_tick = 0;
        if (++pos_log_tick >= 40) {
            pos_log_tick = 0;
            printf("\n");
            fflush(stdout);
            ESP_LOGI(TAG, "[POS] P4_aruco=(%.2f,%.2f)  sent_px4=(%.2f,%.2f,yaw=%.1f)  px4_ned=(%.2f,%.2f,yaw=%.1f)  offset=(%.0f,%.0f)",
                     (double)vp_x,                          (double)vp_y,
                     (double)last_vis_sent_x,               (double)last_vis_sent_y,
                     (double)(vision_yaw * 180.0f / M_PI),
                     (double)px4_pos_x,                     (double)px4_pos_y,
                     (double)(px4_yaw    * 180.0f / M_PI),
                     (double)ned_offset_x,                  (double)ned_offset_y);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
