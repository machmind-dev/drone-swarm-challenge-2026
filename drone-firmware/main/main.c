/* main.c
 *
 * Base:
 * - 4x VL53L1X ToF sensors
 * - MAVLink heartbeat + OBSTACLE_DISTANCE
 * - Drone ID LED burst
 *
 * Added:
 * - micro-ROS
 * - RViz drone flat disc marker (7 inch = 0.18m)
 * - RViz obstacle distance numbers only (TEXT_VIEW_FACING)
 * - Camera image published inside timer_callback
 * - GCS command subscriber — drone_X_video_on / drone_X_video_off
 * - GCS vision pose subscriber — /drone_X/vision_pose
 * - Device hostname from DRONE_ID
 * - Preferred static IP from DRONE_ID
 * - Initial RViz fallback position derived from DRONE_ID
 * - Unique RViz marker IDs per drone — no flicker in multi-drone RViz
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

/* micro-ROS / ROS2 */
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

/* Camera board pinout — XIAO ESP32S3 Sense */
#define CAMERA_MODEL_XIAO_ESP32S3 1
#include "boards.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

static const char *TAG = "drone";

#define DRONE_ID_LED_PIN       GPIO_NUM_1
#define DRONE_ID               1

#define DRONE_DISC_DIAMETER_M  0.18f
#define DRONE_DISC_THICKNESS_M 0.02f
#define DRONE_DEFAULT_Z_M      0.5f
#define VISION_TIMEOUT_MS      1500

/* Marker ID scheme — unique per drone, no RViz flicker:
 * disc  = DRONE_ID * 100
 * text  = DRONE_ID * 100 + 1
 * right = DRONE_ID * 100 + 10
 * top   = DRONE_ID * 100 + 11
 * left  = DRONE_ID * 100 + 12
 * front = DRONE_ID * 100 + 13
 */
#define MARKER_ID_DISC(id)    ((id) * 100)
#define MARKER_ID_TEXT(id)    ((id) * 100 + 1)
#define MARKER_ID_RIGHT(id)   ((id) * 100 + 10)
#define MARKER_ID_TOP(id)     ((id) * 100 + 11)
#define MARKER_ID_LEFT(id)    ((id) * 100 + 12)
#define MARKER_ID_FRONT(id)   ((id) * 100 + 13)

#define DRONE_IP_BASE_OCTET 100
#define DRONE_IP_NETMASK    "255.255.255.0"
#define DRONE_IP_GATEWAY    "192.168.178.1"
#define DRONE_IP_PREFIX     "192.168.178."

/* ══════════════════════════════════════════════════════
 * UART / MAVLink
 * ══════════════════════════════════════════════════════ */

#define UART_TX GPIO_NUM_43
#define UART_RX GPIO_NUM_44

static void uart_mavlink_init(void)
{
    uart_config_t uart_config = {
        .baud_rate  = 57600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_TX, UART_RX,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void mav_send(const mavlink_message_t *msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
    uart_write_bytes(UART_NUM_1, (const char *)buffer, len);
}

static void send_heartbeat_once(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(42, 200, &msg,
        MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    mav_send(&msg);
}

static void __attribute__((unused))
send_distance_sensor_mm(int range_mm, uint8_t sensor_id, uint8_t orientation)
{
    mavlink_message_t msg;
    int distance_cm = range_mm / 10;
    if (distance_cm < 4)   distance_cm = 4;
    if (distance_cm > 400) distance_cm = 400;
    uint32_t time_boot_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    mavlink_msg_distance_sensor_pack(42, 200, &msg,
        time_boot_ms, 4, 400, distance_cm,
        MAV_DISTANCE_SENSOR_LASER, sensor_id, orientation,
        UINT8_MAX, 0.47f, 0.47f, (float[4]){0}, 100);
    mav_send(&msg);
}

/* ══════════════════════════════════════════════════════
 * ToF sensors
 * ══════════════════════════════════════════════════════ */

#define MEASUREMENT_CYCLE_MS 20
#define TIMER_PERIODIC_MS    25

VL53L1_Dev_t tof_array[] = {
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 2, .shutdown_pin = GPIO_NUM_2,
      .interrupt_pin = 0, .distance_mode = DISTANCE_MODE_SHORT,
      .timing_budget = MEASUREMENT_CYCLE_MS, .inter_measurement = TIMER_PERIODIC_MS },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 4, .shutdown_pin = GPIO_NUM_3,
      .interrupt_pin = 0, .distance_mode = DISTANCE_MODE_SHORT,
      .timing_budget = MEASUREMENT_CYCLE_MS, .inter_measurement = TIMER_PERIODIC_MS },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 6, .shutdown_pin = GPIO_NUM_4,
      .interrupt_pin = 0, .distance_mode = DISTANCE_MODE_SHORT,
      .timing_budget = MEASUREMENT_CYCLE_MS, .inter_measurement = TIMER_PERIODIC_MS },
    { .I2cDevAddr = VL53L1_I2C_ADDRESS + 8, .shutdown_pin = GPIO_NUM_41,
      .interrupt_pin = 0, .distance_mode = DISTANCE_MODE_SHORT,
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

    uint16_t right_cm = tof_array[0].range_mm / 10;
    uint16_t top_cm   = tof_array[1].range_mm / 10;
    uint16_t left_cm  = tof_array[2].range_mm / 10;
    uint16_t front_cm = tof_array[3].range_mm / 10;

    if (right_cm < 4)   right_cm = 4;
    if (right_cm > 400) right_cm = 400;
    if (top_cm < 4)     top_cm = 4;
    if (top_cm > 400)   top_cm = 400;
    if (left_cm < 4)    left_cm = 4;
    if (left_cm > 400)  left_cm = 400;
    if (front_cm < 4)   front_cm = 4;
    if (front_cm > 400) front_cm = 400;

    distances[71] = distances[0] = distances[1] = front_cm;
    distances[17] = distances[18] = distances[19] = right_cm;
    distances[35] = distances[36] = distances[37] = top_cm;
    distances[53] = distances[54] = distances[55] = left_cm;

    mavlink_msg_obstacle_distance_pack(42, 200, &msg,
        esp_timer_get_time(), MAV_DISTANCE_SENSOR_LASER,
        distances, 5, 4, 400, 5.0f, 0.0f, MAV_FRAME_BODY_FRD);
    mav_send(&msg);
}

/* ══════════════════════════════════════════════════════
 * Drone ID LED
 * ══════════════════════════════════════════════════════ */

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
                blink_index = 0;
                gpio_set_level(DRONE_ID_LED_PIN, 1);
                last_tick = now; state = 1;
            }
            break;
        case 1:
            if ((now - last_tick) >= on_time) {
                gpio_set_level(DRONE_ID_LED_PIN, 0);
                last_tick = now; blink_index++; state = 2;
            }
            break;
        case 2:
            if (blink_index >= DRONE_ID) {
                state = 0; last_tick = now;
            } else if ((now - last_tick) >= off_time) {
                gpio_set_level(DRONE_ID_LED_PIN, 1);
                last_tick = now; state = 1;
            }
            break;
    }
}

/* ══════════════════════════════════════════════════════
 * Camera config
 * ══════════════════════════════════════════════════════ */

static const camera_config_t camera_config = {
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
    .frame_size   = FRAMESIZE_QQVGA,
    .jpeg_quality = 63,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
};

/* ══════════════════════════════════════════════════════
 * micro-ROS publishers + subscribers + message buffers
 * ══════════════════════════════════════════════════════ */

#define RCCHECK(fn) \
    { rcl_ret_t temp_rc = fn; \
      if (temp_rc != RCL_RET_OK) { \
          printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
          vTaskDelete(NULL); } }

#define RCSOFTCHECK(fn) \
    { rcl_ret_t temp_rc = fn; \
      if (temp_rc != RCL_RET_OK) { \
          printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); } }

static rcl_publisher_t         publisher_image;
static sensor_msgs__msg__Image img_msg;
static struct timespec         ts;

static rcl_publisher_t         publisher_drone_position_marker;
static visualization_msgs__msg__Marker drone_disc_msg;
static visualization_msgs__msg__Marker text_msg;
static visualization_msgs__msg__Marker obstacle_labels[4];

static rcl_subscription_t             gcs_sub;
static std_msgs__msg__String          gcs_msg;

static rcl_subscription_t             vision_pose_sub;
static geometry_msgs__msg__PoseStamped vision_pose_msg;

static volatile bool    camera_streaming     = true;
static volatile bool    vision_pose_valid    = false;
static volatile int64_t last_vision_pose_ms  = 0;

static char topic_image[64];
static char topic_vision_pose[64];
static char topic_camera_frame[32];
static char topic_gcs_command[32];
static char drone_ns[16];   // per-drone namespace e.g. "drone_4"

/* ══════════════════════════════════════════════════════
 * Helpers
 * ══════════════════════════════════════════════════════ */

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static float clamp_m(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static float get_initial_x_from_drone_id(void) { return 1.0f; }
static float get_initial_y_from_drone_id(void) { return (float)DRONE_ID; }
static float get_initial_z_from_drone_id(void) { return DRONE_DEFAULT_Z_M; }

static void set_label_color_by_distance(visualization_msgs__msg__Marker *m, float dist_m)
{
    if (dist_m < 0.30f) {
        m->color.r = 1.0f; m->color.g = 0.0f;
        m->color.b = 0.0f; m->color.a = 1.0f;
    } else if (dist_m < 0.80f) {
        m->color.r = 1.0f; m->color.g = 0.6f;
        m->color.b = 0.0f; m->color.a = 1.0f;
    } else {
        m->color.r = 0.0f; m->color.g = 1.0f;
        m->color.b = 0.0f; m->color.a = 1.0f;
    }
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

static void apply_pose_to_drone_markers(
    float px, float py, float pz,
    float qx, float qy, float qz, float qw)
{
    drone_disc_msg.pose.position.x    = px;
    drone_disc_msg.pose.position.y    = py;
    drone_disc_msg.pose.position.z    = pz;
    drone_disc_msg.pose.orientation.x = qx;
    drone_disc_msg.pose.orientation.y = qy;
    drone_disc_msg.pose.orientation.z = qz;
    drone_disc_msg.pose.orientation.w = qw;

    text_msg.pose.position.x    = px;
    text_msg.pose.position.y    = py;
    text_msg.pose.position.z    = pz + 0.22f;
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

    float right_m = clamp_m(tof_array[0].range_mm / 1000.0f, 0.05f, 4.0f);
    float top_m   = clamp_m(tof_array[1].range_mm / 1000.0f, 0.05f, 4.0f);
    float left_m  = clamp_m(tof_array[2].range_mm / 1000.0f, 0.05f, 4.0f);
    float front_m = clamp_m(tof_array[3].range_mm / 1000.0f, 0.05f, 4.0f);

    int right_cm = tof_array[0].range_mm / 10;
    int top_cm   = tof_array[1].range_mm / 10;
    int left_cm  = tof_array[2].range_mm / 10;
    int front_cm = tof_array[3].range_mm / 10;

    const float offset = 0.20f;

    /* RIGHT */
    obstacle_labels[0].pose.position.x = dx;
    obstacle_labels[0].pose.position.y = dy - offset;
    obstacle_labels[0].pose.position.z = dz;
    snprintf(obstacle_labels[0].text.data, 16, "%d", right_cm);
    obstacle_labels[0].text.size = strlen(obstacle_labels[0].text.data);
    set_label_color_by_distance(&obstacle_labels[0], right_m);

    /* TOP */
    obstacle_labels[1].pose.position.x = dx;
    obstacle_labels[1].pose.position.y = dy;
    obstacle_labels[1].pose.position.z = dz + offset;
    snprintf(obstacle_labels[1].text.data, 16, "%d", top_cm);
    obstacle_labels[1].text.size = strlen(obstacle_labels[1].text.data);
    set_label_color_by_distance(&obstacle_labels[1], top_m);

    /* LEFT */
    obstacle_labels[2].pose.position.x = dx;
    obstacle_labels[2].pose.position.y = dy + offset;
    obstacle_labels[2].pose.position.z = dz;
    snprintf(obstacle_labels[2].text.data, 16, "%d", left_cm);
    obstacle_labels[2].text.size = strlen(obstacle_labels[2].text.data);
    set_label_color_by_distance(&obstacle_labels[2], left_m);

    /* FRONT */
    obstacle_labels[3].pose.position.x = dx + offset;
    obstacle_labels[3].pose.position.y = dy;
    obstacle_labels[3].pose.position.z = dz;
    snprintf(obstacle_labels[3].text.data, 16, "%d", front_cm);
    obstacle_labels[3].text.size = strlen(obstacle_labels[3].text.data);
    set_label_color_by_distance(&obstacle_labels[3], front_m);
}

/* ══════════════════════════════════════════════════════
 * Device hostname / preferred static IP
 * ══════════════════════════════════════════════════════ */

static void set_device_hostname_from_drone_id(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) { ESP_LOGW(TAG, "netif not found, cannot set hostname"); return; }

    char hostname[32];
    snprintf(hostname, sizeof(hostname), "mach-mind-drone-%d", DRONE_ID);
    if (esp_netif_set_hostname(netif, hostname) == ESP_OK)
        ESP_LOGI(TAG, "Hostname: %s", hostname);
    else
        ESP_LOGE(TAG, "Failed to set hostname");
}

static esp_err_t set_preferred_ip_from_drone_id(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) { ESP_LOGE(TAG, "netif not found"); return ESP_FAIL; }

    esp_err_t err = esp_netif_dhcpc_stop(netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "DHCP stop failed: %s", esp_err_to_name(err)); return err;
    }

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));

    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), DRONE_IP_PREFIX "%d",
             DRONE_IP_BASE_OCTET + DRONE_ID);

    ip4addr_aton(ip_str,           &ip_info.ip);
    ip4addr_aton(DRONE_IP_GATEWAY, &ip_info.gw);
    ip4addr_aton(DRONE_IP_NETMASK, &ip_info.netmask);

    err = esp_netif_set_ip_info(netif, &ip_info);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "Static IP: %s  GW: %s", ip_str, DRONE_IP_GATEWAY);
    else
        ESP_LOGE(TAG, "Failed to set static IP: %s", esp_err_to_name(err));
    return err;
}

static void print_current_ip_info(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return;
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
        ESP_LOGI(TAG, "IP: " IPSTR "  GW: " IPSTR "  MASK: " IPSTR,
                 IP2STR(&ip_info.ip), IP2STR(&ip_info.gw),
                 IP2STR(&ip_info.netmask));
}

/* ══════════════════════════════════════════════════════
 * GCS command callback
 * ══════════════════════════════════════════════════════ */

static void gcs_command_callback(const void *msg_in)
{
    const std_msgs__msg__String *cmd = (const std_msgs__msg__String *)msg_in;
    if (!cmd || !cmd->data.data || cmd->data.size == 0) {
        ESP_LOGW(TAG, "GCS: empty message"); return;
    }

    char buf[64] = {0};
    size_t copy_len = cmd->data.size < sizeof(buf) - 1
                      ? cmd->data.size : sizeof(buf) - 1;
    memcpy(buf, cmd->data.data, copy_len);
    for (int i = (int)copy_len - 1; i >= 0; i--) {
        if (buf[i] == ' ' || buf[i] == '\n' ||
            buf[i] == '\r' || buf[i] == '\t') buf[i] = '\0';
        else break;
    }

    char on_cmd[32], off_cmd[32];
    snprintf(on_cmd,  sizeof(on_cmd),  "drone_%d_video_on",  DRONE_ID);
    snprintf(off_cmd, sizeof(off_cmd), "drone_%d_video_off", DRONE_ID);

    if (strcmp(buf, on_cmd) == 0) {
        camera_streaming = true;
        ESP_LOGI(TAG, "GCS: camera streaming ON");
    } else if (strcmp(buf, off_cmd) == 0) {
        camera_streaming = false;
        ESP_LOGI(TAG, "GCS: camera streaming OFF");
    } else {
        ESP_LOGW(TAG, "GCS: no match — '%s'", buf);
    }
}

/* ══════════════════════════════════════════════════════
 * Vision pose callback
 * ══════════════════════════════════════════════════════ */

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

    vision_pose_valid   = true;
    last_vision_pose_ms = now_ms();

    ESP_LOGI(TAG, "Vision pose: (%.2f, %.2f, %.2f)", px, py, pz);
}

/* ══════════════════════════════════════════════════════
 * Timer callback — camera + markers
 * ══════════════════════════════════════════════════════ */

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (!timer) return;

    if (vision_pose_valid) {
        int64_t age_ms = now_ms() - last_vision_pose_ms;
        if (age_ms > VISION_TIMEOUT_MS) {
            vision_pose_valid = false;
            ESP_LOGW(TAG, "Vision pose timeout (%lld ms)", (long long)age_ms);
        }
    }

    /* Camera — gated by GCS command */
    if (camera_streaming) {
        camera_fb_t *pic = esp_camera_fb_get();
        if (pic) {
            if (pic->len <= img_msg.data.capacity) {
                clock_gettime(CLOCK_REALTIME, &ts);
                img_msg.header.stamp.sec     = ts.tv_sec;
                img_msg.header.stamp.nanosec = ts.tv_nsec;
                img_msg.header.frame_id =
                    micro_ros_string_utilities_set(
                        img_msg.header.frame_id, topic_camera_frame);
                img_msg.width     = 160;
                img_msg.height    = 120;
                img_msg.step      = 160;
                img_msg.encoding  =
                    micro_ros_string_utilities_set(img_msg.encoding, "mono8");
                img_msg.data.size = pic->len;
                memcpy(img_msg.data.data, pic->buf, pic->len);
                RCSOFTCHECK(rcl_publish(&publisher_image, &img_msg, NULL));
            }
            esp_camera_fb_return(pic);
        } else {
            ESP_LOGW(TAG, "Camera capture failed");
        }
    } else {
        camera_fb_t *stale = esp_camera_fb_get();
        if (stale) esp_camera_fb_return(stale);
    }

    update_obstacle_markers();

    RCSOFTCHECK(rcl_publish(&publisher_drone_position_marker, &drone_disc_msg, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_drone_position_marker, &text_msg, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_drone_position_marker, &obstacle_labels[0], NULL));
    RCSOFTCHECK(rcl_publish(&publisher_drone_position_marker, &obstacle_labels[1], NULL));
    RCSOFTCHECK(rcl_publish(&publisher_drone_position_marker, &obstacle_labels[2], NULL));
    RCSOFTCHECK(rcl_publish(&publisher_drone_position_marker, &obstacle_labels[3], NULL));
}

/* ══════════════════════════════════════════════════════
 * micro-ROS task
 * ══════════════════════════════════════════════════════ */

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

    RCCHECK(rclc_publisher_init_default(
        &publisher_image, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
        topic_image));

    RCCHECK(rclc_publisher_init_default(
        &publisher_drone_position_marker, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, Marker),
        "/visualization_marker"));

    RCCHECK(rclc_subscription_init_best_effort(
        &gcs_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        topic_gcs_command));

    RCCHECK(rclc_subscription_init_best_effort(
        &vision_pose_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        topic_vision_pose));

    gcs_msg.data.data     = (char *)malloc(64);
    gcs_msg.data.size     = 0;
    gcs_msg.data.capacity = 64;

    geometry_msgs__msg__PoseStamped__init(&vision_pose_msg);

    static micro_ros_utilities_memory_conf_t conf = {};
    conf.max_string_capacity              = 50;
    conf.max_ros2_type_sequence_capacity  = 5;
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
        &img_msg, conf);

    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(&timer, &support,
                                     RCL_MS_TO_NS(100), timer_callback));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &gcs_sub,
                &gcs_msg, &gcs_command_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &vision_pose_sub,
                &vision_pose_msg, &vision_pose_callback, ON_NEW_DATA));

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&publisher_image, &node));
    RCCHECK(rcl_publisher_fini(&publisher_drone_position_marker, &node));
    RCCHECK(rcl_subscription_fini(&gcs_sub, &node));
    RCCHECK(rcl_subscription_fini(&vision_pose_sub, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

/* ══════════════════════════════════════════════════════
 * app_main
 * ══════════════════════════════════════════════════════ */

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    set_device_hostname_from_drone_id();
    ESP_ERROR_CHECK(set_preferred_ip_from_drone_id());
    print_current_ip_info();
#endif

    /* Build per-drone namespace and topic strings */
    snprintf(drone_ns,          sizeof(drone_ns),          "drone_%d",                    DRONE_ID);
    snprintf(topic_image,       sizeof(topic_image),        "/drone_%d/camera/image_raw",  DRONE_ID);
    snprintf(topic_vision_pose, sizeof(topic_vision_pose),  "/drone_%d/vision_pose",       DRONE_ID);
    snprintf(topic_camera_frame,sizeof(topic_camera_frame), "drone_%d/camera",             DRONE_ID);
    snprintf(topic_gcs_command, sizeof(topic_gcs_command),  "/gcs_command");

    ESP_LOGI(TAG, "==============================");
    ESP_LOGI(TAG, "DRONE ID   : %d",              DRONE_ID);
    ESP_LOGI(TAG, "Namespace  : %s",              drone_ns);
    ESP_LOGI(TAG, "Image topic: %s",              topic_image);
    ESP_LOGI(TAG, "Pose topic : %s",              topic_vision_pose);
    ESP_LOGI(TAG, "Hostname   : mach-mind-drone-%d", DRONE_ID);
    ESP_LOGI(TAG, "Expected IP: 192.168.178.%d",  100 + DRONE_ID);
    ESP_LOGI(TAG, "Marker IDs : disc=%d text=%d obs=%d-%d",
             MARKER_ID_DISC(DRONE_ID), MARKER_ID_TEXT(DRONE_ID),
             MARKER_ID_RIGHT(DRONE_ID), MARKER_ID_FRONT(DRONE_ID));
    ESP_LOGI(TAG, "==============================");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DRONE_ID_LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = 0, .pull_down_en = 0,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\r\nDevice Starting — Drone ID %d\r\n", DRONE_ID);

    i2c_init();
    uart_mavlink_init();
    VL53L1X_InitSensorArray(tof_array, sensor_count);

    const esp_timer_create_args_t tof_timer_args = {
        .callback = &periodic_tof_sensor,
        .name     = "tofsensor"
    };
    ESP_ERROR_CHECK(esp_timer_create(&tof_timer_args, &tof_sensor_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tof_sensor_timer,
                                              TIMER_PERIODIC_MS * 1000));

    ESP_LOGI(TAG, "Camera SCCB SDA:GPIO_%d SCL:GPIO_%d",
             CAMERA_PIN_SIOD, CAMERA_PIN_SIOC);
    if (esp_camera_init(&camera_config) != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed — continuing without camera");
        camera_streaming = false;
    } else {
        ESP_LOGI(TAG, "Camera OK");
        sensor_t *s = esp_camera_sensor_get();
        if (s) { s->set_vflip(s, 1); s->set_hmirror(s, 1); }
    }

    const float initial_x = get_initial_x_from_drone_id();
    const float initial_y = get_initial_y_from_drone_id();
    const float initial_z = get_initial_z_from_drone_id();

    /* Drone flat disc — unique ID and namespace per drone */
    visualization_msgs__msg__Marker__init(&drone_disc_msg);
    rosidl_runtime_c__String__assign(&drone_disc_msg.header.frame_id, "map");
    rosidl_runtime_c__String__assign(&drone_disc_msg.ns, drone_ns);
    drone_disc_msg.id     = MARKER_ID_DISC(DRONE_ID);
    drone_disc_msg.type   = visualization_msgs__msg__Marker__CYLINDER;
    drone_disc_msg.action = visualization_msgs__msg__Marker__ADD;
    drone_disc_msg.pose.position.x    = initial_x;
    drone_disc_msg.pose.position.y    = initial_y;
    drone_disc_msg.pose.position.z    = initial_z;
    drone_disc_msg.pose.orientation.w = 1.0f;
    drone_disc_msg.scale.x = DRONE_DISC_DIAMETER_M;
    drone_disc_msg.scale.y = DRONE_DISC_DIAMETER_M;
    drone_disc_msg.scale.z = DRONE_DISC_THICKNESS_M;
    drone_disc_msg.color.r = 0.2f;
    drone_disc_msg.color.g = 0.6f;
    drone_disc_msg.color.b = 1.0f;
    drone_disc_msg.color.a = 0.95f;

    /* Drone text label — unique ID and namespace per drone */
    visualization_msgs__msg__Marker__init(&text_msg);
    rosidl_runtime_c__String__assign(&text_msg.header.frame_id, "map");
    rosidl_runtime_c__String__assign(&text_msg.ns, drone_ns);
    text_msg.id     = MARKER_ID_TEXT(DRONE_ID);
    text_msg.type   = visualization_msgs__msg__Marker__TEXT_VIEW_FACING;
    text_msg.action = visualization_msgs__msg__Marker__ADD;
    text_msg.pose.position.x    = initial_x;
    text_msg.pose.position.y    = initial_y;
    text_msg.pose.position.z    = initial_z + 0.22f;
    text_msg.pose.orientation.w = 1.0f;
    text_msg.scale.z = 0.16f;
    text_msg.color.r = text_msg.color.g = text_msg.color.b = text_msg.color.a = 1.0f;

    char drone_label[8];
    snprintf(drone_label, sizeof(drone_label), "D%d", DRONE_ID);
    rosidl_runtime_c__String__assign(&text_msg.text, drone_label);

    /* Obstacle labels — unique IDs per drone, drone-specific namespace */
    char obs_ns[4][32];
    snprintf(obs_ns[0], sizeof(obs_ns[0]), "%s_obs_right", drone_ns);
    snprintf(obs_ns[1], sizeof(obs_ns[1]), "%s_obs_top",   drone_ns);
    snprintf(obs_ns[2], sizeof(obs_ns[2]), "%s_obs_left",  drone_ns);
    snprintf(obs_ns[3], sizeof(obs_ns[3]), "%s_obs_front", drone_ns);

    init_obstacle_label(&obstacle_labels[0], MARKER_ID_RIGHT(DRONE_ID), obs_ns[0]);
    init_obstacle_label(&obstacle_labels[1], MARKER_ID_TOP(DRONE_ID),   obs_ns[1]);
    init_obstacle_label(&obstacle_labels[2], MARKER_ID_LEFT(DRONE_ID),  obs_ns[2]);
    init_obstacle_label(&obstacle_labels[3], MARKER_ID_FRONT(DRONE_ID), obs_ns[3]);

    apply_pose_to_drone_markers(initial_x, initial_y, initial_z,
                                 0.0f, 0.0f, 0.0f, 1.0f);
    update_obstacle_markers();

    xTaskCreate(micro_ros_task, "uros_task",
                CONFIG_MICRO_ROS_APP_STACK, NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);

    while (1) {
        drone_id_led_update();
        send_heartbeat_once();
        send_obstacle_distance_all4_debug();

        printf("\r RIGHT: %4d cm | TOP: %4d cm | LEFT: %4d cm | "
               "FRONT: %4d cm | VISION: %s    ",
               tof_array[0].range_mm / 10,
               tof_array[1].range_mm / 10,
               tof_array[2].range_mm / 10,
               tof_array[3].range_mm / 10,
               vision_pose_valid ? "OK" : "NO");

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
