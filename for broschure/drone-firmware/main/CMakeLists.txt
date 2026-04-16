idf_component_register(SRCS main.c
  PRIV_INCLUDE_DIRS .
  PRIV_REQUIRES esp_psram micro_ros_espidf_component nvs_flash VL53L1-ULD-ESP
)
