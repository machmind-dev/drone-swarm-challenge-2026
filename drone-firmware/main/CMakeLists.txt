idf_component_register(SRCS main.c aruco_task.cpp
  PRIV_INCLUDE_DIRS .
  PRIV_REQUIRES esp_psram micro_ros_espidf_component nvs_flash VL53L1-ULD-ESP espressif__opencv
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-UEPS")
