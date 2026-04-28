idf_component_register(SRCS main.c aruco_pose.cpp
  PRIV_INCLUDE_DIRS .
  PRIV_REQUIRES nvs_flash esp_timer driver heap esp_pm espressif__opencv
                espressif__esp_cam_sensor espressif__esp_sccb_intf
                esp_driver_isp esp_psram
  LDFRAGMENTS sbss_psram.lf
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-UEPS")
