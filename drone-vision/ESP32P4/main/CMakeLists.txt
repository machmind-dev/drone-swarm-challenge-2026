idf_component_register(SRCS main.c aruco_pose.cpp
  PRIV_INCLUDE_DIRS .
  PRIV_REQUIRES nvs_flash esp_timer driver heap esp_pm espressif__opencv
                espressif__esp_video
                esp_psram esp_mm esp_hw_support vfs
  LDFRAGMENTS sbss_psram.lf
)

# -DCAMERA_VIEW_MODE : color+gray live viewer, no OpenCV -- use with view_frame.py
# Remove -DCAMERA_VIEW_MODE to restore normal ArUco detection mode.
target_compile_options(${COMPONENT_LIB} PRIVATE "-UEPS")
