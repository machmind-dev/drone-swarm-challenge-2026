idf_component_register(SRCS 
  "core/VL53L1X_api.c"
  "core/VL53L1X_calibration.c"
  "platform/vl53l1_platform.c"
  "platform/i2c_platform_esp.c"
  "platform/gpio_platform_esp.c"

  INCLUDE_DIRS "core" "platform"
  REQUIRES mbedtls log driver esp_timer freertos)

