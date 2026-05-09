idf_component_register(SRCS main.c p4_link.c
  PRIV_INCLUDE_DIRS .
  PRIV_REQUIRES esp_psram nvs_flash esp_timer driver heap esp_netif
                micro_ros_espidf_component
)
