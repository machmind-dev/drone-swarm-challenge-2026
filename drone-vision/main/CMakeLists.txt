idf_component_register(SRCS main.c vision_bench.cpp
  PRIV_INCLUDE_DIRS .
  PRIV_REQUIRES esp_psram nvs_flash esp_timer driver heap espressif__opencv psram_bss_scheme
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-UEPS")
