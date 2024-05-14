idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       REQUIRES  "spi_flash" "app_update" "nvs_flash" "esp_wifi" "esp_timer")
                       
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
