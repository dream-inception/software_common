idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES "driver" "spi_flash" "esp_partition"
                       )
                       
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
