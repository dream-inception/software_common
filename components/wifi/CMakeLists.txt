idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include" "src"
                       PRIV_REQUIRES "esp_wifi" "esp_netif")
# target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
