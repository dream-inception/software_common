idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES "esp_event" "esp_hid")
                       
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format" "-Wno-unused-const-variable")
