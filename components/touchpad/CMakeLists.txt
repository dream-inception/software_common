idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES "driver" "touch_element"
                       )
                       
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
