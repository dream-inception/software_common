idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       REQUIRES bus mpu6050)
                       
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
