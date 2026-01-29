# For more information about component CMakeLists.txt files, see:
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#component-cmakelists-txt-files

idf_component_register(SRCS "mpu6050.c"
                    INCLUDE_DIRS "include"
                    REQUIRES kalman_filter log
                    PRIV_REQUIRES main) # This component depends on the kalman_filter component