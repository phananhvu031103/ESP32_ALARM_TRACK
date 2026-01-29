# For more information about component CMakeLists.txt files, see:
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#component-cmakelists-txt-files

idf_component_register(SRCS "kalman_filter.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_INCLUDE_DIRS ".")
