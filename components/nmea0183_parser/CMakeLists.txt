# For more information about component CMakeLists.txt files, see:
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#component-cmakelists-txt-files

idf_component_register(SRCS "nmea_parser.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_event
                    REQUIRES kalman_filter driver log
                    PRIV_REQUIRES main)

