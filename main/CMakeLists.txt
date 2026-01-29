# For more information about component CMakeLists.txt files, see:
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#component-cmakelists-txt-files

# The following five lines of boilerplate have been deprecated since ESP-IDF v4.0 and are not necessary
# idf_component_get_property(res IDF_PATH "idf_path")
# set(EXTRA_COMPONENT_DIRS "")
# if(EXISTS ${res}/components)
#     set(EXTRA_COMPONENT_DIRS ${res}/components)
# endif()


idf_component_register(SRCS 
                            "src/main.c"
                            "src/tasks.c"
                            "src/hardware.c"
                            "src/mqtt_handler.c"
                            "src/sms_handler.c"
                    INCLUDE_DIRS 
                            "include"
                    REQUIRES mpu6050 kalman_filter nmea0183_parser driver nvs_flash esp_timer json esp_adc)
