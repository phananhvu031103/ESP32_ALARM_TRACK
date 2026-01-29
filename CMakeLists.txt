# The following five lines of boilerplate have been deprecated since ESP-IDF v4.0 and are not necessary
# if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
#     set(CMAKE_TOOLCHAIN_FILE ${IDF_PATH}/tools/cmake/toolchain-esp32.cmake)
# endif()
# from
#cmake_minimum_required(VERSION 3.5)
#include($ENV{IDF_PATH}/tools/cmake/project.cmake)
#project(template-app)
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Add our custom components directory
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

project(esp_gps_idf)
