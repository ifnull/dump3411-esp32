# Hardware check for the reference build: L76K GNSS, ePaper driver board and
# panel. Uses the xiao_esp32s3 board pins. See README.md.
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../boards)
set(SDKCONFIG_DEFAULTS "${CMAKE_CURRENT_LIST_DIR}/../../boards/sdkconfig.xiao_esp32s3;sdkconfig.defaults")
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hwcheck)
