# ESP-IDF project. Pick the board with -D BOARD=<name> (default: generic),
# one build directory per board:
#
#   . ~/esp/esp-idf/export.sh
#   idf.py -B build-xiao_esp32s3 -D BOARD=xiao_esp32s3 -p /dev/ttyACM0 flash monitor
#
# Boards are boards/sdkconfig.<name> (settings) + boards/board_<name>.h (pins).
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED BOARD)
    set(BOARD generic)
endif()
if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/boards/sdkconfig.${BOARD}")
    message(FATAL_ERROR "Unknown BOARD '${BOARD}': no boards/sdkconfig.${BOARD}")
endif()
set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;boards/sdkconfig.${BOARD}")
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
set(EXTRA_COMPONENT_DIRS boards)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dump3411-esp32)
