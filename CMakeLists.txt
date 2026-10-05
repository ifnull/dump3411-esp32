# ESP-IDF project. Build and flash with:
#   . ~/esp/esp-idf/export.sh
#   idf.py set-target esp32s3      # once
#   idf.py -p /dev/ttyACM0 flash monitor
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(dump3411-esp32)
