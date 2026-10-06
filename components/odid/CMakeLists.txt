# ESP-IDF component registration. host/CMakeLists.txt compiles the same
# sources natively for the parity tests.
idf_component_register(
    SRCS "odid_parser.c" "odid_wifi.c" "odid_tracker.c"
    INCLUDE_DIRS "include"
)
