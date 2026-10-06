idf_component_register(
    SRCS "main.c" "selftest.c" "wifi_capture.c"
    INCLUDE_DIRS "."
    REQUIRES odid esp_wifi esp_event esp_timer nvs_flash
)
