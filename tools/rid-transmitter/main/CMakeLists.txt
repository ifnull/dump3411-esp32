idf_component_register(
    SRCS "main.c" "sim.c" "wifi_tx.c" "ble_tx.c"
    INCLUDE_DIRS "."
    REQUIRES opendroneid esp_wifi esp_netif esp_event nvs_flash bt
)
