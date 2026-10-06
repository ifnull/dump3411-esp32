idf_component_register(
    SRCS "main.c"
    REQUIRES boards esp_driver_spi esp_driver_gpio esp_driver_uart esp_timer
)
