idf_component_register(
    SRCS "app_main.c" "mc_protocol.c" "mc_uart_diagnostic.c"
    INCLUDE_DIRS "." "/home/PRIMARY_USER/vicon_mecanum_ws/protocol"
    REQUIRES esp_wifi esp_event esp_netif esp_timer nvs_flash lwip esp_driver_uart vfs
)
