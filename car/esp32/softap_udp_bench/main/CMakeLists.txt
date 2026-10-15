idf_component_register(SRCS "app_main.c" INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_event esp_netif esp_timer esp_hw_support nvs_flash lwip)
