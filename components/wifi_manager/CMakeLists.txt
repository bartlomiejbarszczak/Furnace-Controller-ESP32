idf_component_register(
    SRCS "wifi_manager.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt nvs_flash esp_wifi esp_netif esp_event driver freertos
)