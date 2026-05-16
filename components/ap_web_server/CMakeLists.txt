idf_component_register(
    SRCS "ap_web_server.c"
    INCLUDE_DIRS "."
    REQUIRES esp_http_server esp_wifi nvs_flash wifi_manager
)