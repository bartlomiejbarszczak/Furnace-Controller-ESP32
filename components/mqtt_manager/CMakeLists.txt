idf_component_register(SRCS "mqtt_manager.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi nvs_flash mqtt driver cjson furnace_config wifi_manager esp_event mdns
                       WHOLE_ARCHIVE)