idf_component_register(SRCS "main.c"
                        REQUIRES esp_driver_ledc esp_driver_gpio esp_pm nvs_flash freertos fatfs sd_card sd_logger ds18b20 wifi_manager mqtt esp_netif esp_event
                        PRIV_REQUIRES esp_wifi
                        INCLUDE_DIRS "."
                        WHOLE_ARCHIVE)
