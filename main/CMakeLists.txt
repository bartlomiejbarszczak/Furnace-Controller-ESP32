idf_component_register(SRCS "main.c"
                        REQUIRES 
                            esp_driver_ledc 
                            esp_driver_gpio 
                            esp_pm nvs_flash 
                            freertos 
                            fatfs 
                            ota_manager 
                            sd_card 
                            sd_logger
                            ds18b20
                            wifi_manager
                            mqtt_manager 
                            furnace_config
                            esp_netif
                            esp_event
                            ap_web_server
                        PRIV_REQUIRES esp_wifi
                        INCLUDE_DIRS "."
                        WHOLE_ARCHIVE)
