idf_component_register(SRCS "main.c"
                        REQUIRES esp_driver_ledc esp_driver_gpio esp_pm nvs_flash freertos fatfs sd_card 
                        INCLUDE_DIRS "."
                        WHOLE_ARCHIVE)
