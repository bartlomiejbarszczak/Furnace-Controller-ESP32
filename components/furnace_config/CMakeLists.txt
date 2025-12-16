idf_component_register(SRCS "furnace_config.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_driver_gpio esp_event
                       WHOLE_ARCHIVE)