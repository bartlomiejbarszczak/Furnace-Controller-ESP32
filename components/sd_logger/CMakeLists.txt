# Component CMakeLists.txt for SD Logger

idf_component_register(
    SRCS "sd_logger.c"
    INCLUDE_DIRS "."
    REQUIRES driver fatfs sdmmc esp_timer nvs_flash
)