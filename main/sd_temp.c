/* ESP32 SD Card Logger
   Simplified modular logger for SD card using SDMMC 1-bit mode
*/

#include <string.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"

static const char *TAG = "SD_LOGGER";

#define MOUNT_POINT "/sdcard"
#define LOG_FILE_PATH MOUNT_POINT"/log.txt"

// Global variables
static FILE *log_file = NULL;
static sdmmc_card_t *card = NULL;
static bool sd_initialized = false;

// Custom logging function that writes to both serial and SD card
int custom_log_vprintf(const char *fmt, va_list args) {
    // Always print to serial
    vprintf(fmt, args);
    
    // Also write to SD card file if available
    if (log_file) {
        vfprintf(log_file, fmt, args);
        fflush(log_file);
    }
    return 0;
}

/**
 * Initialize SD card and enable logging to file
 * @return true if successful, false otherwise
 */
bool init_sd(void) {
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing SD card (1-bit mode)");

    // Mount configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    // SDMMC host configuration (default 20MHz, 1-bit mode)
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // Slot configuration for 1-bit mode
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;  // 1-bit mode
    
    // Enable internal pullups (ESP-WROVER-KIT has 10k external pullups)
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // ESP-WROVER-KIT uses fixed SDMMC pins:
    // CLK = GPIO14, CMD = GPIO15, D0 = GPIO2

    // Mount the SD card
    ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
        }
        return false;
    }

    ESP_LOGI(TAG, "SD card mounted successfully");
    
    // Print card info
    ESP_LOGI(TAG, "SD card: Name: %s, Type: %s, Speed: %d MHz, Size: %llu MB",
             card->cid.name,
             (card->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC",
             card->max_freq_khz / 1000,
             ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));

    // Open log file in append mode
    log_file = fopen(LOG_FILE_PATH, "a");
    if (log_file == NULL) {
        ESP_LOGE(TAG, "Failed to open log file");
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        return false;
    }

    ESP_LOGI(TAG, "Log file opened: %s", LOG_FILE_PATH);
    
    // Set custom logging function
    esp_log_set_vprintf(custom_log_vprintf);
    
    ESP_LOGI(TAG, "SD card logger initialized - all logs now saved to SD card");
    
    sd_initialized = true;
    return true;
}

/**
 * Flush log data to SD card
 * Ensures all buffered data is written to the SD card
 */
void flush_sd_log(void) {
    if (log_file) {
        fflush(log_file);
        // Force filesystem sync
        fsync(fileno(log_file));
        ESP_LOGI(TAG, "Log data flushed to SD card");
    } else {
        ESP_LOGW(TAG, "Cannot flush - log file not open");
    }
}

/**
 * Unmount SD card and close log file
 * Call this before system shutdown or when you're done logging
 */
void unmount_sd(void) {
    if (!sd_initialized) {
        ESP_LOGW(TAG, "SD card not initialized");
        return;
    }

    ESP_LOGI(TAG, "Unmounting SD card...");

    // Restore default logging (serial only)
    esp_log_set_vprintf(vprintf);

    // Close log file
    if (log_file) {
        fflush(log_file);
        fclose(log_file);
        log_file = NULL;
        ESP_LOGI(TAG, "Log file closed");
    }

    // Unmount filesystem
    if (card) {
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
        card = NULL;
        ESP_LOGI(TAG, "SD card unmounted");
    }

    sd_initialized = false;
}

// Example usage
void app_main(void)
{
    // Initialize SD card logger
    if (!init_sd()) {
        ESP_LOGE(TAG, "SD card initialization failed - continuing without SD logging");
        // You can choose to return here or continue without SD logging
    }

    // Now you can create your tasks
    // xTaskCreate(your_task, "task", 4096, NULL, 5, NULL);
    
    // Example: Log some messages
    ESP_LOGI(TAG, "System started");
    ESP_LOGI(TAG, "This message appears on serial AND SD card");
    
    // Periodically flush logs (optional, but recommended for critical data)
    int counter = 0;
    while(1) {
        ESP_LOGI(TAG, "Loop iteration %d", counter++);
        
        // Flush every 10 iterations to ensure data is saved
        if (counter % 10 == 0) {
            flush_sd_log();
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    // If you ever need to cleanly shutdown (won't reach here due to infinite loop)
    // unmount_sd();
}