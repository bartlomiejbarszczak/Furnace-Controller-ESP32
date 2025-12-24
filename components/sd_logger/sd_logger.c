/**
 * @file sd_logger.c
 * @brief SD Card Logger Implementation using FatFs API
 */

#include "sd_logger.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "ff.h"
#include "diskio.h"
#include "diskio_sdmmc.h"

static const char *TAG = "SD_LOGGER";

// ============== INTERNAL STRUCTURES ==============

typedef struct {
    sd_logger_config_t config;
    sd_logger_status_t status;
    sd_logger_stats_t stats;
    
    // SD card objects
    sdmmc_card_t *card;
    FATFS *fs;          // FatFs filesystem object
    FIL log_file;       // FatFs file object for log
    bool log_file_open;
    
    // Buffering
    char write_buffer[SD_LOG_BUFFER_SIZE];
    uint16_t buffer_pos;
    
    // Synchronization
    SemaphoreHandle_t mutex;
    TaskHandle_t monitor_task;
    TaskHandle_t flush_task;
    
    // State
    bool initialized;
    bool card_present;
    uint32_t last_flush_time;
    
    // Callback
    sd_card_event_cb_t card_event_callback;
    
    // ESP log integration
    vprintf_like_t original_vprintf;
    bool vprintf_enabled;
    
} sd_logger_context_t;

static sd_logger_context_t ctx = {0};

// ============== FORWARD DECLARATIONS ==============
static esp_err_t sd_mount_card(void);
static esp_err_t sd_unmount_card(void);
static esp_err_t sd_open_log_file(void);
static esp_err_t sd_close_log_file(void);
static esp_err_t sd_flush_buffer_internal(void);
static void sd_card_monitor_task(void *pvParameters);
static void sd_flush_task(void *pvParameters);
static bool is_card_present(void);
static int custom_vprintf(const char *fmt, va_list args);

// ============== INTERNAL HELPERS ==============

/**
 * @brief Check if card is physically present (CD pin)
 */
static bool is_card_present(void) {
    if (!ctx.config.enable_card_detect) {
        return true; // Assume always present if CD disabled
    }
    
    // CD pin is active low (0 = card present)
    return (gpio_get_level(ctx.config.cd_pin) == 0);
}

/**
 * @brief Mount SD card using FatFs and SDMMC 1-bit mode
 */
static esp_err_t sd_mount_card(void) {
    if (ctx.status == SD_STATUS_MOUNTED) {
        return ESP_OK; // Already mounted
    }
    
    ESP_LOGI(TAG, "Mounting SD card with FatFs...");
    
    // Configure SDMMC host with proper settings
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;  // Use 1-bit mode
    host.max_freq_khz = SDMMC_FREQ_DEFAULT >> 2;
    
    // Configure slot BEFORE initialization with custom pins
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;  // 1-bit mode
    slot_config.clk = GPIO_NUM_14;
    slot_config.cmd = GPIO_NUM_15;
    slot_config.d0  = GPIO_NUM_2;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    
    // Initialize SDMMC host with proper error handling
    esp_err_t ret = sdmmc_host_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SDMMC host init failed: %s", esp_err_to_name(ret));
        ctx.stats.error_count++;
        return ret;
    }
    
    // Initialize slot with our configuration
    ret = sdmmc_host_init_slot(host.slot, &slot_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SDMMC slot init failed: %s", esp_err_to_name(ret));
        sdmmc_host_deinit();  // Clean up host
        ctx.stats.error_count++;
        return ret;
    }
    
    // Allocate card structure
    if (ctx.card == NULL) {
        ctx.card = (sdmmc_card_t*)malloc(sizeof(sdmmc_card_t));
        if (ctx.card == NULL) {
            ESP_LOGE(TAG, "Failed to allocate card structure");
            sdmmc_host_deinit();
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Clear card structure
    memset(ctx.card, 0, sizeof(sdmmc_card_t));
    
    // Probe and initialize card
    ret = sdmmc_card_init(&host, ctx.card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Card init failed: %s", esp_err_to_name(ret));
        free(ctx.card);
        ctx.card = NULL;
        sdmmc_host_deinit();
        ctx.stats.error_count++;
        return ret;
    }
    
    // Log card info
    sdmmc_card_print_info(stdout, ctx.card);
    
    // Register diskio driver
    ff_diskio_register_sdmmc(0, ctx.card);
    
    // Allocate FatFs object
    if (ctx.fs == NULL) {
        ctx.fs = (FATFS*)malloc(sizeof(FATFS));
        if (ctx.fs == NULL) {
            ESP_LOGE(TAG, "Failed to allocate FATFS structure");
            free(ctx.card);
            ctx.card = NULL;
            sdmmc_host_deinit();
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Mount filesystem
    FRESULT fres = f_mount(ctx.fs, "0:", 1);  // "0:" = drive number, 1 = mount now
    if (fres != FR_OK) {
        ESP_LOGE(TAG, "FatFs mount failed: %d", fres);
        free(ctx.fs);
        ctx.fs = NULL;
        free(ctx.card);
        ctx.card = NULL;
        ff_diskio_register_sdmmc(0, NULL);  // Unregister
        sdmmc_host_deinit();
        ctx.stats.error_count++;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "FatFs mounted successfully");
    
    ctx.status = SD_STATUS_MOUNTED;
    ctx.stats.mount_count++;
    
    return ESP_OK;
}

/**
 * @brief Unmount SD card and cleanup
 */
static esp_err_t sd_unmount_card(void) {
    if (ctx.status != SD_STATUS_MOUNTED) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Unmounting SD card...");
    
    // Close log file first
    sd_close_log_file();
    
    // Unmount FatFs
    if (ctx.fs) {
        f_mount(NULL, "0:", 0);  // Unmount
        free(ctx.fs);
        ctx.fs = NULL;
    }
    
    // Unregister diskio
    ff_diskio_register_sdmmc(0, NULL);
    
    // Free card structure
    if (ctx.card) {
        free(ctx.card);
        ctx.card = NULL;
    }
    
    // Deinitialize SDMMC host
    sdmmc_host_deinit();
    
    ctx.status = SD_STATUS_NO_CARD;
    ESP_LOGI(TAG, "SD card unmounted");
    
    return ESP_OK;
}

/**
 * @brief Open log file in append mode using FatFs
 */
static esp_err_t sd_open_log_file(void) {
    if (ctx.log_file_open) {
        return ESP_OK; // Already open
    }
    
    if (ctx.status != SD_STATUS_MOUNTED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Open file in append mode (FA_OPEN_APPEND | FA_WRITE | FA_READ)
    // If file doesn't exist, create it (FA_OPEN_ALWAYS)
    FRESULT fres = f_open(&ctx.log_file, SD_LOG_FILE_PATH, FA_WRITE | FA_OPEN_APPEND);
    
    if (fres != FR_OK) {
        ESP_LOGE(TAG, "Failed to open log file: %d", fres);
        ctx.stats.error_count++;
        return ESP_FAIL;
    }
    
    // // Seek to end of file for append
    // fres = f_lseek(&ctx.log_file, f_size(&ctx.log_file));
    // if (fres != FR_OK) {
    //     ESP_LOGE(TAG, "Failed to seek to end: %d", fres);
    //     f_close(&ctx.log_file);
    //     ctx.stats.error_count++;
    //     return ESP_FAIL;
    // }
    
    ctx.log_file_open = true;
    ctx.stats.current_file_size = f_size(&ctx.log_file);
    
    ESP_LOGI(TAG, "Log file opened: %s (size: %u bytes)", 
             SD_LOG_FILE_PATH, ctx.stats.current_file_size);
    
    return ESP_OK;
}

/**
 * @brief Close log file
 */
static esp_err_t sd_close_log_file(void) {
    if (ctx.log_file_open) {
        // Flush buffer first
        sd_flush_buffer_internal();
        
        // Close file
        f_close(&ctx.log_file);
        ctx.log_file_open = false;
        ESP_LOGI(TAG, "Log file closed");
    }
    return ESP_OK;
}

/**
 * @brief Internal buffer flush (must be called with mutex held)
 * Uses FatFs f_sync() for proper synchronization
 */
static esp_err_t sd_flush_buffer_internal(void) {
    if (ctx.buffer_pos == 0) {
        return ESP_OK; // Nothing to flush
    }
    
    if (!ctx.log_file_open) {
        ESP_LOGW(TAG, "Cannot flush - log file not open");
        return ESP_ERR_INVALID_STATE;
    }
    
    // xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    
    // Write buffer to file
    UINT bytes_written;
    FRESULT fres = f_write(&ctx.log_file, ctx.write_buffer, ctx.buffer_pos, &bytes_written);
    
    // xSemaphoreGive(ctx.mutex);
    
    if (fres != FR_OK || bytes_written != ctx.buffer_pos) {
        ESP_LOGE(TAG, "Write error: expected %d, wrote %d, result=%d. Error code: %d", 
                 ctx.buffer_pos, bytes_written, fres, fres);
        ctx.stats.error_count++;
        return ESP_FAIL;
    }
    
    // Sync to ensure data is physically written
    fres = f_sync(&ctx.log_file);
    if (fres != FR_OK) {
        ESP_LOGE(TAG, "f_sync failed: %d", fres);
        ctx.stats.error_count++;
        return ESP_FAIL;
    }
    
    // Update statistics
    ctx.stats.total_bytes_written += ctx.buffer_pos;
    ctx.stats.current_file_size += ctx.buffer_pos;
    ctx.stats.flush_count++;
    ctx.buffer_pos = 0;
    ctx.last_flush_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if rotation needed
    if (ctx.config.enable_log_rotation && 
        ctx.stats.current_file_size >= ctx.config.max_file_size_bytes) {
        sd_logger_rotate_log();
    }
    
    return ESP_OK;
}

/**
 * @brief Custom vprintf for ESP-IDF logging integration
 */
static int custom_vprintf(const char *fmt, va_list args) {
    // Always print to serial first
    int ret = 0;
    if (ctx.original_vprintf) {
        ret = ctx.original_vprintf(fmt, args);
    }
    
    // Also write to SD card if available
    if (ctx.status == SD_STATUS_MOUNTED && ctx.log_file_open) {
        // Don't block - use tryTake instead of Take
        if (xSemaphoreTake(ctx.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            va_list args_copy;
            va_copy(args_copy, args);
            
            int remaining = SD_LOG_BUFFER_SIZE - ctx.buffer_pos;
            int written = vsnprintf(&ctx.write_buffer[ctx.buffer_pos], remaining, fmt, args_copy);
            
            if (written > 0 && written < remaining) {
                ctx.buffer_pos += written;
                ctx.stats.total_writes++;
                
                // Flush if buffer is getting full (>75%)
                if (ctx.buffer_pos > (SD_LOG_BUFFER_SIZE * 3 / 4)) {
                    sd_flush_buffer_internal();
                }
            }
            
            xSemaphoreGive(ctx.mutex);
            va_end(args_copy);
        }
        // If mutex not available, skip SD write (don't block)
    }
    
    return ret;
}

/**
 * @brief Card detection monitor task
 * 
 * Polls CD pin and handles card insertion/removal events
 */
static void sd_card_monitor_task(void *pvParameters) {
    bool last_card_state = is_card_present();
    
    ESP_LOGI(TAG, "Card monitor task started (initial state: %s)", 
             last_card_state ? "PRESENT" : "ABSENT");
    
    while (1) {
        bool card_now = is_card_present();
        
        // Debounce: check twice with delay
        vTaskDelay(pdMS_TO_TICKS(50));
        if (card_now != is_card_present()) {
            continue; // State changed during debounce, skip
        }
        
        // Detect state change
        if (card_now != last_card_state) {
            ESP_LOGI(TAG, "Card %s", card_now ? "INSERTED" : "REMOVED");
            
            xSemaphoreTake(ctx.mutex, portMAX_DELAY);
            
            if (card_now) {
                // Wait for card to stabilize after insertion
                vTaskDelay(pdMS_TO_TICKS(200));
                
                // Card inserted - try to mount
                ctx.status = SD_STATUS_CARD_DETECTED;
                
                if (sd_mount_card() == ESP_OK) {
                    if (sd_open_log_file() == ESP_OK) {
                        ESP_LOGI(TAG, "SD card ready for logging");
                    }
                }
            } else {
                // Card removed - unmount
                sd_unmount_card();
            }
            
            ctx.card_present = card_now;
            
            xSemaphoreGive(ctx.mutex);
            
            // Call user callback if registered
            if (ctx.card_event_callback) {
                ctx.card_event_callback(card_now);
            }
            
            last_card_state = card_now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Check every 500ms
    }
}

/**
 * @brief Periodic flush task
 * 
 * Flushes buffered data at regular intervals using f_sync()
 */
static void sd_flush_task(void *pvParameters) {
    ESP_LOGI(TAG, "Flush task started (interval: %u ms)", ctx.config.flush_interval_ms);
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(ctx.config.flush_interval_ms));
        
        if (ctx.status == SD_STATUS_MOUNTED && ctx.buffer_pos > 0) {
            xSemaphoreTake(ctx.mutex, portMAX_DELAY);
            sd_flush_buffer_internal();
            xSemaphoreGive(ctx.mutex);
        }
    }
}

// ============== PUBLIC API IMPLEMENTATION ==============

esp_err_t sd_logger_init(const sd_logger_config_t *config) {
    if (ctx.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "=== Initializing SD Logger (FatFs) ===");
    
    // Copy configuration
    memcpy(&ctx.config, config, sizeof(sd_logger_config_t));
    
    // Initialize mutex
    ctx.mutex = xSemaphoreCreateMutex();
    if (ctx.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Configure CD pin if enabled
    if (ctx.config.enable_card_detect) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << ctx.config.cd_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        
        ESP_LOGI(TAG, "Card detect enabled on GPIO%d", ctx.config.cd_pin);
    }
    
    // Initialize state
    ctx.status = SD_STATUS_UNINITIALIZED;
    ctx.buffer_pos = 0;
    ctx.log_file_open = false;
    ctx.card_present = is_card_present();
    memset(&ctx.stats, 0, sizeof(sd_logger_stats_t));
    
    // Try initial mount if card is present
    if (ctx.card_present) {
        ctx.status = SD_STATUS_CARD_DETECTED;
        if (sd_mount_card() == ESP_OK) {
            sd_open_log_file();
        }
    } else {
        ctx.status = SD_STATUS_NO_CARD;
        ESP_LOGI(TAG, "No SD card detected at startup");
    }
    
    // Create monitoring task
    BaseType_t result = xTaskCreate(
        sd_card_monitor_task,
        "sd_monitor",
        3072,
        NULL,
        5,
        &ctx.monitor_task
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        vSemaphoreDelete(ctx.mutex);
        return ESP_FAIL;
    }
    
    // Create flush task if auto-flush enabled
    if (ctx.config.flush_interval_ms > 0) {
        result = xTaskCreate(
            sd_flush_task,
            "sd_flush",
            4096,
            NULL,
            4,
            &ctx.flush_task
        );
        
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to create flush task");
            vTaskDelete(ctx.monitor_task);
            vSemaphoreDelete(ctx.mutex);
            return ESP_FAIL;
        }
    }
    
    ctx.initialized = true;
    ESP_LOGI(TAG, "SD Logger initialized successfully");
    
    return ESP_OK;
}

esp_err_t sd_logger_deinit(void) {
    if (!ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Deinitializing SD Logger...");
    
    // Restore original vprintf if changed
    if (ctx.vprintf_enabled && ctx.original_vprintf) {
        esp_log_set_vprintf(ctx.original_vprintf);
    }
    
    // Delete tasks
    if (ctx.monitor_task) {
        vTaskDelete(ctx.monitor_task);
        ctx.monitor_task = NULL;
    }
    
    if (ctx.flush_task) {
        vTaskDelete(ctx.flush_task);
        ctx.flush_task = NULL;
    }
    
    // Flush and unmount
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    sd_flush_buffer_internal();
    sd_unmount_card();
    xSemaphoreGive(ctx.mutex);
    
    // Free allocated memory
    if (ctx.card) {
        free(ctx.card);
        ctx.card = NULL;
    }
    
    // Cleanup
    vSemaphoreDelete(ctx.mutex);
    ctx.mutex = NULL;
    ctx.initialized = false;
    
    ESP_LOGI(TAG, "SD Logger deinitialized");
    
    return ESP_OK;
}

sd_logger_status_t sd_logger_get_status(void) {
    return ctx.status;
}

bool sd_logger_is_ready(void) {
    return (ctx.status == SD_STATUS_MOUNTED && ctx.log_file_open);
}

esp_err_t sd_logger_write(const char *message) {
    if (message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!sd_logger_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    
    size_t len = strlen(message);
    size_t remaining = SD_LOG_BUFFER_SIZE - ctx.buffer_pos;
    
    if (len >= remaining) {
        // Message doesn't fit - flush first
        sd_flush_buffer_internal();
        remaining = SD_LOG_BUFFER_SIZE;
    }
    
    // Copy to buffer
    size_t to_copy = (len < remaining) ? len : remaining - 1;
    memcpy(&ctx.write_buffer[ctx.buffer_pos], message, to_copy);
    ctx.buffer_pos += to_copy;
    ctx.stats.total_writes++;
    
    xSemaphoreGive(ctx.mutex);
    
    return ESP_OK;
}

esp_err_t sd_logger_printf(const char *format, ...) {
    if (!sd_logger_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    va_list args;
    va_start(args, format);
    
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    
    int remaining = SD_LOG_BUFFER_SIZE - ctx.buffer_pos;
    int written = vsnprintf(&ctx.write_buffer[ctx.buffer_pos], remaining, format, args);
    
    if (written > 0 && written < remaining) {
        ctx.buffer_pos += written;
        ctx.stats.total_writes++;
    } else if (written >= remaining) {
        // Buffer full - flush and retry
        sd_flush_buffer_internal();
        remaining = SD_LOG_BUFFER_SIZE;
        written = vsnprintf(&ctx.write_buffer[ctx.buffer_pos], remaining, format, args);
        if (written > 0 && written < remaining) {
            ctx.buffer_pos += written;
            ctx.stats.total_writes++;
        }
    }
    
    xSemaphoreGive(ctx.mutex);
    va_end(args);
    
    return ESP_OK;
}

esp_err_t sd_logger_flush(void) {
    if (!ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    esp_err_t ret = sd_flush_buffer_internal();
    xSemaphoreGive(ctx.mutex);
    
    return ret;
}

esp_err_t sd_logger_save_config(const void *config_data, size_t size) {
    if (config_data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!sd_logger_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    
    FIL config_file;
    FRESULT fres = f_open(&config_file, SD_CONFIG_FILE_PATH, 
                          FA_CREATE_ALWAYS | FA_WRITE);
    
    if (fres != FR_OK) {
        ESP_LOGE(TAG, "Failed to open config file for writing: %d", fres);
        xSemaphoreGive(ctx.mutex);
        return ESP_FAIL;
    }
    
    UINT bytes_written;
    fres = f_write(&config_file, config_data, size, &bytes_written);
    
    if (fres == FR_OK) {
        // Sync to ensure data is written
        f_sync(&config_file);
    }
    
    f_close(&config_file);
    
    xSemaphoreGive(ctx.mutex);
    
    if (fres != FR_OK || bytes_written != size) {
        ESP_LOGE(TAG, "Config write incomplete: %d/%d bytes, result=%d", 
                 bytes_written, size, fres);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Configuration saved (%d bytes)", size);
    return ESP_OK;
}

esp_err_t sd_logger_load_config(void *config_data, size_t size) {
    if (config_data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ctx.status != SD_STATUS_MOUNTED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    
    FIL config_file;
    FRESULT fres = f_open(&config_file, SD_CONFIG_FILE_PATH, FA_READ);
    
    if (fres != FR_OK) {
        xSemaphoreGive(ctx.mutex);
        return (fres == FR_NO_FILE) ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    }
    
    UINT bytes_read;
    fres = f_read(&config_file, config_data, size, &bytes_read);
    
    f_close(&config_file);
    
    xSemaphoreGive(ctx.mutex);
    
    if (fres != FR_OK || bytes_read != size) {
        ESP_LOGE(TAG, "Config read size mismatch: %d/%d bytes, result=%d", 
                 bytes_read, size, fres);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Configuration loaded (%d bytes)", size);
    return ESP_OK;
}

esp_err_t sd_logger_save_wifi_config(const char *ssid, size_t ssid_len, const char *password, size_t password_len) 
{
    if (ssid == NULL || ssid_len == 0 || password == NULL || password_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!sd_logger_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    
    FIL wifi_file;
    FRESULT fres = f_open(&wifi_file, SD_WIFI_CONFIG_FILE_PATH, FA_WRITE | FA_CREATE_ALWAYS);
    
    if (fres != FR_OK) {
        ESP_LOGE(TAG, "Failed to open WiFi config file for writing: %d", fres);
        xSemaphoreGive(ctx.mutex);
        return ESP_FAIL;
    }
    
    char ssid_buffer[64] = {0};
    char password_buffer[128] = {0};

    snprintf(ssid_buffer, sizeof(ssid_buffer), "ssid=%s\n", ssid);
    snprintf(password_buffer, sizeof(password_buffer), "password=%s\n", password);

    UINT bytes_written;
    fres = f_write(&wifi_file, ssid_buffer, strlen(ssid_buffer), &bytes_written);
    if (fres != FR_OK || bytes_written != strlen(ssid_buffer)) {
        ESP_LOGE(TAG, "WiFi SSID write failed: %d/%d bytes, result=%d",
                 bytes_written, strlen(ssid_buffer), fres);
        f_close(&wifi_file);
        xSemaphoreGive(ctx.mutex);
        return ESP_FAIL;
    }

    fres = f_write(&wifi_file, password_buffer, strlen(password_buffer), &bytes_written);
    if (fres != FR_OK || bytes_written != strlen(password_buffer)) {
        ESP_LOGE(TAG, "WiFi password write failed: %d/%d bytes, result=%d", 
                 bytes_written, strlen(password_buffer), fres);
        f_close(&wifi_file);
        xSemaphoreGive(ctx.mutex);
        return ESP_FAIL;
    }
    
    // Sync to ensure data is written
    f_sync(&wifi_file);
    
    f_close(&wifi_file);
    
    xSemaphoreGive(ctx.mutex);
    
    ESP_LOGI(TAG, "WiFi configuration saved (SSID: %d bytes, Password: %d bytes)", 
             ssid_len, password_len);
    return ESP_OK;
}

esp_err_t sd_logger_load_wifi_config(char *ssid, size_t ssid_size, char *password, size_t password_size) 
{
    if (ssid == NULL || ssid_size == 0 || password == NULL || password_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (ctx.status != SD_STATUS_MOUNTED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    
    FIL wifi_file;
    FRESULT fres = f_open(&wifi_file, SD_WIFI_CONFIG_FILE_PATH, FA_READ);
    
    if (fres != FR_OK) {
        xSemaphoreGive(ctx.mutex);
        return (fres == FR_NO_FILE) ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    }
    
    char line[128];
    while (f_gets(line, sizeof(line), &wifi_file)) {
        if (strncmp(line, "ssid=", 5) == 0) {
            strncpy(ssid, line + 5, ssid_size - 1);
            ssid[ssid_size - 1] = '\0';
            // Remove newline
            char *newline = strchr(ssid, '\n');
            if (newline) *newline = '\0';
        } else if (strncmp(line, "password=", 9) == 0) {
            strncpy(password, line + 9, password_size - 1);
            password[password_size - 1] = '\0';
            // Remove newline
            char *newline = strchr(password, '\n');
            if (newline) *newline = '\0';
        }
    }
    
    f_close(&wifi_file);
    
    xSemaphoreGive(ctx.mutex);
    
    ESP_LOGI(TAG, "WiFi configuration loaded (SSID: %s)", ssid);
    return ESP_OK;
}

esp_err_t sd_logger_get_stats(sd_logger_stats_t *stats) {
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    memcpy(stats, &ctx.stats, sizeof(sd_logger_stats_t));
    xSemaphoreGive(ctx.mutex);
    
    return ESP_OK;
}

esp_err_t sd_logger_rotate_log(void) {
    if (!sd_logger_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Rotating log file...");
    
    xSemaphoreTake(ctx.mutex, portMAX_DELAY);
    
    // Flush and close current file
    sd_flush_buffer_internal();
    sd_close_log_file();
    
    // Generate new filename with timestamp
    time_t now = time(NULL);
    char new_name[32];
    snprintf(new_name, sizeof(new_name), "0:/furnace_log_%lld.txt", now);
    
    // Rename old file
    FRESULT fres = f_rename(SD_LOG_FILE_PATH, new_name);
    if (fres == FR_OK) {
        ESP_LOGI(TAG, "Old log renamed to: %s", new_name);
    } else {
        ESP_LOGW(TAG, "Failed to rename log file: %d", fres);
    }
    
    // Open new log file
    ctx.stats.current_file_size = 0;
    esp_err_t ret = sd_open_log_file();
    
    xSemaphoreGive(ctx.mutex);
    
    return ret;
}

esp_err_t sd_logger_set_vprintf_handler(bool enable) {
    if (!ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (enable && !ctx.vprintf_enabled) {
        ctx.original_vprintf = esp_log_set_vprintf(custom_vprintf);
        ctx.vprintf_enabled = true;
        ESP_LOGI(TAG, "ESP-IDF log integration enabled");
    } else if (!enable && ctx.vprintf_enabled) {
        esp_log_set_vprintf(ctx.original_vprintf);
        ctx.vprintf_enabled = false;
        ESP_LOGI(TAG, "ESP-IDF log integration disabled");
    }
    
    return ESP_OK;
}

esp_err_t sd_logger_register_card_event_callback(sd_card_event_cb_t callback) {
    ctx.card_event_callback = callback;
    return ESP_OK;
}