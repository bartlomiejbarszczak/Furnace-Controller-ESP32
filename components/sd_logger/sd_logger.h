/**
 * @file sd_logger.h
 * @brief Modular SD Card Logger with Card Detection and Wear Leveling
 * 
 * Features:
 * - Hot-plug card detection (CD pin monitoring)
 * - Automatic initialization and recovery
 * - Buffered writes to reduce write cycles
 * - Periodic flush mechanism using FatFs f_sync()
 * - Configuration storage
 * - Thread-safe operations
 * - Wear leveling through buffering
 * - Direct FatFs API usage (no VFS layer)
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============== CONFIGURATION ==============

#define SD_LOG_BUFFER_SIZE 2048        // Internal buffer size (bytes)
#define SD_LOG_FLUSH_INTERVAL_MS 30000 // Auto-flush every 30 seconds
#define SD_LOG_MAX_FILE_SIZE_MB 10     // Rotate log file after 10 MB
#define SD_LOG_FILE_PATH "0:/furnace_log.txt"     // FatFs path format
#define SD_CONFIG_FILE_PATH "0:/furnace_config.bin"

// ============== TYPES ==============

/**
 * @brief SD card logger status
 */
typedef enum {
    SD_STATUS_UNINITIALIZED = 0,  // Not initialized
    SD_STATUS_NO_CARD,            // No card detected
    SD_STATUS_CARD_DETECTED,      // Card detected but not mounted
    SD_STATUS_MOUNTED,            // Card mounted and ready
    SD_STATUS_ERROR               // Error state
} sd_logger_status_t;

/**
 * @brief SD logger statistics (for monitoring wear)
 */
typedef struct {
    uint32_t total_writes;           // Total write operations
    uint32_t total_bytes_written;    // Total bytes written
    uint32_t flush_count;            // Number of flush operations
    uint32_t mount_count;            // Number of successful mounts
    uint32_t error_count;            // Number of errors
    uint32_t current_file_size;      // Current log file size (bytes)
} sd_logger_stats_t;

/**
 * @brief SD logger initialization configuration
 */
typedef struct {
    gpio_num_t cd_pin;               // Card Detect pin (active low)
    bool enable_card_detect;         // Enable CD pin monitoring
    uint32_t flush_interval_ms;      // Auto-flush interval (0 = manual only)
    uint32_t max_file_size_bytes;    // Max log file size before rotation
    bool enable_log_rotation;        // Enable automatic log rotation
    bool log_to_serial;              // Also log to serial console
} sd_logger_config_t;

// ============== PUBLIC API ==============

/**
 * @brief Initialize SD card logger system
 * 
 * Creates monitoring task and initializes the SD card subsystem.
 * If card is present, it will be mounted automatically.
 * 
 * @param config Configuration structure
 * @return ESP_OK on success
 */
esp_err_t sd_logger_init(const sd_logger_config_t *config);

/**
 * @brief Deinitialize SD card logger and cleanup resources
 * 
 * Flushes all pending data, unmounts card, and deletes tasks.
 * 
 * @return ESP_OK on success
 */
esp_err_t sd_logger_deinit(void);

/**
 * @brief Get current SD logger status
 * 
 * @return Current status
 */
sd_logger_status_t sd_logger_get_status(void);

/**
 * @brief Check if SD card is ready for operations
 * 
 * @return true if card is mounted and ready
 */
bool sd_logger_is_ready(void);

/**
 * @brief Write log message to SD card (buffered)
 * 
 * Data is buffered internally and flushed periodically or when buffer is full.
 * Thread-safe.
 * 
 * @param message Log message to write
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if card not ready
 */
esp_err_t sd_logger_write(const char *message);

/**
 * @brief Write formatted log message to SD card
 * 
 * Similar to printf. Thread-safe.
 * 
 * @param format Printf-style format string
 * @param ... Variable arguments
 * @return ESP_OK on success
 */
esp_err_t sd_logger_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief Force flush all buffered data to SD card
 * 
 * Ensures all pending data is written to physical storage.
 * Thread-safe.
 * 
 * @return ESP_OK on success
 */
esp_err_t sd_logger_flush(void);

/**
 * @brief Save configuration data to SD card
 * 
 * Writes binary configuration blob to dedicated config file.
 * Thread-safe.
 * 
 * @param config_data Pointer to configuration data
 * @param size Size of configuration data in bytes
 * @return ESP_OK on success
 */
esp_err_t sd_logger_save_config(const void *config_data, size_t size);

/**
 * @brief Load configuration data from SD card
 * 
 * Reads binary configuration blob from config file.
 * Thread-safe.
 * 
 * @param config_data Buffer to store configuration
 * @param size Size of buffer (must match saved size)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if file doesn't exist
 */
esp_err_t sd_logger_load_config(void *config_data, size_t size);

/**
 * @brief Get SD logger statistics
 * 
 * Returns statistics about SD card usage for wear monitoring.
 * 
 * @param stats Pointer to statistics structure to fill
 * @return ESP_OK on success
 */
esp_err_t sd_logger_get_stats(sd_logger_stats_t *stats);

/**
 * @brief Manually trigger log file rotation
 * 
 * Closes current log file and starts a new one.
 * Old file is renamed with timestamp.
 * 
 * @return ESP_OK on success
 */
esp_err_t sd_logger_rotate_log(void);

/**
 * @brief Register custom vprintf handler for ESP-IDF logging
 * 
 * Redirects ESP_LOG* macros to also write to SD card.
 * 
 * @param enable true to enable, false to disable
 * @return ESP_OK on success
 */
esp_err_t sd_logger_set_vprintf_handler(bool enable);

// ============== CALLBACKS (Optional) ==============

/**
 * @brief Callback function type for card detection events
 * 
 * @param card_present true if card inserted, false if removed
 */
typedef void (*sd_card_event_cb_t)(bool card_present);

/**
 * @brief Register callback for card insertion/removal events
 * 
 * @param callback Callback function (NULL to unregister)
 * @return ESP_OK on success
 */
esp_err_t sd_logger_register_card_event_callback(sd_card_event_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif // SD_LOGGER_H