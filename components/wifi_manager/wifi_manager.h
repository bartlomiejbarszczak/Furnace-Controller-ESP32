#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include "esp_event.h"
#include <stdbool.h>

// WiFi configuration - uses separate NVS namespace "wifi_config"
#define WIFI_NVS_NAMESPACE "wifi_config"
#define WIFI_MAXIMUM_RETRY  5

// WiFi connection status
typedef enum {
    WIFI_STATUS_DISCONNECTED = 0,
    WIFI_STATUS_CONNECTING,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_FAILED
} wifi_status_t;

/**
 * @brief Initialize WiFi in station mode using credentials from NVS
 * If no credentials in NVS, uses default SSID/PASSWORD defines
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_init_sta(void);

/**
 * @brief Initialize WiFi with specific credentials (and save to NVS)
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return ESP_OK on success
 */
esp_err_t wifi_init_sta_with_credentials(const char *ssid, const char *password);

/**
 * @brief Check if WiFi is connected
 * @return true if connected, false otherwise
 */
bool wifi_is_connected(void);

/**
 * @brief Get current WiFi status
 * @return Current WiFi connection status
 */
wifi_status_t wifi_get_status(void);

/**
 * @brief Manually trigger WiFi reconnection
 * @return ESP_OK on success
 */
esp_err_t wifi_reconnect(void);

/**
 * @brief Get WiFi RSSI (signal strength)
 * @return RSSI value in dBm, 0 if not connected
 */
int8_t wifi_get_rssi(void);

/**
 * @brief Save WiFi credentials to NVS
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return ESP_OK on success
 */
esp_err_t wifi_save_credentials(const char *ssid, const char *password);

/**
 * @brief Load WiFi credentials from NVS
 * @param ssid Buffer for SSID (min 32 bytes)
 * @param password Buffer for password (min 64 bytes)
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not saved
 */
esp_err_t wifi_load_credentials(char *ssid, char *password);

/**
 * @brief Check if WiFi credentials exist in NVS
 * @return true if credentials exist
 */
bool wifi_credentials_exist(void);

/**
 * @brief Erase WiFi credentials from NVS
 * @return ESP_OK on success
 */
esp_err_t wifi_erase_credentials(void);

#endif // WIFI_MANAGER_H