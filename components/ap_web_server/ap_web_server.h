#ifndef AP_WEB_SERVER_H
#define AP_WEB_SERVER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize AP mode and start the configuration web server
 *
 * Creates an open Access Point "FurnaceConfig" and starts an HTTP server
 * as a background service. Returns immediately — does NOT block.
 *
 * The HTTP server serves a config page at http://192.168.4.1 where users
 * can enter WiFi credentials. On submission the credentials are saved to
 * NVS and the device restarts automatically.
 *
 * Safe to call from app_main before FreeRTOS tasks are created; the HTTP
 * server runs in its own IDF-managed task.
 *
 * @param current_ssid  Last attempted SSID shown in the form (NULL or "" is fine)
 * @param current_password  Unused, kept for API compatibility (password is never pre-filled)
 * @return ESP_OK on success, or an esp_err_t code on failure
 */
esp_err_t ap_web_server_init(const char *current_ssid, const char *current_password);

/**
 * @brief Check whether the AP config server is currently running
 * @return true if AP mode is active
 */
bool ap_web_server_is_running(void);

#endif // AP_WEB_SERVER_H