#ifndef AP_WEB_SERVER_H
#define AP_WEB_SERVER_H

#include "esp_err.h"

/**
 * @brief Start AP mode with web server for WiFi configuration
 *
 * Creates an open Access Point "FurnaceConfig" and starts an HTTP server.
 * Serves an HTML form at http://192.168.4.1 where users can enter WiFi credentials.
 * When credentials are submitted, they are saved to NVS and the device restarts.
 *
 * This function blocks until credentials are received and saved, then triggers a restart.
 */
void ap_web_server_start(void);

#endif // AP_WEB_SERVER_H