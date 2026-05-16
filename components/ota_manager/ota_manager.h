#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============== CONFIGURATION ==============

/** Raspberry Pi HTTP server base URL (no trailing slash) */
#define OTA_SERVER_BASE_URL     "http://192.168.2.161:"

/** Path to version descriptor JSON on the server */
#define OTA_VERSION_JSON_URL    OTA_SERVER_BASE_URL "/version.json"

/** How often the OTA task checks for a new version (milliseconds) */
#define OTA_CHECK_INTERVAL_MS   (5 * 60 * 1000)    /* 5 minutes */

/** HTTP client timeout for both version check and firmware download (ms) */
#define OTA_HTTP_TIMEOUT_MS     10000

/** Minimum free heap required to consider a new firmware healthy (bytes) */
#define OTA_MIN_HEAP_HEALTHY    (32 * 1024)

/** Maximum size of the version.json response body (bytes) */
#define OTA_VERSION_JSON_MAX    256

// ============== PUBLIC API ==============

/**
 * @brief Perform rollback self-test on first boot after OTA update.
 *
 * Must be called as early as possible in app_main (right after nvs_flash_init).
 * If the running partition is in PENDING_VERIFY state, it runs basic health
 * checks and either confirms the firmware or triggers a rollback & reboot.
 *
 * Safe to call on every boot — has no effect when no OTA verification is pending.
 */
void ota_init(void);

/**
 * @brief FreeRTOS task that periodically checks for and applies OTA updates.
 *
 * Flow per cycle:
 *   1. Wait for WiFi connection
 *   2. Fetch version.json from the RPi server
 *   3. Parse version string and firmware URL
 *   4. Skip if version matches running firmware or is older
 *   5. Download firmware binary in chunks and flash to inactive OTA slot
 *   6. Set new partition as boot target and restart
 *
 * On any error the task logs a warning and waits for the next cycle.
 *
 * @param pvParameters  Unused (pass NULL when creating the task)
 */
void ota_task(void *pvParameters);

#ifdef __cplusplus
}
#endif