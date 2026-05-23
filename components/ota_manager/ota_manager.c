#include "ota_manager.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_app_desc.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "wifi_manager.h"   /* wifi_is_connected() */

static const char *TAG = "OTA";

// ============== INTERNAL TYPES ==============

/**
 * Parsed content of version.json:
 * {
 *   "version": "1.2.0",
 *   "url":     "http://192.168.1.100:8080/firmware_1.2.0.bin"
 * }
 */
typedef struct {
    char version[32];   /**< Semantic version string, e.g. "1.2.0" */
    char url[128];      /**< Full HTTP URL to the firmware binary    */
} ota_version_info_t;

/**
 * Parsed semantic version — major.minor.patch
 */
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} semver_t;

// ============== INTERNAL HELPERS ==============

/**
 * @brief Parse a "major.minor.patch" string into a semver_t struct.
 *
 * @param str   Version string, e.g. "1.2.0"
 * @param out   Output struct
 * @return true on success, false if the string is malformed
 */
static bool parse_semver(const char *str, semver_t *out)
{
    if (!str || !out) return false;

    int major = 0, minor = 0, patch = 0;
    int matched = sscanf(str, "%d.%d.%d", &major, &minor, &patch);

    if (matched < 2) {
        /* Accept "1.2" as "1.2.0" */
        return false;
    }

    out->major = (uint8_t)major;
    out->minor = (uint8_t)minor;
    out->patch = (uint8_t)patch;
    return true;
}

/**
 * @brief Compare two semver_t values.
 *
 * @return  1 if a > b
 *          0 if a == b
 *         -1 if a < b
 */
static int semver_compare(const semver_t *a, const semver_t *b)
{
    if (a->major != b->major) return (a->major > b->major) ? 1 : -1;
    if (a->minor != b->minor) return (a->minor > b->minor) ? 1 : -1;
    if (a->patch != b->patch) return (a->patch > b->patch) ? 1 : -1;
    return 0;
}

// ============== NVS VERSION CACHE ==============

/** Cached version string loaded once at boot — empty string means not loaded yet */
static char s_nvs_version[32] = { 0 };
static bool s_nvs_version_loaded = false;

/**
 * @brief Load the last OTA-flashed version from NVS (once) and return it.
 *
 * On the very first boot (no NVS entry yet) falls back to
 * esp_app_get_description()->version so the first comparison works correctly.
 *
 * @return Null-terminated version string — never NULL.
 */
static const char *ota_nvs_load_version(void)
{
    if (s_nvs_version_loaded) {
        return s_nvs_version;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(OTA_NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err == ESP_OK) {
        size_t required = sizeof(s_nvs_version);
        err = nvs_get_str(handle, OTA_NVS_KEY_VERSION, s_nvs_version, &required);
        nvs_close(handle);
    }

    if (err != ESP_OK || s_nvs_version[0] == '\0') {
        /* NVS empty — first ever boot; use app descriptor as baseline */
        const esp_app_desc_t *desc = esp_app_get_description();
        strncpy(s_nvs_version, desc->version, sizeof(s_nvs_version) - 1);
        s_nvs_version[sizeof(s_nvs_version) - 1] = '\0';
        ESP_LOGW(TAG, "NVS version not found — using app_desc fallback: %s", s_nvs_version);
    } else {
        ESP_LOGI(TAG, "NVS version loaded: %s", s_nvs_version);
    }

    s_nvs_version_loaded = true;
    return s_nvs_version;
}

/**
 * @brief Persist a version string to NVS after a successful firmware flash.
 *
 * @param version   Null-terminated semver string, e.g. "1.2.0"
 * @return ESP_OK on success
 */
static esp_err_t ota_nvs_save_version(const char *version)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(OTA_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(handle, OTA_NVS_KEY_VERSION, version);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "NVS version saved: %s", version);
    } else {
        ESP_LOGE(TAG, "NVS version save failed: %s", esp_err_to_name(err));
    }

    return err;
}

/**
 * @brief Minimal JSON field extractor — finds the value of a quoted string key.
 *
 * Looks for `"key": "value"` or `"key":"value"` patterns.
 * Does NOT handle nested objects, arrays, or escaped quotes inside values.
 * Sufficient for the flat version.json structure used here.
 *
 * @param json      Null-terminated JSON string
 * @param key       Field name to search for
 * @param out_buf   Buffer to write the extracted value into
 * @param out_size  Size of out_buf
 * @return true if the field was found and fits in out_buf, false otherwise
 */
static bool json_extract_string(const char *json, const char *key,
                                char *out_buf, size_t out_size)
{
    if (!json || !key || !out_buf || out_size == 0) return false;

    /* Build search pattern: "key" */
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *pos = strstr(json, pattern);
    if (!pos) return false;

    /* Advance past "key" */
    pos += strlen(pattern);

    /* Skip whitespace and colon */
    while (*pos == ' ' || *pos == '\t' || *pos == ':') pos++;

    /* Expect opening quote */
    if (*pos != '"') return false;
    pos++; /* skip '"' */

    /* Copy until closing quote or buffer full */
    size_t i = 0;
    while (*pos && *pos != '"' && i < out_size - 1) {
        out_buf[i++] = *pos++;
    }
    out_buf[i] = '\0';

    return (*pos == '"'); /* false if we hit end-of-string before closing quote */
}

// ============== HTTP HELPERS ==============

/**
 * @brief Fetch a small text resource from an HTTP URL into a caller-supplied buffer.
 *
 * @param url       Full HTTP URL
 * @param buf       Output buffer
 * @param buf_size  Size of output buffer (result is always null-terminated)
 * @return ESP_OK on success, or an esp_err_t error code
 */
static esp_err_t http_get_text(const char *url, char *buf, size_t buf_size)
{
    esp_http_client_config_t cfg = {
        .url         = url,
        .timeout_ms  = OTA_HTTP_TIMEOUT_MS,
        .method      = HTTP_METHOD_GET,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int content_len = esp_http_client_fetch_headers(client);
    (void)content_len; /* we rely on buffer overflow protection instead */

    int http_status = esp_http_client_get_status_code(client);
    if (http_status != 200) {
        ESP_LOGE(TAG, "HTTP GET %s returned status %d", url, http_status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    int total_read = 0;
    int remaining  = (int)buf_size - 1; /* reserve space for null terminator */

    while (remaining > 0) {
        int read = esp_http_client_read(client, buf + total_read, remaining);
        if (read < 0) {
            ESP_LOGE(TAG, "HTTP read error");
            err = ESP_FAIL;
            break;
        }
        if (read == 0) break; /* EOF */
        total_read += read;
        remaining  -= read;
    }

    buf[total_read] = '\0';

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return err;
}

// ============== CORE LOGIC ==============

/**
 * @brief Fetch version.json from the RPi and parse it into ota_version_info_t.
 *
 * @param info  Output struct populated on success
 * @return ESP_OK on success
 */
static esp_err_t fetch_version_info(ota_version_info_t *info)
{
    char json_buf[OTA_VERSION_JSON_MAX];

    esp_err_t err = http_get_text(OTA_VERSION_JSON_URL, json_buf, sizeof(json_buf));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not fetch %s", OTA_VERSION_JSON_URL);
        return err;
    }

    ESP_LOGD(TAG, "version.json: %s", json_buf);

    if (!json_extract_string(json_buf, "version", info->version, sizeof(info->version))) {
        ESP_LOGE(TAG, "Failed to parse 'version' field from version.json");
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (!json_extract_string(json_buf, "url", info->url, sizeof(info->url))) {
        ESP_LOGE(TAG, "Failed to parse 'url' field from version.json");
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "Server: version=%s  url=%s", info->version, info->url);
    return ESP_OK;
}

/**
 * @brief Download firmware from url and flash it into the inactive OTA partition.
 *
 * Uses esp_https_ota() which handles begin/write/end internally and verifies
 * the image magic byte before marking the partition as bootable.
 *
 * @param url   Full HTTP URL to the firmware binary
 * @return ESP_OK on success
 */
static esp_err_t download_and_flash(const char *url)
{
    ESP_LOGI(TAG, "Starting firmware download: %s", url);

    esp_http_client_config_t http_cfg = {
        .url               = url,
        .timeout_ms        = OTA_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
        .buffer_size       = 4096,
    };

    esp_https_ota_config_t ota_cfg = {
        .http_config      = &http_cfg,
        .bulk_flash_erase = false,  /* sequential erase — safer for watchdog */
    };

    esp_err_t err = esp_https_ota(&ota_cfg);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Firmware flashed successfully");
    } else {
        ESP_LOGE(TAG, "Firmware flash failed: %s", esp_err_to_name(err));
    }

    return err;
}

/**
 * @brief Single OTA check cycle — called every OTA_CHECK_INTERVAL_MS.
 *
 * @return ESP_OK          — firmware is up to date (no action taken)
 *         ESP_ERR_*       — an error occurred during check or update
 *         (never returns ESP_OK after a successful update — device reboots)
 */
static esp_err_t ota_check_and_update(void)
{
    /* ── 1. Fetch version.json ─────────────────────────────────────────── */
    ota_version_info_t server = { 0 };
    esp_err_t err = fetch_version_info(&server);
    if (err != ESP_OK) return err;

    /* ── 2. Get running firmware version from NVS ──────────────────────── */
    const char *running_version = ota_nvs_load_version();
    ESP_LOGI(TAG, "Running firmware: version=%s (from NVS)", running_version);

    /* ── 3. Parse and compare versions ────────────────────────────────── */
    semver_t sv_server  = { 0 };
    semver_t sv_running = { 0 };

    if (!parse_semver(server.version, &sv_server)) {
        ESP_LOGE(TAG, "Could not parse server version string: '%s'", server.version);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (!parse_semver(running_version, &sv_running)) {
        ESP_LOGW(TAG, "Could not parse running version string: '%s' — treating as 0.0.0", running_version);
        /* sv_running stays {0,0,0} — any valid server version will trigger update */
    }

    int cmp = semver_compare(&sv_server, &sv_running);

    if (cmp == 0) {
        ESP_LOGI(TAG, "Firmware is up to date (%s)", running_version);
        return ESP_OK;
    }

    if (cmp < 0) {
        ESP_LOGW(TAG, "Server version %s is OLDER than running %s — skipping",
                 server.version, running_version);
        return ESP_OK;
    }

    /* ── 4. Newer version found — update ──────────────────────────────── */
    ESP_LOGI(TAG, "New firmware available: %s → %s", running_version, server.version);

    err = download_and_flash(server.url);
    if (err != ESP_OK) return err;

    /* ── 5. Persist new version to NVS before reboot ─────────────────── */
    ota_nvs_save_version(server.version);

    /* ── 6. Reboot into the new firmware ──────────────────────────────── */
    ESP_LOGI(TAG, "Update complete — rebooting in 2 seconds...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

    return ESP_OK; /* unreachable */
}

// ============== ROLLBACK SELF-TEST ==============

/**
 * @brief Basic health checks run after first boot of a newly flashed firmware.
 *
 * Extend this with application-specific checks as needed (e.g. sensor init,
 * NVS read, etc.).
 *
 * @return true if all checks pass, false if any check fails
 */
static bool run_self_test(void)
{
    /* Check 1: Heap is healthy */
    size_t free_heap = esp_get_free_heap_size();
    if (free_heap < OTA_MIN_HEAP_HEALTHY) {
        ESP_LOGE(TAG, "Self-test FAILED: free heap %u < %u bytes",
                 (unsigned)free_heap, (unsigned)OTA_MIN_HEAP_HEALTHY);
        return false;
    }
    ESP_LOGI(TAG, "Self-test: heap OK (%u bytes free)", (unsigned)free_heap);

    /* Check 2: Heap integrity */
    if (!heap_caps_check_integrity_all(true)) {
        ESP_LOGE(TAG, "Self-test FAILED: heap integrity check failed");
        return false;
    }
    ESP_LOGI(TAG, "Self-test: heap integrity OK");

    /*
     * Add your own checks here, for example:
     *   - Verify DS18B20 sensors responded during sensor_task startup
     *   - Check NVS namespace opens cleanly
     *   - Verify a critical GPIO state
     */

    return true;
}

// ============== PUBLIC FUNCTIONS ==============

void ota_init(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t   ota_state;

    esp_err_t err = esp_ota_get_state_partition(running, &ota_state);
    if (err != ESP_OK) {
        /*
         * ESP_ERR_NOT_SUPPORTED is returned when the running partition is not
         * an OTA slot (e.g. factory partition on first ever flash).  This is
         * normal — nothing to do.
         */
        ESP_LOGI(TAG, "ota_init: no OTA state to check (err=%s)", esp_err_to_name(err));
        return;
    }

    if (ota_state != ESP_OTA_IMG_PENDING_VERIFY) {
        /* Normal boot — firmware already confirmed or untouched */
        ESP_LOGI(TAG, "ota_init: partition state=%d, no verification needed", ota_state);
        return;
    }

    /* ── First boot after OTA update ─────────────────────────────────── */
    ESP_LOGI(TAG, "ota_init: new firmware PENDING_VERIFY — running self-test...");

    if (run_self_test()) {
        ESP_LOGI(TAG, "Self-test passed — confirming firmware");
        esp_ota_mark_app_valid_cancel_rollback();
    } else {
        ESP_LOGE(TAG, "Self-test FAILED — rolling back to previous firmware");
        /*
         * This function marks the current app INVALID and reboots.
         * The bootloader will select the previous working OTA slot.
         */
        esp_ota_mark_app_invalid_rollback_and_reboot();
        /* never reached */
    }
}

void ota_task(void *pvParameters)
{
    ESP_LOGI(TAG, "OTA task started (check interval: %d min)",
             OTA_CHECK_INTERVAL_MS / 60000);

    /* Stagger first check by 30 s so other tasks can fully initialise */
    vTaskDelay(pdMS_TO_TICKS(30 * 1000));

    while (1) {
        /* Wait until WiFi is up */
        if (!wifi_is_connected()) {
            ESP_LOGD(TAG, "OTA: waiting for WiFi...");
            vTaskDelay(pdMS_TO_TICKS(10 * 1000));
            continue;
        }

        ESP_LOGI(TAG, "OTA: checking for updates...");
        esp_err_t err = ota_check_and_update();

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "OTA check failed: %s — will retry in %d min",
                     esp_err_to_name(err), OTA_CHECK_INTERVAL_MS / 60000);
        }

        vTaskDelay(pdMS_TO_TICKS(OTA_CHECK_INTERVAL_MS));
    }
}