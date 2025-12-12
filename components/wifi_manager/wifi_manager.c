#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "WiFi_Manager";

// Default WiFi credentials (fallback if nothing in NVS)
#define DEFAULT_WIFI_SSID      "DziwneJaja"
#define DEFAULT_WIFI_PASSWORD  "Haslo123#@!"

// Event group for WiFi events
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
static wifi_status_t s_wifi_status = WIFI_STATUS_DISCONNECTED;
static bool s_auto_reconnect = true;
static bool s_wifi_initialized = false;

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi started, connecting...");
        s_wifi_status = WIFI_STATUS_CONNECTING;
        esp_wifi_connect();
        
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_status = WIFI_STATUS_DISCONNECTED;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
        wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi disconnected, reason: %d", disconn->reason);
        
        if (s_auto_reconnect && s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connecting to WiFi... (%d/%d)", 
                     s_retry_num, WIFI_MAXIMUM_RETRY);
        } else if (s_retry_num >= WIFI_MAXIMUM_RETRY) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            s_wifi_status = WIFI_STATUS_FAILED;
            ESP_LOGE(TAG, "Failed to connect to WiFi after %d attempts", 
                     WIFI_MAXIMUM_RETRY);
            
            // Wait 10 seconds before resetting retry counter
            vTaskDelay(pdMS_TO_TICKS(10000));
            s_retry_num = 0;
            
            // Try again if auto-reconnect is enabled
            if (s_auto_reconnect) {
                ESP_LOGI(TAG, "Attempting reconnection...");
                xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
                esp_wifi_connect();
            }
        }
        
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        s_wifi_status = WIFI_STATUS_CONNECTED;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_save_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "Invalid WiFi credentials");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(ssid) == 0 || strlen(ssid) > 31) {
        ESP_LOGE(TAG, "Invalid SSID length");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(password) > 63) {
        ESP_LOGE(TAG, "Invalid password length");
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save SSID
    ret = nvs_set_str(handle, "ssid", ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }
    
    // Save password
    ret = nvs_set_str(handle, "password", password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save password: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }
    
    // Commit changes
    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "WiFi credentials saved successfully");
    }
    
    nvs_close(handle);
    return ret;
}

esp_err_t wifi_load_credentials(char *ssid, char *password)
{
    if (ssid == NULL || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for reading: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Load SSID
    size_t ssid_len = 32;
    ret = nvs_get_str(handle, "ssid", ssid, &ssid_len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi SSID not found in NVS");
        nvs_close(handle);
        return ret;
    }
    
    // Load password
    size_t pass_len = 64;
    ret = nvs_get_str(handle, "password", password, &pass_len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi password not found in NVS");
        nvs_close(handle);
        return ret;
    }
    
    nvs_close(handle);
    ESP_LOGI(TAG, "WiFi credentials loaded from NVS");
    return ESP_OK;
}

bool wifi_credentials_exist(void)
{
    char ssid[32];
    char password[64];
    return (wifi_load_credentials(ssid, password) == ESP_OK);
}

esp_err_t wifi_erase_credentials(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = nvs_erase_all(handle);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
        ESP_LOGI(TAG, "WiFi credentials erased");
    }
    
    nvs_close(handle);
    return ret;
}

static esp_err_t wifi_init_netif_and_event_loop(void)
{
    esp_err_t ret;
    
    // Initialize network interface (only once)
    static bool netif_initialized = false;
    if (!netif_initialized) {
        ret = esp_netif_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
            return ret;
        }
        netif_initialized = true;
    }
    
    // Create default event loop (only once)
    static bool event_loop_created = false;
    if (!event_loop_created) {
        ret = esp_event_loop_create_default();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
            return ret;
        }
        event_loop_created = true;
    }
    
    return ESP_OK;
}

esp_err_t wifi_init_sta_with_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL) {
        ESP_LOGE(TAG, "Invalid credentials");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Save credentials to NVS first
    esp_err_t ret = wifi_save_credentials(ssid, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save credentials");
        return ret;
    }
    
    // Initialize WiFi with these credentials
    return wifi_init_sta();
}

esp_err_t wifi_init_sta(void)
{
    esp_err_t ret = ESP_OK;
    
    // Create event group if not exists
    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) {
            ESP_LOGE(TAG, "Failed to create event group");
            return ESP_FAIL;
        }
    }
    
    // Initialize network interface and event loop
    ret = wifi_init_netif_and_event_loop();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Create default WiFi station (only once)
    static esp_netif_t *sta_netif = NULL;
    if (sta_netif == NULL) {
        sta_netif = esp_netif_create_default_wifi_sta();
        if (sta_netif == NULL) {
            ESP_LOGE(TAG, "Failed to create default WiFi STA");
            return ESP_FAIL;
        }
    }
    
    // Initialize WiFi with default config (only once)
    if (!s_wifi_initialized) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ret = esp_wifi_init(&cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
            return ret;
        }
        s_wifi_initialized = true;
        
        // Register event handlers (only once)
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            NULL));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            NULL));
    }
    
    // Load credentials from NVS or use defaults
    char ssid[32];
    char password[64];
    
    if (wifi_load_credentials(ssid, password) != ESP_OK) {
        ESP_LOGW(TAG, "No WiFi credentials in NVS, using defaults");
        strncpy(ssid, DEFAULT_WIFI_SSID, sizeof(ssid) - 1);
        strncpy(password, DEFAULT_WIFI_PASSWORD, sizeof(password) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
        password[sizeof(password) - 1] = '\0';
    }
    
    ESP_LOGI(TAG, "Connecting to SSID: %s", ssid);
    
    // Configure WiFi
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    
    // Set WiFi mode and configuration
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Start WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WiFi initialization completed");
    
    // Wait for connection or failure (with timeout)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(30000)); // 30 second timeout
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi successfully");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "Failed to connect to WiFi (will retry in background)");
        return ESP_ERR_WIFI_NOT_CONNECT;
    } else {
        ESP_LOGW(TAG, "WiFi connection timeout (will retry in background)");
        return ESP_ERR_TIMEOUT;
    }
}

bool wifi_is_connected(void)
{
    return (s_wifi_status == WIFI_STATUS_CONNECTED);
}

wifi_status_t wifi_get_status(void)
{
    return s_wifi_status;
}

esp_err_t wifi_reconnect(void)
{
    ESP_LOGI(TAG, "Manual reconnection requested");
    s_retry_num = 0;
    xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    return esp_wifi_connect();
}

int8_t wifi_get_rssi(void)
{
    if (!wifi_is_connected()) {
        return 0;
    }
    
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    
    if (ret == ESP_OK) {
        return ap_info.rssi;
    }
    
    return 0;
}