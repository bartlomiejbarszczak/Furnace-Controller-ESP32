#include "mqtt_manager.h"
#include "wifi_manager.h"
#include "furnace_config.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include "mdns.h"

static const char *TAG = "MQTT_Manager";

// MQTT client handle
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

// mDNS and broker discovery
static bool mdns_initialized = false;
static char *current_broker_uri = NULL;
static uint8_t reconnect_attempt = 0;
static bool rediscover_broker_flag = false;

// External data sources (set by main application)
static furnace_config_t *furnace_config = NULL;
static furnace_runtime_t *furnace_runtime = NULL;
static SemaphoreHandle_t data_mutex = NULL;

// Callbacks
static bool (*wifi_is_connected_fn)(void) = NULL;
static int8_t (*wifi_get_rssi_fn)(void) = NULL;
static bool (*config_save_fn)(void) = NULL;

// Forward declarations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data);
static bool apply_config_from_json(const char* json_str, int json_len);
static bool apply_single_config(furnace_config_t *cfg, const char *key, cJSON *value, int *changes);
static void create_message_payload(char *buffer, size_t buffer_size, const char *command, const char *status, const char *message);
static char* mqtt_discover_broker(void);

// Set data sources
void mqtt_manager_set_data_sources(void *cfg, void *rt, void *mutex) {
    furnace_config = (furnace_config_t *)cfg;
    furnace_runtime = (furnace_runtime_t *)rt;
    data_mutex = (SemaphoreHandle_t)mutex;
}

// Set WiFi callbacks
void mqtt_manager_set_wifi_callbacks(bool (*is_connected)(void), int8_t (*get_rssi)(void)) {
    wifi_is_connected_fn = is_connected;
    wifi_get_rssi_fn = get_rssi;
}

// Set configuration save callback
void mqtt_manager_set_config_save_callback(bool (*save_fn)(void)) {
    config_save_fn = save_fn;
}

// Discover MQTT broker via mDNS
static char* mqtt_discover_broker(void) {
    // Initialize mDNS if not already done
    if (!mdns_initialized) {
        esp_err_t err = mdns_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize mDNS: %s", esp_err_to_name(err));
            return NULL;
        }
        
        mdns_hostname_set("furnace-esp32");
        mdns_instance_name_set("Furnace Controller");
        mdns_initialized = true;
        ESP_LOGI(TAG, "mDNS initialized");
    }
    
    ESP_LOGI(TAG, "Searching for MQTT broker via mDNS...");
    
    mdns_result_t *results = NULL;
    esp_err_t err = mdns_query_ptr("_mqtt", "_tcp", 3000, 20, &results);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS query failed: %s", esp_err_to_name(err));
        return NULL;
    }
    
    if (results == NULL) {
        ESP_LOGW(TAG, "No MQTT brokers found via mDNS");
        return NULL;
    }
    
    // Process first result
    mdns_result_t *r = results;
    mdns_ip_addr_t *a = r->addr;
    char *broker_uri = NULL;
    
    while (a) {
        if (a->addr.type == ESP_IPADDR_TYPE_V4) {
            ESP_LOGI(TAG, "Found MQTT broker:");
            ESP_LOGI(TAG, "  Hostname: %s", r->hostname ? r->hostname : "N/A");
            ESP_LOGI(TAG, "  Instance: %s", r->instance_name ? r->instance_name : "N/A");
            ESP_LOGI(TAG, "  IPv4: " IPSTR, IP2STR(&(a->addr.u_addr.ip4)));
            ESP_LOGI(TAG, "  Port: %d", r->port);
            
            // Allocate memory for URI string
            broker_uri = malloc(64);
            if (broker_uri != NULL) {
                snprintf(broker_uri, 64, "mqtt://" IPSTR ":%d",
                        IP2STR(&(a->addr.u_addr.ip4)), r->port);
                ESP_LOGI(TAG, "Broker URI: %s", broker_uri);
            }
            break;
        }
        a = a->next;
    }
    
    mdns_query_results_free(results);
    return broker_uri;
}

// Initialize MQTT
esp_err_t mqtt_manager_init(void) {
    if (furnace_config == NULL || furnace_runtime == NULL || data_mutex == NULL) {
        ESP_LOGE(TAG, "Data sources not set! Call mqtt_manager_set_data_sources() first");
        return ESP_ERR_INVALID_STATE;
    }

    // Discover broker via mDNS
    char *broker_uri = mqtt_discover_broker();
    
    if (broker_uri == NULL) {
        ESP_LOGW(TAG, "Broker not found via mDNS");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Save URI globally for reconnection
    if (current_broker_uri != NULL) {
        free(current_broker_uri);
    }
    current_broker_uri = broker_uri;

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = current_broker_uri,
        .session.last_will.topic = LWT_TOPIC,
        .session.last_will.msg = LWT_PAYLOAD_OFFLINE,
        .session.last_will.msg_len = strlen(LWT_PAYLOAD_OFFLINE),
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
        .session.keepalive = 15,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, 
                                   mqtt_event_handler, NULL);
    
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "MQTT client started with broker: %s", current_broker_uri);
    return ESP_OK;
}

// Check if broker rediscovery is needed
bool mqtt_manager_needs_rediscovery(void) {
    return rediscover_broker_flag;
}

// Perform broker rediscovery (call from external task, not from event handler)
esp_err_t mqtt_manager_rediscover_broker(void) {
    if (!rediscover_broker_flag) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting broker rediscovery via mDNS...");
    
    // Stop and destroy old client
    if (mqtt_client != NULL) {
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Find broker again via mDNS
    char *new_broker_uri = mqtt_discover_broker();
    
    if (new_broker_uri == NULL) {
        ESP_LOGW(TAG, "Broker not found via mDNS during rediscovery");
        rediscover_broker_flag = false;  // Clear flag to try again later
        return ESP_ERR_NOT_FOUND;
    }
    
    if (current_broker_uri != NULL) {
        free(current_broker_uri);
    }
    current_broker_uri = new_broker_uri;
    
    // Create new MQTT client with new URI
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = current_broker_uri,
        .session.last_will.topic = LWT_TOPIC,
        .session.last_will.msg = LWT_PAYLOAD_OFFLINE,
        .session.last_will.msg_len = strlen(LWT_PAYLOAD_OFFLINE),
        .session.last_will.qos = 1,
        .session.last_will.retain = true,
        .session.keepalive = 15,
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to create new MQTT client");
        rediscover_broker_flag = false;
        return ESP_FAIL;
    }
    
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, 
                                  mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    
    ESP_LOGI(TAG, "Reconnecting to newly discovered broker: %s", current_broker_uri);
    rediscover_broker_flag = false;
    
    return ESP_OK;
}
void mqtt_manager_deinit(void) {
    if (mqtt_client != NULL) {
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }
    
    if (current_broker_uri != NULL) {
        free(current_broker_uri);
        current_broker_uri = NULL;
    }
    
    mqtt_connected = false;
    reconnect_attempt = 0;
    
    ESP_LOGI(TAG, "MQTT manager deinitialized");
}

// Check connection status
bool mqtt_manager_is_connected(void) {
    return mqtt_connected;
}

// Publish to MQTT
int mqtt_manager_publish(const char *topic, const char *data, int qos, bool retain) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return -1;
    }
    
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, 0, qos, retain ? 1 : 0);
    return msg_id;
}

int mqtt_manager_publish_status(char *topic, char *payload, size_t topic_size, size_t payload_size) {
    xSemaphoreTake(data_mutex, portMAX_DELAY);

    furnace_state_t state = furnace_runtime->current_state;

    int16_t temp_furnace = furnace_runtime->temp_furnace;
    int16_t temp_boiler_top = furnace_runtime->temp_boiler_top;
    int16_t temp_boiler_bottom = furnace_runtime->temp_boiler_bottom;
    int16_t temp_main_output = furnace_runtime->temp_main_output;

    bool pump_main_on = furnace_runtime->pump_main_on;
    bool pump_floor_on = furnace_runtime->pump_floor_on;

    uint8_t pump_mixing_power = furnace_runtime->pump_mixing_power;
    uint8_t blower_power = furnace_runtime->blower_power;

    bool error_flag = furnace_runtime->error_flag;

    xSemaphoreGive(data_mutex);

    snprintf(topic, topic_size, "%s/status", MQTT_BASE_TOPIC);
    snprintf(payload, payload_size, 
                 "{"
                 "\"state\":\"%s\","
                 "\"temperatures\":{"
                   "\"furnace\":%d,"
                   "\"boiler_top\":%d,"
                   "\"boiler_bottom\":%d,"
                   "\"main_output\":%d"
                 "},"
                 "\"pumps\":{"
                   "\"main\":\"%s\","
                   "\"floor\":\"%s\","
                   "\"mixing_power\":%d"
                 "},"
                 "\"blower\":{\"power\":%d},"
                 "\"system\":{"
                   "\"error\":\"%s\","
                   "\"rssi\":%d,"
                   "\"heap\":%lu"
                 "}"
                 "}",
                 state_to_string(state),
                 temp_furnace, temp_boiler_top, temp_boiler_bottom, temp_main_output,
                 pump_main_on ? "ON" : "OFF", 
                 pump_floor_on ? "ON" : "OFF",
                 pump_mixing_power,
                 blower_power,
                 error_flag ? "ERROR" : "OK",
                 wifi_get_rssi(), 
                 esp_get_free_heap_size());

    return mqtt_manager_publish(topic, payload, 1, false);
}

// Apply single config parameter
static bool apply_single_config(furnace_config_t *cfg, const char *key, cJSON *value, int *changes) {
    // Temperature thresholds
    bool found_key = false;

    if (strcmp(key, "temp_target_work") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int16_t val = (int16_t)value->valueint;
        if (val >= 40 && val <= 80 && val != cfg->temp_target_work) {
            cfg->temp_target_work = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "temp_ignition") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int16_t val = (int16_t)value->valueint;
        if (val >= 20 && val <= 35 && val != cfg->temp_ignition) {
            cfg->temp_ignition = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "temp_shutdown") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int16_t val = (int16_t)value->valueint;
        if (val >= 30 && val <= 50 && val != cfg->temp_shutdown) {
            cfg->temp_shutdown = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "temp_safe_after_overheat") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int16_t val = (int16_t)value->valueint;
        if (val >= 60 && val <= 80 && val != cfg->temp_safe_after_overheat) {
            cfg->temp_safe_after_overheat = val;
            (*changes)++;
            return true;
        }
    }

    // Main pump
    else if (strcmp(key, "temp_activ_pump_main") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int16_t val = (int16_t)value->valueint;
        if (val >= 30 && val <= 80 && val != cfg->temp_activ_pump_main) {
            cfg->temp_activ_pump_main = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "temp_activ_pump_main_hysteresis") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int16_t val = (int16_t)value->valueint;
        if (val >= 2 && val <= 10 && val != cfg->temp_activ_pump_main_hysteresis) {
            cfg->temp_activ_pump_main_hysteresis = val;
            (*changes)++;
            return true;
        }
    }

    // Boiler
    else if (strcmp(key, "temp_diff_furnace_boiler") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int16_t val = (int16_t)value->valueint;
        if (val >= 3 && val <= 10 && val != cfg->temp_diff_furnace_boiler) {
            cfg->temp_diff_furnace_boiler = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "temp_diff_boiler_vertical") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int16_t val = (int16_t)value->valueint;
        if (val >= 3 && val <= 10 && val != cfg->temp_diff_boiler_vertical) {
            cfg->temp_diff_boiler_vertical = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "boiler_top_priority_enabled") == 0 && cJSON_IsBool(value)) {
        found_key = true;
        bool val = cJSON_IsTrue(value);
        if (val != cfg->boiler_top_priority_enabled) {
            cfg->boiler_top_priority_enabled = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "temp_boiler_top_priority") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int16_t val = (int16_t)value->valueint;
        if (val >= 30 && val <= 70 && val != cfg->temp_boiler_top_priority) {
            cfg->temp_boiler_top_priority = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "boiler_mixing_in_shutdown") == 0 && cJSON_IsBool(value)) {
        found_key = true;
        bool val = cJSON_IsTrue(value);
        if (val != cfg->boiler_mixing_in_shutdown) {
            cfg->boiler_mixing_in_shutdown = val;
            (*changes)++;
            return true;
        }
    }

    // Mixing pump
    else if (strcmp(key, "pump_mixing_power_min") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        uint8_t val = (uint8_t)value->valueint;
        if (val <= 100 && val != cfg->pump_mixing_power_min) {
            cfg->pump_mixing_power_min = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "pump_mixing_power_max") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        uint8_t val = (uint8_t)value->valueint;
        if (val <= 100 && val != cfg->pump_mixing_power_max) {
            cfg->pump_mixing_power_max = val;
            (*changes)++;
            return true;
        }
    }

    // Blower
    else if (strcmp(key, "blower_enabled") == 0 && cJSON_IsBool(value)) {
        found_key = true;
        bool val = cJSON_IsTrue(value);
        if (val != cfg->blower_enabled) {
            cfg->blower_enabled = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "blower_power_min") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        uint8_t val = (uint8_t)value->valueint;
        if (val >= 20 && val <= 100 && val != cfg->blower_power_min) {
            cfg->blower_power_min = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "blower_power_max") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        uint8_t val = (uint8_t)value->valueint;
        if (val >= 20 && val <= 100 && val != cfg->blower_power_max) {
            cfg->blower_power_max = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "time_blowthrough_sec") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        uint16_t val = (uint16_t)value->valueint;
        if (val >= 10 && val <= 180 && val != cfg->time_blowthrough_sec) {
            cfg->time_blowthrough_sec = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "time_blowthrough_interval_min") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        uint16_t val = (uint16_t)value->valueint;
        if (val <= 30 && val != cfg->time_blowthrough_interval_min) {
            cfg->time_blowthrough_interval_min = val;
            (*changes)++;
            return true;
        }
    }
    // Floor pump
    else if (strcmp(key, "pump_floor_enabled") == 0 && cJSON_IsBool(value)) {
        found_key = true;
        bool val = cJSON_IsTrue(value);
        if (val != cfg->pump_floor_enabled) {
            cfg->pump_floor_enabled = val;
            (*changes)++;
            return true;
        }
    }
    // Temperature corrections
    else if (strcmp(key, "temp_correction_furnace") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int8_t val = (int8_t)value->valueint;
        if (val >= -5 && val <= 5 && val != cfg->temp_correction_furnace) {
            cfg->temp_correction_furnace = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "temp_correction_boiler_top") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int8_t val = (int8_t)value->valueint;
        if (val >= -5 && val <= 5 && val != cfg->temp_correction_boiler_top) {
            cfg->temp_correction_boiler_top = val;
            (*changes)++;
            return true;
        }
    }
    else if (strcmp(key, "temp_correction_boiler_bottom") == 0 && cJSON_IsNumber(value)) {
        found_key = true;
        int8_t val = (int8_t)value->valueint;
        if (val >= -5 && val <= 5 && val != cfg->temp_correction_boiler_bottom) {
            cfg->temp_correction_boiler_bottom = val;
            (*changes)++;
            return true;
        }
    }

    if (found_key) {
        return true;
    }
    ESP_LOGW(TAG, "Unknown or invalid config key: %s", key);
    return false;
}

// Apply configuration from JSON
static bool apply_config_from_json(const char* json_str, int json_len) {
    cJSON *root = cJSON_ParseWithLength(json_str, json_len);
    if (root == NULL) {
        ESP_LOGE(TAG, "JSON parse error");
        return false;
    }
    
    if (furnace_config == NULL || data_mutex == NULL) {
        ESP_LOGE(TAG, "Config or mutex not set");
        cJSON_Delete(root);
        return false;
    }
    
    // Create temporary config copy
    furnace_config_t new_config;
    
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    memcpy(&new_config, furnace_config, sizeof(furnace_config_t));
    xSemaphoreGive(data_mutex);
    
    int changes = 0;
    int errors = 0;
    
    // Iterate through all received keys
    cJSON *item = NULL;
    cJSON_ArrayForEach(item, root) {
        if (item->string) {
            if (!apply_single_config(&new_config, item->string, item, &changes)) {
                errors++;
            }
        }
    }
    
    cJSON_Delete(root);
    
    ESP_LOGI(TAG, "Config update: %d changes, %d errors", changes, errors);
    
    // Apply changes if any valid changes were made
    if (changes > 0) {
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        memcpy(furnace_config, &new_config, sizeof(furnace_config_t));
        xSemaphoreGive(data_mutex);
        char status_payload[256];
        
        // Save to NVS
        if (config_save_fn != NULL && config_save_fn()) {
            ESP_LOGI(TAG, "Configuration updated and saved to NVS");
            
            char status_msg[128];
            snprintf(status_msg, sizeof(status_msg), "{\"changes\":%d,\"errors\":%d}", changes, errors);
            create_message_payload(status_payload, sizeof(status_payload), "config", "ok", status_msg);
            mqtt_manager_publish(MQTT_BASE_TOPIC "/command/status", status_payload, 1, false);
        } else {
            ESP_LOGE(TAG, "Failed to save configuration to NVS");
            create_message_payload(status_payload, sizeof(status_payload), "config", "error", "Failed to save to NVS");
            mqtt_manager_publish(MQTT_BASE_TOPIC "/command/status", status_payload, 1, false);
        }
        
        return true;
    }
    
    return false;
}

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to broker");
            mqtt_connected = true;
            reconnect_attempt = 0;  // Reset reconnect counter on successful connection
            
            esp_mqtt_client_publish(client, LWT_TOPIC, LWT_PAYLOAD_ONLINE, 0, 1, true);
            
            // Subscribe to command topics
            esp_mqtt_client_subscribe(client, MQTT_BASE_TOPIC "/command/ignition", 0);
            esp_mqtt_client_subscribe(client, MQTT_BASE_TOPIC "/command/shutdown", 0);
            esp_mqtt_client_subscribe(client, MQTT_BASE_TOPIC "/command/restart", 0);
            esp_mqtt_client_subscribe(client, MQTT_BASE_TOPIC "/command/config", 0);
            esp_mqtt_client_subscribe(client, MQTT_BASE_TOPIC "/command/config/get", 0);
            ESP_LOGI(TAG, "Subscribed to command topics");
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected from broker");
            mqtt_connected = false;
            reconnect_attempt++;
            
            if (reconnect_attempt <= 3) {
                // First 3 attempts - ESP-IDF MQTT client will automatically reconnect
                ESP_LOGI(TAG, "Reconnecting to last known broker... (%d/3)", reconnect_attempt);
                // Do nothing - library handles reconnection automatically
            } else {
                // After 3 failed reconnects - set flag for external task to handle
                ESP_LOGI(TAG, "3 reconnect attempts failed, triggering broker rediscovery");
                reconnect_attempt = 0;
                rediscover_broker_flag = true;
            }
            break;
            
        case MQTT_EVENT_DATA: {
            ESP_LOGI(TAG, "Data received on topic: %.*s", event->topic_len, event->topic);
            char status_payload[256];
            
            // Handle commands
            if (strncmp(event->topic, MQTT_BASE_TOPIC "/command/ignition", event->topic_len) == 0) {
                if (furnace_runtime && data_mutex) {
                    xSemaphoreTake(data_mutex, portMAX_DELAY);
                    furnace_runtime->manual_ignition_requested = true;
                    xSemaphoreGive(data_mutex);
                    ESP_LOGI(TAG, "Manual ignition requested via MQTT");
                    create_message_payload(status_payload, sizeof(status_payload), "ignition", "ok", "");
                    mqtt_manager_publish(MQTT_BASE_TOPIC "/command/status", status_payload, 1, false);
                }
            }
            else if (strncmp(event->topic, MQTT_BASE_TOPIC "/command/shutdown", event->topic_len) == 0) {
                if (furnace_runtime && data_mutex) {
                    xSemaphoreTake(data_mutex, portMAX_DELAY);
                    furnace_runtime->manual_shutdown_requested = true;
                    xSemaphoreGive(data_mutex);
                    ESP_LOGI(TAG, "Manual shutdown requested via MQTT");
                    create_message_payload(status_payload, sizeof(status_payload), "shutdown", "ok", "");
                    mqtt_manager_publish(MQTT_BASE_TOPIC "/command/status", status_payload, 1, false);
                }
            }
            else if (strncmp(event->topic, MQTT_BASE_TOPIC "/command/restart", event->topic_len) == 0) {
                ESP_LOGI(TAG, "Restart requested via MQTT");
                create_message_payload(status_payload, sizeof(status_payload), "restart", "ok", "");
                mqtt_manager_publish(MQTT_BASE_TOPIC "/command/status", status_payload, 1, false);
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
            else if (strncmp(event->topic, MQTT_BASE_TOPIC "/command/config", event->topic_len) == 0) {
                ESP_LOGI(TAG, "Configuration update received");
                if (apply_config_from_json(event->data, event->data_len)) {
                    ESP_LOGI(TAG, "Configuration applied successfully");
                } else {
                    ESP_LOGW(TAG, "No valid configuration changes found");
                    create_message_payload(status_payload, sizeof(status_payload), "config", "error", "No valid configuration changes found");
                    mqtt_manager_publish(MQTT_BASE_TOPIC "/command/status", status_payload, 1, false);
                }
            }
            else if (strncmp(event->topic, MQTT_BASE_TOPIC "/command/config/get", event->topic_len) == 0) {
                char payload[1024];
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                snprintf(payload, sizeof(payload),
                    "{"
                    "\"temp_target_work\":%d,"
                    "\"temp_ignition\":%d,"
                    "\"temp_shutdown\":%d,"
                    "\"temp_safe_after_overheat\":%d,"
                    "\"temp_activ_pump_main\":%d,"
                    "\"temp_activ_pump_main_hysteresis\":%d,"
                    "\"temp_diff_furnace_boiler\":%d,"
                    "\"temp_diff_boiler_vertical\":%d,"
                    "\"boiler_top_priority_enabled\":%s,"
                    "\"temp_boiler_top_priority\":%d,"
                    "\"boiler_mixing_in_shutdown\":%s,"
                    "\"pump_mixing_power_min\":%d,"
                    "\"pump_mixing_power_max\":%d,"
                    "\"blower_enabled\":%s,"
                    "\"blower_power_min\":%d,"
                    "\"blower_power_max\":%d,"
                    "\"time_blowthrough_sec\":%d,"
                    "\"time_blowthrough_interval_min\":%d,"
                    "\"pump_floor_enabled\":%s,"
                    "\"temp_correction_furnace\":%d,"
                    "\"temp_correction_boiler_top\":%d,"
                    "\"temp_correction_boiler_bottom\":%d"
                    "}",
                    furnace_config->temp_target_work,
                    furnace_config->temp_ignition,
                    furnace_config->temp_shutdown,
                    furnace_config->temp_safe_after_overheat,
                    furnace_config->temp_activ_pump_main,
                    furnace_config->temp_activ_pump_main_hysteresis,
                    furnace_config->temp_diff_furnace_boiler,
                    furnace_config->temp_diff_boiler_vertical,
                    furnace_config->boiler_top_priority_enabled ? "true" : "false",
                    furnace_config->temp_boiler_top_priority,
                    furnace_config->boiler_mixing_in_shutdown ? "true" : "false",
                    furnace_config->pump_mixing_power_min,
                    furnace_config->pump_mixing_power_max,
                    furnace_config->blower_enabled ? "true" : "false",
                    furnace_config->blower_power_min,
                    furnace_config->blower_power_max,
                    furnace_config->time_blowthrough_sec,
                    furnace_config->time_blowthrough_interval_min,
                    furnace_config->pump_floor_enabled ? "true" : "false",
                    furnace_config->temp_correction_furnace,
                    furnace_config->temp_correction_boiler_top,
                    furnace_config->temp_correction_boiler_bottom
                );
                xSemaphoreGive(data_mutex);
                create_message_payload(status_payload, sizeof(status_payload), "config_get", "ok", "");
                mqtt_manager_publish(MQTT_BASE_TOPIC "/config/get-response", payload, 1, false);
                mqtt_manager_publish(MQTT_BASE_TOPIC "/command/status", status_payload, 1, false);
            }
            break;
        }
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error occurred");
            mqtt_connected = false;
            break;
            
        default:
            break;
    }
}

static void create_message_payload(char *buffer, size_t buffer_size, const char *command, const char *status, const char *message) 
{
    snprintf(buffer, buffer_size,
             "{\"command\":\"%s\",\"status\":\"%s\",\"message\":\"%s\"}",
             command, status, message);
}