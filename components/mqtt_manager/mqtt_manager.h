#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include "esp_err.h"
#include "mqtt_client.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// MQTT Configuration
#define MQTT_BASE_TOPIC "furnace"
#define LWT_TOPIC       MQTT_BASE_TOPIC "/availability"
#define LWT_PAYLOAD_OFFLINE "offline"
#define LWT_PAYLOAD_ONLINE  "online"
#define MQTT_SERVICE        "_mqtt"
#define MQTT_PROTO          "_tcp"

/**
 * @brief Set data sources for MQTT manager
 * Must be called before mqtt_manager_init()
 * 
 * @param cfg Pointer to furnace_config_t
 * @param rt Pointer to furnace_runtime_t
 * @param mutex Pointer to SemaphoreHandle_t
 */
void mqtt_manager_set_data_sources(void *cfg, void *rt, void *mutex);

/**
 * @brief Set WiFi callback functions
 * 
 * @param is_connected Function pointer to check WiFi connection status
 * @param get_rssi Function pointer to get WiFi RSSI
 */
void mqtt_manager_set_wifi_callbacks(bool (*is_connected)(void), int8_t (*get_rssi)(void));

/**
 * @brief Set configuration save callback
 * 
 * @param save_fn Function pointer to save configuration
 */
void mqtt_manager_set_config_save_callback(bool (*save_fn)(void));

/**
 * @brief Initialize MQTT manager and discover broker via mDNS
 * Searches for MQTT broker on local network using mDNS service discovery
 * 
 * @return ESP_OK on success
 *         ESP_ERR_NOT_FOUND if broker not found via mDNS
 *         ESP_ERR_INVALID_STATE if data sources not set
 *         ESP_FAIL on other errors
 */
esp_err_t mqtt_manager_init(void);

/**
 * @brief Deinitialize MQTT manager and free resources
 * Stops MQTT client and frees allocated memory
 */
void mqtt_manager_deinit(void);

/**
 * @brief Check if broker rediscovery is needed
 * This flag is set after 3 failed reconnection attempts
 * 
 * @return true if rediscovery is needed, false otherwise
 */
bool mqtt_manager_needs_rediscovery(void);

/**
 * @brief Perform broker rediscovery via mDNS
 * Should be called from external task (not from event handler) when
 * mqtt_manager_needs_rediscovery() returns true
 * 
 * @return ESP_OK on success
 *         ESP_ERR_NOT_FOUND if broker not found
 *         ESP_FAIL on other errors
 */
esp_err_t mqtt_manager_rediscover_broker(void);

/**
 * @brief Check if MQTT is connected
 * 
 * @return true if connected, false otherwise
 */
bool mqtt_manager_is_connected(void);

/**
 * @brief Publish message to MQTT broker
 * 
 * @param topic MQTT topic
 * @param data Message data
 * @param qos Quality of Service (0, 1, or 2)
 * @param retain Retain flag
 * @return Message ID on success, -1 on failure
 */
int mqtt_manager_publish(const char *topic, const char *data, int qos, bool retain);

/**
 * @brief Publish furnace status to MQTT
 * Publishes comprehensive status including temperatures, pump states, etc.
 * 
 * @param topic Buffer to store topic (min 96 bytes)
 * @param payload Buffer to store payload (min 512 bytes)
 * @param topic_size Size of topic buffer
 * @param payload_size Size of payload buffer
 * @return Message ID on success, -1 on failure
 */
int mqtt_manager_publish_status(char *topic, char *payload, size_t topic_size, size_t payload_size);

#ifdef __cplusplus
}
#endif

#endif // MQTT_MANAGER_H