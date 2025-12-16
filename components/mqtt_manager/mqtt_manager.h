#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

// MQTT Configuration
#define MQTT_BROKER_URI "mqtt://192.168.0.113:1883"
#define MQTT_BASE_TOPIC "furnace"
#define LWT_TOPIC       MQTT_BASE_TOPIC "/availability"
#define LWT_PAYLOAD_OFFLINE "offline"
#define LWT_PAYLOAD_ONLINE  "online"

/**
 * @brief Initialize MQTT client and connect to broker
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_manager_init(void);

/**
 * @brief Check if MQTT is connected
 * 
 * @return true if connected, false otherwise
 */
bool mqtt_manager_is_connected(void);

/**
 * @brief Publish data to MQTT topic
 * 
 * @param topic Topic to publish to (will be prefixed with base topic if relative)
 * @param data Data to publish
 * @param qos QoS level (0, 1, or 2)
 * @param retain Retain flag
 * @return Message ID on success, -1 on failure
 */
int mqtt_manager_publish(const char *topic, const char *data, int qos, bool retain);

/**
 * @brief Publish furnace status data
 * 
 * This function should be called periodically to publish current state,
 * temperatures, pump states, etc.
 * 
 * @return Message ID on success, -1 on failure
 */
int mqtt_manager_publish_status(char *topic, char *payload, size_t topic_size, size_t payload_size);

/**
 * @brief Set configuration pointer for MQTT manager
 * 
 * Must be called before mqtt_manager_init()
 * 
 * @param cfg Pointer to furnace configuration structure
 * @param rt Pointer to furnace runtime structure
 * @param mutex Mutex for accessing config and runtime safely
 */
void mqtt_manager_set_data_sources(void *cfg, void *rt, void *mutex);

/**
 * @brief Set WiFi status callback
 * 
 * @param is_connected Function pointer to check WiFi connection status
 * @param get_rssi Function pointer to get WiFi RSSI
 */
void mqtt_manager_set_wifi_callbacks(bool (*is_connected)(void), int8_t (*get_rssi)(void));

/**
 * @brief Set configuration save callback
 * 
 * @param save_fn Function pointer to save configuration to NVS
 */
void mqtt_manager_set_config_save_callback(bool (*save_fn)(void));

#ifdef __cplusplus
}
#endif

#endif // MQTT_MANAGER_H