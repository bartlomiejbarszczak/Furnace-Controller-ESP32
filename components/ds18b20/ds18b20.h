#ifndef DS18B20_H_
#define DS18B20_H_

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>

// DS18B20 Commands
#define DS18B20_SKIP_ROM           0xCC
#define DS18B20_CONVERT_T          0x44
#define DS18B20_READ_SCRATCHPAD    0xBE
#define DS18B20_WRITE_SCRATCHPAD   0x4E
#define DS18B20_READ_ROM           0x33
#define DS18B20_MATCH_ROM          0x55
#define DS18B20_SEARCH_ROM         0xF0
#define DS18B20_ALARM_SEARCH       0xEC

// Return codes
#define DS18B20_OK                 0
#define DS18B20_ERROR_NO_DEVICE   -1
#define DS18B20_ERROR_CRC         -2
#define DS18B20_ERROR_TIMEOUT     -3

// Sensor configuration structure
typedef struct {
    gpio_num_t gpio_pin;
    char *name;
    float last_temperature;
    bool is_valid;
} ds18b20_sensor_t;

/**
 * @brief Initialize DS18B20 sensor array
 * @param sensors Array of sensor structures with GPIO pins configured
 * @param num_sensors Number of sensors in the array
 * @return ESP_OK on success
 */
esp_err_t ds18b20_init(ds18b20_sensor_t *sensors, uint8_t num_sensors);

/**
 * @brief Read temperature from a specific sensor
 * @param sensor Pointer to sensor structure
 * @param temperature Pointer to store the temperature result
 * @return DS18B20_OK on success, error code otherwise
 */
int ds18b20_read_temperature(ds18b20_sensor_t *sensor, float *temperature);

/**
 * @brief Read temperature from all sensors
 * @param sensors Array of sensor structures
 * @param num_sensors Number of sensors
 * @param temperatures Array to store temperature results
 * @return Number of successfully read sensors
 */
int ds18b20_read_all(ds18b20_sensor_t *sensors, uint8_t num_sensors, float *temperatures);

/**
 * @brief Trigger temperature conversion on a specific sensor
 * @param sensor Pointer to sensor structure
 * @return DS18B20_OK on success
 */
int ds18b20_trigger_conversion(ds18b20_sensor_t *sensor);

/**
 * @brief Trigger temperature conversion on all sensors
 * @param sensors Array of sensor structures
 * @param num_sensors Number of sensors
 * @return DS18B20_OK on success
 */
int ds18b20_trigger_all_conversions(ds18b20_sensor_t *sensors, uint8_t num_sensors);

#endif /* DS18B20_H_ */