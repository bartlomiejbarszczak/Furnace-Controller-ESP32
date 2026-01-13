#include "ds18b20.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

static const char *TAG = "DS18B20";

// Critical section macros for FreeRTOS
#define DS18B20_ENTER_CRITICAL() portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; taskENTER_CRITICAL(&mux)
#define DS18B20_EXIT_CRITICAL() taskEXIT_CRITICAL(&mux)

// Timing constants (microseconds) - based on DS18B20 datasheet
#define TIMING_RESET_PULSE      500     // 480
#define TIMING_PRESENCE_WAIT    80      // 70
#define TIMING_PRESENCE_READ    420     // 410
#define TIMING_WRITE_1_LOW      8       // 6
#define TIMING_WRITE_1_HIGH     70      // 64
#define TIMING_WRITE_0_LOW      65      // 60
#define TIMING_WRITE_0_HIGH     10      
#define TIMING_READ_INIT        8       // 6
#define TIMING_READ_WAIT        12      // 9
#define TIMING_READ_RELEASE     60      // 55
#define TIMING_SLOT_RECOVERY    2       // 1

#define DS18B20_CONVERSION_TIME_MS  750 // Conversion time (milliseconds)
#define MAX_READ_RETRIES 3

// Static helper functions
static void ds18b20_set_output(gpio_num_t pin);
static void ds18b20_set_input(gpio_num_t pin);
static void ds18b20_write_low(gpio_num_t pin);
static void ds18b20_release_bus(gpio_num_t pin);
static uint8_t ds18b20_read_bit(gpio_num_t pin);
static int ds18b20_reset(gpio_num_t pin);
static void ds18b20_write_bit(gpio_num_t pin, uint8_t bit);
static void ds18b20_write_byte(gpio_num_t pin, uint8_t byte);
static uint8_t ds18b20_read_byte(gpio_num_t pin);
static uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len);

// ============== GPIO LOW-LEVEL FUNCTIONS ==============

static inline void ds18b20_set_output(gpio_num_t pin) {
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

static inline void ds18b20_set_input(gpio_num_t pin) {
    gpio_set_direction(pin, GPIO_MODE_INPUT);
}

static inline void ds18b20_write_low(gpio_num_t pin) {
    gpio_set_level(pin, 0);
}

static inline void ds18b20_release_bus(gpio_num_t pin) {
    gpio_set_direction(pin, GPIO_MODE_INPUT);
}

// ============== ONE-WIRE PROTOCOL FUNCTIONS ==============

/**
 * @brief Reset pulse and presence detection
 * @return DS18B20_OK if device present, DS18B20_ERROR_NO_DEVICE otherwise
 */
static int ds18b20_reset(gpio_num_t pin) {
    uint8_t presence;
    
    // Pull bus low for reset pulse
    ds18b20_set_output(pin);
    ds18b20_write_low(pin);
    ets_delay_us(TIMING_RESET_PULSE);
    
    // Release bus and wait for presence pulse
    ds18b20_release_bus(pin);
    ets_delay_us(TIMING_PRESENCE_WAIT);
    
    // Read presence pulse
    presence = gpio_get_level(pin);
    
    // Wait for presence pulse to finish
    ets_delay_us(TIMING_PRESENCE_READ);
    
    return (presence == 0) ? DS18B20_OK : DS18B20_ERROR_NO_DEVICE;
}

/**
 * @brief Write a single bit to the bus
 */
static void ds18b20_write_bit(gpio_num_t pin, uint8_t bit) {
    if (bit & 1) {
        // Write '1' bit
        DS18B20_ENTER_CRITICAL();
        ds18b20_set_output(pin);
        ds18b20_write_low(pin);
        ets_delay_us(TIMING_WRITE_1_LOW);
        ds18b20_release_bus(pin);
        ets_delay_us(TIMING_WRITE_1_HIGH);
        DS18B20_EXIT_CRITICAL();
    } else {
        // Write '0' bit
        DS18B20_ENTER_CRITICAL();
        ds18b20_set_output(pin);
        ds18b20_write_low(pin);
        ets_delay_us(TIMING_WRITE_0_LOW);
        ds18b20_release_bus(pin);
        ets_delay_us(TIMING_WRITE_0_HIGH);
        DS18B20_EXIT_CRITICAL();
    }
}

/**
 * @brief Read a single bit from the bus
 */
static uint8_t ds18b20_read_bit(gpio_num_t pin) {
    uint8_t bit;
    
    DS18B20_ENTER_CRITICAL();
    ds18b20_set_output(pin);
    ds18b20_write_low(pin);
    ets_delay_us(TIMING_READ_INIT);
    ds18b20_release_bus(pin);
    ets_delay_us(TIMING_READ_WAIT);
    bit = gpio_get_level(pin);
    ets_delay_us(TIMING_READ_RELEASE);
    DS18B20_EXIT_CRITICAL();
    
    return bit;
}

/**
 * @brief Write a byte to the bus
 */
static void ds18b20_write_byte(gpio_num_t pin, uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        ds18b20_write_bit(pin, byte & 0x01);
        byte >>= 1;
    }
    ets_delay_us(TIMING_SLOT_RECOVERY);
}

/**
 * @brief Read a byte from the bus
 */
static uint8_t ds18b20_read_byte(gpio_num_t pin) {
    uint8_t byte = 0;
    
    for (uint8_t i = 0; i < 8; i++) {
        byte >>= 1;
        if (ds18b20_read_bit(pin)) {
            byte |= 0x80;
        }
    }
    ets_delay_us(TIMING_SLOT_RECOVERY);
    
    return byte;
}

/**
 * @brief Calculate CRC8 checksum
 */
static uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            byte >>= 1;
        }
    }
    
    return crc;
}

// ============== PUBLIC API FUNCTIONS ==============

esp_err_t ds18b20_init(ds18b20_sensor_t *sensors, uint8_t num_sensors) {
    if (sensors == NULL || num_sensors == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    for (uint8_t i = 0; i < num_sensors; i++) {
        // Configure GPIO pin
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << sensors[i].gpio_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        
        // Initialize sensor state
        sensors[i].last_temperature = 0.0f;
        sensors[i].is_valid = false;
        
        // Test presence
        if (ds18b20_reset(sensors[i].gpio_pin) != DS18B20_OK) {
            ESP_LOGW(TAG, "Sensor on GPIO %d -%s- not detected", sensors[i].gpio_pin, sensors[i].name);
        } else {
            ESP_LOGI(TAG, "Sensor on GPIO %d -%s- initialized", sensors[i].gpio_pin, sensors[i].name);
        }
    }
    
    return ESP_OK;
}

int ds18b20_trigger_conversion(ds18b20_sensor_t *sensor) {
    if (sensor == NULL) {
        return DS18B20_ERROR_NO_DEVICE;
    }
    
    // Reset and check presence
    if (ds18b20_reset(sensor->gpio_pin) != DS18B20_OK) {
        return DS18B20_ERROR_NO_DEVICE;
    }
    
    // Skip ROM (single sensor per pin)
    ds18b20_write_byte(sensor->gpio_pin, DS18B20_SKIP_ROM);
    
    // Start temperature conversion
    ds18b20_write_byte(sensor->gpio_pin, DS18B20_CONVERT_T);
    
    return DS18B20_OK;
}

int ds18b20_trigger_all_conversions(ds18b20_sensor_t *sensors, uint8_t num_sensors) {
    int success_count = 0;
    
    for (uint8_t i = 0; i < num_sensors; i++) {
        if (ds18b20_trigger_conversion(&sensors[i]) == DS18B20_OK) {
            success_count++;
        }
    }
    
    return success_count;
}

int ds18b20_read_temperature(ds18b20_sensor_t *sensor, float *temperature) {
    uint8_t scratchpad[9];
    int16_t raw_temp;
    int retry_count = 0;
    
    if (sensor == NULL || temperature == NULL) {
        return DS18B20_ERROR_NO_DEVICE;
    }
    
    // Wait for conversion to complete
    vTaskDelay(pdMS_TO_TICKS(DS18B20_CONVERSION_TIME_MS));
    
    // Retry logic for CRC errors
    while (retry_count < MAX_READ_RETRIES) {
        // Reset and check presence
        if (ds18b20_reset(sensor->gpio_pin) != DS18B20_OK) {
            sensor->is_valid = false;
            return DS18B20_ERROR_NO_DEVICE;
        }
        
        // Skip ROM and read scratchpad
        ds18b20_write_byte(sensor->gpio_pin, DS18B20_SKIP_ROM);
        ds18b20_write_byte(sensor->gpio_pin, DS18B20_READ_SCRATCHPAD);
        
        // Read 9 bytes of scratchpad
        for (uint8_t i = 0; i < 9; i++) {
            scratchpad[i] = ds18b20_read_byte(sensor->gpio_pin);
        }
        
        // Verify CRC
        if (ds18b20_crc8(scratchpad, 8) == scratchpad[8]) {
            // CRC OK - break out of retry loop
            break;
        }
        
        retry_count++;
        if (retry_count < MAX_READ_RETRIES) {
            ESP_LOGW(TAG, "CRC error on GPIO %d -%s-, retry %d/%d", 
                    sensor->gpio_pin, sensor->name, retry_count, MAX_READ_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay before retry
        } else {
            ESP_LOGE(TAG, "CRC error on GPIO %d -%s- after %d retries", 
                    sensor->gpio_pin, sensor->name, MAX_READ_RETRIES);
            sensor->is_valid = false;
            return DS18B20_ERROR_CRC;
        }
    }
    
    // Calculate temperature
    raw_temp = (scratchpad[1] << 8) | scratchpad[0];
    *temperature = (float)raw_temp / 16.0f;
    
    // Check for power-on reset value (85°C)
    if (*temperature == 85.0f) {
        ESP_LOGW(TAG, "Invalid reading (85°C) on GPIO %d -%s-", sensor->gpio_pin, sensor->name);
        sensor->is_valid = false;
        return DS18B20_ERROR_TIMEOUT;
    }
    
    // Update sensor state
    sensor->last_temperature = *temperature;
    sensor->is_valid = true;
    
    return DS18B20_OK;
}

int ds18b20_read_all(ds18b20_sensor_t *sensors, uint8_t num_sensors, float *temperatures) {
    int success_count = 0;
    
    if (sensors == NULL || temperatures == NULL) {
        return 0;
    }
    
    // Trigger all conversions first
    ds18b20_trigger_all_conversions(sensors, num_sensors);
    
    // Wait for conversions to complete
    vTaskDelay(pdMS_TO_TICKS(DS18B20_CONVERSION_TIME_MS));
    
    // Read all sensors with retry logic
    for (uint8_t i = 0; i < num_sensors; i++) {
        int retry_count = 0;
        bool read_success = false;
        
        while (retry_count < MAX_READ_RETRIES && !read_success) {
            // Reset and check presence
            if (ds18b20_reset(sensors[i].gpio_pin) != DS18B20_OK) {
                temperatures[i] = 500.0f;  // Error value
                sensors[i].is_valid = false;
                break;
            }
            
            // Skip ROM and read scratchpad
            ds18b20_write_byte(sensors[i].gpio_pin, DS18B20_SKIP_ROM);
            ds18b20_write_byte(sensors[i].gpio_pin, DS18B20_READ_SCRATCHPAD);
            
            // Read scratchpad
            uint8_t scratchpad[9];
            for (uint8_t j = 0; j < 9; j++) {
                scratchpad[j] = ds18b20_read_byte(sensors[i].gpio_pin);
            }
            
            // Verify CRC
            if (ds18b20_crc8(scratchpad, 8) != scratchpad[8]) {
                retry_count++;
                if (retry_count < MAX_READ_RETRIES) {
                    ESP_LOGW(TAG, "CRC error on GPIO %d -%s-, retry %d/%d", 
                            sensors[i].gpio_pin, sensors[i].name, retry_count, MAX_READ_RETRIES);
                    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay before retry
                    continue;
                } else {
                    ESP_LOGE(TAG, "CRC error on GPIO %d -%s- after %d retries", 
                            sensors[i].gpio_pin, sensors[i].name, MAX_READ_RETRIES);
                    temperatures[i] = 500.0f;  // Error value
                    sensors[i].is_valid = false;
                    break;
                }
            }
            
            // Calculate temperature
            int16_t raw_temp = (scratchpad[1] << 8) | scratchpad[0];
            temperatures[i] = (float)raw_temp / 16.0f;
            
            // Check for power-on reset value
            if (temperatures[i] == 85.0f) {
                ESP_LOGW(TAG, "Invalid reading (85°C) on GPIO %d -%s-", sensors[i].gpio_pin, sensors[i].name);
                temperatures[i] = 500.0f;  // Error value
                sensors[i].is_valid = false;
                break;
            }
            
            // Success!
            sensors[i].last_temperature = temperatures[i];
            sensors[i].is_valid = true;
            success_count++;
            read_success = true;
        }
    }
    
    return success_count;
}