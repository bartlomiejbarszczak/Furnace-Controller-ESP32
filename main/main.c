#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_sntp.h"

#include "furnace_config.h"
#include "mqtt_manager.h"
#include "sd_logger.h"
#include "ds18b20.h"
#include "wifi_manager.h"

// ============== DEFINES ==============
#define TEMP_OVERHEAT 95        // Overheat temperature threshold [°C]
#define TEMP_FREEZE_THRESHOLD 4 // Temperature freeze alert threshold [°C]
#define NUM_TEMP_SENSORS 4      // Number of DS18B20 temperature sensors
    
static const char *TAG = "FURNACE";

// ============== GLOBAL VARIABLES ==============
static furnace_config_t config;              /**< Furnace configuration structure */
static furnace_runtime_t runtime = { 0 };    /**< Runtime state and telemetry */
static SemaphoreHandle_t furnace_mutex;      /**< Mutex for thread-safe access to runtime */
static ds18b20_sensor_t temp_sensors[NUM_TEMP_SENSORS]; /**< Temperature sensor array */
static char wifi_ssid[32];                   /**< WiFi SSID buffer */
static char wifi_password[64];               /**< WiFi password buffer */

// ============== FUNCTION PROTOTYPES ==============
void set_pump_main(bool state);
void set_pump_floor(bool state);
void set_pump_mixing_power(uint8_t power);
void set_blower_power(uint8_t power);
uint32_t millis(void);
bool is_temp_rising(void);
void apply_temp_corrections(void);
uint8_t interpolate_linear(int16_t value, int16_t min_val, int16_t max_val, uint8_t min_power, uint8_t max_power);
void update_pump_main(void);
void update_pump_floor(void);
void update_pump_mixing(void);
void update_blower(void);
void fsm_update(void);
bool save_config_to_sd(const furnace_config_t *config);
bool load_config_from_sd(furnace_config_t *config);
bool save_wifi_config_to_sd(const char *ssid, const size_t ssid_len, const char *password, const size_t password_len);
bool load_wifi_config_from_sd(char *ssid, size_t ssid_size, char *password, size_t password_size);
void initialize_time(void);

/**
 * @brief Wrapper for saving configuration to both NVS and SD card
 * 
 * Used as callback to save configuration to multiple storage locations.
 * 
 * @return true if both NVS and SD saves succeed, false otherwise
 */
bool config_save_wrapper(void) {
    xSemaphoreTake(furnace_mutex, portMAX_DELAY);
    bool result_nvs = config_save_to_nvs(&config, &runtime.error_flag, &runtime.error_code);
    xSemaphoreGive(furnace_mutex);
    bool result_sd = save_config_to_sd(&config);

    return result_nvs && result_sd;
}


// ============== HELPER FUNCTIONS ==============

/**
 * @brief Apply temperature corrections to raw sensor readings
 * 
 * Applies configured correction offsets to raw temperature values from sensors.
 * This compensates for sensor calibration errors.
 */
void apply_temp_corrections(void) {
    runtime.temp_furnace = runtime.temp_furnace_raw + config.temp_correction_furnace;
    runtime.temp_boiler_top = runtime.temp_boiler_top_raw + config.temp_correction_boiler_top;
    runtime.temp_boiler_bottom = runtime.temp_boiler_bottom_raw + config.temp_correction_boiler_bottom;
    runtime.temp_main_output = runtime.temp_main_output_raw;
}

/**
 * @brief Detect if furnace temperature is rising
 * 
 * Analyzes the temperature history buffer to determine if furnace temperature
 * is rising. If the difference between max and min in the buffer exceeds 4°C,
 * temperature is considered rising.
 * 
 * @return true if temperature is rising, false otherwise
 */
bool is_temp_rising(void) {
    int16_t min_temp = runtime.temp_furnace_history[0];
    int16_t max_temp = runtime.temp_furnace_history[0];

    for (uint8_t i = 1; i < BUFFER_SIZE; i++) {
        int16_t temp = runtime.temp_furnace_history[i];
        if (temp < min_temp) min_temp = temp;
        if (temp > max_temp) max_temp = temp;
    }

    // If difference between max and min exceeds 4°C, consider temperature rising
    if ((max_temp - min_temp) >= 4) {
        ESP_LOGI(TAG, "Temperature rising detected (max: %d, min: %d)", max_temp, min_temp);
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Perform linear interpolation between two points
 * 
 * Maps a value from one range to another using linear interpolation.
 * Clamps results to ensure output stays within specified bounds.
 * 
 * @param value The input value to interpolate
 * @param min_val Minimum input value
 * @param max_val Maximum input value
 * @param min_power Minimum output power
 * @param max_power Maximum output power
 * @return Interpolated power value (0-100)
 */
uint8_t interpolate_linear(int16_t value, int16_t min_val, int16_t max_val, uint8_t min_power, uint8_t max_power) {
    if (value <= min_val) return min_power;
    if (value >= max_val) return max_power;
    
    int32_t power = min_power + ((value - min_val) * (max_power - min_power)) / (max_val - min_val);
    
    if (power < min_power) power = min_power;
    if (power > max_power) power = max_power;
    
    return (uint8_t)power;
}

/**
 * @brief Get elapsed time in milliseconds since FreeRTOS start
 * 
 * @return Current tick count converted to milliseconds
 */
uint32_t millis(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/**
 * @brief Save furnace configuration to SD card
 * 
 * Persists the current furnace configuration to the SD card.
 * Returns success if card is not available (assumes user doesn't have SD).
 * 
 * @param config Pointer to configuration structure to save
 * @return true if saved or SD not available, false if save failed
 */
bool save_config_to_sd(const furnace_config_t *config) {
    sd_logger_status_t sd_status = sd_logger_get_status();

    // If card is not mounted, return true (user may not have SD card)
    if (sd_status == SD_STATUS_NO_CARD) {
        ESP_LOGW(TAG, "Cannot save config to SD card - SD card not mounted");
        return true;
    }

    if (!sd_logger_is_ready()) {
        ESP_LOGW(TAG, "Cannot save config to SD card - SD card not ready");
        return false;
    }
    
    esp_err_t err = sd_logger_save_config(config, sizeof(furnace_config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save config to SD card: %s", esp_err_to_name(err));
        return false;
    } else {
        ESP_LOGI(TAG, "Configuration saved to SD card");
        return true;
    }
}

/**
 * @brief Load furnace configuration from SD card
 * 
 * Attempts to load configuration from SD card.
 * Returns false if SD card is not ready.
 * 
 * @param config Pointer to configuration structure to load into
 * @return true if successfully loaded, false otherwise
 */
bool load_config_from_sd(furnace_config_t *config) {
    if (!sd_logger_is_ready()) {
        ESP_LOGW(TAG, "Cannot load config form SD card - SD card not ready");
        return false;
    }
    
    esp_err_t err = sd_logger_load_config(config, sizeof(furnace_config_t));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Configuration loaded from SD card");
        return true;
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "No configuration file found on SD card");
    } else {
        ESP_LOGE(TAG, "Failed to load configuration");
    }
    return false;
}


/**
 * @brief Save WiFi credentials to SD card
 * 
 * Persists WiFi SSID and password to SD card for persistent storage.
 * 
 * @param ssid WiFi SSID string
 * @param ssid_len Length of SSID
 * @param password WiFi password string
 * @param password_len Length of password
 * @return true if saved, false if save failed
 */
bool save_wifi_config_to_sd(const char *ssid, const size_t ssid_len, const char *password, const size_t password_len) {
    sd_logger_status_t sd_status = sd_logger_get_status();

    // If card is not mounted, return to avoid false error due to no SD
    if (sd_status == SD_STATUS_NO_CARD) {
        ESP_LOGW(TAG, "Cannot save WiFi config to SD card - SD card not mounted");
        return false;
    }

    if (!sd_logger_is_ready()) {
        ESP_LOGW(TAG, "Cannot save WiFi config to SD card - SD card not ready");
        return false;
    }
    
    esp_err_t err = sd_logger_save_wifi_config(ssid, ssid_len, password, password_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save WiFi config to SD card: %s", esp_err_to_name(err));
        return false;
    } else {
        ESP_LOGI(TAG, "WiFi configuration saved to SD card");
        return true;
    }
}


/**
 * @brief Load WiFi credentials from SD card
 * 
 * Attempts to load WiFi SSID and password from SD card.
 * 
 * @param ssid Buffer to load SSID into
 * @param ssid_size Size of SSID buffer
 * @param password Buffer to load password into
 * @param password_size Size of password buffer
 * @return true if successfully loaded, false otherwise
 */
bool load_wifi_config_from_sd(char *ssid, size_t ssid_size, char *password, size_t password_size) {
    if (!sd_logger_is_ready()) {
        ESP_LOGW(TAG, "Cannot load WiFi config from SD card - SD card not ready");
        return false;
    }
    
    esp_err_t err = sd_logger_load_wifi_config(ssid, ssid_size, password, password_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi configuration loaded from SD card");
        return true;
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "No WiFi configuration file found on SD card");
    } else {
        ESP_LOGE(TAG, "Failed to load WiFi configuration from SD card: %s", esp_err_to_name(err));
    }
    return false;
}


/**
 * @brief Callback when SD card is inserted or removed
 * 
 * Handles initialization and cleanup of SD card logging when card presence changes.
 * Saves current configuration when card is inserted.
 * 
 * @param card_present true if card was inserted, false if removed
 */
void on_card_event(bool card_present) {
    if (card_present) {
        sd_logger_set_vprintf_handler(true);
        ESP_LOGI(TAG, "SD card inserted - logging enabled");
        
        esp_err_t err = sd_logger_save_config(&config, sizeof(furnace_config_t));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save config to SD card: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Configuration saved to SD card");
        }

        err = sd_logger_save_wifi_config(wifi_ssid, strlen(wifi_ssid), wifi_password, strlen(wifi_password));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save WiFi config to SD card: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "WiFi configuration saved to SD card");
        }

    } else {
        sd_logger_set_vprintf_handler(false);
        ESP_LOGW(TAG, "SD card removed - logging to serial only");
    }
}

/**
 * @brief Initialize system time via SNTP
 * 
 * Sets up SNTP client to synchronize system time with NTP servers.
 * Configures timezone for Warsaw (CET/CEST).
 * Blocks until time synchronization succeeds.
 */
void initialize_time(void) {
    ESP_LOGI(TAG, "Initializing SNTP for Warsaw...");

    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "tempus1.gum.gov.pl");
    esp_sntp_setservername(1, "pool.ntp.org");
    esp_sntp_init();

    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Time synchronization finished.");
}

// ============== PUMP CONTROL LOGIC ==============

/**
 * @brief Update main circulation pump state based on temperature
 * 
 * Implements hysteresis-based control logic:
 * - In overheat/cooling states: pump ON
 * - In work/blowthrough states: ON/OFF based on furnace temperature
 * - In other states: pump OFF
 */
void update_pump_main(void) {
    if (runtime.current_state == STATE_OVERHEAT) {
        // Overheat condition - enable all pumps
        set_pump_main(true);
        return;
    }
    
    if (runtime.current_state == STATE_COOLING) {
        // Cooling condition - enable main pump
        set_pump_main(true);
        return;
    }
    
    if (runtime.current_state == STATE_WORK || runtime.current_state == STATE_BLOWTHROUGH) {
        // Temperature-based control with hysteresis
        int16_t half_hysteresis = config.temp_activ_pump_main_hysteresis / 2;
        int16_t temp_on = config.temp_activ_pump_main + half_hysteresis;
        int16_t temp_off = config.temp_activ_pump_main - half_hysteresis;
        
        if (runtime.temp_furnace > temp_on) {
            set_pump_main(true);
        } else if (runtime.temp_furnace < temp_off) {
            set_pump_main(false);
        }
        // Between thresholds - maintain current state
        return;
    }
    
    // In other states - disable pump
    set_pump_main(false);
}

/**
 * @brief Update floor heating pump state
 * 
 * Floor pump only operates when main pump is running in work/blowthrough states.
 * Can be manually overridden at runtime.
 */
void update_pump_floor(void) {
    if (runtime.current_state == STATE_OVERHEAT) {
        // Overheat condition - enable all pumps
        set_pump_floor(true);
        return;
    }

    if (!config.pump_floor_enabled && !runtime.runtime_underfloor_pump_enabled) {
        set_pump_floor(false);
        return;
    }
    
    // Floor pump only operates when main pump is running
    if (runtime.current_state == STATE_WORK || runtime.current_state == STATE_BLOWTHROUGH) {
        set_pump_floor(runtime.pump_main_on);
        return;
    }
    
    set_pump_floor(false);
}

/**
 * @brief Update mixing pump power based on boiler temperature differential
 * 
 * Controls pump power through linear interpolation of boiler vertical temperature
 * difference. Active in shutdown, work, and blowthrough states.
 * Respects boiler top priority setting if enabled.
 */
void update_pump_mixing(void) {
    if (runtime.current_state == STATE_OVERHEAT) {
        // Overheat condition - enable all pumps at maximum power
        set_pump_mixing_power(config.pump_mixing_power_max);
        return;
    }
    
    // Active in shutdown, work, and blowthrough states
    if (runtime.current_state == STATE_SHUTDOWN ||
        runtime.current_state == STATE_WORK ||
        runtime.current_state == STATE_BLOWTHROUGH) {

        // Calculate boiler vertical temperature difference (top - bottom)
        int16_t diff_top_bottom = runtime.temp_boiler_top - runtime.temp_boiler_bottom;
        
        // Mixing conditions
        bool cond1 = runtime.temp_furnace > runtime.temp_boiler_top + config.temp_diff_furnace_boiler;
        bool cond2 = diff_top_bottom > config.temp_diff_boiler_vertical;
        
        // Check boiler top priority setting
        bool priority_ok = true;
        if (config.boiler_top_priority_enabled) {
            priority_ok = runtime.temp_boiler_top > config.temp_boiler_top_priority;
        }
        
        if (cond1 && cond2 && priority_ok) {
            // Power depends on boiler vertical temperature difference
            uint8_t power = interpolate_linear(
                diff_top_bottom,
                config.temp_diff_boiler_vertical,
                config.temp_diff_boiler_vertical * 2,
                config.pump_mixing_power_min,
                config.pump_mixing_power_max
            );
            set_pump_mixing_power(power);
        } else {
            set_pump_mixing_power(0);
        }
        return;
    }
    
    // In other states - disable pump
    set_pump_mixing_power(0);
}

/**
 * @brief Update blower fan power
 * 
 * Controls blower power based on furnace state:
 * - Ignition: maximum power
 * - Blowthrough: interpolated power (high to low over duration)
 * - Other states: off
 */
void update_blower(void) {
    if (!config.blower_enabled) {
        set_blower_power(0);
        return;
    }
    
    if (runtime.current_state == STATE_IGNITION) {
        // Ignition phase - maximum power
        set_blower_power(config.blower_power_max);
        return;
    }
    
    if (runtime.current_state == STATE_BLOWTHROUGH) {
        // Blowthrough phase - power tapers from high to low over time
        uint32_t time_in_state = millis() - runtime.state_entry_time;
        uint32_t total_time = config.time_blowthrough_sec * 1000;
        uint32_t remaining_time = total_time - time_in_state;
        
        if (remaining_time > total_time) remaining_time = total_time;
        
        // Start power: half of max (but at least min)
        // End power: min power
        uint8_t start_power = config.blower_power_max / 2;
        if (start_power < config.blower_power_min) {
            start_power = config.blower_power_min;
        }
        
        // Linear interpolation over time
        uint8_t power = interpolate_linear(
            remaining_time,
            0,
            total_time,
            config.blower_power_min,
            start_power
        );
        
        set_blower_power(power);
        return;
    }
    
    // In other states - disable blower
    set_blower_power(0);
}

// ============== FINITE STATE MACHINE ==============

/**
 * @brief Update furnace finite state machine
 * 
 * Implements the main control logic FSM. Processes state transitions based on:
 * - Temperature thresholds
 * - Manual user requests
 * - Time-based conditions
 * - Safety conditions (overheat detection)
 * 
 * States:
 * - IDLE: Furnace not working, just monitoring
 * - IGNITION: Starting furnace, reaching work temperature
 * - WORK: Normal operation with pumps and fans
 * - BLOWTHROUGH: Periodic ventilation cycle
 * - SHUTDOWN: Cooling down the furnace
 * - OVERHEAT: Safety state when temperature is too high
 * - COOLING: Emergency cooling when temperature is too low
 */
void fsm_update(void) {
    uint32_t time_in_state = millis() - runtime.state_entry_time;
    uint32_t time_since_last_blowthrough = millis() - runtime.last_blowthrough_time;
    furnace_state_t next_state = runtime.current_state;

    runtime.last_fsm_update_time = millis();
    
    // Global overheat safety check - high priority
    if (runtime.temp_furnace >= TEMP_OVERHEAT && runtime.current_state != STATE_OVERHEAT) {
        runtime.state_before_overheat = runtime.current_state;
        next_state = STATE_OVERHEAT;
        goto state_change;
    }
    
    switch (runtime.current_state) {
        
        case STATE_IDLE:
            // All actuators off
            set_blower_power(0);
            set_pump_main(false);
            set_pump_floor(false);
            set_pump_mixing_power(0);

            // Disable floor pump if it was manually enabled
            if (runtime.runtime_underfloor_pump_enabled) {
                runtime.runtime_underfloor_pump_enabled = false;
            }  
            
            // Transitions:
            // → IGNITION: temperature rising OR manual request OR temp > ignition threshold
            if (is_temp_rising() || 
                runtime.manual_ignition_requested || 
                runtime.temp_furnace > config.temp_ignition) {
                next_state = STATE_IGNITION;
                runtime.manual_ignition_requested = false;
            }
            // → COOLING: temp < 4°C (but not more than once per 30 minutes)
            else if (runtime.temp_furnace < TEMP_FREEZE_THRESHOLD) {
                uint32_t time_since_last_alert = millis() - runtime.last_cooling_alert_time;
                if (time_since_last_alert > (30 * 60 * 1000)) {
                    next_state = STATE_COOLING;
                    runtime.last_cooling_alert_time = millis();
                }
            }
            break;
            
        case STATE_IGNITION:
            // Ignition phase - full power if blower enabled
            update_blower();
            set_pump_main(false);
            set_pump_floor(false);
            set_pump_mixing_power(0);
            
            // Transitions:
            // → IDLE: timeout 30 minutes
            if (time_in_state > (30 * 60 * 1000)) {
                next_state = STATE_IDLE;
                runtime.error_flag = true;
                runtime.error_code |= FURNACE_ERROR_IGNITION_TIMEOUT;
                ESP_LOGE(TAG, "Ignition timeout!");
            }
            // → WORK: target work temperature reached
            else if (runtime.temp_furnace >= config.temp_target_work) {
                next_state = STATE_WORK;
                runtime.last_blowthrough_time = millis();
            }

            // → SHUTDOWN: manual shutdown request
            else if (runtime.manual_shutdown_requested) {
                next_state = STATE_SHUTDOWN;
                runtime.manual_shutdown_requested = false;
            }
            break;
            
        case STATE_WORK:
            // Blower off during normal work
            set_blower_power(0);
            
            // Pumps controlled based on temperature
            update_pump_main();
            update_pump_floor();
            update_pump_mixing();
            
            // Transitions:
            // → BLOWTHROUGH: periodic cycle (only if blower enabled and interval > 0)
            if (config.blower_enabled && config.time_blowthrough_interval_min > 0) {
                uint32_t interval_ms = config.time_blowthrough_interval_min * 60 * 1000;
                if (time_since_last_blowthrough > interval_ms) {
                    next_state = STATE_BLOWTHROUGH;
                }
            }
            // → SHUTDOWN: temp < (shutdown threshold - 1) OR manual shutdown
            if (runtime.temp_furnace < (config.temp_shutdown - 1) || 
                runtime.manual_shutdown_requested) {
                next_state = STATE_SHUTDOWN;
                runtime.manual_shutdown_requested = false;
            }
            break;
            
        case STATE_BLOWTHROUGH:
            // Blower with interpolated power
            update_blower();
            
            // Pumps operate same as in work state
            update_pump_main();
            update_pump_floor();
            update_pump_mixing();
            
            // Transitions:
            // → WORK: blowthrough duration expired
            if (time_in_state > (config.time_blowthrough_sec * 1000)) {
                next_state = STATE_WORK;
                runtime.last_blowthrough_time = millis();
            }
            break;
            
        case STATE_SHUTDOWN:
            // Only mixing pump can operate if enabled
            set_blower_power(0);
            set_pump_main(false);
            set_pump_floor(false);
            
            if (config.boiler_mixing_in_shutdown) {
                update_pump_mixing();
            } else {
                set_pump_mixing_power(0);
            }
              
            // Transitions:
            // → IDLE: temp < (ignition threshold - 1)
            if (runtime.temp_furnace < (config.temp_ignition - 1)) {
                next_state = STATE_IDLE;
            }
            // → IGNITION: temperature rising OR manual request
            else if (is_temp_rising() || runtime.manual_ignition_requested) {
                next_state = STATE_IGNITION;
                runtime.manual_ignition_requested = false;
            }
            break;
            
        case STATE_OVERHEAT:
            // Overheat state - enable all pumps, disable blower
            set_blower_power(0);
            set_pump_main(true);
            set_pump_floor(true);
            set_pump_mixing_power(config.pump_mixing_power_max);
            
            // Transition:
            // → PREVIOUS STATE: temp < safe temperature
            if (runtime.temp_furnace < config.temp_safe_after_overheat) {
                next_state = runtime.state_before_overheat;
                ESP_LOGI(TAG, "Overheat cleared, returning to %s", 
                        state_to_string(next_state));
            }
            break;
            
        case STATE_COOLING:
            // Cooling state - main pump only, enable cooling circulation
            set_blower_power(0);
            set_pump_main(true);
            set_pump_floor(false);
            set_pump_mixing_power(0);
            
            // Transitions:
            // → IDLE: timeout 10 minutes OR temp > 5°C
            if (time_in_state > (10 * 60 * 1000) || 
                runtime.temp_furnace > 5) {
                next_state = STATE_IDLE;
            }
            break;
    }

    // Additional overheat safety check based on output water temperature
    if (runtime.temp_main_output > TEMP_OVERHEAT - 5) {
        ESP_LOGW(TAG, "High output water temperature detected: %d°C", runtime.temp_main_output);
        next_state = STATE_OVERHEAT;
    }
    
state_change:
    // Handle state transition
    if (next_state != runtime.current_state) {
        runtime.previous_state = runtime.current_state;
        runtime.current_state = next_state;
        runtime.state_entry_time = millis();
        runtime.last_state_change = millis();
        
        ESP_LOGI(TAG, "FSM: %s -> %s (T=%d°C)", 
               state_to_string(runtime.previous_state),
               state_to_string(runtime.current_state),
               runtime.temp_furnace);
    }
}

// ============== FREERTOS TASKS ==============

/**
 * @brief Temperature sensor reading task (High priority)
 * 
 * Reads all DS18B20 temperature sensors every second.
 * - Invalid readings are replaced with last known value
 * - Applies temperature corrections
 * - Logs temperature values every 60 seconds
 * 
 * @param pvParameters Unused
 */
void sensor_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float temperatures[NUM_TEMP_SENSORS];
    int16_t print_temperatures[NUM_TEMP_SENSORS] = { 0 };
    int16_t last_temperatures[NUM_TEMP_SENSORS] = { 0 };
    uint32_t print_count = 0;
    
    while (1) {
        int success = ds18b20_read_all(temp_sensors, NUM_TEMP_SENSORS, temperatures);
        
        if (success < NUM_TEMP_SENSORS) {
            ESP_LOGW(TAG, "Only %d/%d sensors read successfully", success, NUM_TEMP_SENSORS);
            xSemaphoreTake(furnace_mutex, portMAX_DELAY);
            runtime.error_flag = true;
            runtime.error_flag |= FURNACE_ERROR_SENSOR_FAILURE;
            xSemaphoreGive(furnace_mutex);
        }

        xSemaphoreTake(furnace_mutex, portMAX_DELAY);
        
        // Read all DS18B20 sensors (RAW values)
        // Invalid readings use previous value
        runtime.temp_furnace_raw = temp_sensors[0].is_valid ? (int16_t)temperatures[0] : runtime.temp_furnace_raw;
        runtime.temp_boiler_top_raw = temp_sensors[1].is_valid ? (int16_t)temperatures[1] : runtime.temp_boiler_top_raw;
        runtime.temp_boiler_bottom_raw = temp_sensors[2].is_valid ? (int16_t)temperatures[2] : runtime.temp_boiler_bottom_raw;
        runtime.temp_main_output_raw = temp_sensors[3].is_valid ? (int16_t)temperatures[3] : runtime.temp_main_output_raw;
                
        // Apply temperature corrections
        apply_temp_corrections();

        if (print_count % 60 == 0) {
            print_temperatures[0] = runtime.temp_furnace;
            print_temperatures[1] = runtime.temp_boiler_top;
            print_temperatures[2] = runtime.temp_boiler_bottom;
            print_temperatures[3] = runtime.temp_main_output;
        }
            
        xSemaphoreGive(furnace_mutex);

        // Log temperatures every 60 seconds (every minute)
        if (print_count % 60 == 0) {
            ESP_LOGI(TAG, "Temperatures: Furnace=%d°C, Boiler Top=%d°C, Boiler Bottom=%d°C, Main Output=%d°C",
                     print_temperatures[0],
                     print_temperatures[1],
                     print_temperatures[2],
                     print_temperatures[3]);
        }



        print_count = (print_count + 1) % 60;
        
        // Read every 1 second
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Furnace temperature history update task (Medium priority)
 * 
 * Maintains a sliding buffer of furnace temperature readings sampled every minute.
 * Used by is_temp_rising() to detect temperature trends.
 * 
 * @param pvParameters Unused
 */
void temp_furnace_history_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds for temperature initialization
    for (int i = 0; i < BUFFER_SIZE; i++) {
        runtime.temp_furnace_history[i] = runtime.temp_furnace;
    }
    runtime.temp_history_index = 0;
    
    while (1) {
        // Update furnace temperature history every 1 minute
        xSemaphoreTake(furnace_mutex, portMAX_DELAY);
        
        runtime.temp_furnace_history[runtime.temp_history_index] = runtime.temp_furnace;
        runtime.temp_history_index = (runtime.temp_history_index + 1) % BUFFER_SIZE;
        
        xSemaphoreGive(furnace_mutex);
        
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(60 * 1000));
    }
}

/**
 * @brief Main furnace control FSM task (Highest priority)
 * 
 * Runs the finite state machine update at 500ms intervals.
 * Controls all furnace actuators based on current state and sensor inputs.
 * 
 * @param pvParameters Unused
 */
void control_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        xSemaphoreTake(furnace_mutex, portMAX_DELAY);
        fsm_update();
        xSemaphoreGive(furnace_mutex);
        
        // FSM update every 500ms
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

/**
 * @brief MQTT communication task (Low priority)
 * 
 * Publishes furnace status to MQTT broker every 5 seconds.
 * Handles MQTT broker discovery via mDNS and automatic reconnection.
 * Requires WiFi connection to be active.
 * 
 * @param pvParameters Unused
 */
void mqtt_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char topic[96];
    char payload[512];
    uint32_t send_count = 0;
    furnace_mqtt_status_t current_status = { 0 };
    furnace_mqtt_status_t last_status = { 0 };
    bool is_changed = false;
    
    ESP_LOGI(TAG, "MQTT Task started, waiting for WiFi...");
    
    // Wait for WiFi connection
    while (!wifi_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Initialize MQTT with retry loop (mDNS broker discovery)
    ESP_LOGI(TAG, "WiFi connected, searching for MQTT broker...");
    esp_err_t err;
    while ((err = mqtt_manager_init()) != ESP_OK) {
        ESP_LOGW(TAG, "MQTT init failed (broker not found), retrying in 10s...");
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        // Check if WiFi is still connected
        if (!wifi_is_connected()) {
            ESP_LOGW(TAG, "WiFi disconnected during MQTT init, waiting...");
            while (!wifi_is_connected()) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            ESP_LOGI(TAG, "WiFi reconnected, resuming MQTT broker search...");
        }
    }
    
    ESP_LOGI(TAG, "MQTT initialized successfully");
    
    while (1) {
        // Check if WiFi is connected
        if (!wifi_is_connected()) {
            ESP_LOGW(TAG, "WiFi not connected, waiting...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // Check if broker rediscovery is needed
        if (mqtt_manager_needs_rediscovery()) {
            ESP_LOGI(TAG, "Broker rediscovery needed, searching...");
            err = mqtt_manager_rediscover_broker();
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Broker rediscovered successfully");
            } else {
                ESP_LOGW(TAG, "Broker rediscovery failed, will retry in 10s");
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            continue;
        }
        
        // Check MQTT connection
        if (!mqtt_manager_is_connected()) {
            ESP_LOGW(TAG, "MQTT not connected, waiting...");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        xSemaphoreTake(furnace_mutex, portMAX_DELAY);

        current_status.state = runtime.current_state;
        current_status.temp_furnace = runtime.temp_furnace;
        current_status.temp_boiler_top = runtime.temp_boiler_top;
        current_status.temp_boiler_bottom = runtime.temp_boiler_bottom;
        current_status.temp_main_output = runtime.temp_main_output;
        current_status.pump_main_on = runtime.pump_main_on;
        current_status.pump_floor_on = runtime.pump_floor_on;
        current_status.pump_mixing_power = runtime.pump_mixing_power;
        current_status.blower_power = runtime.blower_power;
        current_status.error_flag = runtime.error_flag;
        current_status.error_code = runtime.error_code;

        xSemaphoreGive(furnace_mutex);

        // Compare current status with last status
        if (memcmp(&current_status, &last_status, sizeof(furnace_mqtt_status_t)) != 0) {
            memcpy(&last_status, &current_status, sizeof(furnace_mqtt_status_t));
            is_changed = true;
            
            ESP_LOGI(TAG, "Runtime Status Change Detected");
        }
        
        if (is_changed) {
            is_changed = false;

             // Publish furnace status
            int msg_id = mqtt_manager_publish_status(topic, payload, sizeof(topic), sizeof(payload));

            if (msg_id >= 0) {
                if (send_count++ % 600 == 0) {
                    ESP_LOGI(TAG, "Published: msg_id=%d", msg_id);
                }
            } else {
                ESP_LOGE(TAG, "MQTT Publish failed!");
            }
        }

        // Update every 100 milliseconds
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Watchdog task (Critical)
 * 
 * Monitors FSM execution health. Triggers system restart if the FSM doesn't
 * update within 30 seconds, indicating a system hang or deadlock.
 * Runs every 2 seconds for continuous monitoring.
 * 
 * @param pvParameters Unused
 */
void watchdog_task(void *pvParameters) {
    while (1) {
        xSemaphoreTake(furnace_mutex, portMAX_DELAY);
        uint32_t current = runtime.current_state;
        uint32_t last_fsm_update = runtime.last_fsm_update_time;
        xSemaphoreGive(furnace_mutex);
        
        uint32_t time_without_change = millis() - last_fsm_update;
        const uint32_t max_time_in_state = 30 * 1000;

        if (time_without_change > max_time_in_state) {
            ESP_LOGE(TAG, "WATCHDOG: FSM did not update within 30 seconds! Last state: %s. Restarting now!", state_to_string(current));
            esp_restart();
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000)); // Check every 2 seconds
    }
}


// ============== CONTROL FUNCTIONS ==============

/**
 * @brief Set main circulation pump state
 * 
 * Controls main pump GPIO and logs state changes.
 * 
 * @param state true to turn pump on, false to turn off
 */
void set_pump_main(bool state) {
    if (runtime.pump_main_on != state) {
        gpio_set_level(config.pin_pump_main, state);
        runtime.pump_main_on = state;
        ESP_LOGI(TAG, "Pump Main: %s", state ? "ON" : "OFF");
    }
}

/**
 * @brief Set floor heating pump state
 * 
 * Controls floor pump GPIO and logs state changes.
 * 
 * @param state true to turn pump on, false to turn off
 */
void set_pump_floor(bool state) {
    if (runtime.pump_floor_on != state) {
        gpio_set_level(config.pin_pump_floor, state);
        runtime.pump_floor_on = state;
        ESP_LOGI(TAG, "Pump Floor: %s", state ? "ON" : "OFF");
    }
}

/**
 * @brief Set mixing pump power level
 * 
 * Controls mixing pump TRIAC power for boiler mixing control.
 * Power is clamped to 0-100%.
 * 
 * @param power Power level (0-100%)
 */
void set_pump_mixing_power(uint8_t power) {
    if (power > 100) power = 100;
    
    if (runtime.pump_mixing_power != power) {
        runtime.pump_mixing_power = power;
        
        // TODO: Implement TRIAC control
        //  triac_set_power(config.pin_pump_mixing_control, power);
        
        ESP_LOGI(TAG, "Pump Mixing Power: %d%%", power);
    }
}

/**
 * @brief Set blower fan power level
 * 
 * Controls blower fan TRIAC power for ignition and blowthrough phases.
 * Power is clamped to 0-100%.
 * 
 * @param power Power level (0-100%)
 */
void set_blower_power(uint8_t power) {
    if (power > 100) power = 100;
    
    if (runtime.blower_power != power) {
        runtime.blower_power = power;
        
        // TODO: Implement TRIAC control
        //  triac_set_power(config.pin_blower_control, power);
        
        ESP_LOGI(TAG, "Blower Power: %d%%", power);
    }
}

// ============== MAIN ENTRY POINT ==============

/**
 * @brief Main application entry point
 * 
 * Initializes all system components in sequence:
 * 1. Non-Volatile Storage (NVS) for persistent configuration
 * 2. SD card logger with hot-plug detection
 * 3. Configuration loading (SD card, NVS, or defaults)
 * 4. WiFi connection and credentials
 * 5. System time synchronization (SNTP)
 * 6. GPIO initialization for pumps and TRIAC controls
 * 7. Temperature sensor initialization (DS18B20)
 * 8. MQTT manager configuration
 * 9. FreeRTOS task creation (sensor, control, communication, watchdog)
 * 
 * Blocks initialization if critical components fail (NVS, WiFi, sensors).
 */
void app_main(void) {
    ESP_LOGI(TAG, "=== Furnace Controller Starting ===");
    
    // Initialize Non-Volatile Storage (NVS) for configuration persistence
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS erasing and reinitializing...");
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS!");
        esp_restart();
    }

    // Initialize SD card logger with hot-plug detection
    sd_logger_config_t sd_config = {
        .cd_pin = GPIO_NUM_33,
        .enable_card_detect = true,
        .flush_interval_ms = 30000,
        .max_file_size_bytes = 10 * 1024 * 1024,
        .enable_log_rotation = true,    
        .log_to_serial = true
    };

    err = sd_logger_init(&sd_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD Logger initialization failed! Error: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "SD Logger initialized successfully");
    }

    sd_logger_register_card_event_callback(on_card_event);

    // Create FreeRTOS synchronization mutex
    furnace_mutex = xSemaphoreCreateMutex();
    if (furnace_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex!");
        esp_restart();
    }

    // Load configuration from SD card, NVS, or use defaults
    if (!load_config_from_sd(&config)) {
        xSemaphoreTake(furnace_mutex, portMAX_DELAY);

        esp_err_t err = config_load_from_nvs(&config, &runtime.error_flag, &runtime.error_code);
        if (err != ESP_OK) {
            memcpy(&config, &default_config, sizeof(furnace_config_t));
            config_save_to_nvs(&config, &runtime.error_flag, &runtime.error_code);
            sd_logger_save_config(&config, sizeof(furnace_config_t));
            
        }

        xSemaphoreGive(furnace_mutex);
    }

    

    // Display current configuration
    ESP_LOGI(TAG, "=== Configuration ===");
    ESP_LOGI(TAG, "Target work temp: %d°C", config.temp_target_work);
    ESP_LOGI(TAG, "Ignition temp: %d°C", config.temp_ignition);
    ESP_LOGI(TAG, "Shutdown temp: %d°C", config.temp_shutdown);
    ESP_LOGI(TAG, "Main pump activation: %d°C ±%d°C", 
           config.temp_activ_pump_main, config.temp_activ_pump_main_hysteresis);
    ESP_LOGI(TAG, "Boiler top priority: %s (temp=%d°C)",
           config.boiler_top_priority_enabled ? "ENABLED" : "DISABLED",
           config.temp_boiler_top_priority);
    ESP_LOGI(TAG, "Blower: %s (power %d-%d%%)", 
           config.blower_enabled ? "ENABLED" : "DISABLED",
           config.blower_power_min, config.blower_power_max);
    ESP_LOGI(TAG, "Blowthrough: %ds every %d min", 
           config.time_blowthrough_sec, config.time_blowthrough_interval_min);
    ESP_LOGI(TAG, "Floor pump: %s", config.pump_floor_enabled ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "Temp corrections: Furnace=%d°C, Boiler_top=%d°C, Boiler_bot=%d°C",
           config.temp_correction_furnace, 
           config.temp_correction_boiler_top,
           config.temp_correction_boiler_bottom);

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    
    // Load WiFi credentials from SD card or NVS
    if (!load_wifi_config_from_sd(wifi_ssid, sizeof(wifi_ssid), wifi_password, sizeof(wifi_password))) {
        // If not on SD card, try loading from NVS
        err = wifi_load_credentials(wifi_ssid, wifi_password);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Loaded WiFi credentials: SSID='%s'", wifi_ssid);
            if (save_wifi_config_to_sd(wifi_ssid, strlen(wifi_ssid), wifi_password, strlen(wifi_password))) {
                ESP_LOGI(TAG, "WiFi credentials loaded from NVS saved to SD card");
            } else {
                ESP_LOGW(TAG, "Failed to save loaded from NVS WiFi credentials to SD card");
            }
        } else {
            ESP_LOGW(TAG, "No WiFi credentials found");
            vTaskDelay(pdMS_TO_TICKS(10000000));
            //TODO: Start AP mode for configuration
        }
    }

    err = wifi_init_sta_with_credentials(wifi_ssid, wifi_password);
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "WiFi initialization failed, restarting");
        esp_restart();
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed, will retry in background. Error: %s", esp_err_to_name(err));
    }

    if (sd_logger_is_ready()) {
        sd_logger_set_vprintf_handler(true);
        ESP_LOGI(TAG, "SD card logging enabled");
    }
    
    // Initialize system time synchronization (SNTP)
    initialize_time();

    // Initialize GPIO pins for pump relays (ON/OFF)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config.pin_pump_main) | (1ULL << config.pin_pump_floor),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Initialize GPIO pins for TRIAC power control
    gpio_config_t triac_conf = {
        .pin_bit_mask = (1ULL << config.pin_pump_mixing_control) | (1ULL << config.pin_blower_control),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&triac_conf);
    
    // TODO: Initialize TRIAC control drivers
    // triac_init(config.pin_pump_mixing_control);
    // triac_init(config.pin_blower_control);
    
    // Initialize DS18B20 temperature sensors
    temp_sensors[0].gpio_pin = config.pin_temp_sensor_furnace;
    temp_sensors[0].name = "Furnace";

    temp_sensors[1].gpio_pin = config.pin_temp_sensor_boiler_top;
    temp_sensors[1].name = "Boiler Top";

    temp_sensors[2].gpio_pin = config.pin_temp_sensor_boiler_bottom;
    temp_sensors[2].name = "Boiler Bottom";

    temp_sensors[3].gpio_pin = config.pin_temp_sensor_main_output;
    temp_sensors[3].name = "Main Output";

    err = ds18b20_init(temp_sensors, NUM_TEMP_SENSORS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DS18B20 sensors!");
    }
    
    // Initialize runtime state variables
    runtime.current_state = STATE_IDLE;
    runtime.state_entry_time = millis();
    runtime.last_blowthrough_time = millis();
    runtime.last_fsm_update_time = millis();
    runtime.last_cooling_alert_time = 0;
    runtime.temp_history_index = 0;
    
    // Initialize temperature history buffer to zero
    for (int i = 0; i < BUFFER_SIZE; i++) {
        runtime.temp_furnace_history[i] = 0;
    }
    
    // Configure MQTT manager with data sources and callbacks
    mqtt_manager_set_data_sources(&config, &runtime, furnace_mutex);
    mqtt_manager_set_wifi_callbacks(wifi_is_connected, wifi_get_rssi);
    mqtt_manager_set_config_save_callback(config_save_wrapper);
    
    ESP_LOGI(TAG, "Creating FreeRTOS tasks...");
    
    // Create FreeRTOS tasks
    BaseType_t result;
    
    result = xTaskCreatePinnedToCore(sensor_task, "Sensors", 4096, NULL, 8, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Sensors task! Error: %s", esp_err_to_name(err));
        esp_restart();
    }

    result = xTaskCreatePinnedToCore(temp_furnace_history_task, "FurnaceHist", 2048, NULL, 5, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Furnace Temperature History task! Error: %s", esp_err_to_name(err));
        esp_restart();
    }
    
    result = xTaskCreatePinnedToCore(control_task, "Control", 4096, NULL, 10, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Control task! Error: %s", esp_err_to_name(err));
        esp_restart();
    }
    
    result = xTaskCreatePinnedToCore(mqtt_task, "MQTT", 8192, NULL, 3, NULL, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MQTT task! Error: %s", esp_err_to_name(err));
        esp_restart();
    }
    
    result = xTaskCreatePinnedToCore(watchdog_task, "Watchdog", 2048, NULL, 12, NULL, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Watchdog task! Error: %s", esp_err_to_name(err));
        esp_restart();
    }
    
    ESP_LOGI(TAG, "=== System Ready ===");
}