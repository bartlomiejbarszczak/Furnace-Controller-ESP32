#include "furnace_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

#define NVS_NAMESPACE "furnace"
#define NVS_CONFIG_KEY "config"

static const char *TAG = "CONFIG";

// Default configuration
const furnace_config_t default_config = {
    .temp_target_work = 60,
    .temp_ignition = 25,
    .temp_shutdown = 45,
    .temp_safe_after_overheat = 80,
    
    .temp_activ_pump_main = 45,
    .temp_activ_pump_main_hysteresis = 4,
    
    .temp_diff_furnace_boiler = 5,
    .temp_diff_boiler_vertical = 5,
    .boiler_top_priority_enabled = false,
    .temp_boiler_top_priority = 55,
    .boiler_mixing_in_shutdown = true,
    
    .pump_mixing_power_min = 30,
    .pump_mixing_power_max = 100,
    
    .blower_enabled = false,
    .blower_power_min = 30,
    .blower_power_max = 100,
    .time_blowthrough_sec = 60,
    .time_blowthrough_interval_min = 10,
    
    .pump_floor_enabled = false,
    
    .temp_correction_furnace = 0,
    .temp_correction_boiler_top = 0,
    .temp_correction_boiler_bottom = 0,
    
    .pin_pump_main = GPIO_NUM_25,
    .pin_pump_floor = GPIO_NUM_26,
    .pin_pump_mixing_control = GPIO_NUM_27,
    .pin_blower_control = GPIO_NUM_32,

    .pin_temp_sensor_furnace = GPIO_NUM_4,
    .pin_temp_sensor_boiler_top = GPIO_NUM_5,
    .pin_temp_sensor_boiler_bottom = GPIO_NUM_18,
    .pin_temp_sensor_main_output = GPIO_NUM_19,
};

// Save configuration to NVS
bool config_save_to_nvs(const furnace_config_t *config) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return false;
    }
    
    err = nvs_set_blob(nvs_handle, NVS_CONFIG_KEY, config, sizeof(furnace_config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write failed: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Configuration saved to NVS");
        return true;
    } else {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
        return false;
    }
}

// Load configuration from NVS
bool config_load_from_nvs(furnace_config_t *config) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed (first boot?), using default config");
        return false;
    }
    
    size_t required_size = sizeof(furnace_config_t);
    err = nvs_get_blob(nvs_handle, NVS_CONFIG_KEY, config, &required_size);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK && required_size == sizeof(furnace_config_t)) {
        ESP_LOGI(TAG, "Configuration loaded from NVS");
        return true;
    } else {
        ESP_LOGW(TAG, "NVS read failed: %s, using default config", esp_err_to_name(err));
        return false;
    }
}

// Convert state to string
const char* state_to_string(furnace_state_t state) {
    switch(state) {
        case STATE_IDLE: return "IDLE";
        case STATE_IGNITION: return "IGNITION";
        case STATE_WORK: return "WORK";
        case STATE_BLOWTHROUGH: return "BLOWTHROUGH";
        case STATE_SHUTDOWN: return "SHUTDOWN";
        case STATE_OVERHEAT: return "OVERHEAT";
        case STATE_COOLING: return "COOLING";
        default: return "UNKNOWN";
    }
}