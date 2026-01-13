#ifndef FURNACE_CONFIG_H
#define FURNACE_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============== CONFIGURATION STRUCTURE ==============
typedef struct {
    // Furnace temperature thresholds [°C]
    int16_t temp_target_work;               // 1. Target work temperature [40-80]
    int16_t temp_ignition;                  // 2. Ignition temperature [20-35]
    int16_t temp_shutdown;                  // 3. Shutdown temperature [30-50]
    int16_t temp_safe_after_overheat;       // 4. Safe temperature after overheat [60-80]
    
    // Main circulation pump
    int16_t temp_activ_pump_main;           // 5. Pump activation temperature [30-80]
    int16_t temp_activ_pump_main_hysteresis; // 6. Pump activation hysteresis [2-10]
    
    // Boiler - mixing
    int16_t temp_diff_furnace_boiler;       // 7. Furnace-boiler temp diff for mixing [3-10]
    int16_t temp_diff_boiler_vertical;      // 8. Boiler top-bottom temp diff [3-10]
    bool boiler_top_priority_enabled;       // 9. Boiler top priority enabled [true/false]
    int16_t temp_boiler_top_priority;       // 10. Boiler top priority temperature [30-70]
    bool boiler_mixing_in_shutdown;         // 11. Boiler mixing during shutdown [true/false]
    
    // Mixing pump - power [%]
    uint8_t pump_mixing_power_min;          // 12. Min mixing pump power [0-100%]
    uint8_t pump_mixing_power_max;          // 13. Max mixing pump power [0-100%]
    
    // Blower
    bool blower_enabled;                    // 14. Blower enabled [true/false]
    uint8_t blower_power_min;               // 15. Min blower power [20-100%]
    uint8_t blower_power_max;               // 16. Max blower power [20-100%]
    uint16_t time_blowthrough_sec;          // 17. Blowthrough duration [10-180s]
    uint16_t time_blowthrough_interval_min; // 18. Blowthrough interval [0-30 min]
    
    // Floor heating
    bool pump_floor_enabled;                // 19. Floor pump enabled [true/false]
    
    // Temperature corrections [°C]
    int8_t temp_correction_furnace;         // 20. Furnace temp correction [-5 to 5]
    int8_t temp_correction_boiler_top;      // 21. Boiler top temp correction [-5 to 5]
    int8_t temp_correction_boiler_bottom;   // 22. Boiler bottom temp correction [-5 to 5]
    
    // GPIO pins
    gpio_num_t pin_pump_main;               // Main pump - ON/OFF
    gpio_num_t pin_pump_floor;              // Floor pump - ON/OFF
    gpio_num_t pin_pump_mixing_control;     // Mixing pump TRIAC control
    gpio_num_t pin_blower_control;          // Blower TRIAC control

    gpio_num_t pin_temp_sensor_furnace;         // Furnace temperature sensor
    gpio_num_t pin_temp_sensor_boiler_top;      // Boiler top temperature sensor
    gpio_num_t pin_temp_sensor_boiler_bottom;   // Boiler bottom temperature sensor
    gpio_num_t pin_temp_sensor_main_output;     // Output water temperature sensor
    
} furnace_config_t;

// ============== RUNTIME STATE STRUCTURE ==============
typedef enum {
    STATE_IDLE,             // Idle state
    STATE_IGNITION,         // Ignition phase
    STATE_WORK,             // Normal operation
    STATE_BLOWTHROUGH,      // Blowthrough ventilation cycle
    STATE_SHUTDOWN,         // Shutdown phase
    STATE_OVERHEAT,         // Overheat safety state
    STATE_COOLING           // Emergency cooling state
} furnace_state_t;

#define BUFFER_SIZE 5  // Temperature history buffer size

typedef struct {
    furnace_state_t current_state;
    furnace_state_t previous_state;
    furnace_state_t state_before_overheat;  // State before overheat (for recovery)
    
    // Raw sensor readings [°C] (without corrections)
    int16_t temp_furnace_raw;
    int16_t temp_boiler_top_raw;
    int16_t temp_boiler_bottom_raw;
    int16_t temp_main_output_raw;
    
    // Corrected readings [°C] (with applied corrections)
    int16_t temp_furnace;
    int16_t temp_boiler_top;
    int16_t temp_boiler_bottom;
    int16_t temp_main_output;
    
    // Temperature history (for rise detection)
    int16_t temp_furnace_history[BUFFER_SIZE];  // Last 5 readings
    uint8_t temp_history_index;
    
    // Actuator states
    bool pump_main_on;
    bool pump_floor_on;
    uint8_t pump_mixing_power;
    uint8_t blower_power;
    
    // Timers
    uint32_t state_entry_time;
    uint32_t last_state_change;
    uint32_t last_blowthrough_time;
    uint32_t last_fsm_update_time;
    uint32_t last_cooling_alert_time;      // Last cooling alert (max once per 30 min)
    
    // Control flags
    bool manual_shutdown_requested;
    bool manual_ignition_requested;
    bool runtime_underfloor_pump_enabled;
    bool error_flag;
    bool is_changed; 
    
} furnace_runtime_t;


typedef struct {
    bool pump_main_on;
    bool pump_floor_on;
    bool error_flag;
    uint8_t pump_mixing_power;
    uint8_t blower_power;
    
    int16_t temp_furnace;
    int16_t temp_boiler_top;
    int16_t temp_boiler_bottom;
    int16_t temp_main_output;
    
    furnace_state_t state;
    
} furnace_mqtt_status_t;

// ============== DEFAULT CONFIGURATION ==============
extern const furnace_config_t default_config;

// ============== FUNCTION PROTOTYPES ==============

/**
 * @brief Save configuration to NVS
 * @param config Pointer to configuration structure
 * @return true if successful, false otherwise
 */
bool config_save_to_nvs(const furnace_config_t *config);

/**
 * @brief Load configuration from NVS
 * @param config Pointer to configuration structure to load into
 * @return true if successful, false otherwise
 */
bool config_load_from_nvs(furnace_config_t *config);

/**
 * @brief Convert furnace state to string
 * @param state State to convert
 * @return String representation of state
 */
const char* state_to_string(furnace_state_t state);

#ifdef __cplusplus
}
#endif

#endif // FURNACE_CONFIG_H