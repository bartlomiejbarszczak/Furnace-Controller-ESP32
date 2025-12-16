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
    // Progi temperatury pieca [°C]
    int16_t temp_target_work;               // 1. Docelowa temperatura pieca (przejście do pracy) [40-80]
    int16_t temp_ignition;                  // 2. Temperatura rozpalania [20-35]
    int16_t temp_shutdown;                  // 3. Temperatura wygaszania [30-50]
    int16_t temp_safe_after_overheat;       // 4. Temperatura bezpieczna po przegrzaniu [60-80]
    
    // Główna pompa obiegowa
    int16_t temp_activ_pump_main;           // 5. Temperatura pracy głównej pompy [30-80]
    int16_t temp_activ_pump_main_hysteresis; // 6. Histereza pracy głównej pompy [2-10]
    
    // Bojler - mieszanie
    int16_t temp_diff_furnace_boiler;       // 7. Różnica temp piec-bojler do mieszania [3-10]
    int16_t temp_diff_boiler_vertical;      // 8. Różnica temp góra-dół bojlera [3-10]
    bool boiler_top_priority_enabled;       // 9. Priorytet grzania tylko góry zbiornika [true/false]
    int16_t temp_boiler_top_priority;       // 10. Temperatura priorytetu [30-70]
    bool boiler_mixing_in_shutdown;         // 11. Mieszanie bojlera podczas wygaszania [true/false]
    
    // Pompa mieszająca - moc [%]
    uint8_t pump_mixing_power_min;          // 12. Min moc pompy mieszającej [0-100%]
    uint8_t pump_mixing_power_max;          // 13. Max moc pompy mieszającej [0-100%]
    
    // Dmuchawa
    bool blower_enabled;                    // 14. Korzystanie z dmuchawy [true/false]
    uint8_t blower_power_min;               // 15. Min moc dmuchawy [20-100%]
    uint8_t blower_power_max;               // 16. Max moc dmuchawy [20-100%]
    uint16_t time_blowthrough_sec;          // 17. Czas trwania przedmuchu [10-180s]
    uint16_t time_blowthrough_interval_min; // 18. Przerwa między przedmuchami [0-30 min]
    
    // Podłogówka
    bool pump_floor_enabled;                // 19. Korzystanie z podłogówki [true/false]
    
    // Korekty temperatur [°C]
    int8_t temp_correction_furnace;         // 20. Korekta wskazania temp pieca [-5 do 5]
    int8_t temp_correction_boiler_top;      // 21. Korekta wskazania temp góry bojlera [-5 do 5]
    int8_t temp_correction_boiler_bottom;   // 22. Korekta wskazania temp dołu bojlera [-5 do 5]
    
    // GPIO
    gpio_num_t pin_pump_main;               // Pompa główna obiegowa - ON/OFF
    gpio_num_t pin_pump_floor;              // Pompa podłogówki - ON/OFF
    gpio_num_t pin_pump_mixing_control;     // Sterowanie triakiem pompy mieszającej
    gpio_num_t pin_blower_control;          // Sterowanie triakiem dmuchawy

    gpio_num_t pin_temp_sensor_furnace;         // Czujnik temperatury pieca
    gpio_num_t pin_temp_sensor_boiler_top;      // Czujnik temperatury góry bojlera
    gpio_num_t pin_temp_sensor_boiler_bottom;   // Czujnik temperatury dołu bojlera
    gpio_num_t pin_temp_sensor_main_output;     // Czujnik temperatury wody wyjściowej
    
} furnace_config_t;

// ============== RUNTIME STATE STRUCTURE ==============
typedef enum {
    STATE_IDLE,             // Spoczynek
    STATE_IGNITION,         // Rozpalanie
    STATE_WORK,             // Praca
    STATE_BLOWTHROUGH,      // Przedmuch
    STATE_SHUTDOWN,         // Wygaszanie
    STATE_OVERHEAT,         // Przegrzanie
    STATE_COOLING           // Ochłodzenie
} furnace_state_t;

#define BUFFER_SIZE 5  // Rozmiar bufora historii temperatur

typedef struct {
    furnace_state_t current_state;
    furnace_state_t previous_state;
    furnace_state_t state_before_overheat;  // Stan przed przegrzaniem (do powrotu)
    
    // Temperatury RAW (bez korekcji) [°C]
    int16_t temp_furnace_raw;
    int16_t temp_boiler_top_raw;
    int16_t temp_boiler_bottom_raw;
    int16_t temp_main_output_raw;
    
    // Temperatury SKORYGOWANE (z korektą) [°C]
    int16_t temp_furnace;
    int16_t temp_boiler_top;
    int16_t temp_boiler_bottom;
    int16_t temp_main_output;
    
    // Historia temperatur (do detekcji wzrostu)
    int16_t temp_furnace_history[BUFFER_SIZE];       // Ostatnie 5 pomiarów
    uint8_t temp_history_index;
    
    // Stany wykonawcze
    bool pump_main_on;
    bool pump_floor_on;
    uint8_t pump_mixing_power;
    uint8_t blower_power;
    
    // Timery
    uint32_t state_entry_time;
    uint32_t last_state_change;
    uint32_t last_blowthrough_time;
    uint32_t last_fsm_update_time;
    uint32_t last_cooling_alert_time;      // Ostatnie ostrzeżenie o ochłodzeniu (max co 30 min)
    
    // Flagi
    bool manual_shutdown_requested;
    bool manual_ignition_requested;
    bool error_flag;
    
} furnace_runtime_t;

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