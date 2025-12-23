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

// ============== DEFINICJE ==============
#define TEMP_OVERHEAT 95        // Próg temperatury przegrzania
#define TEMP_FREEZE_THRESHOLD 4 // Próg temperatury do alertu ochłodzenia
#define NUM_TEMP_SENSORS 4      // Liczba czujników DS18B20
    
static const char *TAG = "FURNACE";

// ============== GLOBALNE ==============
static furnace_config_t config;
static furnace_runtime_t runtime = {0};
static SemaphoreHandle_t furnace_mutex;
static ds18b20_sensor_t temp_sensors[NUM_TEMP_SENSORS];
static char wifi_ssid[32];
static char wifi_password[64];

// ============== PROTOTYPY ==============
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

// Wrapper dla config_save_to_nvs (do przekazania callbacku)
bool config_save_wrapper(void) {
    bool result_nvs = config_save_to_nvs(&config);
    bool result_sd = save_config_to_sd(&config);

    return result_nvs && result_sd;
}


// ============== FUNKCJE POMOCNICZE ==============

// Zastosuj korekty temperatur
void apply_temp_corrections(void) {
    runtime.temp_furnace = runtime.temp_furnace_raw + config.temp_correction_furnace;
    runtime.temp_boiler_top = runtime.temp_boiler_top_raw + config.temp_correction_boiler_top;
    runtime.temp_boiler_bottom = runtime.temp_boiler_bottom_raw + config.temp_correction_boiler_bottom;
    runtime.temp_main_output = runtime.temp_main_output_raw;
}

// Sprawdź czy temperatura pieca rośnie (analiza różnicy max-min w buforze)
bool is_temp_rising(void) {
    int16_t min_temp = runtime.temp_furnace_history[0];
    int16_t max_temp = runtime.temp_furnace_history[0];

    for (uint8_t i = 1; i < BUFFER_SIZE; i++) {
        int16_t temp = runtime.temp_furnace_history[i];
        if (temp < min_temp) min_temp = temp;
        if (temp > max_temp) max_temp = temp;
    }

    // Jeśli różnica między max a min jest większa niż 4°C, uznajemy że temp rośnie
    if ((max_temp - min_temp) >= 4) {
        return true;
    } else {
        return false;
    }
}

// Interpolacja liniowa
uint8_t interpolate_linear(int16_t value, int16_t min_val, int16_t max_val, uint8_t min_power, uint8_t max_power) {
    if (value <= min_val) return min_power;
    if (value >= max_val) return max_power;
    
    int32_t power = min_power + ((value - min_val) * (max_power - min_power)) / (max_val - min_val);
    
    if (power < min_power) power = min_power;
    if (power > max_power) power = max_power;
    
    return (uint8_t)power;
}

uint32_t millis(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

bool save_config_to_sd(const furnace_config_t *config) {
    sd_logger_status_t sd_status = sd_logger_get_status();

    // If card is not mounted, return true to avoid false error due to no SD
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

// ============== LOGIKA POMP ==============

// Logika pompy głównej obiegowej
void update_pump_main(void) {
    if (runtime.current_state == STATE_OVERHEAT) {
        // Przegrzanie - wszystkie pompy ON
        set_pump_main(true);
        return;
    }
    
    if (runtime.current_state == STATE_COOLING) {
        // Ochłodzenie - główna pompa ON
        set_pump_main(true);
        return;
    }
    
    if (runtime.current_state == STATE_WORK || runtime.current_state == STATE_BLOWTHROUGH) {
        // Sterowanie z histerezą
        int16_t half_hysteresis = config.temp_activ_pump_main_hysteresis / 2;
        int16_t temp_on = config.temp_activ_pump_main + half_hysteresis;
        int16_t temp_off = config.temp_activ_pump_main - half_hysteresis;
        
        if (runtime.temp_furnace > temp_on) {
            set_pump_main(true);
        } else if (runtime.temp_furnace < temp_off) {
            set_pump_main(false);
        }
        // Pomiędzy - zachowaj poprzedni stan
        return;
    }
    
    // W pozostałych stanach - wyłącz
    set_pump_main(false);
}

// Logika pompy podłogówki
void update_pump_floor(void) {
    if (runtime.current_state == STATE_OVERHEAT) {
        // Przegrzanie - wszystkie pompy ON
        set_pump_floor(true);
        return;
    }

    if (!config.pump_floor_enabled) {
        set_pump_floor(false);
        return;
    }
    
    // Podłogówka działa gdy główna pompa działa
    if (runtime.current_state == STATE_WORK || runtime.current_state == STATE_BLOWTHROUGH) {
        set_pump_floor(runtime.pump_main_on);
        return;
    }
    
    set_pump_floor(false);
}

// Logika pompy mieszającej
void update_pump_mixing(void) {
    if (runtime.current_state == STATE_OVERHEAT) {
        // Przegrzanie - wszystkie pompy ON na max
        set_pump_mixing_power(config.pump_mixing_power_max);
        return;
    }
    
    // Lista stanów, w których działa pompa mieszająca
    if (runtime.current_state == STATE_SHUTDOWN ||
        runtime.current_state == STATE_WORK ||
        runtime.current_state == STATE_BLOWTHROUGH) {

        // Oblicz różnicę temperatur góra-dół
        int16_t diff_top_bottom = runtime.temp_boiler_top - runtime.temp_boiler_bottom;
        
        // Warunki do mieszania
        bool cond1 = runtime.temp_furnace > runtime.temp_boiler_top + config.temp_diff_furnace_boiler;
        bool cond2 = diff_top_bottom > config.temp_diff_boiler_vertical;
        
        // Sprawdź priorytet góry bojlera
        bool priority_ok = true;
        if (config.boiler_top_priority_enabled) {
            priority_ok = runtime.temp_boiler_top > config.temp_boiler_top_priority;
        }
        
        if (cond1 && cond2 && priority_ok) {
            // Moc zależna od różnicy góra-dół
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
    
    // W pozostałych stanach - wyłącz
    set_pump_mixing_power(0);
}

// Logika dmuchawy
void update_blower(void) {
    if (!config.blower_enabled) {
        set_blower_power(0);
        return;
    }
    
    if (runtime.current_state == STATE_IGNITION) {
        // Rozpalanie - max moc
        set_blower_power(config.blower_power_max);
        return;
    }
    
    if (runtime.current_state == STATE_BLOWTHROUGH) {
        // Przedmuch - moc zależna od czasu (interpolacja od połowy max do min)
        uint32_t time_in_state = millis() - runtime.state_entry_time;
        uint32_t total_time = config.time_blowthrough_sec * 1000;
        uint32_t remaining_time = total_time - time_in_state;
        
        if (remaining_time > total_time) remaining_time = total_time;
        
        // Na początku: połowa max (ale nie mniej niż min)
        // Na końcu: min moc
        uint8_t start_power = config.blower_power_max / 2;
        if (start_power < config.blower_power_min) {
            start_power = config.blower_power_min;
        }
        
        // Interpolacja liniowa w czasie
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
    
    // W pozostałych stanach - wyłącz
    set_blower_power(0);
}

// ============== MASZYNA STANÓW ==============
void fsm_update(void) {
    uint32_t time_in_state = millis() - runtime.state_entry_time;
    uint32_t time_since_last_blowthrough = millis() - runtime.last_blowthrough_time;
    furnace_state_t next_state = runtime.current_state;

    runtime.last_fsm_update_time = millis();
    
    // Globalny warunek przegrzania
    if (runtime.temp_furnace >= TEMP_OVERHEAT && runtime.current_state != STATE_OVERHEAT) {
        runtime.state_before_overheat = runtime.current_state;
        next_state = STATE_OVERHEAT;
        goto state_change;
    }
    
    switch (runtime.current_state) {
        
        case STATE_IDLE:
            // Wszystko wyłączone
            set_blower_power(0);
            set_pump_main(false);
            set_pump_floor(false);
            set_pump_mixing_power(0);
            
            // Przejścia
            // → Rozpalanie: temp rośnie LUB user kliknął LUB temp > temp_ignition
            if (is_temp_rising() || 
                runtime.manual_ignition_requested || 
                runtime.temp_furnace > config.temp_ignition) {
                next_state = STATE_IGNITION;
                runtime.manual_ignition_requested = false;
            }
            // → Ochłodzenie: temp < 4°C (ale nie częściej niż co 30 min)
            else if (runtime.temp_furnace < TEMP_FREEZE_THRESHOLD) {
                uint32_t time_since_last_alert = millis() - runtime.last_cooling_alert_time;
                if (time_since_last_alert > (30 * 60 * 1000)) {
                    next_state = STATE_COOLING;
                    runtime.last_cooling_alert_time = millis();
                }
            }
            break;
            
        case STATE_IGNITION:
            update_blower();  // Max moc jeśli enabled
            set_pump_main(false);
            set_pump_floor(false);
            set_pump_mixing_power(0);
            
            // Przejścia
            // → Spoczynek: timeout 15 minut
            if (time_in_state > (15 * 60 * 1000)) {
                next_state = STATE_IDLE;
                runtime.error_flag = true;
                ESP_LOGE(TAG, "Ignition timeout!");
            }
            // → Praca: osiągnięto docelową temperaturę
            else if (runtime.temp_furnace >= config.temp_target_work) {
                next_state = STATE_WORK;
                runtime.last_blowthrough_time = millis();
            }
            break;
            
        case STATE_WORK:
            // Dmuchawa wyłączona w pracy
            set_blower_power(0);
            
            // Pompy sterowane
            update_pump_main();
            update_pump_floor();
            update_pump_mixing();
            
            // Przejścia
            // → Przedmuch: cykliczne (tylko jeśli dmuchawa enabled i interval > 0)
            if (config.blower_enabled && config.time_blowthrough_interval_min > 0) {
                uint32_t interval_ms = config.time_blowthrough_interval_min * 60 * 1000;
                if (time_since_last_blowthrough > interval_ms) {
                    next_state = STATE_BLOWTHROUGH;
                }
            }
            // → Wygaszanie: temp < (temp_shutdown - 1) LUB manual shutdown
            if (runtime.temp_furnace < (config.temp_shutdown - 1) || 
                runtime.manual_shutdown_requested) {
                next_state = STATE_SHUTDOWN;
                runtime.manual_shutdown_requested = false;
            }
            break;
            
        case STATE_BLOWTHROUGH:
            update_blower();  // Interpolowana moc
            
            // Pompy działają jak w pracy
            update_pump_main();
            update_pump_floor();
            update_pump_mixing();
            
            // Przejścia
            // → Praca: czas przedmuchu minął
            if (time_in_state > (config.time_blowthrough_sec * 1000)) {
                next_state = STATE_WORK;
                runtime.last_blowthrough_time = millis();
            }
            break;
            
        case STATE_SHUTDOWN:
            // Tylko pompa może działać (jeśli włączona)
            set_blower_power(0);
            set_pump_main(false);
            set_pump_floor(false);
            
            if (config.boiler_mixing_in_shutdown) {
                update_pump_mixing();
            } else {
                set_pump_mixing_power(0);
            }
              
            // Przejścia
            // → Spoczynek: temp < (temp_ignition - 1)
            if (runtime.temp_furnace < (config.temp_ignition - 1)) {
                next_state = STATE_IDLE;
            }
            // → Rozpalanie: temp rośnie
            else if (is_temp_rising()) {
                next_state = STATE_IGNITION;
            }
            break;
            
        case STATE_OVERHEAT:
            // Wszystkie pompy ON, dmuchawa OFF
            set_blower_power(0);
            set_pump_main(true);
            set_pump_floor(true);
            set_pump_mixing_power(config.pump_mixing_power_max);
            
            // Przejścia
            // → Do poprzedniego stanu: temp < bezpieczna
            if (runtime.temp_furnace < config.temp_safe_after_overheat) {
                next_state = runtime.state_before_overheat;
                ESP_LOGI(TAG, "Overheat cleared, returning to %s", 
                        state_to_string(next_state));
            }
            break;
            
        case STATE_COOLING:
            // Główna pompa ON, reszta OFF
            set_blower_power(0);
            set_pump_main(true);
            set_pump_floor(false);
            set_pump_mixing_power(0);
            
            // Przejścia
            // → Spoczynek: minęło 10 minut LUB temp > 5°C
            if (time_in_state > (10 * 60 * 1000) || 
                runtime.temp_furnace > 5) {
                next_state = STATE_IDLE;
            }
            break;
    }

    // Dodatkowy warunek przegrzania gdy temperatura wyjściowa wody jest wysoka
    if (runtime.temp_main_output > TEMP_OVERHEAT - 5) {
        ESP_LOGW(TAG, "High output water temperature detected: %d°C", runtime.temp_main_output);
        next_state = STATE_OVERHEAT;
    }
    
state_change:
    // Zmiana stanu
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

// ============== TASKI FREERTOS ==============
// TASK 1: Odczyt czujników (wysoki priorytet)
void sensor_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float temperatures[NUM_TEMP_SENSORS];
    
    while (1) {
        int success = ds18b20_read_all(temp_sensors, NUM_TEMP_SENSORS, temperatures);
        
        if (success < NUM_TEMP_SENSORS) {
            ESP_LOGW(TAG, "Only %d/%d sensors read successfully", success, NUM_TEMP_SENSORS);
        }

        xSemaphoreTake(furnace_mutex, portMAX_DELAY);
        
        // Odczyt wszystkich czujników DS18B20 (RAW)
        runtime.temp_furnace_raw = (int16_t)temperatures[0];
        runtime.temp_boiler_top_raw = (int16_t)temperatures[1];
        runtime.temp_boiler_bottom_raw = (int16_t)temperatures[2];
        runtime.temp_main_output_raw = (int16_t)temperatures[3];

        //TODO: co w przypadku błędu odczytu?
                
        // Zastosowanie korekty temperatur
        apply_temp_corrections();
            
        xSemaphoreGive(furnace_mutex);
        
        // Co sekundę
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

// TASK 2: Aktualizacja histori temperatury pieca (średni priorytet)
void temp_furnace_history_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Aktualizuj historię temperatury pieca co 1 minute
        xSemaphoreTake(furnace_mutex, portMAX_DELAY);
        
        runtime.temp_furnace_history[runtime.temp_history_index] = runtime.temp_furnace;
        runtime.temp_history_index = (runtime.temp_history_index + 1) % BUFFER_SIZE;
        
        xSemaphoreGive(furnace_mutex);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(60 * 1000));
    }
}

// TASK 3: Główna logika FSM (najwyższy priorytet)
void control_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        xSemaphoreTake(furnace_mutex, portMAX_DELAY);
        fsm_update();
        xSemaphoreGive(furnace_mutex);
        
        // FSM aktualizacja co 500ms
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

// TASK 4: Komunikacja MQTT (niski priorytet)
void mqtt_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char topic[96];
    char payload[512];
    
    ESP_LOGI(TAG, "MQTT Task started, waiting for WiFi...");
    
    // Czekaj na połączenie WiFi
    while (!wifi_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Inicjalizacja MQTT
    esp_err_t err = mqtt_manager_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT manager");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // Sprawdź połączenie WiFi
        if (!wifi_is_connected()) {
            ESP_LOGW(TAG, "WiFi not connected, waiting...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // Sprawdź połączenie MQTT
        if (!mqtt_manager_is_connected()) {
            ESP_LOGW(TAG, "MQTT not connected, waiting...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // Publikuj status pieca
        int msg_id = mqtt_manager_publish_status(topic, payload, sizeof(topic), sizeof(payload));

        if (msg_id >= 0) {
            ESP_LOGI(TAG, "Published: msg_id=%d", msg_id);
        } else {
            ESP_LOGE(TAG, "MQTT Publish failed!");
        }
        
        // Co 5 sekund
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
    }
}

// TASK 5: Watchdog (krytyczny)
void watchdog_task(void *pvParameters) {
    while (1) {
        xSemaphoreTake(furnace_mutex, portMAX_DELAY);
        uint32_t current = runtime.current_state;
        uint32_t last_fsm_update = runtime.last_fsm_update_time;
        xSemaphoreGive(furnace_mutex);
        
        uint32_t time_without_change = millis() - last_fsm_update;
        const uint32_t max_time_in_state = 30 * 1000;

        if (time_without_change > max_time_in_state) {
            ESP_LOGE(TAG, "WATCHDOG: FSM did not update within 60 seconds! Last state: %s. Restarting now!", state_to_string(current));
            esp_restart();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // Co 10 sekund
    }
}

// ============== FUNKCJE STERUJĄCE ==============
void set_pump_main(bool state) {
    if (runtime.pump_main_on != state) {
        gpio_set_level(config.pin_pump_main, state);
        runtime.pump_main_on = state;
        ESP_LOGI(TAG, "Pump Main: %s", state ? "ON" : "OFF");
    }
}

void set_pump_floor(bool state) {
    if (runtime.pump_floor_on != state) {
        gpio_set_level(config.pin_pump_floor, state);
        runtime.pump_floor_on = state;
        ESP_LOGI(TAG, "Pump Floor: %s", state ? "ON" : "OFF");
    }
}

void set_pump_mixing_power(uint8_t power) {
    if (power > 100) power = 100;
    
    if (runtime.pump_mixing_power != power) {
        runtime.pump_mixing_power = power;
        
        // TODO: Implementacja sterowania triakiem
        //  triac_set_power(config.pin_pump_mixing_control, power);
        
        ESP_LOGI(TAG, "Pump Mixing Power: %d%%", power);
    }
}

void set_blower_power(uint8_t power) {
    if (power > 100) power = 100;
    
    if (runtime.blower_power != power) {
        runtime.blower_power = power;
        
        // TODO: Implementacja sterowania triakiem
        //  triac_set_power(config.pin_blower_control, power);
        
        ESP_LOGI(TAG, "Blower Power: %d%%", power);
    }
}

// ============== MAIN ==============
void app_main(void) {
    ESP_LOGI(TAG, "=== Furnace Controller Starting ===");
    
    // Inicjalizacja NVS
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

    // Inicjalizacja karty SD
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

    // Wczytywanie konfiguracji z karty SD, NVS lub domyślnej
    if (!load_config_from_sd(&config)) {
        if (!config_load_from_nvs(&config)) {
            memcpy(&config, &default_config, sizeof(furnace_config_t));
            config_save_to_nvs(&config);
            sd_logger_save_config(&config, sizeof(furnace_config_t));
        }
    }

    // Wyświetl aktualną konfigurację
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

    // Inicjalizacja WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    
    // Wczytywanie danych konfiguracyjnych WiFi
    if (!load_wifi_config_from_sd(wifi_ssid, sizeof(wifi_ssid), wifi_password, sizeof(wifi_password))) {
        // Jeśli brak na karcie SD, to z NVS
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
    
    // Inicjalizacja czasu
    initialize_time();

    // Inicjalizacja GPIO dla pomp ON/OFF
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config.pin_pump_main) | (1ULL << config.pin_pump_floor),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Inicjalizacja GPIO dla sterowania triakami
    gpio_config_t triac_conf = {
        .pin_bit_mask = (1ULL << config.pin_pump_mixing_control) | (1ULL << config.pin_blower_control),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&triac_conf);
    
    // TODO: Inicjalizacja sterowania triakami
    // triac_init(config.pin_pump_mixing_control);
    // triac_init(config.pin_blower_control);
    
    // Inicjalizacja czujników temperatury DS18B20
    temp_sensors[0].gpio_pin = config.pin_temp_sensor_furnace;
    temp_sensors[1].gpio_pin = config.pin_temp_sensor_boiler_top;
    temp_sensors[2].gpio_pin = config.pin_temp_sensor_boiler_bottom;
    temp_sensors[3].gpio_pin = config.pin_temp_sensor_main_output;

    err = ds18b20_init(temp_sensors, NUM_TEMP_SENSORS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DS18B20 sensors!");
    }
    
    // Mutex
    furnace_mutex = xSemaphoreCreateMutex();
    if (furnace_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex!");
        esp_restart();
    }
    
    // Inicjalizacja stanu runtime
    runtime.current_state = STATE_IDLE;
    runtime.state_entry_time = millis();
    runtime.last_blowthrough_time = millis();
    runtime.last_fsm_update_time = millis();
    runtime.last_cooling_alert_time = 0;
    runtime.temp_history_index = 0;
    
    // Wyzeruj historię temperatur
    for (int i = 0; i < BUFFER_SIZE; i++) {
        runtime.temp_furnace_history[i] = 0;
    }
    
    // Konfiguracja MQTT managera
    mqtt_manager_set_data_sources(&config, &runtime, furnace_mutex);
    mqtt_manager_set_wifi_callbacks(wifi_is_connected, wifi_get_rssi);
    mqtt_manager_set_config_save_callback(config_save_wrapper);
    
    ESP_LOGI(TAG, "Creating tasks...");
    
    // Tworzenie tasków
    BaseType_t result;
    
    result = xTaskCreatePinnedToCore(sensor_task, "Sensors", 4096, NULL, 8, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Sensors task!");
        esp_restart();
    }

    result = xTaskCreatePinnedToCore(temp_furnace_history_task, "FurnaceHist", 2048, NULL, 5, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Furnace Temperature History task!");
        esp_restart();
    }
    
    result = xTaskCreatePinnedToCore(control_task, "Control", 4096, NULL, 10, NULL, 0);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Control task!");
        esp_restart();
    }
    
    result = xTaskCreatePinnedToCore(mqtt_task, "MQTT", 8192, NULL, 3, NULL, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MQTT task!");
        esp_restart();
    }
    
    result = xTaskCreatePinnedToCore(watchdog_task, "Watchdog", 2048, NULL, 12, NULL, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Watchdog task!");
        esp_restart();
    }
    
    ESP_LOGI(TAG, "=== System Ready ===");
}