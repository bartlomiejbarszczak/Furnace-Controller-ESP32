#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "sd_logger.h"
#include "ds18b20.h"
#include "wifi_manager.h"

// ============== DEFINICJE ==============
#define NVS_NAMESPACE "furnace"
#define NVS_CONFIG_KEY "config"

#define BUFFER_SIZE 5           // Rozmiar bufora historii temperatur
#define TEMP_OVERHEAT 95        // Próg temperatury przegrzania
#define TEMP_FREEZE_THRESHOLD 4 // Próg temperatury do alertu ochłodzenia
#define NUM_TEMP_SENSORS 4      // Liczba czujników DS18B20

#define MQTT_BROKER_URI "mqtt://192.168.0.113:1883"  // Adres brokera MQTT --- mqtt://192.168.1.100:1883
#define MQTT_BASE_TOPIC "furnace"     // Podstawowy temat MQTT
#define LWT_TOPIC       MQTT_BASE_TOPIC "/availability" // Temat LWT
#define LWT_PAYLOAD_OFFLINE "offline"                   // Ładunek LWT offline
#define LWT_PAYLOAD_ONLINE  "online"                    // Ładunek LWT online

static const char *TAG = "FURNACE";


// ============== KONFIGURACJA - STRUKTURA ==============
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

// Domyślna konfiguracja
static const furnace_config_t default_config = {
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

// ============== TYPY ==============
typedef enum {
    STATE_IDLE,             // Spoczynek
    STATE_IGNITION,         // Rozpalanie
    STATE_WORK,             // Praca
    STATE_BLOWTHROUGH,      // Przedmuch
    STATE_SHUTDOWN,         // Wygaszanie
    STATE_OVERHEAT,         // Przegrzanie
    STATE_COOLING           // Ochłodzenie
} furnace_state_t;

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
    uint32_t last_cooling_alert_time;      // Ostatnie ostrzeżenie o ochłodzeniu (max co 30 min)
    
    // Flagi
    bool manual_shutdown_requested;
    bool manual_ignition_requested;
    bool error_flag;
    
} furnace_runtime_t;

// ============== GLOBALNE ==============
static furnace_config_t config;
static furnace_runtime_t runtime = {0};
static SemaphoreHandle_t furnace_mutex;
static ds18b20_sensor_t temp_sensors[NUM_TEMP_SENSORS];
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;


// ============== PROTOTYPY ==============
void set_pump_main(bool state);
void set_pump_floor(bool state);
void set_pump_mixing_power(uint8_t power);
void set_blower_power(uint8_t power);
int16_t read_temp_sensor(int sensor_id);
uint32_t millis(void);
const char* state_to_string(furnace_state_t state);
bool config_save_to_nvs(void);
bool config_load_from_nvs(void);
bool is_temp_rising(void);
void apply_temp_corrections(void);

// ============== NVS - ZAPIS/ODCZYT KONFIGURACJI ==============
bool config_save_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return false;
    }
    
    err = nvs_set_blob(nvs_handle, NVS_CONFIG_KEY, &config, sizeof(furnace_config_t));
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

bool config_load_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed (first boot?), using default config");
        return false;
    }
    
    size_t required_size = sizeof(furnace_config_t);
    err = nvs_get_blob(nvs_handle, NVS_CONFIG_KEY, &config, &required_size);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK && required_size == sizeof(furnace_config_t)) {
        ESP_LOGI(TAG, "Configuration loaded from NVS");
        return true;
    } else {
        ESP_LOGW(TAG, "NVS read failed: %s, using default config", esp_err_to_name(err));
        return false;
    }
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

static bool mqtt_data_equals(esp_mqtt_event_handle_t event, const char* expected) {
    size_t expected_len = strlen(expected);
    return (event->data_len == expected_len && strncmp(event->data, expected, expected_len) == 0);
}


// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected to broker");
            mqtt_connected = true;
            esp_mqtt_client_publish(client, LWT_TOPIC, LWT_PAYLOAD_ONLINE, 0, 1, true);
            
            // Subscribe to command topics
            esp_mqtt_client_subscribe(client, MQTT_BASE_TOPIC "/command/ignition", 0);
            esp_mqtt_client_subscribe(client, MQTT_BASE_TOPIC "/command/shutdown", 0);
            esp_mqtt_client_subscribe(client, MQTT_BASE_TOPIC "/command/config", 0);
            ESP_LOGI(TAG, "Subscribed to command topics");
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT Disconnected");
            mqtt_connected = false;
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT Data received on topic: %.*s", 
                     event->topic_len, event->topic);
            
            // Handle commands
            if (strncmp(event->topic, MQTT_BASE_TOPIC "/command/ignition", event->topic_len) == 0) {
                if (mqtt_data_equals(event, "START")) {
                    xSemaphoreTake(furnace_mutex, portMAX_DELAY);
                    runtime.manual_ignition_requested = true;
                    xSemaphoreGive(furnace_mutex);
                    ESP_LOGI(TAG, "Manual ignition requested via MQTT");
                }
            }
            else if (strncmp(event->topic, MQTT_BASE_TOPIC "/command/shutdown", event->topic_len) == 0) {
                if (mqtt_data_equals(event, "STOP")) {
                    xSemaphoreTake(furnace_mutex, portMAX_DELAY);
                    runtime.manual_shutdown_requested = true;
                    xSemaphoreGive(furnace_mutex);
                    ESP_LOGI(TAG, "Manual shutdown requested via MQTT");
                }
            }
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error occurred");
            mqtt_connected = false;
            break;
            
        default:
            break;
    }
}

// Inicjalizacja klienta MQTT
static void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .session.last_will.topic = LWT_TOPIC,
        .session.last_will.msg = LWT_PAYLOAD_OFFLINE,
        .session.last_will.msg_len = strlen(LWT_PAYLOAD_OFFLINE),
        .session.last_will.qos = 1,
        .session.last_will.retain = true,

        // Authentication:
        // .credentials.username = "username",
        // .credentials.password = "password",
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, 
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI("MQTT", "client started");
}

// Funkcja publikująca dane do MQTT
static int mqtt_publish_data(const char *topic, const char *data)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        return -1;
    }
    
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, 0, 1, 0);
    return msg_id;
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
    
    // Globalny warunek przegrzania (z każdego stanu)
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
                
        // Zastosuj korekty
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
    mqtt_init();
    
    while (1) {
        // Sprawdź połączenie WiFi
        if (!wifi_is_connected()) {
            ESP_LOGW(TAG, "WiFi not connected, waiting...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // Sprawdź połączenie MQTT
        if (!mqtt_connected) {
            ESP_LOGW(TAG, "MQTT not connected, waiting...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        // Pobierz dane do publikacji
        xSemaphoreTake(furnace_mutex, portMAX_DELAY);
        
        furnace_state_t state = runtime.current_state;
        int16_t temp_furnace = runtime.temp_furnace;
        int16_t temp_boiler_top = runtime.temp_boiler_top;
        int16_t temp_boiler_bottom = runtime.temp_boiler_bottom;
        int16_t temp_main = runtime.temp_main_output;
        bool pump_main = runtime.pump_main_on;
        bool pump_floor = runtime.pump_floor_on;
        uint8_t pump_mixing = runtime.pump_mixing_power;
        uint8_t blower = runtime.blower_power;
        bool error = runtime.error_flag;
        
        xSemaphoreGive(furnace_mutex);
        
        // Publikuj dane w formacie JSON
        snprintf(topic, sizeof(topic), "%s/status", MQTT_BASE_TOPIC);
        snprintf(payload, sizeof(payload), 
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
                   "\"heap\":%lu,"
                 "}"
                 "}",
                 state_to_string(state),
                 temp_furnace, temp_boiler_top, temp_boiler_bottom, temp_main,
                 pump_main ? "ON" : "OFF", 
                 pump_floor ? "ON" : "OFF",
                 pump_mixing,
                 blower,
                 error ? "ERROR" : "OK",
                 wifi_get_rssi(), 
                 esp_get_free_heap_size());

        int msg_id = mqtt_publish_data(topic, payload);
        
        if (msg_id >= 0) {
            ESP_LOGI(TAG, "MQTT Published: State=%s, T_furnace=%d°C, msg_id=%d",
                     state_to_string(state), temp_furnace, msg_id);
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
        uint32_t current_update = runtime.last_state_change;
        furnace_state_t current = runtime.current_state;
        xSemaphoreGive(furnace_mutex);
        
        // Sprawdź czy FSM działa
        uint32_t time_without_change = millis() - current_update;
        
        // WORK i IDLE mogą trwać długo - to OK
        // Inne stany nie powinny trwać bardzo długo
        uint32_t max_time_in_state = (10 * 60 * 1000); // 10 minut
        
        if (current == STATE_WORK || current == STATE_IDLE) {
            max_time_in_state = UINT32_MAX; // Bez limitu
        } else if (current == STATE_IGNITION) {
            max_time_in_state = 20 * 60 * 1000; // 20 minut (z marginesem)
        }
        
        if (time_without_change > max_time_in_state) {
            ESP_LOGE(TAG, "WATCHDOG: FSM stuck in state %s for too long! RESTART!", 
                   state_to_string(current));
            esp_restart();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // Co 10 sekund
    }
}


// ============== FUNKCJE POMOCNICZE ==============
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
        // triac_set_power(config.pin_pump_mixing_control, power);
        
        ESP_LOGI(TAG, "Pump Mixing Power: %d%%", power);
    }
}

void set_blower_power(uint8_t power) {
    if (power > 100) power = 100;
    
    if (runtime.blower_power != power) {
        runtime.blower_power = power;
        
        // TODO: Implementacja sterowania triakiem
        // triac_set_power(config.pin_blower_control, power);
        
        ESP_LOGI(TAG, "Blower Power: %d%%", power);
    }
}

int16_t read_temp_sensor(int sensor_id) {
    if (sensor_id < 0 || sensor_id >= NUM_TEMP_SENSORS) {
        return 500; // Zwróć wartość błędu dla nieprawidłowego ID
    }

    if (temp_sensors[sensor_id].is_valid) {
        return (int16_t)temp_sensors[sensor_id].last_temperature;
    }

    // Zwróć wartość błędu jeśli czujnik nie jest dostępny
    return 500;
}

uint32_t millis(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

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
    
    // Wczytaj konfigurację z NVS lub użyj domyślnej
    if (!config_load_from_nvs()) {
        memcpy(&config, &default_config, sizeof(furnace_config_t));
        config_save_to_nvs();
    }

    //TODO: Inicjalizacja karty SD i jeśli poprawna, to wczytanie konfiguracji z pliku, a jeśli nie to defaultowej
    //TODO: Aktywowanie zapisywania logów również do pliku na karcie SD
    
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
    err = wifi_init_sta();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi initialization failed, will retry in background");
    }
    
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
        ESP_LOGE(TAG, "Failed to initialize DS18B20 sensors!\nRestarting...");
        // esp_restart();
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
    runtime.last_cooling_alert_time = 0;
    runtime.temp_history_index = 0;
    
    // Wyzeruj historię temperatur
    for (int i = 0; i < BUFFER_SIZE; i++) {
        runtime.temp_furnace_history[i] = 0;
    }
    
    ESP_LOGI(TAG, "Creating tasks...");
    
    // Tworzenie tasków
    BaseType_t result;
    
    result = xTaskCreatePinnedToCore(sensor_task, "Sensors", 2048, NULL, 8, NULL, 0);
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
    
    result = xTaskCreatePinnedToCore(mqtt_task, "MQTT", 4096, NULL, 3, NULL, 1);
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