# Furnace Controller ESP32 

ESP32-based furnace controller for managing multiple circulation pumps, floor heating pumps, boiler mixing pump, and blower. Includes SD card logging, WiFi connectivity, and MQTT integration for remote monitoring and control.

## Features

### Core Functionality
- **Multi-State FSM Control**: 7-state finite state machine
- **4 Temperature Sensors**: DS18B20 digital temperature monitoring for furnace, boiler (top/bottom), and main water output
- **Pump Control**: 
  - Main circulation pump (on/off)
  - Floor heating pump (on/off with runtime override)
  - Boiler mixing pump (Power control 0-100%)
- **Blower Control**: Power control with automatic ramping during blowthrough cycles
- **Safety Features**: Overheat detection, freeze protection, watchdog monitoring

### Communication & Storage
- **MQTT Integration**: 
  - mDNS broker discovery
  - Subscribe to control commands (ignition, shutdown, configuration)
  - Last-Will & Testament (LWT) for availability tracking
- **WiFi**: Station mode with automatic reconnection with also NVS and SD card credential storage
- **SD Card Logging**:
  - Hot-plug card detection with GPIO CD pin
  - Buffered writes with automatic flushing
  - Configuration persistence (binary and WiFi config files)
  - Log rotation after size threshold
  - FatFs filesystem with wear leveling through buffering

<!-- ## Diagram -->
<!-- //Connection diagram coming soon. -->

## Hardware Requirements

### Microcontroller
- **ESP32-DevKitC** or compatible ESP32 board

### Peripherals
- **4x DS18B20** Temperature sensors (GPIO 4, 5, 18, 19)
- **Relay Modules** for pump control (GPIO 25, 26)
- **TRIAC Driver** for power control (GPIO 27, 32)
- **SD Card Slot** with SDMMC interface (1-bit mode)
  - CLK: GPIO 14
  - CMD: GPIO 15
  - D0: GPIO 2
  - CD: GPIO 33 (optional card detect)
- **WiFi**: Built-in ESP32 antenna

### Recommended Connections
```
DS18B20 Sensors (One-Wire):
  - Furnace:       GPIO 4
  - Boiler Top:    GPIO 5
  - Boiler Bottom: GPIO 18
  - Main Output:   GPIO 19

Pump Control (GPIO Output):
  - Main Pump:     GPIO 25 (Relay)
  - Floor Pump:    GPIO 26 (Relay)
  - Mixing Pump:   GPIO 27 (Triac)
  - Blower:        GPIO 32 (Triac)

SD Card (SDMMC 1-bit mode):
  - CLK:  GPIO 14
  - CMD:  GPIO 15
  - D0:   GPIO 2
  - CD:   GPIO 33 (active low)
```

## Getting Started

### Prerequisites
- **ESP-IDF v6.1.0** or later
- **Visual Studio Code** with ESP-IDF extension
- **Python 3.7+** (required by ESP-IDF)

### Installation

1. **Clone the repository and then**
   ```bash
   cd Furnace-Controller-ESP32
   ```

2. **Set up ESP-IDF environment**
   ```bash
   . $IDF_PATH/export.sh
   ```

3. **Configure project for your ESP32**
   ```bash
   idf.py menuconfig
   ```
   - Select your ESP32 board variant
   - Adjust other ESP-IDF settings as needed

4. **Build the project**
   ```bash
   idf.py build
   ```

5. **Flash to device**
   ```bash
   idf.py flash
   ```

6. **Monitor serial output**
   ```bash
   idf.py monitor
   ```

## Project Structure

```
furnace-controller/
├── main/
│   ├── main.c                    # Main application and FSM
│   ├── CMakeLists.txt
│   └── idf_component.yml         # Component dependencies
│
├── components/
│   ├── ds18b20/                  # Temperature sensor driver
│   ├── furnace_config/           # Configuration management
│   ├── mqtt_manager/             # MQTT communication & mDNS
│   ├── sd_logger/                # SD card logging (FatFs)
│   └── wifi_manager/             # WiFi connectivity
│
├── CMakeLists.txt                # Root project configuration
├── topics.txt                    # MQTT topics documentation
└── README.md                     # This file
```

## System Architecture

### Task Priority (Higher = Higher Priority)
1. **Watchdog Task** (Priority 12) - Monitors FSM execution health
2. **Control Task** (Priority 10) - FSM update at 500ms interval
3. **Sensor Task** (Priority 8) - Temperature reading at 1s interval
4. **Furnace History Task** (Priority 5) - Temperature history buffer update
5. **MQTT Task** (Priority 3) - Status publishing on change

### Timing
```
Sensor Reading:       1 second
FSM Update:           500 milliseconds
Temperature History:  60 seconds
Status Publish:       Publish on change (100ms task)
SD Card Flush:        30 seconds
Watchdog Check:       2 seconds (max 30s without FSM update)
```

## MQTT Configuration

### Broker Discovery
The controller automatically discovers MQTT brokers using mDNS (`_mqtt._tcp`). Ensure your broker advertises this service:

**Mosquitto Configuration** (`/etc/mosquitto/mosquitto.conf`):
```conf
listener 1883
protocol mqtt
```

Enable mDNS advertising on your broker (if supported).

### Topics

**Published Topics** (Status):
```
furnace/status                - Runtime status updates
furnace/availability          - LWT topic (online/offline)
furnace/config/get-response   - Configuration response
furnace/command/status        - Command execution status
```

**Subscribed Topics** (Commands):
```
furnace/command/ignition           - Trigger manual ignition
furnace/command/shutdown           - Trigger manual shutdown
furnace/command/pump/underfloor    - Data: "ON" or "OFF"
furnace/command/clear-errors       - Clear error flag
furnace/command/restart            - Restart ESP32
furnace/command/config             - JSON configuration update
furnace/command/config/get         - Request current configuration
```

## Configuration Parameters

### Temperature Thresholds
| Parameter | Range | Default | Unit |
|-----------|-------|---------|------|
| `temp_target_work` | 40-80 | 60 | °C |
| `temp_ignition` | 20-35 | 25 | °C |
| `temp_shutdown` | 30-50 | 45 | °C |
| `temp_safe_after_overheat` | 60-80 | 80 | °C |

### Pump Control
| Parameter | Range | Default | Unit |
|-----------|-------|---------|------|
| `temp_activ_pump_main` | 30-80 | 45 | °C |
| `temp_activ_pump_main_hysteresis` | 2-10 | 4 | °C |
| `pump_mixing_power_min` | 0-100 | 30 | % |
| `pump_mixing_power_max` | 0-100 | 100 | % |
| `pump_floor_enabled` | true/false | false | - |

### Boiler Mixing
| Parameter | Range | Default | Unit |
|-----------|-------|---------|------|
| `temp_diff_furnace_boiler` | 3-10 | 5 | °C |
| `temp_diff_boiler_vertical` | 3-10 | 5 | °C |
| `boiler_top_priority_enabled` | true/false | false | - |
| `temp_boiler_top_priority` | 30-70 | 55 | °C |
| `boiler_mixing_in_shutdown` | true/false | true | - |

### Blower
| Parameter | Range | Default | Unit |
|-----------|-------|---------|------|
| `blower_enabled` | true/false | false | - |
| `blower_power_min` | 20-100 | 30 | % |
| `blower_power_max` | 20-100 | 100 | % |
| `time_blowthrough_sec` | 10-180 | 60 | s |
| `time_blowthrough_interval_min` | 0-30 | 10 | min |

### Temperature Corrections
| Parameter | Range | Default | Unit |
|-----------|-------|---------|------|
| `temp_correction_furnace` | -5 to 5 | 0 | °C |
| `temp_correction_boiler_top` | -5 to 5 | 0 | °C |
| `temp_correction_boiler_bottom` | -5 to 5 | 0 | °C |

## Default Configuration

On boot, the controller loads:
1. Configuration from SD card (`/furnace_config.bin`) if available
2. Configuration from NVS flash if SD not available
3. Default hardcoded configuration if neither available

The default configuration is intended for typical heating systems and may require adjustment for specific installations. Adjust via MQTT or re-flash with modified `default_config` in `furnace_config.c`.

## Safety Features

- **Overheat Detection** (>95°C): All pumps engaged, blower disabled
- **Freeze Protection** (<4°C): Emergency cooling circulation (max once per 30 min)
- **Ignition Timeout**: 30-minute maximum, error flag set if exceeded
- **Watchdog**: Restarts system if FSM doesn't update for 30 seconds

## Wear Leveling
- Auto-flush every 30 seconds or when buffer >75% full
- Log rotation at 10MB prevents single file bloat

## TODOs

- [ ] **Display Integration**

- [ ] **WiFi Configuration Portal (AP Mode)**

## Troubleshooting

### WiFi Issues
- **Won't connect**: Check SSID/password saved on SD card
- **Frequent disconnects**: Check WiFi signal strength (RSSI), adjust antenna position
- **mDNS broker not found**: Ensure MQTT broker advertises `_mqtt._tcp` service

### SD Card Issues
- **Card not detected**: Verify CD pin connection, check if `enable_card_detect=true`
- **Mount failures**: Check FatFs filesystem integrity, try power cycle

### MQTT Issues
- **Can't find broker**: Verify mDNS is working, check mosquitto logs
- **No status updates**: Check MQTT connection and WiFi connectivity

### Temperature Reading Failures
- **Invalid readings (85°C)**: Sensor not yet initialized or power-on default
- **CRC errors**: Check One-Wire wiring, try reducing cable length, adjust timings.
- **Sensor missing**: Verify GPIO pin in config, check DS18B20 parasite power mode

## Contributing

To extend functionality:
1. Add new components in `/components/` directory
2. Create CMakeLists.txt for each component
3. Add to main CMakeLists.txt `REQUIRES` list
4. Update configuration structure if adding new parameters

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
