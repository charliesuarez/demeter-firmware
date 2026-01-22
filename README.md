# Demeter Firmware - Pure ESP-IDF

This is the **pure ESP-IDF** version of the Demeter hydroponics system firmware, completely ported from Arduino to native ESP-IDF APIs.

## Overview

This firmware has been fully converted from Arduino to ESP-IDF, removing all Arduino framework dependencies. All sensor libraries and hardware interfaces now use native ESP-IDF drivers and APIs.

## What Was Ported

### Arduino APIs Replaced with ESP-IDF

1. **Serial/UART**
   - `Serial.begin()` → `uart_driver_install()` + `uart_param_config()`
   - `Serial.print()` → `uart_write_bytes()`
   - `Serial.read()` → `uart_read_bytes()`

2. **I2C (Wire)**
   - `Wire.begin()` → `i2c_param_config()` + `i2c_driver_install()`
   - `Wire.write()` → `i2c_master_write_to_device()`
   - `Wire.read()` → `i2c_master_read_from_device()`

3. **ADC**
   - `analogRead()` → `adc_oneshot_read()` with calibration
   - Proper voltage calibration using `adc_cali_*` APIs

4. **GPIO**
   - `pinMode()` → `gpio_set_direction()`
   - `digitalWrite()` → `gpio_set_level()`
   - `digitalRead()` → `gpio_get_level()`

5. **Timing**
   - `delay()` → `vTaskDelay()`
   - `delayMicroseconds()` → `esp_rom_delay_us()`
   - `millis()` → `esp_timer_get_time()`

### Sensor Libraries Ported

All sensor libraries have been completely rewritten for ESP-IDF:

1. **OneWire Protocol** - Pure ESP-IDF GPIO bit-banging implementation
2. **DS18B20 (DallasTemperature)** - Native OneWire implementation
3. **BME280** - Direct I2C communication with full calibration
4. **BH1750** - I2C light sensor driver
5. **DFRobot pH Sensor** - ADC-based with NVS calibration storage
6. **GravityTDS** - ADC-based TDS/EC measurement
7. **Dissolved Oxygen Sensor** - ADC-based DO measurement
8. **Ultrasonic Distance Sensor** - UART-based communication

## Project Structure

```
demeter-firmware/
├── CMakeLists.txt              # Top-level project configuration
├── sdkconfig.defaults          # Default ESP-IDF configuration
├── main/
│   ├── CMakeLists.txt          # Component configuration
│   └── main.cpp                # Main firmware code
└── README.md
```

## Hardware Configuration

### Pin Assignments

- **I2C Bus**: SDA=GPIO21, SCL=GPIO22
  - BME280 (Temperature/Humidity/Pressure): 0x76
  - BH1750 (Light): 0x23

- **OneWire**: GPIO4 (DS18B20 water temperature)

- **ADC Channels**:
  - pH Sensor: GPIO34 (ADC1_CH6)
  - TDS Sensor: GPIO35 (ADC1_CH7)
  - DO Sensor: GPIO33 (ADC1_CH5)

- **UART2**: RX=GPIO16, TX=GPIO17 (Ultrasonic distance sensor)

## Building

### Prerequisites

- ESP-IDF v5.0 or later
- ESP32 toolchain

### Build Commands

```bash
# Configure the project
idf.py menuconfig

# Build the firmware
idf.py build

# Flash to ESP32
idf.py -p PORT flash

# Monitor serial output
idf.py -p PORT monitor
```

## Configuration

Edit the following constants in `main/main.cpp`:

```cpp
#define WIFI_SSID              "YOUR_WIFI_SSID"
#define WIFI_PASSWORD          "YOUR_WIFI_PASSWORD"
#define BACKEND_URL            "http://YOUR_SPRING_BOOT_IP:8080/api/sensor-data"
```

## Features

### Connectivity

- **WiFi**: Connects to WiFi network and sends sensor data via HTTP POST
- **Bluetooth LE**: GATT server for local data access and configuration

### Sensors

All sensors are read every 10 seconds:
- pH level (calibrated)
- Water temperature (DS18B20)
- Air temperature, humidity, pressure (BME280)
- TDS and EC (electrical conductivity)
- Light intensity (lux)
- Dissolved oxygen
- Water level distance

### Data Storage

- pH calibration values stored in NVS (Non-Volatile Storage)
- Persistent across reboots

## Key Improvements Over Arduino

1. **No Framework Overhead**: Direct ESP-IDF APIs for better performance
2. **Better Resource Management**: Proper FreeRTOS task management
3. **Native WiFi & BLE**: Full ESP32 wireless stack utilization
4. **Proper Error Handling**: ESP-IDF error checking throughout
5. **ADC Calibration**: Hardware-specific voltage calibration
6. **NVS Integration**: Proper non-volatile storage for calibration

## Troubleshooting

### Sensor Not Detected

Check I2C connections and addresses:
```bash
idf.py monitor
```

Look for initialization messages for each sensor.

### ADC Readings Incorrect

Ensure:
- ADC calibration is enabled in sdkconfig
- Proper voltage reference (3.3V)
- Correct attenuation (11dB for 0-3.3V range)

### WiFi Connection Issues

- Verify SSID and password
- Check WiFi signal strength
- Review network security settings (WPA2)

## License

MIT License - See LICENSE.txt

## Credits

Original Arduino libraries ported to ESP-IDF:
- DFRobot pH Sensor
- DFRobot GravityTDS
- OneWire
- DallasTemperature
- Adafruit BME280
- BH1750

All libraries have been completely rewritten for native ESP-IDF compatibility.
