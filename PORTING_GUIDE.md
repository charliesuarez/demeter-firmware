# Arduino to ESP-IDF Porting Guide

This document details the complete porting process from Arduino framework to pure ESP-IDF for the Demeter hydroponics firmware.

## Overview of Changes

The entire codebase has been converted from Arduino framework to native ESP-IDF APIs. This eliminates the Arduino abstraction layer and provides direct access to ESP32 hardware capabilities.

## Arduino API Replacements

### 1. Serial Communication (UART)

**Arduino:**
```cpp
Serial.begin(115200);
Serial.println("Hello");
Serial2.begin(9600, SERIAL_8N1, RXD2, -1);
byte data = Serial2.read();
```

**ESP-IDF:**
```cpp
uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};
uart_param_config(UART_NUM_2, &uart_config);
uart_set_pin(UART_NUM_2, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
uart_driver_install(UART_NUM_2, 256, 0, 0, NULL, 0);

uint8_t data[4];
uart_read_bytes(UART_NUM_2, data, 4, 100 / portTICK_PERIOD_MS);
```

### 2. I2C Communication (Wire Library)

**Arduino:**
```cpp
Wire.begin();
Wire.beginTransmission(address);
Wire.write(reg);
Wire.endTransmission();
Wire.requestFrom(address, length);
byte data = Wire.read();
```

**ESP-IDF:**
```cpp
i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = 21,
    .scl_io_num = 22,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,
};
i2c_param_config(I2C_NUM_0, &conf);
i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

uint8_t write_buf[2] = {reg_addr, data};
i2c_master_write_to_device(I2C_NUM_0, dev_addr, write_buf, 2, 1000 / portTICK_PERIOD_MS);

uint8_t data[len];
i2c_master_write_read_device(I2C_NUM_0, dev_addr, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
```

### 3. Analog-to-Digital Conversion (ADC)

**Arduino:**
```cpp
pinMode(34, INPUT);
int value = analogRead(34);
float voltage = value / 4096.0 * 3.3;
```

**ESP-IDF:**
```cpp
// Initialize ADC
adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT_1,
};
adc_oneshot_new_unit(&init_config, &adc1_handle);

adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
};
adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config);

// Setup calibration
adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
};
adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);

// Read value
int adc_raw, voltage;
adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw);
adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage);
```

### 4. GPIO Operations

**Arduino:**
```cpp
pinMode(4, OUTPUT);
digitalWrite(4, HIGH);
int state = digitalRead(4);
```

**ESP-IDF:**
```cpp
gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
gpio_set_level(GPIO_NUM_4, 1);
int state = gpio_get_level(GPIO_NUM_4);

// For input with pull-up
gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);
```

### 5. Timing Functions

**Arduino:**
```cpp
delay(1000);              // 1 second
delayMicroseconds(100);   // 100 microseconds
unsigned long time = millis();
```

**ESP-IDF:**
```cpp
vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1 second
esp_rom_delay_us(100);                   // 100 microseconds
int64_t time = esp_timer_get_time() / 1000;  // Convert to milliseconds
```

### 6. EEPROM/Non-Volatile Storage

**Arduino:**
```cpp
EEPROM.begin(32);
EEPROM.write(address, value);
byte value = EEPROM.read(address);
EEPROM.commit();
```

**ESP-IDF:**
```cpp
nvs_handle_t nvs_handle;
nvs_open("namespace", NVS_READWRITE, &nvs_handle);

// Write
nvs_set_blob(nvs_handle, "key", &data, sizeof(data));
nvs_commit(nvs_handle);

// Read
size_t required_size = sizeof(data);
nvs_get_blob(nvs_handle, "key", &data, &required_size);

nvs_close(nvs_handle);
```

## Sensor Library Conversions

### OneWire Protocol

Implemented bit-banging protocol using ESP-IDF GPIO:
- `onewire_reset()` - Bus reset and presence detection
- `onewire_write_bit()` - Write single bit
- `onewire_read_bit()` - Read single bit
- `onewire_write_byte()` - Write byte
- `onewire_read_byte()` - Read byte
- `onewire_crc8()` - CRC calculation

### DS18B20 Temperature Sensor

Built on OneWire implementation:
- Device detection via presence pulse
- Temperature conversion command
- Scratchpad reading
- CRC verification
- Temperature calculation (16-bit to float)

### BME280 Environment Sensor

Direct I2C implementation:
- Calibration data reading from device
- Temperature compensation algorithm
- Pressure compensation algorithm
- Humidity compensation algorithm
- All using manufacturer's formulas

### BH1750 Light Sensor

Simple I2C implementation:
- Power on command
- Continuous high-resolution mode
- Direct lux reading

### pH Sensor

ADC-based with calibration:
- Voltage reading via ADC
- Two-point calibration (acid/neutral)
- Temperature compensation
- NVS storage for calibration values

### TDS/EC Sensor

ADC-based conductivity measurement:
- Multiple sample averaging
- Temperature compensation
- EC to TDS conversion
- K-value calibration support

### Dissolved Oxygen Sensor

ADC voltage to DO conversion:
- Multi-sample averaging
- Temperature compensation
- Calibration voltage references

## Project Structure Changes

**Arduino:**
```
project/
├── project.ino
├── config.h
└── sensors.cpp
```

**ESP-IDF:**
```
project/
├── CMakeLists.txt           # Top-level build config
├── sdkconfig.defaults       # Default configuration
├── main/
│   ├── CMakeLists.txt       # Component build config
│   └── main.cpp             # Application code
└── README.md
```

## Build System Changes

**Arduino:**
- Uses Arduino IDE or PlatformIO
- `setup()` and `loop()` functions
- Automatic library management

**ESP-IDF:**
- CMake-based build system
- `app_main()` entry point
- Manual component registration
- FreeRTOS task management

## Key Benefits of ESP-IDF

1. **Performance**: No Arduino abstraction overhead
2. **Memory**: Better control over heap and stack
3. **Features**: Full access to ESP32 capabilities
4. **Debugging**: Better tooling and error messages
5. **Optimization**: Compiler optimizations
6. **Compatibility**: Standard ESP32 development

## Migration Checklist

- [x] Replace all `Serial` with UART drivers
- [x] Replace `Wire` with I2C drivers
- [x] Replace `analogRead()` with ADC drivers
- [x] Replace `pinMode/digitalWrite/digitalRead` with GPIO
- [x] Replace `delay/delayMicroseconds` with FreeRTOS
- [x] Replace `EEPROM` with NVS
- [x] Port OneWire library
- [x] Port DS18B20 library
- [x] Port BME280 library
- [x] Port BH1750 library
- [x] Port pH sensor library
- [x] Port TDS sensor library
- [x] Create proper CMake build files
- [x] Remove Arduino framework dependency
- [x] Update documentation

## Common Pitfalls

1. **GPIO Pull-ups**: ESP-IDF requires explicit pull-up configuration
2. **ADC Calibration**: Must be enabled in sdkconfig
3. **Task Stack Size**: FreeRTOS tasks need adequate stack
4. **I2C Timeout**: Must specify timeout for blocking operations
5. **UART Buffer**: Must allocate RX buffer for uart_driver_install()

## Testing Recommendations

1. Test each sensor independently
2. Verify I2C communication with i2c-tools
3. Check ADC calibration values
4. Monitor heap usage
5. Verify WiFi and BLE functionality
6. Test under various loads

## Further Reading

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
