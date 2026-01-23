// ==================
// Standard C Includes
// ==================
#include <stdio.h>
#include <string.h>
#include <math.h>

// ==================
// FreeRTOS Includes
// ==================
#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// ==================
// ESP32 Includes
// ==================
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "hal/adc_types.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "nvs.h"

// ========================
//  System Configuration
// ========================
static const char *TAG = "ESP32_FIRMWARE";

typedef struct {
  float tds;
  float ec;
} TDS_EC;

typedef struct {
  float temperature;
  float humidity;
  float pressure;
} AirQuality;

// Sensor data structure
typedef struct {
  float      ph;
  float      water_temperature;
  AirQuality air_quality;
  TDS_EC     tds_ec;
  int        distance;
  float      lux;
  float      dissolved_oxygen;
  long       timestamp;
} SensorData;

// =====================
//  Wi-Fi Configuration
// =====================
#define WIFI_SSID              "YOUR_WIFI_SSID"
#define WIFI_PASSWORD          "YOUR_WIFI_PASSWORD"
#define BACKEND_URL            "http://YOUR_SPRING_BOOT_IP:8080/api/sensor-data"
#define MAX_HTTP_RECV_BUFFER   512
#define MAX_HTTP_OUTPUT_BUFFER 2048

// WiFi event group
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;

// Connection status
#define MAX_RETRY 5

static bool wifi_connected = false;
static int s_retry_num     = 0;

// ==================
//    Bluetooth LE
// ==================
#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_NUM_HANDLE     4
#define DEVICE_NAME          "ESP32_Sensor"
#define MANUFACTURER_NAME    "ESP32_Manufacturer"

// BLE variables
static uint16_t gatts_if_global = ESP_GATT_IF_NONE;
static uint16_t conn_id_global  = 0;
static uint16_t char_handle     = 0;
static uint8_t  char_value[100] = {0};
static bool     ble_connected   = false;

// ==================
//  I2C Configuration
// ==================
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// ==================
//  ADC Configuration
// ==================
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc_calibrated = false;

// ===============
//   pH Sensor Configuration
// ===============
#define PH_PIN ADC_CHANNEL_6  // GPIO34
#define ESPADC 4096.0
#define ESPVOLTAGE 3300

// pH calibration values stored in NVS
#define PH_NAMESPACE "ph_sensor"
#define PHVALUEADDR "ph_calib"

typedef struct {
    float acidVoltage;
    float neutralVoltage;
} PHCalibration;

static PHCalibration ph_calib = {2010.0, 1500.0};  // Default calibration values

// ==============================
//    Water Temperature Sensor Configuration
// ==============================
#define ONE_WIRE_BUS GPIO_NUM_4

// OneWire timing constants (microseconds)
#define ONEWIRE_RESET_LOW_TIME      480
#define ONEWIRE_RESET_WAIT_TIME     70
#define ONEWIRE_RESET_PRESENCE_TIME 410
#define ONEWIRE_WRITE_LOW_0         60
#define ONEWIRE_WRITE_LOW_1         10
#define ONEWIRE_WRITE_RECOVERY      10
#define ONEWIRE_READ_LOW            3
#define ONEWIRE_READ_WAIT           10
#define ONEWIRE_READ_RECOVERY       53

// DS18B20 Commands
#define DS18B20_SKIP_ROM        0xCC
#define DS18B20_CONVERT_T       0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

// ==================
// BME280 Configuration
// ==================
#define BME280_ADDR 0x76

// BME280 Registers
#define BME280_REG_TEMP_MSB   0xFA
#define BME280_REG_TEMP_LSB   0xFB
#define BME280_REG_TEMP_XLSB  0xFC
#define BME280_REG_PRESS_MSB  0xF7
#define BME280_REG_PRESS_LSB  0xF8
#define BME280_REG_PRESS_XLSB 0xF9
#define BME280_REG_HUM_MSB    0xFD
#define BME280_REG_HUM_LSB    0xFE
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_ID         0xD0

// BME280 Calibration data
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} BME280_Calib;

static BME280_Calib bme280_calib;
static int32_t t_fine;

// ==================
//   TDS/EC Sensor Configuration
// ==================
#define TDS_PIN ADC_CHANNEL_7  // GPIO35
#define VREF    3.3
#define SCOUNT  30

static float tds_temperature = 25.0;
static float tds_kvalue = 1.0;

// ==============================
//  Ultrasonic Distance Sensor Configuration
// ==============================
#define UART_NUM UART_NUM_2
#define RXD2 GPIO_NUM_16
#define TXD2 GPIO_NUM_17

// ========================
//  BH1750 Light Sensor Configuration
// ========================
#define BH1750_ADDR 0x23

// BH1750 Commands
#define BH1750_POWER_ON   0x01
#define BH1750_RESET      0x07
#define BH1750_CONT_H_RES 0x10  // Continuous H-Resolution Mode

// ===========================
// Dissolved Oxygen Sensor Configuration
// ===========================
#define DO_PIN ADC_CHANNEL_5  // GPIO33
#define DO_VREF 3300  // mV
#define ADC_RES 4096

#define CAL1_V  1127  // Calibration voltage at 100% air saturation (mV)
#define CAL1_T  25    // Calibration temperature
#define CAL2_T  25    // Two-point calibration temperature
#define CAL2_V  1678  // Calibration voltage at 0% (N2 atmosphere)

// ==================
//  Helper Functions
// ==================

static void delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

// ==================
//  I2C Functions
// ==================

esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_write_cmd(uint8_t dev_addr, uint8_t cmd) {
    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, &cmd, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// ==================
//  ADC Functions
// ==================

esp_err_t adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PH_PIN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, TDS_PIN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, DO_PIN, &config));

    // ADC Calibration
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle);
    if (ret == ESP_OK) {
        adc_calibrated = true;
    }

    return ESP_OK;
}

int read_adc_voltage(adc_channel_t channel) {
    int adc_raw;
    int voltage = 0;

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw));

    if (adc_calibrated) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage));
    } else {
        voltage = (int)((float)adc_raw / ESPADC * ESPVOLTAGE);
    }

    return voltage;
}

// ==================
// OneWire Functions
// ==================

void onewire_init(gpio_num_t pin) {
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
}

uint8_t onewire_reset(gpio_num_t pin) {
    uint8_t presence;

    gpio_set_level(pin, 0);
    delay_us(ONEWIRE_RESET_LOW_TIME);

    gpio_set_level(pin, 1);
    delay_us(ONEWIRE_RESET_WAIT_TIME);

    presence = !gpio_get_level(pin);
    delay_us(ONEWIRE_RESET_PRESENCE_TIME);

    return presence;
}

void onewire_write_bit(gpio_num_t pin, uint8_t bit) {
    if (bit) {
        gpio_set_level(pin, 0);
        delay_us(ONEWIRE_WRITE_LOW_1);
        gpio_set_level(pin, 1);
        delay_us(ONEWIRE_WRITE_LOW_0 - ONEWIRE_WRITE_LOW_1 + ONEWIRE_WRITE_RECOVERY);
    } else {
        gpio_set_level(pin, 0);
        delay_us(ONEWIRE_WRITE_LOW_0);
        gpio_set_level(pin, 1);
        delay_us(ONEWIRE_WRITE_RECOVERY);
    }
}

uint8_t onewire_read_bit(gpio_num_t pin) {
    uint8_t bit;

    gpio_set_level(pin, 0);
    delay_us(ONEWIRE_READ_LOW);
    gpio_set_level(pin, 1);
    delay_us(ONEWIRE_READ_WAIT);

    bit = gpio_get_level(pin);
    delay_us(ONEWIRE_READ_RECOVERY);

    return bit;
}

void onewire_write_byte(gpio_num_t pin, uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(pin, byte & 0x01);
        byte >>= 1;
    }
}

uint8_t onewire_read_byte(gpio_num_t pin) {
    uint8_t byte = 0;

    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (onewire_read_bit(pin)) {
            byte |= 0x80;
        }
    }

    return byte;
}

uint8_t onewire_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }

    return crc;
}

// ==================
// DS18B20 Functions
// ==================

float read_water_temperature_c(void) {
    uint8_t data[9];

    if (!onewire_reset(ONE_WIRE_BUS)) {
        ESP_LOGW(TAG, "DS18B20 not detected");
        return -999.0;
    }

    onewire_write_byte(ONE_WIRE_BUS, DS18B20_SKIP_ROM);
    onewire_write_byte(ONE_WIRE_BUS, DS18B20_CONVERT_T);

    vTaskDelay(750 / portTICK_PERIOD_MS);  // Wait for conversion

    if (!onewire_reset(ONE_WIRE_BUS)) {
        return -999.0;
    }

    onewire_write_byte(ONE_WIRE_BUS, DS18B20_SKIP_ROM);
    onewire_write_byte(ONE_WIRE_BUS, DS18B20_READ_SCRATCHPAD);

    for (int i = 0; i < 9; i++) {
        data[i] = onewire_read_byte(ONE_WIRE_BUS);
    }

    if (onewire_crc8(data, 8) != data[8]) {
        ESP_LOGW(TAG, "DS18B20 CRC error");
        return -999.0;
    }

    int16_t raw = (data[1] << 8) | data[0];
    float temp = (float)raw / 16.0;

    return temp;
}

float read_water_temperature_f(void) {
    float tempC = read_water_temperature_c();
    if (tempC == -999.0) return -999.0;
    return tempC * 9.0 / 5.0 + 32.0;
}

// ==================
//  BME280 Functions
// ==================

esp_err_t bme280_read_calibration(void) {
    uint8_t calib_data[32];

    // Read temperature and pressure calibration data (0x88-0x9F)
    esp_err_t ret = i2c_read_bytes(BME280_ADDR, 0x88, calib_data, 24);
    if (ret != ESP_OK) return ret;

    bme280_calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    bme280_calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    bme280_calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    bme280_calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    bme280_calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    bme280_calib.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    bme280_calib.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    bme280_calib.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    bme280_calib.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    bme280_calib.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    bme280_calib.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    bme280_calib.dig_P9 = (calib_data[23] << 8) | calib_data[22];

    // Read humidity calibration data
    ret = i2c_read_bytes(BME280_ADDR, 0xA1, &bme280_calib.dig_H1, 1);
    if (ret != ESP_OK) return ret;

    ret = i2c_read_bytes(BME280_ADDR, 0xE1, calib_data, 7);
    if (ret != ESP_OK) return ret;

    bme280_calib.dig_H2 = (calib_data[1] << 8) | calib_data[0];
    bme280_calib.dig_H3 = calib_data[2];
    bme280_calib.dig_H4 = (calib_data[3] << 4) | (calib_data[4] & 0x0F);
    bme280_calib.dig_H5 = (calib_data[5] << 4) | (calib_data[4] >> 4);
    bme280_calib.dig_H6 = calib_data[6];

    return ESP_OK;
}

esp_err_t bme280_init(void) {
    uint8_t chip_id;
    esp_err_t ret = i2c_read_bytes(BME280_ADDR, BME280_REG_ID, &chip_id, 1);

    if (ret != ESP_OK || chip_id != 0x60) {
        ESP_LOGE(TAG, "BME280 not found!");
        return ESP_FAIL;
    }

    ret = bme280_read_calibration();
    if (ret != ESP_OK) return ret;

    // Configure humidity oversampling
    i2c_write_byte(BME280_ADDR, BME280_REG_CTRL_HUM, 0x01);

    // Configure temp/pressure oversampling and mode
    i2c_write_byte(BME280_ADDR, BME280_REG_CTRL_MEAS, 0x27);

    // Configure standby time and filter
    i2c_write_byte(BME280_ADDR, BME280_REG_CONFIG, 0xA0);

    return ESP_OK;
}

int32_t bme280_compensate_temperature(int32_t adc_T) {
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bme280_calib.dig_T1 << 1))) * ((int32_t)bme280_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1))) >> 12) * ((int32_t)bme280_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T;
}

uint32_t bme280_compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bme280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bme280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bme280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bme280_calib.dig_P3) >> 8) + ((var1 * (int64_t)bme280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bme280_calib.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bme280_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_calib.dig_P7) << 4);

    return (uint32_t)p;
}

uint32_t bme280_compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_calib.dig_H4) << 20) - (((int32_t)bme280_calib.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)bme280_calib.dig_H6)) >> 10) *
                                                    (((v_x1_u32r * ((int32_t)bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                                                  ((int32_t)2097152)) * ((int32_t)bme280_calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bme280_calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12);
}

AirQuality read_air_quality(void) {
    AirQuality air = {0};
    uint8_t data[8];

    esp_err_t ret = i2c_read_bytes(BME280_ADDR, BME280_REG_PRESS_MSB, data, 8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BME280");
        return air;
    }

    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int32_t adc_H = (data[6] << 8) | data[7];

    int32_t temp = bme280_compensate_temperature(adc_T);
    uint32_t press = bme280_compensate_pressure(adc_P);
    uint32_t hum = bme280_compensate_humidity(adc_H);

    air.temperature = (float)temp / 100.0;
    air.pressure = (float)press / 25600.0;
    air.humidity = (float)hum / 1024.0;

    return air;
}

// ==================
//  BH1750 Functions
// ==================

esp_err_t bh1750_init(void) {
    esp_err_t ret = i2c_write_cmd(BH1750_ADDR, BH1750_POWER_ON);
    if (ret != ESP_OK) return ret;

    vTaskDelay(10 / portTICK_PERIOD_MS);

    return i2c_write_cmd(BH1750_ADDR, BH1750_CONT_H_RES);
}

float read_lux(void) {
    uint8_t data[2];

    esp_err_t ret = i2c_master_read_from_device(I2C_MASTER_NUM, BH1750_ADDR, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BH1750");
        return -1.0;
    }

    uint16_t level = (data[0] << 8) | data[1];
    return (float)level / 1.2;
}

// ===============
//   pH Sensor Functions
// ===============

void ph_load_calibration(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(PH_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err == ESP_OK) {
        size_t required_size = sizeof(PHCalibration);
        nvs_get_blob(nvs_handle, PHVALUEADDR, &ph_calib, &required_size);
        nvs_close(nvs_handle);
    }
}

float read_ph(void) {
    int voltage = read_adc_voltage(PH_PIN);
    float temperature = 25.0;  // Can be updated with actual water temp

    float slope = (7.0 - 4.0) / ((ph_calib.neutralVoltage - 1500.0) / 3.0 - (ph_calib.acidVoltage - 1500.0) / 3.0);
    float intercept = 7.0 - slope * (ph_calib.neutralVoltage - 1500.0) / 3.0;
    float phValue = slope * (voltage - 1500.0) / 3.0 + intercept;

    return phValue;
}

// ==================
//   TDS/EC Sensor Functions
// ==================

TDS_EC read_tds_ec(void) {
    TDS_EC result = {0};
    float voltage_sum = 0;

    // Average multiple readings
    for (int i = 0; i < SCOUNT; i++) {
        int voltage = read_adc_voltage(TDS_PIN);
        voltage_sum += voltage;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    float avgVoltage = voltage_sum / SCOUNT / 1000.0;  // Convert to volts

    // Temperature compensation
    float compensationCoefficient = 1.0 + 0.02 * (tds_temperature - 25.0);
    float compensationVoltage = avgVoltage / compensationCoefficient;

    // TDS calculation
    result.ec = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
                 - 255.86 * compensationVoltage * compensationVoltage
                 + 857.39 * compensationVoltage) * tds_kvalue;
    result.tds = result.ec * 0.5;

    return result;
}

// ==============================
//  Ultrasonic Distance Sensor Functions
// ==============================

esp_err_t uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0));

    return ESP_OK;
}

int read_distance(void) {
    uint8_t data[4];
    int len = uart_read_bytes(UART_NUM, data, 4, 100 / portTICK_PERIOD_MS);

    if (len == 4 && data[0] == 0xFF) {
        uint8_t checksum = (data[1] + data[2]) & 0xFF;
        if (checksum == data[3]) {
            int distance = (data[1] << 8) | data[2];
            return distance;
        }
    }

    return -1;
}

// ===========================
// Dissolved Oxygen Sensor Functions
// ===========================

float read_do_value(void) {
    int adcValue = 0;

    // Average 10 readings
    for (int i = 0; i < 10; i++) {
        adcValue += read_adc_voltage(DO_PIN);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    adcValue = adcValue / 10;

    float voltage_mv = (float)adcValue;
    float temperature_c = read_water_temperature_c();
    if (temperature_c == -999.0) temperature_c = 25.0;

    float V_saturation = CAL1_V + 35 * (temperature_c - CAL1_T);
    float doValue = (voltage_mv * 20.0) / V_saturation;

    return doValue;
}

// ========================
//  WiFi Functions
// ========================

static void wifi_event_handler(void* arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void* event_data)
{
  if      (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < MAX_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "Retry to connect to the AP");
    }
    else xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);

    wifi_connected = false;
    ESP_LOGI(TAG, "Connect to the AP failed");

  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

    ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));

    s_retry_num    = 0;
    wifi_connected = true;

    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold = {
                .authmode = WIFI_AUTH_WPA2_PSK,
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// ========================
//  HTTP Functions
// ========================

esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
  switch(evt->event_id) {
  case HTTP_EVENT_ERROR:
    ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
    break;
  case HTTP_EVENT_ON_CONNECTED:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
    break;
  case HTTP_EVENT_HEADER_SENT:
    ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
    break;
  case HTTP_EVENT_ON_HEADER:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
    break;
  case HTTP_EVENT_ON_DATA:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
    break;
  case HTTP_EVENT_ON_FINISH:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
    break;
  case HTTP_EVENT_DISCONNECTED:
    ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
    break;
  default:
    break;
  }
  return ESP_OK;
}

void send_data_via_wifi(SensorData *data)
{
  if (!wifi_connected) {
    ESP_LOGW(TAG, "WiFi not connected, skipping HTTP send");
    return;
  }

  char post_data[512];
  snprintf(post_data, sizeof(post_data),
           "{\"ph\":%.2f,\"waterTemp\":%.2f,\"airTemp\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f,"
           "\"tds\":%.2f,\"ec\":%.2f,\"distance\":%d,\"lux\":%.2f,\"dissolvedOxygen\":%.2f,\"timestamp\":%ld}",
           data->ph, data->water_temperature, data->air_quality.temperature,
           data->air_quality.humidity, data->air_quality.pressure,
           data->tds_ec.tds, data->tds_ec.ec, data->distance, data->lux,
           data->dissolved_oxygen, data->timestamp);

  esp_http_client_config_t config = {
    .url = BACKEND_URL,
    .method = HTTP_METHOD_POST,
    .event_handler = http_event_handler,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);

  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_post_field(client, post_data, strlen(post_data));

  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %lld",
             esp_http_client_get_status_code(client),
             esp_http_client_get_content_length(client));
    ESP_LOGI(TAG, "Data sent via WiFi: %s", post_data);
  } else {
    ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
}

// ========================
//  BLE Functions
// ========================

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
  switch (event) {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    {
      esp_ble_adv_params_t params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
      };      
      
      esp_ble_gap_start_advertising(&params);
    }
    break;

  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    {
      if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
	ESP_LOGE(TAG, "Advertising start failed");
      } else {
	ESP_LOGI(TAG, "Advertising started successfully");
      }
    }
    break;

  default:
    break;
  }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param)
{
  switch (event) {
  case ESP_GATTS_REG_EVT:
    {
      ESP_LOGI(TAG, "GATT server registered, status %d, app_id %d", param->reg.status, param->reg.app_id);
      gatts_if_global = gatts_if;
      
      esp_ble_gap_set_device_name(DEVICE_NAME);
      
      esp_ble_adv_data_t adv_data = {
	.set_scan_rsp = false,
	.include_name = true,
	.include_txpower = true,
	.min_interval = 0x0006,
	.max_interval = 0x0010,
	.appearance = 0x00,
	.manufacturer_len = 0,
	.p_manufacturer_data = NULL,
	.service_data_len = 0,
	.p_service_data = NULL,
	.service_uuid_len = 0,
	.p_service_uuid = NULL,
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
      };
      esp_ble_gap_config_adv_data(&adv_data);

      esp_gatt_srvc_id_t service_id = {
	  .id = {
	    .uuid = {
	      .len = ESP_UUID_LEN_16,
	      .uuid = {.uuid16 = GATTS_SERVICE_UUID},
	    },
	    .inst_id = 0,
	  },
	  .is_primary = true,
      };        
      esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
    }
    break;

  case ESP_GATTS_CREATE_EVT:
    {
      ESP_LOGI(TAG, "Service created, status %d, service_handle %d", param->create.status, param->create.service_handle);
      esp_ble_gatts_start_service(param->create.service_handle);

      esp_bt_uuid_t bt_uuid = {
	.len = ESP_UUID_LEN_16,
	.uuid = {.uuid16 = GATTS_CHAR_UUID},
      };
      esp_attr_value_t attr_value = {
	.attr_max_len = 100,
	.attr_len = 0,
	.attr_value = char_value,
      };        
      esp_ble_gatts_add_char(param->create.service_handle,
			     &bt_uuid,
			     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
			     ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
			     &attr_value,
			     NULL);
    }
    break;

  case ESP_GATTS_ADD_CHAR_EVT:
    {
      ESP_LOGI(TAG, "Characteristic added, status %d, attr_handle %d", param->add_char.status, param->add_char.attr_handle);
      char_handle = param->add_char.attr_handle;
    }
    break;

  case ESP_GATTS_CONNECT_EVT:
    {
      ESP_LOGI(TAG, "Client connected, conn_id %d", param->connect.conn_id);
      conn_id_global = param->connect.conn_id;
      ble_connected = true;
    }
    break;

  case ESP_GATTS_DISCONNECT_EVT:
    {
      ESP_LOGI(TAG, "Client disconnected");
      ble_connected = false;
      esp_ble_adv_params_t ble_adv_params = {
	.adv_int_min = 0x20,
	.adv_int_max = 0x40,
	.adv_type = ADV_TYPE_IND,
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
      };        
      esp_ble_gap_start_advertising(&ble_adv_params);
    }
    break;

  case ESP_GATTS_WRITE_EVT:
    {
      ESP_LOGI(TAG, "Write event, value len %d", param->write.len);
      if (param->write.len > 0) {
	ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
      }
    }
    break;

  default:
    break;
  }
}

void send_data_via_bluetooth(SensorData *data)
{
  if (!ble_connected || gatts_if_global == ESP_GATT_IF_NONE) {
    ESP_LOGW(TAG, "BLE not connected, skipping BLE send");
    return;
  }

  char ble_data[100];
  snprintf(ble_data, sizeof(ble_data),
           "PH:%.2f,WT:%.2f,AT:%.2f,H:%.2f,TDS:%.2f",
           data->ph, data->water_temperature, data->air_quality.temperature,
           data->air_quality.humidity, data->tds_ec.tds);

  esp_ble_gatts_set_attr_value(char_handle, strlen(ble_data), (uint8_t *)ble_data);

  esp_ble_gatts_send_indicate(gatts_if_global, conn_id_global, char_handle,
                              strlen(ble_data), (uint8_t *)ble_data, false);

  ESP_LOGI(TAG, "Data sent via BLE: %s", ble_data);
}

void bluetooth_init(void)
{
  esp_err_t ret;

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
    return;
  }

  esp_ble_gatts_register_callback(gatts_profile_event_handler);
  esp_ble_gap_register_callback(gap_event_handler);
  esp_ble_gatts_app_register(0);

  ESP_LOGI(TAG, "Bluetooth initialized");
}

// ========================
//  Sensor Subsystem
// ========================

SensorData read_sensor_data(void)
{
  SensorData data;

  data.ph               = read_ph();
  data.water_temperature = read_water_temperature_c();
  data.air_quality      = read_air_quality();
  data.tds_ec           = read_tds_ec();
  data.distance         = read_distance();
  data.lux              = read_lux();
  data.dissolved_oxygen = read_do_value();
  data.timestamp        = esp_timer_get_time() / 1000000; // Convert to seconds

  return data;
}

void sensor_init(void)
{
  ESP_LOGI(TAG, "Initializing sensors...");

  // Initialize I2C
  i2c_master_init();

  // Initialize ADC
  adc_init();

  // Initialize UART for distance sensor
  uart_init();

  // Initialize OneWire GPIO
  onewire_init(ONE_WIRE_BUS);

  // Initialize BME280
  if (bme280_init() == ESP_OK) {
    ESP_LOGI(TAG, "BME280 initialized");
  } else {
    ESP_LOGE(TAG, "BME280 initialization failed");
  }

  // Initialize BH1750
  if (bh1750_init() == ESP_OK) {
    ESP_LOGI(TAG, "BH1750 initialized");
  } else {
    ESP_LOGE(TAG, "BH1750 initialization failed");
  }

  // Load pH calibration from NVS
  ph_load_calibration();

  ESP_LOGI(TAG, "All sensors initialized");
}

// ============
//  Main Task
// ============

void data_sender_task(void *pvParameters)
{
  while (1) {
    SensorData sensor_data = read_sensor_data();

    ESP_LOGI(TAG, "Sensor Data: pH=%.2f, WaterTemp=%.2f, AirTemp=%.2f, Humidity=%.2f, TDS=%.2f",
             sensor_data.ph, sensor_data.water_temperature,
             sensor_data.air_quality.temperature, sensor_data.air_quality.humidity,
             sensor_data.tds_ec.tds);

    // Send via WiFi
    send_data_via_wifi(&sensor_data);

    // Send via Bluetooth
    send_data_via_bluetooth(&sensor_data);

    // Wait 10 seconds before next reading
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "ESP32 Firmware Starting...");

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize sensors
  sensor_init();

  // Initialize WiFi
  ESP_LOGI(TAG, "Initializing WiFi...");
  wifi_init_sta();

  // Initialize Bluetooth
  ESP_LOGI(TAG, "Initializing Bluetooth...");
  bluetooth_init();

  // Create data sender task
  xTaskCreate(data_sender_task, "data_sender_task", 8192, NULL, 5, NULL);

  ESP_LOGI(TAG, "Firmware initialization complete");
}
