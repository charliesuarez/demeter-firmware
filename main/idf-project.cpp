// ==================
// Standard C Includes
// ==================
#include <stdio.h>
#include <string.h>

// ==================
// FreeRTOS Includes
// ==================
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
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"



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



// ===============
//   pH Sensor:
// ===============
/* #include "EEPROM.h" */
#include "DFRobot_ESP_PH.h"

#define PH_PIN 34
#define ESPADC 4096.0
#define ESPVOLTAGE 3300

DFRobot_ESP_PH ph;
float voltage, phValue, temperature = 25.0;

float read_ph() {
    voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE;
    phValue = ph.readPH(voltage, temperature);
    
    return phValue
}



// ==============================
//    Water Temperature Sensor:
// ==============================
#include "OneWire.h"
#include "DallasTemperature.h"

#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature water_sensor(&oneWire);

// Water Temperature (Fahrenheit)
float read_water_temperature_f() {
    sensors.requestTemperatures();
    float tempF = sensors.getTempFByIndex(0);
    return tempF;
}

// Water Temperature (Celsius)
float read_water_temperature_c() {
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    return tempC;
}



// ==================
// For Humidity Sensor:
// ==================
#include <Wire.h>
#include "Adafruit_BME280.h"

Adafruit_BME280 bme;

AirQuality read_air_quality() {
  AirQuality air;

  air.temp     = bme.readTemperature();
  air.humidity = bme.readHumidity();
  air.pressure = bme.readPressure() / 100.0F;
  
  return air;
}



// ==================
//   TDS/EC Sensor:
// ==================
#include "GravityTDS.h"

#define TDS_PIN 35
#define VREF    3.3
#define SCOUNT  30

GravityTDS gravityTds;

int   analogBuffer[SCOUNT];
int   analogBufferIndex = 0;
float temperature       = 25.0;


TDS_EC read_tds_ec() {
  gravityTds.setTemperature(temperature);
  gravityTds.update();
    
  TDS_EC tds_ec;

  tds_ec.tdsValue = gravityTds.getTdsValue();
  tds_ec.ecValue  = tdsValue * 2.0; // EC = TDS * 2 (approximate)

  return tds_ec;
}



// ==============================
//  Ultrasonic Distance Sensor:
// ==============================
#define RXD2 16

int read_distance() {
    if (Serial2.available() >= 4) {
        byte header = Serial2.read();
        
        if (header == 0xFF) {
            byte highByte = Serial2.read();
            byte lowByte  = Serial2.read();
            byte checksum = Serial2.read();
            
            if ((highByte + lowByte) & 0xFF == checksum) {
                int distance = (highByte << 8) + lowByte;
                
		return distance;
            }
        }
    }
}



// ========================
//  Light Intensity Sensor:
// ========================
#include "Wire.h"
#include "BH1750.h"

BH1750 lightMeter(0x23); // Default I2C address

float read_lux() {
  float lux = lightMeter.readLightLevel();
  return lux;
}



//===========================
// Dissolved Oxygen Sensor:
//===========================
#define DO_PIN  33
#define VREF    3300 // mV
#define ADC_RES 4096

#define CAL1_V  1127 // Calibration voltage at 100% air saturation (mV)
#define CAL1_T  25   // Calibration temperature
#define CAL2_T  25   // Two-point calibration temperature
#define CAL2_V  1678 // Calibration voltage at 0% (N2 atmosphere)

float temperature = 25.0; // Can be read from DS18B20

float read_do_value() {
  int adcValue = 0;
  
  // Average 10 readings
  for (int i = 0; i < 10; i++) {
    adcValue += analogRead(DO_PIN);
    delay(50);
  }
  adcValue = adcValue / 10;
  
  float voltage_mv   = (float)adcValue / ADC_RES * VREF;
  float V_saturation = CAL1_V + 35 * temperature_c - CAL1_T * 35;
  float doValue      = (voltage_mv * 20.0) / V_saturation;
  
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
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
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
  
  char post_data[256];
  snprintf(post_data, sizeof(post_data),
	   "{\"temperature\":%.2f,\"humidity\":%.2f,\"lightLevel\":%d,\"timestamp\":%ld}",
	   data->temperature, data->humidity, data->light_level, data->timestamp);
  
  esp_http_client_config_t config = {
    .url = BACKEND_URL,
    .event_handler = http_event_handler,
    .method = HTTP_METHOD_POST,
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
    esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
	.adv_int_min = 0x20,
	.adv_int_max = 0x40,
	.adv_type = ADV_TYPE_IND,
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
      });
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "Advertising start failed");
    } else {
      ESP_LOGI(TAG, "Advertising started successfully");
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
    
    esp_ble_gatts_create_service(gatts_if, &(esp_gatt_srvc_id_t){
	.is_primary = true,
	.id = {
	  .uuid = {
	    .len = ESP_UUID_LEN_16,
	    .uuid = {.uuid16 = GATTS_SERVICE_UUID},
	  },
	  .inst_id = 0,
	},
      }, GATTS_NUM_HANDLE);
    break;
    
  case ESP_GATTS_CREATE_EVT:
    ESP_LOGI(TAG, "Service created, status %d, service_handle %d", param->create.status, param->create.service_handle);
    esp_ble_gatts_start_service(param->create.service_handle);
    
    esp_ble_gatts_add_char(param->create.service_handle,
			   &(esp_bt_uuid_t){
			     .len = ESP_UUID_LEN_16,
			     .uuid = {.uuid16 = GATTS_CHAR_UUID},
			   },
			   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
			   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
			   &(esp_attr_value_t){
			     .attr_max_len = 100,
			     .attr_len = 0,
			     .attr_value = char_value,
			   },
			   NULL);
    break;
    
  case ESP_GATTS_ADD_CHAR_EVT:
    ESP_LOGI(TAG, "Characteristic added, status %d, attr_handle %d", param->add_char.status, param->add_char.attr_handle);
    char_handle = param->add_char.attr_handle;
    break;
    
  case ESP_GATTS_CONNECT_EVT:
    ESP_LOGI(TAG, "Client connected, conn_id %d", param->connect.conn_id);
    conn_id_global = param->connect.conn_id;
    ble_connected = true;
    break;
    
  case ESP_GATTS_DISCONNECT_EVT:
    ESP_LOGI(TAG, "Client disconnected");
    ble_connected = false;
    esp_ble_gap_start_advertising(&(esp_ble_adv_params_t){
	.adv_int_min = 0x20,
	.adv_int_max = 0x40,
	.adv_type = ADV_TYPE_IND,
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
      });
    break;
    
  case ESP_GATTS_WRITE_EVT:
    ESP_LOGI(TAG, "Write event, value len %d", param->write.len);
    if (param->write.len > 0) {
      ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
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
	   "T:%.2f,H:%.2f,L:%d,TS:%ld",
	   data->temperature, data->humidity, data->light_level, data->timestamp);

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
  
  // Simulate sensor readings (replace with actual sensor code)
  data.ph               = read_ph();
  data.temperature      = read_water_temperature_c();
  data.air_quality      = read_air_quality();
  data.tds_ec           = read_tds_ec();
  data.distance         = read_distance();
  data.lux              = read_lux();
  data.dissolved_oxygen = read_do_value();
  data.timestamp        = esp_timer_get_time() / 1'000'000; // Convert to seconds
  
  return data;
}

void sensor_init()
{
  // Initialize ESP32 EEPROM
  Serial.begin(115200);
  /* EEPROM.begin(32); */

  // Initialize pH Sensor
  ph.begin();
  pinMode(PH_PIN, INPUT);

  // Initialize Water Temperature Sensor
  sensors.begin();

  // Initialize Humidity Sensor
  Wire.begin();
  
  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found!");
  }
  
  // Initialize TDS/EC Sensor
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(VREF);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();

  // Initialize Ultrasonic Distance Sensor:
  Serial2.begin(9600, SERIAL_8N1, RXD2, -1);
  
  // Initialize Light Intensity Sensor
  Wire.begin(21, 22); // SDA, SCL
  
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 initialized");
  } else {
    Serial.println("Error initializing BH1750");
  }

  pinMode(DO_PIN, INPUT);
}

// ============
//  Main Task
// ============

void data_sender_task(void *pvParameters)
{
  while (1) {
    SensorData sensor_data = read_sensor_data();
    
    ESP_LOGI(TAG, "Sensor Data: Temp=%.2f, Humid=%.2f, Light=%d",
	     sensor_data.temperature, sensor_data.humidity, sensor_data.light_level);
    
    // Send via WiFi
    send_data_via_wifi(&sensor_data);
    
    // Send via Bluetooth
    send_data_via_bluetooth(&sensor_data);
    
    // Wait 10 seconds before next reading
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
  ESP_LOGI(TAG, "ESP32 Firmware Starting...");
  
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  
  // Initialize WiFi
  ESP_LOGI(TAG, "Initializing WiFi...");
  wifi_init_sta();
  
  // Initialize Bluetooth
  ESP_LOGI(TAG, "Initializing Bluetooth...");
  bluetooth_init();
  
  // Create data sender task
  xTaskCreate(data_sender_task, "data_sender_task", 4096, NULL, 5, NULL);
  
  ESP_LOGI(TAG, "Firmware initialization complete");
}
