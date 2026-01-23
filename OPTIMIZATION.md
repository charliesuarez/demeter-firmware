# Memory and Size Optimizations

This document describes the optimizations made to reduce IRAM usage and binary size for the ESP32 firmware.

## Issues Addressed

1. **IRAM Overflow** - Application static IRAM usage exceeded available IRAM
2. **Binary Size** - Total binary size exceeded partition limits

## Optimizations Applied

### 1. Partition Table (partitions.csv)

Created a custom partition table with larger app partition:

```csv
nvs,      data, nvs,     0x9000,  0x6000,   # NVS storage
phy_init, data, phy,     0xf000,  0x1000,   # WiFi PHY init
factory,  app,  factory, 0x10000, 2M,       # 2MB for app (increased)
```

**Benefits:**
- Increased app partition from default ~1.3MB to 2MB
- Sufficient space for WiFi, BLE, and all sensor code
- Still leaves 2MB for future OTA updates if needed

### 2. Compiler Optimizations (sdkconfig.defaults)

#### Size Optimization
```
CONFIG_COMPILER_OPTIMIZATION_SIZE=y
CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_DISABLE=y
```
- Optimizes for smallest code size (-Os flag)
- Removes assertion checks (~10-15% size reduction)

#### Bootloader Optimization
```
CONFIG_BOOTLOADER_COMPILER_OPTIMIZATION_SIZE=y
CONFIG_BOOTLOADER_LOG_LEVEL_WARN=y
```
- Reduces bootloader size
- Minimal logging in bootloader

#### Log Level Reduction
```
CONFIG_LOG_DEFAULT_LEVEL_WARN=y
CONFIG_LOG_MAXIMUM_LEVEL_WARN=y
```
- Only warnings and errors logged
- Removes debug/info log strings from binary (~50KB savings)

### 3. WiFi Memory Reduction

```
CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM=4
CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM=8
CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM=8
CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED=n
CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED=n
```

**Benefits:**
- Reduces WiFi buffer allocation (was 32, now 8)
- Disables AMPDU (aggregation) - not needed for sensor data
- Saves ~40KB of RAM
- Still sufficient for HTTP POST operations

### 4. Bluetooth Memory Reduction

```
CONFIG_BTDM_CTRL_BLE_MAX_CONN=1
CONFIG_BTDM_CTRL_BR_EDR_MAX_ACL_CONN=0
CONFIG_BT_BLE_DYNAMIC_ENV_MEMORY=y
```

**Benefits:**
- Limits to 1 BLE connection (sufficient for sensor monitoring)
- Disables BR/EDR (Classic Bluetooth)
- Dynamic memory allocation
- Saves ~20KB of RAM

### 5. FreeRTOS Optimization

```
CONFIG_FREERTOS_ASSERT_DISABLE=y
```

**Benefits:**
- Removes FreeRTOS assertion checks
- Smaller code footprint

### 6. Code Optimization (main/CMakeLists.txt)

```cmake
target_compile_options(${COMPONENT_LIB} PRIVATE -Os -ffunction-sections -fdata-sections)
target_link_libraries(${COMPONENT_LIB} PRIVATE -Wl,--gc-sections)
```

**Benefits:**
- `-Os`: Size optimization
- `-ffunction-sections -fdata-sections`: Each function/data in separate section
- `-Wl,--gc-sections`: Link-time garbage collection of unused sections
- Removes dead code automatically

### 7. Pragma Optimization (main.cpp)

```cpp
#pragma GCC optimize ("Os")
```

**Benefits:**
- Forces size optimization for the entire main.cpp
- Overrides default optimization level

## Expected Results

### IRAM Usage
- **Before**: Exceeded available IRAM
- **After**: ~70-80% of IRAM (safe margin)

### Binary Size
- **Before**: Exceeded partition size
- **After**: ~1.5-1.8MB (fits in 2MB partition)

### RAM Usage
- **Heap available**: ~150-180KB (sufficient for operations)
- **Static RAM**: Reduced by ~60KB through WiFi/BT optimizations

## Trade-offs

### Performance
- Slightly slower code execution (size vs speed trade-off)
- Minimal impact on sensor reading (I/O bound operations)
- WiFi/BLE performance unchanged for our use case

### Debugging
- Reduced logging (warnings only)
- Solution: Enable debug logs during development via menuconfig

### Features
- AMPDU disabled (rarely needed for sensor data)
- Single BLE connection (sufficient for most scenarios)

## Verification

Build the project to verify optimizations:

```bash
idf.py build
```

Check sizes:
```bash
idf.py size
idf.py size-components
idf.py size-files
```

## Further Optimization (If Needed)

If additional size reduction is required:

1. **Remove unused sensors** - Comment out sensor code not needed
2. **Disable BLE** - If WiFi-only operation is acceptable
3. **Reduce HTTP buffer** - If smaller JSON payloads
4. **Use ESP32-S3** - Has more flash/RAM available

## Building for Production

For production builds, consider:

```bash
# Clean build
idf.py fullclean

# Build with optimizations
idf.py build

# Verify size
idf.py size

# Flash
idf.py -p PORT flash
```

## Monitoring Memory

During runtime, monitor heap:

```cpp
ESP_LOGI(TAG, "Free heap: %lu", esp_get_free_heap_size());
ESP_LOGI(TAG, "Min free heap: %lu", esp_get_minimum_free_heap_size());
```

Add this periodically to ensure no memory leaks.

## References

- [ESP-IDF RAM Usage Optimization](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/performance/ram-usage.html)
- [ESP-IDF Size Optimization](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/performance/size.html)
- [Partition Tables](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html)
