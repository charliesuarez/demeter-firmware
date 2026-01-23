idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
<<<<<<< HEAD
                       REQUIRES nvs_flash esp_wifi esp_http_client esp_timer driver bt esp_adc)
=======
                       REQUIRES nvs_flash esp_wifi esp_http_client esp_timer driver)

# Optimize for size
target_compile_options(${COMPONENT_LIB} PRIVATE -Os -ffunction-sections -fdata-sections)
target_link_libraries(${COMPONENT_LIB} PRIVATE -Wl,--gc-sections)
>>>>>>> suspicious-tu
