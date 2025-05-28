// AHT21_ENS160.c

#include "ENS160.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_err.h"


static const char *TAG_ENS160 = "ENS160";


static i2c_master_dev_handle_t ens160_dev_handle = NULL;

// --- ENS160 Functions ---
// Accepts the bus handle to add the device
esp_err_t ens160_init(i2c_master_bus_handle_t bus_handle) {
    uint8_t part_id[2];
    esp_err_t ret;

    ESP_LOGI(TAG_ENS160, "Initializing ENS160");

    // --- Add ENS160 device to the bus ---
    i2c_device_config_t ens160_dev_cfg = {
        .device_address = ENS160_ADDR,
        .scl_speed_hz = 100000, // Standard speed 100kHz
    };
    ret = i2c_master_bus_add_device(bus_handle, &ens160_dev_cfg, &ens160_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENS160, "Failed to add ENS160 device to bus: %s", esp_err_to_name(ret));
        ens160_dev_handle = NULL; // Ensure handle is NULL on failure
        return ret;
    }
    ESP_LOGI(TAG_ENS160, "ENS160 device added to I2C bus");
    // --- Device added ---

    // Read Part ID
    // Use the device handle and i2c_master_transmit_receive
    uint8_t reg_addr_pid = ENS160_REG_PART_ID;
    ret = i2c_master_transmit(ens160_dev_handle, &reg_addr_pid, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENS160, "Failed to send ENS160 register address: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(2));  // neliela aizture

    ret = i2c_master_receive(ens160_dev_handle, part_id, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENS160, "Failed to receive ENS160 part ID: %s", esp_err_to_name(ret));
        return ret;
    }
    uint16_t pid = (part_id[1] << 8) | part_id[0]; // Corrected order (LSB first)
    ESP_LOGI(TAG_ENS160, "ENS160 Part ID: 0x%04X", pid);
    if (pid != 0x0160) {
        ESP_LOGE(TAG_ENS160, "ENS160 Part ID mismatch. Expected 0x0160, got 0x%04X", pid);
        return ESP_FAIL;
    }

    // Set Operation Mode
    uint8_t write_buf[2] = {ENS160_REG_OPMODE, 0x02}; // Standard operation
    // Use the device handle
    ret = i2c_master_transmit(ens160_dev_handle, write_buf, sizeof(write_buf), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENS160, "Failed to set ENS160 operation mode: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // Wait for mode change
    ESP_LOGI(TAG_ENS160, "ENS160 initialized successfully");
    return ESP_OK;
}

esp_err_t read_ens160(uint8_t *aqi, uint16_t *tvoc, uint16_t *eco2) {
    uint8_t data[6];
    esp_err_t ret;

    // Check if device handle is valid
    if (ens160_dev_handle == NULL) {
        ESP_LOGE(TAG_ENS160, "ENS160 device handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Use the device handle and i2c_master_transmit_receive
    uint8_t reg_addr_data = ENS160_REG_DATA_AQI;
    ret = i2c_master_transmit_receive(ens160_dev_handle, &reg_addr_data, 1, data, sizeof(data), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENS160, "Failed to read ENS160 data: %s", esp_err_to_name(ret));
        return ret;
    }

    *aqi = data[0];
    *tvoc = (data[2] << 8) | data[1]; // MSB is data[2]
    *eco2 = (data[4] << 8) | data[3]; // MSB is data[4]
    uint8_t status = data[5];

    ESP_LOGD(TAG_ENS160, "ENS160 Status: 0x%02X", status);

    return ESP_OK;
}

// --- ENS160 Sleep and Wake Functions ---
esp_err_t ens160_sleep() {
    if (ens160_dev_handle == NULL) {
        ESP_LOGE(TAG_ENS160, "ENS160 device handle not initialized for sleep");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t write_buf[2] = { ENS160_REG_OPMODE, 0x00 }; // Sleep mode (Operating Mode = 0x00)
    esp_err_t ret = i2c_master_transmit(ens160_dev_handle, write_buf, sizeof(write_buf), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENS160, "Failed to set ENS160 sleep mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_ENS160, "ENS160 entered sleep mode");
    return ESP_OK;
}

esp_err_t ens160_wake() {
    if (ens160_dev_handle == NULL) {
        ESP_LOGE(TAG_ENS160, "ENS160 device handle not initialized for wake");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t write_buf[2] = { ENS160_REG_OPMODE, 0x02 }; // Standard operation mode (Normal Mode = 0x02)
    esp_err_t ret = i2c_master_transmit(ens160_dev_handle, write_buf, sizeof(write_buf), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ENS160, "Failed to set ENS160 normal mode: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // Give time for mode switch

    ESP_LOGI(TAG_ENS160, "ENS160 woke up successfully");
    return ESP_OK;
}
