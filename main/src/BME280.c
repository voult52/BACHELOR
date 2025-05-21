#include "BME280.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

// --- Constants ---
#define BME280_ADDR             0x76
#define BME280_REG_RESET        0xE0
#define BME280_RESET_VALUE      0xB6
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_DATA         0xF7
#define BME280_DATA_LEN         8

#define ENS160_ADDR             0x53
#define ENS160_REG_PART_ID      0x00
#define ENS160_REG_OPMODE       0x10
#define ENS160_REG_DATA_AQI     0x21

static const char *TAG_BME280 = "BME280";
static const char *TAG_ENS160 = "ENS160";

// --- Static Device Handles ---
static i2c_master_dev_handle_t bme280_dev_handle = NULL;
static i2c_master_dev_handle_t ens160_dev_handle = NULL;

// --- BME280 Functions ---
esp_err_t bme280_init(i2c_master_bus_handle_t bus_handle) {
    esp_err_t ret;

    ESP_LOGI(TAG_BME280, "Initializing BME280");

    i2c_device_config_t dev_cfg = {
        .device_address = BME280_ADDR,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &bme280_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BME280, "Failed to add BME280 to bus: %s", esp_err_to_name(ret));
        bme280_dev_handle = NULL;
        return ret;
    }

    // Reset
    uint8_t reset_cmd[] = {BME280_REG_RESET, BME280_RESET_VALUE};
    ret = i2c_master_transmit(bme280_dev_handle, reset_cmd, 2, -1);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    // Humidity oversampling = x1
    uint8_t hum_cfg[] = {BME280_REG_CTRL_HUM, 0x01};
    ret = i2c_master_transmit(bme280_dev_handle, hum_cfg, 2, -1);
    if (ret != ESP_OK) return ret;

    // Normal mode, Temp/Press oversampling x1
    uint8_t meas_cfg[] = {BME280_REG_CTRL_MEAS, 0x27};
    ret = i2c_master_transmit(bme280_dev_handle, meas_cfg, 2, -1);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG_BME280, "BME280 initialized successfully");
    return ESP_OK;
}

esp_err_t read_bme280(float *temperature, float *humidity, float *pressure) {
    if (bme280_dev_handle == NULL) {
        ESP_LOGE(TAG_BME280, "Device handle not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg = BME280_REG_DATA;
    uint8_t data[BME280_DATA_LEN];

    esp_err_t ret = i2c_master_transmit_receive(bme280_dev_handle, &reg, 1, data, BME280_DATA_LEN, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BME280, "Failed to read sensor data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Raw data extraction (no compensation!)
    uint32_t adc_pres = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    uint32_t adc_temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    uint16_t adc_hum  = (data[6] << 8) | data[7];

    ESP_LOGW(TAG_BME280, "Raw read values - T:%lu, P:%lu, H:%u", adc_temp, adc_pres, adc_hum);

    // Dummy conversion (replace with real compensation!)
    *temperature = adc_temp / 100000.0f;
    *pressure    = adc_pres / 25600.0f;
    *humidity    = adc_hum / 1024.0f;

    return ESP_OK;
}

