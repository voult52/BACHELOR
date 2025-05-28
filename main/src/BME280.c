#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#define BME280_ADDR             0x76
#define BME280_REG_RESET        0xE0
#define BME280_RESET_VALUE      0xB6
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_DATA         0xF7
#define BME280_REG_CALIB_00     0x88
#define BME280_REG_CALIB_26     0xE1

static const char *TAG_BME280 = "BME280";

static i2c_master_dev_handle_t bme280_dev_handle = NULL;
static struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4, dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine;
} calib;

static esp_err_t read_calibration_data() {
    uint8_t buf[26], hbuf[7];
    esp_err_t ret;

    ret = i2c_master_transmit_receive(bme280_dev_handle, (uint8_t[]){BME280_REG_CALIB_00}, 1, buf, sizeof(buf), -1);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_transmit_receive(bme280_dev_handle, (uint8_t[]){BME280_REG_CALIB_26}, 1, hbuf, sizeof(hbuf), -1);
    if (ret != ESP_OK) return ret;

    calib.dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    calib.dig_T2 = (int16_t)(buf[3] << 8 | buf[2]);
    calib.dig_T3 = (int16_t)(buf[5] << 8 | buf[4]);

    calib.dig_H1 = buf[25];
    calib.dig_H2 = (int16_t)(hbuf[1] << 8 | hbuf[0]);
    calib.dig_H3 = hbuf[2];
    calib.dig_H4 = (int16_t)((hbuf[3] << 4) | (hbuf[4] & 0x0F));
    calib.dig_H5 = (int16_t)((hbuf[5] << 4) | (hbuf[4] >> 4));
    calib.dig_H6 = (int8_t)hbuf[6];

    return ESP_OK;
}

esp_err_t bme280_init(i2c_master_bus_handle_t bus_handle) {
    i2c_device_config_t dev_cfg = {
        .device_address = BME280_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &bme280_dev_handle));

    ESP_LOGI(TAG_BME280, "Resetting sensor...");
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, (uint8_t[]){BME280_REG_RESET, BME280_RESET_VALUE}, 2, -1));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, (uint8_t[]){BME280_REG_CTRL_HUM, 0x01}, 2, -1));
    ESP_ERROR_CHECK(i2c_master_transmit(bme280_dev_handle, (uint8_t[]){BME280_REG_CTRL_MEAS, 0x25}, 2, -1));

    ESP_LOGI(TAG_BME280, "Reading calibration data...");
    return read_calibration_data();
}

static int32_t compensate_temp(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    calib.t_fine = var1 + var2;
    return (calib.t_fine * 5 + 128) >> 8;
}

static uint32_t compensate_hum(int32_t adc_H) {
    int32_t v_x1_u32r = calib.t_fine - ((int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) +
                  ((int32_t)16384)) >> 15) *
                (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                  ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r -= (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

esp_err_t bme280_read(float *temperature, float *humidity) {
    uint8_t reg = BME280_REG_DATA;
    uint8_t data[8];

    esp_err_t ret = i2c_master_transmit_receive(bme280_dev_handle, &reg, 1, data, 8, -1);
    if (ret != ESP_OK) return ret;

    int32_t adc_T = (((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4));
    int32_t adc_H = ((int32_t)data[6] << 8) | data[7];

    int32_t temp = compensate_temp(adc_T);
    uint32_t hum = compensate_hum(adc_H);

    *temperature = temp / 100.0f;
    *humidity = hum / 1024.0f;

    return ESP_OK;
}
