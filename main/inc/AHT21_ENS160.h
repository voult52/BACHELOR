// AHT21_ENS160.h
#ifndef AHT21_ENS160_H
#define AHT21_ENS160_H

#include "driver/i2c_master.h" // Include for i2c_master_bus_handle_t
#include "esp_err.h"
#include <stdint.h>

// --- AHT21 Definitions ---
#define AHT21_ADDR                      0x38
#define AHT21_CMD_SOFT_RESET            0xBA
#define AHT21_CMD_CALIBRATION           0xBE
#define AHT21_CMD_TRIGGER_MEASUREMENT   0xAC
#define AHT21_STATUS_BUSY               0x80
#define AHT21_STATUS_CALIBRATED         0x08
#define AHT21_DATA_LEN                  7 // Status + 6 data bytes

// --- ENS160 Definitions ---
#define ENS160_ADDR                     0x53 // Or 0x52 depending on ADDR pin
#define ENS160_REG_PART_ID              0x00
#define ENS160_REG_OPMODE               0x10
#define ENS160_REG_DATA_AQI             0x21
#define ENS160_REG_DATA_TVOC            0x22
#define ENS160_REG_DATA_ECO2            0x24
#define ENS160_REG_DATA_STATUS          0x20 // Status register

// Function Prototypes
// Modified to accept the bus handle
esp_err_t aht21_init(i2c_master_bus_handle_t bus_handle);
esp_err_t read_aht21(float *temperature, float *humidity);

// Modified to accept the bus handle
esp_err_t ens160_init(i2c_master_bus_handle_t bus_handle);
esp_err_t read_ens160(uint8_t *aqi, uint16_t *tvoc, uint16_t *eco2);
// --- AHT21 Sleep/Wake ---
esp_err_t aht21_sleep(void);
esp_err_t aht21_wake(void);

// --- ENS160 Sleep/Wake ---
esp_err_t ens160_sleep(void);
esp_err_t ens160_wake(void);

#endif // AHT21_ENS160_H
