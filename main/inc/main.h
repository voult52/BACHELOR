// main.h
#ifndef MAIN_H_
#define MAIN_H_

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include "driver/gpio.h" // Include for GPIO_NUM_ definitions

// I2C Master Configuration
#define I2C_MASTER_SCL_IO GPIO_NUM_7
#define I2C_MASTER_SDA_IO GPIO_NUM_6
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 1000

// Global I2C Bus Handle
extern i2c_master_bus_handle_t i2c_master_bus_handle;

// Function Prototypes for I2C Helper Functions (if defined in main.c)
esp_err_t i2c_master_write_slave(uint8_t addr, uint8_t *data, size_t len);
esp_err_t i2c_master_read_slave(uint8_t addr, uint8_t *data, size_t len);
esp_err_t i2c_master_read_slave_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

// Function Prototype for I2C Initialization (optional, but good practice)
esp_err_t i2c_master_init();

#endif // MAIN_H_