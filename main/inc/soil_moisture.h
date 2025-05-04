#ifndef SOIL_MOISTURE_H
#define SOIL_MOISTURE_H

#include <stdint.h>
#include "esp_adc/adc_oneshot.h"

// --- Configuration ---
// ADC Channel (GPIO1 = ADC1_CHANNEL_0 on ESP32-C3)
#define SOIL_MOISTURE_ADC_CHANNEL ADC_CHANNEL_0
#define SOIL_MOISTURE_ADC_UNIT    ADC_UNIT_1

// Attenuation and resolution
#define SOIL_MOISTURE_ADC_ATTEN   ADC_ATTEN_DB_12
#define SOIL_MOISTURE_BIT_WIDTH   ADC_BITWIDTH_DEFAULT

// --- Calibration Thresholds ---
#define SOIL_MOISTURE_DRY_ADC     3000
#define SOIL_MOISTURE_WET_ADC     1500

// --- API ---

/**
 * @brief Configure the ADC for the soil moisture sensor.
 */
void configure_soil_moisture_adc(void);

/**
 * @brief Read the raw ADC value from the soil moisture sensor.
 *
 * @return Raw ADC value (0-4095 for 12-bit), or negative on failure.
 */
int read_soil_moisture_raw(void);

/**
 * @brief Calculate the soil moisture percentage based on calibration thresholds.
 *
 * @param raw_adc Raw ADC value.
 * @return Moisture % (0–100).
 */
uint8_t get_soil_moisture_percent(int raw_adc);

/**
 * @brief Optional: log level based on ADC.
 */
void analyze_moisture_level_category(int raw_adc);

int read_supercap_voltage_mv(void);


#endif // SOIL_MOISTURE_H
