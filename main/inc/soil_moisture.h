#ifndef SOIL_MOISTURE_H
#define SOIL_MOISTURE_H

#include <stdint.h>
#include "driver/adc.h" // For ADC types if needed elsewhere
#include "esp_adc_cal.h" // For esp_adc_cal_characteristics_t

// --- Configuration ---
// ADC Channel (e.g., GPIO36)
#define SOIL_MOISTURE_ADC_CHANNEL ADC1_CHANNEL_0
// ADC Attenuation
#define SOIL_MOISTURE_ADC_ATTEN ADC_ATTEN_DB_11
// ADC Width
#define SOIL_MOISTURE_ADC_WIDTH ADC_WIDTH_BIT_12
// Default Vref (mV) - Used if eFuse Vref not available
#define SOIL_MOISTURE_DEFAULT_VREF 1100

// --- Calibration Thresholds (Adjust based on your sensor and soil) ---
// Raw ADC value corresponding to completely dry soil (or max reading)
#define SOIL_MOISTURE_DRY_ADC 3000
// Raw ADC value corresponding to fully saturated soil (or min reading in water)
#define SOIL_MOISTURE_WET_ADC 1500

// --- Public Function Prototypes ---

/**
 * @brief Configure the ADC for the soil moisture sensor.
 */
void configure_soil_moisture_adc(void);

/**
 * @brief Read the raw ADC value from the soil moisture sensor.
 *
 * @return Raw ADC value (0-4095 for 12-bit), or a negative value on error (less common for raw read).
 */
int read_soil_moisture_raw(void);

/**
 * @brief Convert a raw ADC value to voltage using pre-calculated calibration characteristics.
 *
 * @param raw_adc The raw ADC value.
 * @return Voltage in millivolts (mV), or 0 if calibration characteristics are not available or raw_adc is invalid.
 */
uint32_t get_soil_moisture_voltage(int raw_adc);

/**
 * @brief Calculate the soil moisture percentage based on raw ADC reading and calibration thresholds.
 *
 * Maps the raw ADC value to a percentage (0-100%), where 0% is dry and 100% is wet.
 * Clamps the input raw value to the defined DRY/WET thresholds before mapping.
 * Handles potential invalid raw_adc input and invalid threshold configurations.
 *
 * @param raw_adc The raw ADC value from the sensor.
 * @return Soil moisture percentage (0-100). Returns 0 if raw_adc is negative or thresholds are invalid.
 */
uint8_t get_soil_moisture_percent(int raw_adc);

/**
 * @brief Logs the general moisture level based on raw ADC reading and separate categorization thresholds.
 *
 * @param raw_adc The raw ADC value.
 */
void analyze_moisture_level_category(int raw_adc);

#endif // SOIL_MOISTURE_H
