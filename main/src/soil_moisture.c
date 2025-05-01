#include "soil_moisture.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include <inttypes.h>

static const char *TAG = "soil_moisture";

#define ADC_CHANNEL      ADC1_CHANNEL_0
#define ADC_ATTEN        ADC_ATTEN_DB_12
#define ADC_WIDTH        ADC_WIDTH_BIT_12
#define DEFAULT_VREF     1100

static esp_adc_cal_characteristics_t adc_chars;

void configure_soil_moisture_adc(void)
{
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    esp_adc_cal_value_t val_type =
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, &adc_chars);
    uint32_t vref = adc_chars.vref;

    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "ADC characterized using eFuse Vref: %" PRIu32 " mV", vref);
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "ADC characterized using Two Point Vref: %" PRIu32 " mV", vref);
    } else {
        ESP_LOGI(TAG, "ADC characterized using default Vref: %" PRIu32 " mV", vref);
    }
}

int read_soil_moisture_raw(void)
{
    return adc1_get_raw(ADC_CHANNEL);
}

uint8_t get_soil_moisture_percent(int raw_adc)
{
    const int DRY = 3000;
    const int WET = 1500;
    int percent = 100 - ((raw_adc - WET) * 100) / (DRY - WET);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return (uint8_t)percent;
}
