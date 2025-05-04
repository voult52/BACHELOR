#include "soil_moisture.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "soil_moisture";



// Superkondensatora mērījuma definīcijas
#define SUPERCAP_ADC_CHANNEL     ADC_CHANNEL_3      // GPIO3 = ADC1_CH3
#define SUPERCAP_ADC_UNIT        ADC_UNIT_1
#define SUPERCAP_ADC_ATTEN       ADC_ATTEN_DB_12    // Līdz ~3.3V
#define SUPERCAP_BIT_WIDTH       ADC_BITWIDTH_12
static adc_oneshot_unit_handle_t adc_handle;  // tikai viens

void configure_soil_moisture_adc(void) 
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    // Augsnes mitruma ADC kanāla konfigurācija
    adc_oneshot_chan_cfg_t chan_cfg_soil = {
        .atten = SOIL_MOISTURE_ADC_ATTEN,
        .bitwidth = SOIL_MOISTURE_BIT_WIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, SOIL_MOISTURE_ADC_CHANNEL, &chan_cfg_soil));

    // Superkondensatora ADC kanāla konfigurācija
    adc_oneshot_chan_cfg_t chan_cfg_batt = {
        .atten = SUPERCAP_ADC_ATTEN,
        .bitwidth = SUPERCAP_BIT_WIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, SUPERCAP_ADC_CHANNEL, &chan_cfg_batt));

    ESP_LOGI(TAG, "Configured ADC1 for soil moisture (ch %d) and supercap (ch %d)", 
             SOIL_MOISTURE_ADC_CHANNEL, SUPERCAP_ADC_CHANNEL);
}


int read_soil_moisture_raw(void)
{
    int raw = 0;
    if (adc_oneshot_read(adc_handle, SOIL_MOISTURE_ADC_CHANNEL, &raw) == ESP_OK) {
        return raw;
    }
    ESP_LOGW(TAG, "Failed to read soil moisture ADC");
    return -1;
}

uint8_t get_soil_moisture_percent(int raw_adc)
{
    if (raw_adc < 0) return 0;

    int percent = 100 - ((raw_adc - SOIL_MOISTURE_WET_ADC) * 100) / 
                         (SOIL_MOISTURE_DRY_ADC - SOIL_MOISTURE_WET_ADC);
    if (percent < 0) return 0;
    if (percent > 100) return 100;
    return (uint8_t)percent;
}

// Jaunā funkcija — superkondensatora sprieguma mērīšana
int read_supercap_voltage_mv(void)
{
    int raw = 0;
    if (adc_oneshot_read(adc_handle, SUPERCAP_ADC_CHANNEL, &raw) != ESP_OK) {  // <-- izmanto adc_handle
        ESP_LOGW(TAG, "Failed to read supercap ADC");
        return -1;
    }

    int mv = (raw * 3300) / 4095;
    ESP_LOGI(TAG, "Supercap voltage: %d mV (raw=%d)", mv, raw);
    return mv;
}
