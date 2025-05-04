// ble_sensor_node.c - Deep Sleep BLE Sensor Node with Optimized BLE for Supercap

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"

// NimBLE
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_hs_adv.h"
#include "store/config/ble_store_config.h"

#include "AHT21_ENS160.h"
#include "soil_moisture.h"

#define TAG "BLE_SENSOR"
#define DEVICE_NAME "ESP32_SensorNode"
#define SLEEP_INTERVAL_US (10 * 60 * 1000000ULL)

void ble_store_config_init(void);

static const ble_uuid128_t sensor_service_uuid = BLE_UUID128_INIT(0x00,0x20,0xB5,0xA3,0xF3,0x93,0xE0,0xA9,0xE5,0x0E,0xDC,0x24,0x01,0x00,0xCA,0x9E);
static const ble_uuid128_t sensor_data_char_uuid = BLE_UUID128_INIT(0x00,0x20,0xB5,0xA3,0xF3,0x93,0xE0,0xA9,0xE5,0x0E,0xDC,0x24,0x02,0x00,0xCA,0x9E);

static uint8_t ble_addr_type;
static uint16_t sensor_data_val_handle;
static int64_t start_us = 0;
static bool data_sent = false;

RTC_DATA_ATTR static bool cold_boot = true;

typedef struct __attribute__((packed)) {
    int16_t temperature;
    int16_t humidity;
    uint8_t battery;
    uint8_t light;
    uint8_t soil_moisture;
    uint8_t aqi;
    uint16_t tvoc;
    uint16_t eco2;
} sensor_payload_t;

static sensor_payload_t sensor_data;

static void read_all_sensors(sensor_payload_t *p) {
    float temp = 0, hum = 0;
    uint8_t aqi = 0; 
    uint16_t tvoc = 0, eco2 = 0;
    int raw_adc = 0;

    read_aht21(&temp, &hum);
    read_ens160(&aqi, &tvoc, &eco2);

    p->temperature = (int16_t)(temp * 10);
    p->humidity = (int16_t)(hum * 10);
    p->aqi = aqi;
    p->tvoc = tvoc;
    p->eco2 = eco2;
    p->soil_moisture = get_soil_moisture_percent(read_soil_moisture_raw());
    p->battery = read_supercap_voltage_mv();
    p->light = 0xFF;
}

static int gatt_read_cb(uint16_t conn, uint16_t handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (handle == sensor_data_val_handle) {
        os_mbuf_append(ctxt->om, &sensor_data, sizeof(sensor_data));
        data_sent = true;
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static void go_to_sleep() {
    int64_t end_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Total active time: %lld ms", (end_us - start_us) / 1000);
    esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
    esp_deep_sleep_start();
}

static void check_and_sleep(void *arg) {
    ble_gap_adv_stop();
    if (data_sent) {
        ESP_LOGI(TAG, "✅ Data sent successfully");
    } else {
        ESP_LOGI(TAG, "⚠️ No data received");
    }
    
    go_to_sleep();
}

static void start_advertising(uint32_t adv_time_s) {
    struct ble_gap_adv_params advp = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = 32,
        .itvl_max = 48
    };
    struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .uuids128 = (ble_uuid128_t*)&sensor_service_uuid,
        .num_uuids128 = 1,
        .uuids128_is_complete = 1
    };
    ble_gap_adv_set_fields(&fields);
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &advp, NULL, NULL);
    ESP_LOGI(TAG, "📡 Advertising started for %lu sec", (unsigned long)adv_time_s);
    static esp_timer_handle_t adv_timer;
    esp_timer_create_args_t tcfg = {
        .callback = check_and_sleep,
        .name = "adv_timeout"
    };
    esp_timer_create(&tcfg, &adv_timer);
    esp_timer_start_once(adv_timer, adv_time_s * 1000000ULL);
}

static void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    start_advertising(cold_boot ? 20 : 6);
    cold_boot = false;
}

static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Booting...");
    start_us = esp_timer_get_time();
    data_sent = false;

    nvs_flash_init();

    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_7,
        .sda_io_num = GPIO_NUM_6,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    i2c_new_master_bus(&cfg, &bus);
    configure_soil_moisture_adc();
    aht21_init(bus);
    ens160_init(bus);

    read_all_sensors(&sensor_data);

    nimble_port_init();
    ble_store_config_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    static struct ble_gatt_chr_def chr[] = {{
        .uuid = (ble_uuid_t*)&sensor_data_char_uuid,
        .access_cb = gatt_read_cb,
        .val_handle = &sensor_data_val_handle,
        .flags = BLE_GATT_CHR_F_READ
    }, {0}};

    static struct ble_gatt_svc_def svc[] = {{
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t*)&sensor_service_uuid,
        .characteristics = chr
    }, {0}};

    ble_gatts_count_cfg(svc);
    ble_gatts_add_svcs(svc);
    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(host_task);
}
