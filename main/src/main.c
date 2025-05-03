#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
// NimBLE includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "host/ble_store.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_hs_adv.h"
#include "store/config/ble_store_config.h"
#include "esp_attr.h"
// Sensor includes
#include "AHT21_ENS160.h"
#include "soil_moisture.h"
static bool notify_enabled = false;
// Initialize NVS-based BLE store
void ble_store_config_init(void);

// Define CCC_NOTIFY/INDICATE if not already
#ifndef BLE_GATT_CCC_NOTIFY
#define BLE_GATT_CCC_NOTIFY 0x0001
#endif
#ifndef BLE_GATT_CCC_INDICATE
#define BLE_GATT_CCC_INDICATE 0x0002
#endif

static const char *TAG = "BLE_SENSOR_SERVER";
static const char *DEVICE_NAME = "ESP32_SensorNode";

// Custom 128-bit service & characteristic UUIDs
static const ble_uuid128_t sensor_service_uuid = BLE_UUID128_INIT(
    0x6E,0x40,0x00,0x01, 0xB5,0xA3,0xF3,0x93,
    0xE0,0xA9,0xE5,0x0E, 0x24,0xDC,0xCA,0x9E
);
static const ble_uuid128_t sensor_data_char_uuid = BLE_UUID128_INIT(
    0x6E,0x40,0x00,0x02, 0xB5,0xA3,0xF3,0x93,
    0xE0,0xA9,0xE5,0x0E, 0x24,0xDC,0xCA,0x9E
);

// CCCD (Client Characteristic Configuration Descriptor) UUID (0x2902)
static const ble_uuid16_t cccd_uuid = BLE_UUID16_INIT(0x2902);

// Packed sensor payload (10 bytes)
typedef struct {
    uint8_t temperature;
    uint8_t humidity;
    uint8_t battery;
    uint8_t light;
    uint8_t soil_moisture;
    uint8_t aqi;
    uint16_t tvoc;
    uint16_t eco2;
} __attribute__((packed)) sensor_payload_t;

static uint16_t sensor_data_val_handle;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t ble_addr_type;

#define SENSOR_UPDATE_INTERVAL_MS 10000

// Forward declarations
static void ble_app_advertise(void);
static void sensor_update_task(void *param);
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static void ble_app_on_sync(void);
static void host_task(void *param);

// Read all sensors into payload
static void read_all_sensors(sensor_payload_t *payload) {
    esp_err_t err;
    float temp_c = 0, hum_rh = 0;
    uint8_t aqi = 0;
    uint16_t tvoc = 0, eco2 = 0;
    int raw_adc = 0;

    err = read_aht21(&temp_c, &hum_rh);
    if (err == ESP_OK) {
        payload->temperature = (uint8_t)(temp_c + 0.5f);
        payload->humidity    = (uint8_t)(hum_rh + 0.5f);
    } else {
        ESP_LOGW(TAG, "Failed to read AHT21 sensor");
        payload->temperature = 0xFF;
        payload->humidity    = 0xFF;
    }

    err = read_ens160(&aqi, &tvoc, &eco2);
    if (err == ESP_OK) {
        payload->aqi  = aqi;
        payload->tvoc = tvoc;
        payload->eco2 = eco2;
    } else {
        ESP_LOGW(TAG, "Failed to read ENS160 sensor");
        payload->aqi  = 0xFF;
        payload->tvoc = 0xFFFF;
        payload->eco2 = 0xFFFF;
    }

    raw_adc = read_soil_moisture_raw();
    const int DRY = 3000, WET = 1500;
    int m = 100 - ((raw_adc - WET) * 100) / (DRY - WET);
    payload->soil_moisture = (uint8_t)(m < 0 ? 0 : (m > 100 ? 100 : m));

    // Placeholder battery and light
    payload->battery = 0xFF;
    payload->light   = 0xFF;
}

// Characteristic read callback
static int gatt_svr_chr_access(uint16_t conn, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (attr_handle != sensor_data_val_handle) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        sensor_payload_t data;
        read_all_sensors(&data);
        int rc = os_mbuf_append(ctxt->om, &data, sizeof(data));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// CCCD read/write callback
// CCCD read/write callback — now tracks notify_enabled
static int
ble_gatt_chr_cccd_access(uint16_t conn, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t ccc;
        if (os_mbuf_copydata(ctxt->om, 0, sizeof(ccc), &ccc) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        notify_enabled = (ccc & BLE_GATT_CCC_NOTIFY) != 0;
        ESP_LOGI(TAG, "Notifications %s", notify_enabled ? "ENABLED" : "DISABLED");
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}
// CCCD descriptor definition
static const struct ble_gatt_dsc_def gatt_svr_chr_cccd[] = {
    {
        .uuid      = &cccd_uuid.u,
        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
        .access_cb = ble_gatt_chr_cccd_access,
    },
    {0}
};

// GATT characteristic definition with CCCD
static const struct ble_gatt_chr_def gatt_svr_chr[] = {
    {
        .uuid        = (const ble_uuid_t *)&sensor_data_char_uuid.u,
        .access_cb   = gatt_svr_chr_access,
        .val_handle  = &sensor_data_val_handle,
        .flags       = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .descriptors = gatt_svr_chr_cccd,
    },
    {0}
};

// GATT service definition
const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type           = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid           = (const ble_uuid_t *)&sensor_service_uuid.u,
        .characteristics = gatt_svr_chr,
    },
    {0}
};




RTC_DATA_ATTR static int boot_count = 0;

static void sensor_update_task(void *param)
{
    const uint64_t deep_sleep_duration_us = 600 * 1000000ULL; // 10 minutes
    boot_count++;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    sensor_payload_t data;
    read_all_sensors(&data);

    uint32_t elapsed = 0;
    const uint32_t max_wait_time_ms = (boot_count == 1) ? 120000 : 30000; // First boot: wait max 2 mins, subsequent boots: 30s
    bool data_sent = false;

    ESP_LOGI(TAG, "Waiting for client to connect and subscribe/read (max %lu ms)...", (unsigned long)max_wait_time_ms);


    while (elapsed < max_wait_time_ms) {
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(&data, sizeof(data));
            if (om) {
                int rc = ble_gatts_notify_custom(conn_handle, sensor_data_val_handle, om);
                if (rc == 0) {
                    ESP_LOGI(TAG, "Notification sent successfully.");
                    data_sent = true;
                } else {
                    ESP_LOGW(TAG, "Notification failed: %d", rc);
                    os_mbuf_free_chain(om);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(2000)); // wait a bit for client to process
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        elapsed += 1000;
    }

    if (!data_sent) {
        ESP_LOGW(TAG, "No client subscribed or read data; going to sleep anyway.");
    }

    ESP_LOGI(TAG, "Entering deep sleep for 10 minutes...");
    esp_sleep_enable_timer_wakeup(deep_sleep_duration_us);
    esp_deep_sleep_start();

    vTaskDelete(NULL); // Should never reach here
}






// GAP event handler
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected (handle %d)", conn_handle);
        } else {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected, reason %d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Start advertising
// Start advertising
static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};
    struct ble_hs_adv_fields scan_rsp = {0};

    // Set up advertising fields: flags + your custom service UUID
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t const *)&sensor_service_uuid.u;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // Set up scan response with device name
    const char *name = DEVICE_NAME;
    scan_rsp.name = (uint8_t *)name;
    scan_rsp.name_len = strlen(name);
    scan_rsp.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&scan_rsp);

    // Connectable advertising, general discoverable
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // **This** is the correct call:**
    int rc = ble_gap_adv_start(
        ble_addr_type,          // own address type
        NULL,                   // no raw adv data; we used ble_gap_adv_set_fields()
        BLE_HS_FOREVER,         // advertise indefinitely
        &adv_params,            // our adv parameters
        ble_gap_event_cb,       // GAP event handler
        NULL                    // handler argument
    );

    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as \"%s\"", DEVICE_NAME);
    }
}


// Called once NimBLE is ready
static void ble_app_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    assert(rc == 0);
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    ble_app_advertise();
}

// Main application entry
void app_main(void) {
    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_7,
        .sda_io_num = GPIO_NUM_6,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));
    configure_soil_moisture_adc();
    ESP_ERROR_CHECK(aht21_init(bus));
    ESP_ERROR_CHECK(ens160_init(bus));

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(nvs_err);

    nimble_port_init();
    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.store_read_cb   = ble_store_config_read;
    ble_hs_cfg.store_write_cb  = ble_store_config_write;
    ble_hs_cfg.store_delete_cb = ble_store_config_delete;

    ble_hs_cfg.sm_io_cap       = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding      = 1;
    ble_hs_cfg.sm_mitm         = 0;
    ble_hs_cfg.sm_sc           = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_hs_cfg.sync_cb  = ble_app_on_sync;
    ble_hs_cfg.reset_cb = NULL;

    nimble_port_freertos_init(host_task);
    xTaskCreate(sensor_update_task, "sensor_task", 4096, NULL, 1, NULL);
}

// NimBLE host task
static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}
