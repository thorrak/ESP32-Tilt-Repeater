// ESP32 Tilt Repeater
// Original by David Gray, rewritten for ESP-IDF

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_sleep.h"

static const char *TAG = "tilt";

/*--- USER SETTINGS ---*/
#define SCAN_TIME_S       5       // Duration to scan for BLE devices (seconds)
#define SLEEP_TIME_S      15      // Duration to deep sleep between scans (seconds)
#define FAST_SLEEP_DIV    1       // Divide sleep time when no Tilts found (1 = disable)
#define REPEAT_COLOUR     0       // 0=All, 1=Red, 2=Green, 3=Black, 4=Purple, 5=Orange, 6=Blue, 7=Yellow, 8=Pink
#define SCAN_POWER        ESP_PWR_LVL_P9
#define REPEAT_POWER      ESP_PWR_LVL_P9

/*--- TILT IDENTIFICATION ---*/
// Tilt UUID pattern: a495bb[10-80]-c5b1-4b44-b512-1370f02d74de
// Byte 3 encodes colour: 0x10=Red, 0x20=Green, ... 0x80=Pink
static const uint8_t TILT_UUID_PREFIX[] = {0xa4, 0x95, 0xbb};
static const uint8_t TILT_UUID_SUFFIX[] = {0xc5, 0xb1, 0x4b, 0x44,
                                           0xb5, 0x12, 0x13, 0x70,
                                           0xf0, 0x2d, 0x74, 0xde};

static const char *COLOUR_NAMES[] = {
    "Unknown", "Red", "Green", "Black", "Purple",
    "Orange", "Blue", "Yellow", "Pink"
};

typedef struct {
    uint8_t colour;       // 1-8
    uint16_t temp_f;      // Temperature in Fahrenheit
    uint16_t gravity;     // Specific gravity * 1000
    uint8_t uuid[16];
    esp_bd_addr_t addr;
    int rssi;
} tilt_data_t;

#define MAX_TILTS 8
static tilt_data_t found_tilts[MAX_TILTS];
static int tilt_count = 0;
static int device_count = 0;

static SemaphoreHandle_t scan_done_sem;
static SemaphoreHandle_t ble_op_sem;

// Parse raw BLE advertising data for a Tilt iBeacon.
// iBeacon AD structure: [1A] [FF] [4C 00] [02 15] [UUID:16] [Major:2] [Minor:2] [TxPwr:1]
static bool parse_tilt_from_adv(const uint8_t *adv, uint8_t adv_len, tilt_data_t *tilt) {
    int pos = 0;
    while (pos < adv_len - 1) {
        uint8_t len = adv[pos];
        if (len == 0 || pos + 1 + len > adv_len) break;

        uint8_t type = adv[pos + 1];
        if (type == 0xFF && len >= 26) {
            const uint8_t *d = &adv[pos + 2];
            // Apple company ID (0x004C LE) + iBeacon type (0x0215)
            if (d[0] == 0x4C && d[1] == 0x00 && d[2] == 0x02 && d[3] == 0x15) {
                const uint8_t *uuid = &d[4];

                if (memcmp(uuid, TILT_UUID_PREFIX, sizeof(TILT_UUID_PREFIX)) != 0 ||
                    memcmp(uuid + 4, TILT_UUID_SUFFIX, sizeof(TILT_UUID_SUFFIX)) != 0) {
                    pos += len + 1;
                    continue;
                }

                tilt->colour = uuid[3] >> 4;
                if (tilt->colour < 1 || tilt->colour > 8) return false;

                memcpy(tilt->uuid, uuid, 16);
                tilt->temp_f  = (d[20] << 8) | d[21];
                tilt->gravity = (d[22] << 8) | d[23];
                return true;
            }
        }
        pos += len + 1;
    }
    return false;
}

static void log_tilt(const tilt_data_t *tilt) {
    ESP_LOGI(TAG, "--------------------------------");
    ESP_LOGI(TAG, "Colour: %s", COLOUR_NAMES[tilt->colour]);
    ESP_LOGI(TAG, "Temp: %dF", tilt->temp_f);
    ESP_LOGI(TAG, "Gravity: %.3f", tilt->gravity / 1000.0f);
    ESP_LOGI(TAG, "RSSI: %d", tilt->rssi);
    ESP_LOGI(TAG, "--------------------------------");
}

static void advertise_tilt(const tilt_data_t *tilt) {
    uint8_t adv_data[30];
    int idx = 0;

    // Flags
    adv_data[idx++] = 0x02;  // length
    adv_data[idx++] = 0x01;  // type = Flags
    adv_data[idx++] = 0x04;  // BR_EDR_NOT_SUPPORTED

    // iBeacon manufacturer-specific data
    adv_data[idx++] = 0x1A;  // length = 26
    adv_data[idx++] = 0xFF;  // type = Manufacturer Specific
    adv_data[idx++] = 0x4C;  // Apple company ID (LE)
    adv_data[idx++] = 0x00;
    adv_data[idx++] = 0x02;  // iBeacon type
    adv_data[idx++] = 0x15;  // iBeacon data length
    memcpy(&adv_data[idx], tilt->uuid, 16);
    idx += 16;
    adv_data[idx++] = tilt->temp_f >> 8;
    adv_data[idx++] = tilt->temp_f & 0xFF;
    adv_data[idx++] = tilt->gravity >> 8;
    adv_data[idx++] = tilt->gravity & 0xFF;
    adv_data[idx++] = 0xC5;  // TX power (-59 dBm)

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, REPEAT_POWER);

    esp_ble_gap_config_adv_data_raw(adv_data, idx);
    xSemaphoreTake(ble_op_sem, pdMS_TO_TICKS(1000));

    esp_ble_adv_params_t adv_params = {
        .adv_int_min       = 0x20,
        .adv_int_max       = 0x40,
        .adv_type          = ADV_TYPE_NONCONN_IND,
        .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
        .channel_map       = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    esp_ble_gap_start_advertising(&adv_params);
    xSemaphoreTake(ble_op_sem, pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Advertising %s Tilt", COLOUR_NAMES[tilt->colour]);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_ble_gap_stop_advertising();
    xSemaphoreTake(ble_op_sem, pdMS_TO_TICKS(1000));
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(SCAN_TIME_S);
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            device_count++;
            tilt_data_t tilt = {0};
            if (!parse_tilt_from_adv(param->scan_rst.ble_adv,
                                     param->scan_rst.adv_data_len, &tilt)) {
                break;
            }

            if (REPEAT_COLOUR != 0 && REPEAT_COLOUR != tilt.colour) {
                ESP_LOGI(TAG, "%s Tilt ignored (colour filter).", COLOUR_NAMES[tilt.colour]);
                break;
            }

            // Skip duplicate colours
            bool dup = false;
            for (int i = 0; i < tilt_count; i++) {
                if (found_tilts[i].colour == tilt.colour) { dup = true; break; }
            }
            if (dup) break;

            if (tilt_count < MAX_TILTS) {
                memcpy(tilt.addr, param->scan_rst.bda, sizeof(esp_bd_addr_t));
                tilt.rssi = param->scan_rst.rssi;
                found_tilts[tilt_count++] = tilt;
                log_tilt(&tilt);
            }
        } else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            xSemaphoreGive(scan_done_sem);
        }
        break;

    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        xSemaphoreGive(ble_op_sem);
        break;

    default:
        break;
    }
}

void app_main(void) {
    // NVS is required by the BT stack
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Release classic BT memory (BLE only)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    scan_done_sem = xSemaphoreCreateBinary();
    ble_op_sem    = xSemaphoreCreateBinary();

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, SCAN_POWER);

    ESP_LOGI(TAG, "Scanning...");

    esp_ble_scan_params_t scan_params = {
        .scan_type          = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval      = 0x50,
        .scan_window        = 0x30,
        .scan_duplicate     = BLE_SCAN_DUPLICATE_ENABLE,
    };
    esp_ble_gap_set_scan_params(&scan_params);
    // Scanning starts in gap_event_handler after params are set

    xSemaphoreTake(scan_done_sem, portMAX_DELAY);
    ESP_LOGI(TAG, "%d devices found, %d Tilt(s).", device_count, tilt_count);

    // Re-broadcast each found Tilt
    for (int i = 0; i < tilt_count; i++) {
        advertise_tilt(&found_tilts[i]);
    }

    if (tilt_count == 0) {
        ESP_LOGI(TAG, "No Tilts repeated.");
    }

    // Deep sleep
    int sleep_time = (tilt_count > 0) ? SLEEP_TIME_S : (SLEEP_TIME_S / FAST_SLEEP_DIV);
    ESP_LOGI(TAG, "Sleeping for %d seconds", sleep_time);

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_enable_timer_wakeup((uint64_t)sleep_time * 1000000ULL);
    esp_deep_sleep_start();
}
