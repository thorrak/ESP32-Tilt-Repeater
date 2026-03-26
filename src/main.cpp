// ESP32 Tilt Repeater
// Original by David Gray, rewritten for ESP-IDF with NimBLE

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "NimBLEDevice.h"

static const char *TAG = "tilt";

/*--- USER SETTINGS ---*/
#define SCAN_TIME_S       5       // Duration to scan for BLE devices (seconds)
#define SLEEP_TIME_S      15      // Duration to deep sleep between scans (seconds)
#define FAST_SLEEP_DIV    1       // Divide sleep time when no Tilts found (1 = disable)
#define REPEAT_COLOUR     0       // 0=All, 1=Red, 2=Green, 3=Black, 4=Purple, 5=Orange, 6=Blue, 7=Yellow, 8=Pink
#define SCAN_POWER_DBM    9       // TX power in dBm while scanning
#define REPEAT_POWER_DBM  9       // TX power in dBm while repeating

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
    uint8_t addr[6];
    int rssi;
} tilt_data_t;

#define MAX_TILTS 8
static tilt_data_t found_tilts[MAX_TILTS];
static int tilt_count = 0;
static int device_count = 0;

static void log_tilt(const tilt_data_t *tilt) {
    ESP_LOGI(TAG, "--------------------------------");
    ESP_LOGI(TAG, "Colour: %s", COLOUR_NAMES[tilt->colour]);
    ESP_LOGI(TAG, "Temp: %dF", tilt->temp_f);
    ESP_LOGI(TAG, "Gravity: %.3f", tilt->gravity / 1000.0f);
    ESP_LOGI(TAG, "RSSI: %d", tilt->rssi);
    ESP_LOGI(TAG, "--------------------------------");
}

class ScanCallbacks : public NimBLEScanCallbacks {
    void onDiscovered(const NimBLEAdvertisedDevice *device) override {
        device_count++;

        std::string mfr = device->getManufacturerData();
        if (mfr.length() < 25) return;

        // Check Apple iBeacon header: company ID 0x004C, type 0x0215
        if ((uint8_t)mfr[0] != 0x4C || (uint8_t)mfr[1] != 0x00 ||
            (uint8_t)mfr[2] != 0x02 || (uint8_t)mfr[3] != 0x15) return;

        const uint8_t *uuid = (const uint8_t *)&mfr[4];

        if (memcmp(uuid, TILT_UUID_PREFIX, sizeof(TILT_UUID_PREFIX)) != 0 ||
            memcmp(uuid + 4, TILT_UUID_SUFFIX, sizeof(TILT_UUID_SUFFIX)) != 0) return;

        uint8_t colour = uuid[3] >> 4;
        if (colour < 1 || colour > 8) return;

        if (REPEAT_COLOUR != 0 && REPEAT_COLOUR != colour) {
            ESP_LOGI(TAG, "%s Tilt ignored (colour filter).", COLOUR_NAMES[colour]);
            return;
        }

        // Skip duplicate colours
        for (int i = 0; i < tilt_count; i++) {
            if (found_tilts[i].colour == colour) return;
        }
        if (tilt_count >= MAX_TILTS) return;

        tilt_data_t &tilt = found_tilts[tilt_count++];
        tilt.colour  = colour;
        memcpy(tilt.uuid, uuid, 16);
        tilt.temp_f  = ((uint8_t)mfr[20] << 8) | (uint8_t)mfr[21];
        tilt.gravity = ((uint8_t)mfr[22] << 8) | (uint8_t)mfr[23];
        tilt.rssi    = device->getRSSI();
        memcpy(tilt.addr, device->getAddress().getVal(), 6);

        log_tilt(&tilt);
    }
};

static ScanCallbacks scan_callbacks;

static void advertise_tilt(const tilt_data_t *tilt) {
    // Build iBeacon manufacturer data payload (25 bytes)
    uint8_t mfr_data[25];
    mfr_data[0] = 0x4C;  // Apple company ID (LE)
    mfr_data[1] = 0x00;
    mfr_data[2] = 0x02;  // iBeacon type
    mfr_data[3] = 0x15;  // iBeacon data length
    memcpy(&mfr_data[4], tilt->uuid, 16);
    mfr_data[20] = tilt->temp_f >> 8;
    mfr_data[21] = tilt->temp_f & 0xFF;
    mfr_data[22] = tilt->gravity >> 8;
    mfr_data[23] = tilt->gravity & 0xFF;
    mfr_data[24] = 0xC5;  // TX power (-59 dBm)

    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData advData;
    advData.setFlags(0x04);  // BR_EDR_NOT_SUPPORTED
    advData.setManufacturerData(mfr_data, sizeof(mfr_data));
    pAdv->setAdvertisementData(advData);
    pAdv->setConnectableMode(BLE_GAP_CONN_MODE_NON);

    NimBLEDevice::setPower(REPEAT_POWER_DBM, NimBLETxPowerType::Advertise);

    pAdv->start();
    ESP_LOGI(TAG, "Advertising %s Tilt", COLOUR_NAMES[tilt->colour]);
    vTaskDelay(pdMS_TO_TICKS(100));
    pAdv->stop();
}

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    NimBLEDevice::init("");

    while (true) {
        tilt_count = 0;
        device_count = 0;

        NimBLEDevice::setPower(SCAN_POWER_DBM, NimBLETxPowerType::Scan);

        NimBLEScan *pScan = NimBLEDevice::getScan();
        pScan->setScanCallbacks(&scan_callbacks);
        pScan->setActiveScan(true);
        pScan->setMaxResults(0);

        ESP_LOGI(TAG, "Scanning...");
        pScan->start(SCAN_TIME_S * 1000);
        while (pScan->isScanning()) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ESP_LOGI(TAG, "%d devices found, %d Tilt(s).", device_count, tilt_count);

        for (int i = 0; i < tilt_count; i++) {
            advertise_tilt(&found_tilts[i]);
        }

        if (tilt_count == 0) {
            ESP_LOGI(TAG, "No Tilts repeated.");
        }

        int sleep_time = (tilt_count > 0) ? SLEEP_TIME_S : (SLEEP_TIME_S / FAST_SLEEP_DIV);
        ESP_LOGI(TAG, "Sleeping for %d seconds", sleep_time);

        NimBLEDevice::deinit();
        esp_sleep_enable_timer_wakeup((uint64_t)sleep_time * 1000000ULL);
        esp_light_sleep_start();
        NimBLEDevice::init("");
    }
}
