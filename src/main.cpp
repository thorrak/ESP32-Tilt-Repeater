// ESP32 Tilt Repeater
// Original by David Gray, rewritten for ESP-IDF with NimBLE

#include <cstring>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "NimBLEDevice.h"

static const char *TAG = "tilt";

/*--- USER SETTINGS ---*/
#define REPEAT_INTERVAL_S 15      // Minimum seconds between rebroadcasts per Tilt per message type
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
    bool is_version_msg;  // true if temp==999 (version/info message)
    uint8_t uuid[16];
    uint8_t addr[6];
    uint8_t addr_type;    // BLE_ADDR_PUBLIC or BLE_ADDR_RANDOM
    int rssi;
} tilt_data_t;

typedef struct {
    uint8_t colour;       // 1-8
    uint8_t addr[6];      // BLE MAC address
    int64_t last_rebroadcast_readings_us;  // Last rebroadcast of a temp/gravity message
    int64_t last_rebroadcast_version_us;   // Last rebroadcast of a version/info message (temp==999)
} tilt_device_t;

static std::vector<tilt_device_t> tracked_tilts;

#define MAX_PENDING 32  // 8 colours × 2 message types x 2 (why not)
static tilt_data_t pending_tilts[MAX_PENDING];
static int pending_count = 0;

static tilt_device_t* find_tracked_tilt(uint8_t colour, const uint8_t *addr) {
    for (auto &t : tracked_tilts) {
        if (t.colour == colour && memcmp(t.addr, addr, 6) == 0)
            return &t;
    }
    return nullptr;
}

static bool should_rebroadcast(uint8_t colour, const uint8_t *addr, bool is_version_msg) {
    tilt_device_t *tracked = find_tracked_tilt(colour, addr);
    if (!tracked) return true;

    int64_t now = esp_timer_get_time();
    int64_t interval = (int64_t)REPEAT_INTERVAL_S * 1000000LL;
    int64_t last = is_version_msg ? tracked->last_rebroadcast_version_us
                                  : tracked->last_rebroadcast_readings_us;
    return (now - last) >= interval;
}

static void update_tracked_tilt(const tilt_data_t *tilt) {
    int64_t now = esp_timer_get_time();
    tilt_device_t *tracked = find_tracked_tilt(tilt->colour, tilt->addr);

    if (!tracked) {
        tilt_device_t entry = {};
        entry.colour = tilt->colour;
        memcpy(entry.addr, tilt->addr, 6);
        if (tilt->is_version_msg)
            entry.last_rebroadcast_version_us = now;
        else
            entry.last_rebroadcast_readings_us = now;
        tracked_tilts.push_back(entry);
    } else {
        if (tilt->is_version_msg)
            tracked->last_rebroadcast_version_us = now;
        else
            tracked->last_rebroadcast_readings_us = now;
    }
}

static void log_tilt(const tilt_data_t *tilt) {
    if (tilt->is_version_msg) {
        ESP_LOGI(TAG, "Received %s Tilt version info (fw=%d, RSSI=%d)",
                 COLOUR_NAMES[tilt->colour], tilt->gravity, tilt->rssi);
    } else {
        ESP_LOGI(TAG, "Received %s Tilt readings (temp=%dF, gravity=%.3f, RSSI=%d)",
                 COLOUR_NAMES[tilt->colour], tilt->temp_f, tilt->gravity / 1000.0f, tilt->rssi);
    }
}

class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *device) override {

        std::string mfr = device->getManufacturerData();
        if (mfr.length() < 24) return;

        // Check Apple iBeacon header: company ID 0x004C, type 0x0215
        if ((uint8_t)mfr[0] != 0x4C || (uint8_t)mfr[1] != 0x00 ||
            (uint8_t)mfr[2] != 0x02 || (uint8_t)mfr[3] != 0x15) return;

        const uint8_t *uuid = (const uint8_t *)&mfr[4];

        if (memcmp(uuid, TILT_UUID_PREFIX, sizeof(TILT_UUID_PREFIX)) != 0 ||
            memcmp(uuid + 4, TILT_UUID_SUFFIX, sizeof(TILT_UUID_SUFFIX)) != 0) return;

        uint8_t colour = uuid[3] >> 4;
        if (colour < 1 || colour > 8) return;

        if (REPEAT_COLOUR != 0 && REPEAT_COLOUR != colour) return;

        const uint8_t *addr = device->getAddress().getVal();
        uint16_t temp_f  = ((uint8_t)mfr[20] << 8) | (uint8_t)mfr[21];
        bool is_version = (temp_f == 999);

        // Check if this colour+addr+msg_type needs rebroadcasting
        if (!should_rebroadcast(colour, addr, is_version)) return;

        // Skip if already pending for this colour+addr+msg_type
        for (int i = 0; i < pending_count; i++) {
            if (pending_tilts[i].colour == colour &&
                pending_tilts[i].is_version_msg == is_version &&
                memcmp(pending_tilts[i].addr, addr, 6) == 0) return;
        }
        if (pending_count >= MAX_PENDING) return;

        tilt_data_t &tilt = pending_tilts[pending_count++];
        tilt.colour  = colour;
        memcpy(tilt.uuid, uuid, 16);
        tilt.temp_f  = temp_f;
        tilt.gravity = ((uint8_t)mfr[22] << 8) | (uint8_t)mfr[23];
        tilt.is_version_msg = is_version;
        tilt.rssi    = device->getRSSI();
        memcpy(tilt.addr, addr, 6);
        tilt.addr_type = device->getAddress().getType();

        log_tilt(&tilt);
    }
};

static ScanCallbacks scan_callbacks;

static void advertise_tilt(const tilt_data_t *tilt) {
    // Spoof advertising address to match the original Tilt's MAC and address type
    if (tilt->addr_type == BLE_ADDR_PUBLIC) {
        // Reverse NimBLE LSB-first to ESP-IDF MSB-first byte order
        uint8_t mac[6];
        for (int j = 0; j < 6; j++) mac[j] = tilt->addr[5 - j];
        NimBLEDevice::deinit();
        esp_iface_mac_addr_set(mac, ESP_MAC_BT);
        NimBLEDevice::init("");
    } else {
        uint8_t rnd_addr[6];
        memcpy(rnd_addr, tilt->addr, 6);
        rnd_addr[5] |= 0xC0;  // Set top 2 bits for valid random static address
        NimBLEDevice::setOwnAddr(rnd_addr);
        NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
    }

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
    ESP_LOGI(TAG, "Rebroadcasting %s Tilt %s", COLOUR_NAMES[tilt->colour],
             tilt->is_version_msg ? "version info" : "readings");
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
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    NimBLEDevice::setPower(SCAN_POWER_DBM, NimBLETxPowerType::Scan);

    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(&scan_callbacks);
    pScan->setActiveScan(false);
    pScan->setDuplicateFilter(false);
    pScan->setMaxResults(0);

    ESP_LOGI(TAG, "Starting continuous scan...");
    pScan->start(0);  // 0 = scan indefinitely

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (pending_count == 0) continue;

        // Stop scan so no callbacks fire while we rebroadcast
        pScan->stop();
        while (pScan->isScanning()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        int count = pending_count;
        for (int i = 0; i < count; i++) {
            advertise_tilt(&pending_tilts[i]);
            update_tracked_tilt(&pending_tilts[i]);
        }
        ESP_LOGI(TAG, "Rebroadcast %d message(s).", count);
        pending_count = 0;

        // Resume scanning
        NimBLEDevice::setPower(SCAN_POWER_DBM, NimBLETxPowerType::Scan);
        pScan->start(0);
    }
}
