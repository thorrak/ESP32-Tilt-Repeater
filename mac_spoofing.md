# MAC Address Spoofing for Tilt Repeater

## Why

When the ESP32 repeats a Tilt's iBeacon broadcast, it currently uses the ESP32's own MAC address as the source. This means receivers see a different MAC than the original Tilt. While most Tilt receivers (TiltPi, Tilt app, TiltBridge) identify Tilts by iBeacon UUID/major/minor, some software may track or deduplicate by MAC address. Spoofing the original Tilt's MAC makes the repeater fully transparent.

## What

For each Tilt detected during a scan, capture its BLE address from the scan results. When re-broadcasting that Tilt's iBeacon data, set the ESP32's advertising address to match the original Tilt's MAC before starting the advertisement.

## Approach

Use `esp_ble_gap_set_rand_addr()` after BLE init to set the advertising address to the Tilt's MAC (with the top 2 bits of byte[0] set to mark it as a valid random static address). Then start advertising with `own_addr_type = BLE_ADDR_TYPE_RANDOM` so the BLE stack uses the spoofed address instead of the device's real public address.

## Known trade-off

The address type field in the advertising PDU will be "random" rather than "public" (which is what real Tilts use). This is unlikely to matter for iBeacon receivers but is worth noting.

## Blockers encountered on Arduino framework

- `esp_base_mac_addr_set()` rejects MACs that aren't unicast (bit 0 of first byte must be 0), and requires subtracting 2 from the target MAC since BT MAC = base + 2.
- `esp_iface_mac_addr_set()` (which sets the BT MAC directly) is only available in ESP-IDF 5.x, not in the ESP-IDF 4.4 bundled with Arduino ESP32 v2.x.
- The Arduino `BLEAdvertising` wrapper hardcodes `own_addr_type = BLE_ADDR_TYPE_PUBLIC` and doesn't expose a setter (in v2.x), requiring direct ESP-IDF API calls to bypass it.
- Repeated `BLEDevice::init()`/`deinit()` cycles cause heap corruption, which is a known issue with the Arduino BLE library and complicates any approach that needs to reinitialize BLE per-Tilt.

## Recommendation

Rewrite the project using ESP-IDF directly. This gives access to `esp_iface_mac_addr_set()`, full control over advertising parameters, and avoids the Arduino BLE library's init/deinit instability.
