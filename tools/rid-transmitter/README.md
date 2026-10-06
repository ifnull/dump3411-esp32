# Bench Remote ID transmitter

Firmware for an ESP32-C3 (built for the Seeed XIAO ESP32-C3) that broadcasts a simulated drone over all four Remote ID transports at once: Wi-Fi Beacon, Wi-Fi NAN, Bluetooth 4 legacy and Bluetooth 5 long range (coded PHY). It gives receivers on the bench (dump3411, this repo's firmware) known traffic to decode. It is a test tool, not a flight transmitter.

Every message is encoded by the OpenDroneID reference library (`third_party/opendroneid-core-c`, Apache-2.0, pinned to the same commit as the parity fixtures). The Wi-Fi frames come straight from its frame builders and are sent raw with `esp_wifi_80211_tx`. The Bluetooth service-data framing (`0x16`, UUID `0xFFFA`, app code `0x0D`, counter, message) is written here following ASTM F3411.

## What it sends

- **UAS ID** `DUMP3411-BENCH-TX-01` (serial number, multirotor), Self ID `BENCH TEST TRANSMITTER`, Operator ID `TEST-OPERATOR-0001`.
- **Flight:** clockwise 150 m circles around the configured center, 60 m AGL with a slow ±20 m climb and descent. Each minute is 40 s at 12 m/s, then 20 s at 70 m/s to exercise the high-speed encoding.
- **Operator:** live GNSS at the circle's center, 250 m altitude.
- **Default center:** the geographic center of the contiguous US (39.8283, -98.5795), an obviously fake spot.
- **Rates:** Wi-Fi Beacon and NAN each send a Message Pack once a second on channel 6. BT4 rotates through the five message types, so each type goes out once a second. BT5 sends a Message Pack once a second.
- **Serial log:** the console prints the true position, track, speed and height once a second, so a receiver's decode can be compared against it.

Transmit power is turned down (Wi-Fi 2 dBm, BLE -12 dBm) so the test drone stays on the bench and doesn't show up on detectors next door.

## Build and flash

```sh
git submodule update --init third_party/opendroneid-core-c
. ~/esp/esp-idf/export.sh
cd tools/rid-transmitter
idf.py set-target esp32c3        # once
idf.py menuconfig                # optional: "RID bench transmitter" for center, channel, power
idf.py -p /dev/ttyACMx flash monitor
```

With more than one board plugged in, check which port is the C3 first, e.g. `esptool --port /dev/ttyACMx chip-id`.
