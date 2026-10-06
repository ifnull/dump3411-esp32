# Radio notes

Why this project's radio code is built the way it is, and how it compares with two other ESP32 Remote ID projects: [Sky-Spy](https://github.com/colonelpanichacks/Sky-Spy), a receiver, and [ArduRemoteID](https://github.com/ArduPilot/ArduRemoteID), a transmitter. Both were read for reference (Sky-Spy at `1930c9b`, ArduRemoteID at `fc42673`). No code was taken from either.

## Wi-Fi capture in this project

- **Mode.** Station mode that never associates, plus promiscuous mode filtered to management frames. Beacons and NAN action frames are both management frames.
- **Frame length.** The promiscuous callback's `sig_len` includes the 4-byte FCS. It's stripped before parsing; otherwise the beacon IE walk would try to read the FCS as an element.
- **What the callback does.** It runs in the Wi-Fi driver's task, so it only does the bounded IE walk (`odid_wifi_parse_frame`). It queues a frame only if it carries Remote ID, so the ordinary beacons from nearby networks never reach the queue (16 entries of 1 KB). Decoding and the tracker run in their own task, which owns the tracker outright.
- **Channels.**
  - By default it hops channels 1–11 with a 200 ms dwell, matching dump3411's defaults; menuconfig can pin one channel.
  - Beacon Remote ID goes out on whatever channel the drone's own Wi-Fi uses, so a fixed channel misses some drones. NAN uses channel 6, the 2.4 GHz NAN discovery channel.
  - A drone beaconing once a second on one channel is heard about once per 2.2 s hop cycle in the best case. Against the bench transmitter that worked out to one or two frames every few seconds.
- **Accepted beacon elements.**
  - ASTM: OUI `FA:0B:BC`, type `0x0D`.
  - Parrot: OUI `90:3A:E6`, used by Parrot and the French Direct Remote ID scheme. Only accepted when a structurally valid Message Pack follows, because Parrot uses the OUI for unrelated elements and the type byte's value isn't documented.
  - Both carry a one-byte send counter before the message.
- **RSSI** comes from `rx_ctrl.rssi`; no radiotap header is involved on the ESP32.

## Bluetooth (not in the firmware yet)

The [coexistence smoke test](../tests/coex/README.md) shows that a full-time BLE scan on the same chip leaves promiscuous Wi-Fi capture no airtime. Solo mode needs an explicit radio schedule. Scanning both the 1M PHY (BT4) and the coded PHY (BT5 long range) doubles BLE's radio time, because the controller gives each PHY its own window.

## Sky-Spy (S3 build)

| | Sky-Spy | This project |
|---|---|---|
| Framework | Arduino on PlatformIO | ESP-IDF |
| Wi-Fi channel | Fixed on 6, no hopping | Hops 1–11, or one fixed channel |
| Decoding | In the promiscuous callback, with opendroneid-core-c's decoder | Callback only filters; a task decodes with our dump3411-tested port |
| Beacon elements | `FA:0B:BC` or `90:3A:E6`; neither the type byte nor the pack is checked | `FA:0B:BC` with type `0x0D`; `90:3A:E6` only with a valid pack |
| BLE | Arduino BLE library: active scan in 1 s bursts with a 100 ms pause, legacy adverts only, window at the library default | Not yet; coded PHY planned |
| Wi-Fi + BLE together | Both run continuously on one S3; no measurement | Measured: needs a schedule |
| Output | JSON over USB serial, buzzer and LED alerts | Serial log now; e-ink display planned |

Notes:

- **Channel 6 only.** Fixed on channel 6, Sky-Spy hears all NAN traffic, but only those beacons from drones whose Wi-Fi happens to be on channel 6.
- **Wi-Fi capture with BLE running.** Given the coexistence result, its BLE scan pattern probably leaves Wi-Fi only short gaps for capture. That's untested on our side.
- **5 GHz.** Its `xiao-c5-5g` build runs on an ESP32-C5. It alternates between channel 6 and the 5 GHz channels 149, 153, 157, 161 and 165, with a 30 ms dwell (about a 180 ms cycle).

## ArduRemoteID (transmit side)

- **Wi-Fi beacon:** it runs a soft-AP and attaches the Remote ID data to that AP's own beacons as a vendor element (`esp_wifi_set_vendor_ie`). On the air, Remote ID beacons therefore come with a real SSID and other elements around them, which our IE walk handles in any order.
- **NAN:** sent as raw action frames (`esp_wifi_80211_tx`).
- **BLE:** two extended-advertising sets on Bluedroid. One is legacy non-connectable adverts rotating through the message types (BT4); the other carries a Message Pack on the coded PHY (BT5). Intervals come from configured rates.
- **Why we didn't just use it as the bench transmitter:**
  - It only starts broadcasting once a flight controller sends a position over MAVLink or DroneCAN, unless it's set to broadcast on power-up.
  - Wi-Fi Beacon and NAN are off by default.
  - That's why `tools/rid-transmitter` exists; it builds its frames with the same opendroneid-core-c library ArduRemoteID uses.
- **Transmit-side coexistence:** its Wi-Fi and BLE share one radio, as on our C3 transmitter. Some of the frame loss seen in bench tests is on the transmit side.

## 5.8 GHz Wi-Fi

**Requirement (decided 2026-10-05):** capture Wi-Fi Remote ID on 5.8 GHz as well as 2.4 GHz.

- **On the air:** Remote ID Wi-Fi beacons and NAN can go out on 5 GHz. NAN's 5 GHz discovery channel is 149 (44 where 149 isn't allowed), and beacons follow the drone's own channel, typically 149–165 in the 5.8 GHz band.
- **Hardware:** the ESP32-S3 and ESP32-C3 are 2.4 GHz only. The ESP32-C5 has dual-band Wi-Fi 6 (2.4 + 5 GHz) and BLE 5, and comes as a XIAO board with the same footprint and socket as the S3/C3. It's a single-core RISC-V chip with one radio, so it hops between bands and can't listen to both at once.
- **Software:** ESP-IDF 5.5 supports the C5 (`esp32c5` target, `WIFI_BAND_MODE_AUTO`). The decoder, frame parser and tracker are plain C and need no changes. The capture code needs a channel plan that covers both bands, and a dwell budget across about 16 channels instead of 11.
- **Testing:** the bench transmitter needs a 5 GHz-capable board (a second C5) to produce 5.8 GHz traffic.
- **Antennas:** the boards' stock antennas are 2.4 GHz; 5.8 GHz needs dual-band antennas.

How the C5 fits the build tiers, whether it replaces the S3 as the Wi-Fi board or adds to it, gets decided once the hardware is on the bench.
