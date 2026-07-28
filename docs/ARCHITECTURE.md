# Architecture

## Context

dump3411 is a pure-Python (3.10+) Remote ID drone detector built for a Raspberry Pi Zero W: it decodes ASTM F3411/OpenDroneID broadcasts over Bluetooth LE (via `bleak`/BlueZ) and Wi-Fi Beacon/NAN (via a raw-socket monitor-mode USB adapter), tracks currently-airborne drones, and serves a dashboard/JSON feed/MQTT stream. This project's goal is a portable, battery-powered version — a handheld unit good for a few hours of use between recharges, not an unattended field post.

No SDR/DSP is involved anywhere in dump3411's code — both radio paths receive already-demodulated frames from the host's radio hardware and do byte/bitfield parsing on top. That's the good news for portability: the *algorithm* is small and simple. The bad news is that essentially the entire "wire it up to the OS" layer — BlueZ/D-Bus, `AF_PACKET` raw sockets, `iw`/NetworkManager/`rfkill` shell-outs, systemd/journald, `sqlite3`-on-a-real-filesystem — is Linux-specific and has no equivalent in an RTOS/bare-metal target. **This is a from-scratch C/C++ rewrite against ESP-IDF, not a literal port of the Python.** The parsing logic itself (Basic ID/Location/System/Operator ID/Self-ID message decoding in dump3411's `ble_feeder.py` and `wifi_feeder.py`) transfers almost mechanically, and can finally be de-duplicated into a single shared decoder — something dump3411's own `TODO.md` already flags as wanted for the Python side too.

Two design decisions shaped this architecture:

1. **Two separate radio boards, not one shared-radio chip.** A single ESP32 has one 2.4 GHz radio time-sliced between Wi-Fi and BLE by ESP-IDF's coexistence arbiter — continuous Wi-Fi promiscuous capture + channel hopping *and* continuous BLE scanning, sustained, has no known-good precedent, and Espressif's own docs/forums confirm BLE scan windows can get truncated by Wi-Fi activity. Using **two boards, each owning one radio** (mirroring dump3411's current two-independent-radios model on the Pi, which is already proven) removes this risk entirely instead of requiring an open-ended validation spike.
2. **BLE GATT to a companion phone app, not an onboard Wi-Fi dashboard.** The Wi-Fi radio has to stay in promiscuous mode, hopping channels 1–11, to catch Wi-Fi RID beacons — it can't also host a stable SoftAP for a phone/laptop to browse a dashboard (an AP needs a fixed channel to keep clients connected). Layering an "AP window" on top would just be a second coexistence problem. Instead, the BLE board runs a connectable GATT peripheral *alongside* its RID scan — a standard, well-supported NimBLE multi-role pattern — so a phone can pull live detections without ever touching the busy Wi-Fi radio. The phone's own GPS + compass + map then supersede adding onboard GPS/compass hardware for bearing/distance display. **This repo covers the on-device firmware and the BLE GATT protocol only; the phone app itself is a separate follow-on project.**

## Recommended architecture

**Framework:** ESP-IDF v5.x native (PlatformIO `framework = espidf`, or `idf.py`) on both boards, not Arduino — direct access to `esp_wifi_set_promiscuous`/`esp_wifi_set_channel` and NimBLE multi-role APIs.

**Two boards, one battery:**

| Board | Chip | Job |
|---|---|---|
| **Wi-Fi sensor board** | ESP32-S3 (dual-core, more RAM headroom for promiscuous-frame buffering, native USB for flash/debug) | `esp_wifi_set_promiscuous` capture, channel-hop task (1–11), Beacon vendor-IE (`FA:0B:BC`) + NAN (`50:6F:9A`) parsing via the shared ODID parser. Forwards decoded messages (not raw frames) to the coordinator board over UART. No radio contention — this chip does nothing but Wi-Fi. |
| **BLE + coordinator board** | ESP32-C3 (single-core is fine — it's not sharing its radio with Wi-Fi promiscuous capture, so none of the S3's core-pinning coexistence concerns apply here) | NimBLE RID scan (own detections, via the shared ODID parser) **plus** a connectable GATT peripheral role running concurrently (standard NimBLE multi-role support — unlike Wi-Fi/BLE coexistence this is a well-trodden pattern, but still worth a bench check, see Phase 3). Receives the S3's forwarded Wi-Fi detections over UART, merges everything into one tracker, and is the single point a phone connects to. Drives a status LED (power/detecting/phone-connected). |

Powering both boards from a single LiPo is straightforward — whichever board carries the charge circuit (see BOM) feeds the other off its 3.3V rail; no second battery or charger needed.

**Component mapping** (dump3411 source files are the porting reference, not code to reuse directly — paths below are relative to the `dump3411` repo checked out alongside this one):

| Piece | Source of truth | ESP32 target |
|---|---|---|
| Wi-Fi RID scan | `wifi_feeder.py` (888 lines: `AF_PACKET`/`SOCK_RAW` capture at `wifi_feeder.py:832-833`, vendor-IE OUI matching, `ChannelHopper` class) | S3 board: `esp_wifi_set_promiscuous(true)` + RX callback (hand off to a queue/parser task, don't do heavy work in the ISR-context callback) + a FreeRTOS timer task calling `esp_wifi_set_channel` |
| BLE RID scan | `ble_feeder.py` (service-data AD extraction, `ble_feeder.py:22,338`) | C3 board: NimBLE GAP observer/scan, filtering AD type 0x16 service-data for the ASTM UUID |
| Shared ODID parser | Currently duplicated between the two feeders (flagged in dump3411's `TODO.md` under "Extract shared decoder") | One `odid_parser.c`/`.h`, compiled into both firmwares — each board parses its own transport's payload locally, then forwards the *parsed struct* (not raw bytes) upstream, which keeps the UART link small |
| Tracker | `tracker.py` (TTL-keyed per-drone dict, unit conversion) | C3 board only: merges its own BLE detections with the S3's UART-forwarded Wi-Fi detections into one MAC-keyed table with TTL eviction. Trivial RAM footprint. |
| Board-to-board link | N/A (new) | Simple UART frame protocol, S3 → C3: parsed ODID message + MAC + RSSI per detection. Low bandwidth (RID beacons are ~1 Hz per transmitter). |
| Phone connectivity | `feed_server.py` (JSON feed shape is the useful reference for what fields to expose), `mqtt_publisher.py` (topic/payload shape as prior art) | C3 board: BLE GATT service with a notify characteristic streaming tracker updates (define characteristics for: device status, per-drone detection record). **Protocol design only for now — no phone app in this repo.** |
| Explicitly dropped for v1 | `history.py` (SQLite), `/map` (Leaflet+OSM), onboard Wi-Fi HTTP dashboard, onboard GPS/compass | History/enrichment stays off-device (consistent with dump3411's own "thin producer" philosophy — see its `TODO.md` "Considered and declined" entries); mapping/bearing/distance is the phone app's job using its own GPS+compass, once that follow-on project exists |
| Deployment | `install.sh`, `dump3411.service`, journald config | N/A — replaced by `idf.py flash`/OTA per board; `ESP_LOG*` over USB-CDC serial for logging, `esp_task_wdt` watchdog instead of systemd restart-on-failure |

## Hardware / BOM (handheld build)

- **Wi-Fi board:** ESP32-S3 module/dev board with a U.FL external antenna option preferred over PCB-trace-only (range matters for a detector). ~$8-15 (or a Feather-format board if it's the one carrying the battery — see below).
- **BLE/coordinator board:** ESP32-C3 dev board, cheap and low-power since it's single-purpose (BLE + UART + tracker + GATT). ~$4-6.
- **Battery + charging:** single-cell 3.7V LiPo, **1000-1500 mAh**, on whichever board has built-in USB-C LiPo charging (e.g. Adafruit Feather ESP32-S3 or Unexpected Maker TinyS3 for the S3 role) — fastest path to a working prototype with zero custom charge-circuit design. ~$15-20 for the charging-capable board + ~$5-10 for the cell.
- **Status LED:** one WS2812 on the C3 board for power/detecting/phone-connected state. ~$1.
- **UART wiring:** just two GPIO-to-GPIO(+ground) connections between the boards, no extra component.
- **Enclosure:** small 3D-printed or off-the-shelf project box housing both boards + battery, with antenna pass-through. ~$5-15.
- **Rough total per prototype unit: ~$35-60** (slightly more than a single-chip design, offset by removing the OLED/GPS/compass that the single-chip-plus-display design would have needed).

## Power budget & runtime (handheld sizing)

Representative ESP32-family draw: Wi-Fi promiscuous RX ~95-100 mA (S3 board), BLE scan + GATT peripheral roughly similar order of magnitude, likely lower on the C3's simpler radio workload — **combined, uncontended, system draw is realistically still in the ~150-220 mA range**, similar ballpark to a single-chip design's estimate, but now *predictable* rather than degraded by radio arbitration. This needs bench confirmation once hardware is in hand (Phase 4).

RID beacons are spec'd at ~1 Hz, which leaves some duty-cycle slack in principle, but Wi-Fi channel-hopping across 11 channels already constrains per-channel dwell time. **Run continuously for v1** rather than adding sleep-based duty-cycling — that's a later optimization once the baseline is measured, not a v1 requirement for a handheld unit used in a few-hour session and recharged after.

At ~150-220 mA continuous with ~15-20% real-world derating (regulator loss, LED overhead, battery aging):

| Battery | Realistic runtime |
|---|---|
| 1000 mAh | ~4-4.5 hours |
| 1500 mAh | ~6-7 hours |

## Phased build plan

1. **Phase 1 — Wi-Fi-only decode on the S3 board, serial output.** ESP-IDF promiscuous mode + channel hop + Beacon/NAN vendor-IE parsing, shared `odid_parser` module, log decoded messages over USB-CDC serial. This is the phase to start ordering hardware and writing firmware for now. Medium effort — mechanical port of `wifi_feeder.py`'s frame-walking logic to C, no radio-contention risk since it's the only radio on this chip.
2. **Phase 2 — BLE-only decode on the C3 board, serial output, plus the UART link.** NimBLE scanner reusing the shared parser; define and implement the S3→C3 UART frame protocol so the C3 receives the S3's parsed Wi-Fi detections. Both boards still just log locally over serial at this point — this phase is about proving the shared parser and the inter-board link, not the tracker or phone connectivity yet.
3. **Phase 3 — Tracker + BLE GATT peripheral, with a bench validation step.** Implement the merged tracker on the C3 (BLE detections + UART-forwarded Wi-Fi detections). Add the GATT peripheral role running concurrently with the NimBLE scan — validate this multi-role behavior on real hardware (does concurrent scan+peripheral hold up under sustained RID scanning, does advertising/connection overhead measurably reduce scan-window coverage). This is lower-risk than the single-chip coexistence question it replaces, but still the phase's one thing to actually measure rather than assume. Define the GATT service/characteristics (device status, per-drone detection records) that a future phone app will consume.
4. **Phase 4 — Battery bring-up.** Move from USB power to the charging-capable board's LiPo circuit, measure real combined current draw across both boards on hardware, confirm the runtime table above, decide whether 1000 vs 1500 mAh better fits actual handheld use.

Phone app implementation is intentionally out of scope here — once Phase 3's GATT protocol is stable, that becomes its own follow-on project.

## Verification

- Phase 1: confirm decoded Wi-Fi Beacon/NAN RID messages over serial match a known transmitter's broadcast (dump3411's existing `ArduPilot/ArduRemoteID`-on-ESP32-S3 bench transmitter setup described in its `TESTING.md` is directly reusable here), field-for-field against what `wifi_feeder.py` decodes from the same transmission on the Pi.
- Phase 2: same check for BLE-path decode correctness on the C3, plus confirm UART-forwarded Wi-Fi detections arrive intact and are logged correctly on the C3 side.
- Phase 3: connect a BLE central (a phone's Bluetooth settings, or a generic BLE explorer app like nRF Connect) to confirm the GATT service is discoverable and streams detection notifications correctly while a scan is actively running; separately measure whether enabling the GATT peripheral role changes BLE RID capture rate versus scan-only.
- Phase 4: measure actual current draw with a USB power meter under continuous operation, compare against the ~150-220 mA budget and runtime table above.
