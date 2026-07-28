# Architecture

## Context

dump3411 is a pure-Python (3.10+) Remote ID drone detector built for a Raspberry Pi Zero W: it decodes ASTM F3411/OpenDroneID broadcasts over Bluetooth LE (via `bleak`/BlueZ) and Wi-Fi Beacon/NAN (via a raw-socket monitor-mode USB adapter), tracks currently-airborne drones, and serves a dashboard/JSON feed/MQTT stream. This project's goal is a portable, battery-powered version — a handheld unit good for a few hours of use between recharges, not an unattended field post.

No SDR/DSP is involved anywhere in dump3411's code — both radio paths receive already-demodulated frames from the host's radio hardware and do byte/bitfield parsing on top. That's the good news for portability: the *algorithm* is small and simple. The bad news is that essentially the entire "wire it up to the OS" layer — BlueZ/D-Bus, `AF_PACKET` raw sockets, `iw`/NetworkManager/`rfkill` shell-outs, systemd/journald, `sqlite3`-on-a-real-filesystem — is Linux-specific and has no equivalent in an RTOS/bare-metal target. **This is a from-scratch C/C++ rewrite against ESP-IDF, not a literal port of the Python.** The parsing logic itself (Basic ID/Location/System/Operator ID/Self-ID message decoding in dump3411's `ble_feeder.py` and `wifi_feeder.py`) transfers almost mechanically, and can finally be de-duplicated into a single shared decoder — something dump3411's own `TODO.md` already flags as wanted for the Python side too.

No hardware has been ordered yet, so this doc documents **two build tiers** rather than committing to one fixed design — pick based on budget and how much detection reliability matters, or start with the shared groundwork and decide later.

## Operating modes

An ESP32 has a single 2.4 GHz radio shared between Wi-Fi and BLE. Running continuous Wi-Fi promiscuous capture (with channel hopping) *and* continuous BLE scanning on that one radio has no known-good precedent — Espressif's own coexistence docs note BLE scan windows can get truncated by Wi-Fi activity. That risk is the deciding factor between the two modes below.

### Solo mode — one board, budget tier

A single ESP32-S3 does both radio jobs, time-sliced by ESP-IDF's coexistence arbiter (mitigated by pinning the Wi-Fi task and the NimBLE host/controller task to separate cores — the standard recommendation, and the reason to use the dual-core S3 here rather than the single-core C3). This is the cheapest way to get something working with one board and no extra modules, but it directly inherits the coexistence risk described above — **detection reliability under simultaneous Wi-Fi+BLE load is unvalidated and must be bench-measured, not assumed.** Treat this as the budget/experimental tier, not the recommended default.

Because the radio is already carrying two jobs, solo mode does **not** also run a BLE GATT peripheral — adding a third role would contend further with an already-strained radio. Output is one or both of:

- **No-solder display** — a pin-header or STEMMA QT/JST-mount OLED (e.g. an Adafruit-style FeatherWing that plugs straight into header pins, no soldering) showing a scrolling list of current detections. Phone-free, platform-agnostic.
- **USB-serial tether to Android** — the ESP32-S3 has native USB, so it can present as a USB-CDC serial device a phone reads directly. This is **Android-only in practice**: iOS requires Apple's MFi certification for any accessory to use its External Accessory framework over USB/Lightning, which is a nonstarter for a hobbyist build. The upside is it costs zero radio time (doesn't touch Wi-Fi or BLE at all) and, since charging and data share the same USB-C port on most S3 boards, tethering this way charges the battery as a side effect.

Both are documented so whoever builds it can pick based on what they have — a display for a fully standalone unit, USB-serial if an Android phone is already going to be along for the ride.

### Dual-radio mode — two boards, reliability tier

Two boards, each owning one radio outright — closer to how dump3411 already runs two independent physical radios on the Pi, and the mode with no open coexistence question:

- **Wi-Fi sensor board** (ESP32-S3) — promiscuous-mode capture, channel hop, Wi-Fi Beacon/NAN RID decode. Does nothing else; no radio contention.
- **BLE + coordinator board** (ESP32-C3) — BLE RID scan, merges in the S3's detections over UART, and runs a BLE GATT peripheral so a phone can pull live detections without ever touching the busy Wi-Fi radio. Concurrent BLE scan + GATT peripheral is a standard NimBLE multi-role pattern (still worth a bench check, but a well-trodden one, unlike Wi-Fi/BLE coexistence).

Output here is BLE GATT to a companion phone app — works on both iOS and Android, no certification needed. **This repo covers the on-device firmware and the BLE GATT protocol only; the phone app itself is a separate follow-on project.** A phone's own GPS + compass + map supersede adding onboard GPS/compass hardware for bearing/distance display in this mode too.

### Antenna: independent of mode

Onboard-PCB-trace-antenna vs. external-U.FL-antenna is a module/BOM choice at purchase time, not a firmware mode — it applies the same way whether you're building solo or dual-radio. External antenna costs a little more and needs a slightly bigger enclosure, but range matters for a detector, so it's worth it if the unit will spend most of its time outdoors.

## Shared foundation

Regardless of mode, both share the same core logic:

- **Shared ODID parser** (`odid_parser.c`/`.h`) — one C module decoding Basic ID / Location / System / Operator ID / Self-ID messages, used by both the Wi-Fi and BLE RX paths in either mode. This is the direct equivalent of what dump3411's own `TODO.md` flags as wanted for the Python side ("Extract shared decoder") — currently duplicated between `ble_feeder.py` and `wifi_feeder.py`.
- **Tracker** — a TTL-keyed MAC→drone-state table (port of `tracker.py`'s logic). In solo mode it runs on the one chip; in dual-radio mode it runs on the C3, fed by its own BLE detections plus the S3's UART-forwarded Wi-Fi detections.

**Component mapping** (dump3411 source files are the porting reference, not code to reuse directly — paths below are relative to the `dump3411` repo checked out alongside this one):

| Piece | Source of truth | ESP32 target |
|---|---|---|
| Wi-Fi RID scan | `wifi_feeder.py` (888 lines: `AF_PACKET`/`SOCK_RAW` capture at `wifi_feeder.py:832-833`, vendor-IE OUI matching, `ChannelHopper` class) | `esp_wifi_set_promiscuous(true)` + RX callback (hand off to a queue/parser task, don't do heavy work in the ISR-context callback) + a FreeRTOS timer task calling `esp_wifi_set_channel`. Runs on the S3 in either mode. |
| BLE RID scan | `ble_feeder.py` (service-data AD extraction, `ble_feeder.py:22,338`) | NimBLE GAP observer/scan, filtering AD type 0x16 service-data for the ASTM UUID. Runs on the same S3 (solo mode) or the C3 (dual-radio mode). |
| Shared ODID parser | Currently duplicated between the two feeders (flagged in dump3411's `TODO.md` under "Extract shared decoder") | One `odid_parser.c`/`.h`, compiled into whichever firmware image(s) need it — each radio path parses its own transport's payload locally |
| Tracker | `tracker.py` (TTL-keyed per-drone dict, unit conversion) | A fixed-size MAC-keyed table with periodic TTL eviction. Solo mode: runs standalone on the one chip. Dual-radio mode: runs on the C3, merging its own BLE detections with the S3's UART-forwarded Wi-Fi detections. |
| Board-to-board link (dual-radio mode only) | N/A (new) | Simple UART frame protocol, S3 → C3: parsed ODID message + MAC + RSSI per detection. Low bandwidth (RID beacons are ~1 Hz per transmitter). |
| Phone connectivity (dual-radio mode) | `feed_server.py` (JSON feed shape is the useful reference for what fields to expose), `mqtt_publisher.py` (topic/payload shape as prior art) | C3 board: BLE GATT service with a notify characteristic streaming tracker updates. **Protocol design only for now — no phone app in this repo.** |
| Explicitly dropped for v1 | `history.py` (SQLite), `/map` (Leaflet+OSM), onboard Wi-Fi HTTP dashboard, onboard GPS/compass | History/enrichment stays off-device (consistent with dump3411's own "thin producer" philosophy — see its `TODO.md` "Considered and declined" entries); mapping/bearing/distance is the phone app's job using its own GPS+compass, once that follow-on project exists |
| Deployment | `install.sh`, `dump3411.service`, journald config | N/A — replaced by `idf.py flash`/OTA per board; `ESP_LOG*` over USB-CDC serial for logging, `esp_task_wdt` watchdog instead of systemd restart-on-failure |

**Framework:** ESP-IDF v5.x native (PlatformIO `framework = espidf`, or `idf.py`) on every board, not Arduino — direct access to `esp_wifi_set_promiscuous`/`esp_wifi_set_channel` and NimBLE multi-role APIs.

## Hardware / BOM

### Solo mode

- ESP32-S3 dev board, preferably with built-in USB-C LiPo charging (Adafruit Feather ESP32-S3 or Unexpected Maker TinyS3) — one board covers both radios, charging, and native USB for the Android-tether option. ~$15-20.
- Battery: single-cell 3.7V LiPo, 1000-1500 mAh. ~$5-10.
- Optional no-solder display: a header/STEMMA-QT-mount OLED. ~$8-12.
- Optional: nothing extra for the USB-serial-to-Android path (uses the board's existing USB-C port).
- Enclosure. ~$5-15.
- **Rough total: ~$20-45** depending on whether the display is included.

### Dual-radio mode

- ESP32-S3 board (Wi-Fi sensor), U.FL external antenna variant preferred if range matters. ~$8-15 (or ~$15-20 if it's the board carrying the battery/charging).
- ESP32-C3 dev board (BLE + coordinator) — cheap, single-purpose. ~$4-6.
- Battery + charging on whichever board has it built in. ~$5-15.
- Status LED (WS2812) on the C3. ~$1.
- UART wiring between boards: two GPIO + ground, no extra component.
- Enclosure housing both boards + battery. ~$5-15.
- **Rough total: ~$35-60.**

## Power budget & runtime (handheld sizing)

Representative ESP32-family draw: Wi-Fi promiscuous RX ~95-100 mA, BLE scan roughly similar order of magnitude depending on scan window aggressiveness.

- **Solo mode:** one chip, one baseline overhead, radios time-sliced rather than truly concurrent — realistically **~120-180 mA** combined, lower than dual-radio mode's total since there's only one MCU's overhead, but the *effective* per-radio duty cycle is reduced by the arbiter (this is the same tradeoff as the reliability risk above, just expressed as a power number instead of a detection-rate number). Add ~10-20 mA if the OLED is in use; the USB-serial path adds negligible draw.
- **Dual-radio mode:** two chips, each fully dedicated to one radio — realistically **~150-220 mA** combined, slightly higher raw draw than solo mode, but predictable rather than degraded by arbitration.

Both need bench confirmation once hardware is in hand. RID beacons are spec'd at ~1 Hz, which leaves some duty-cycle slack in principle, but Wi-Fi channel-hopping across 11 channels already constrains per-channel dwell time — **run continuously for v1** in either mode rather than adding sleep-based duty-cycling, which is a later optimization once a baseline is measured.

At ~150-220 mA continuous with ~15-20% real-world derating (regulator loss, peripheral overhead, battery aging):

| Battery | Solo mode | Dual-radio mode |
|---|---|---|
| 1000 mAh | ~4.5-5.5 hours | ~4-4.5 hours |
| 1500 mAh | ~7-8 hours | ~6-7 hours |

## Build order

Phase 1 below is identical regardless of which mode you eventually land on — it only needs one ESP32-S3, so it's the right thing to build first even before deciding solo vs. dual-radio.

1. **Phase 1 (shared) — Wi-Fi-only decode on an ESP32-S3, serial output.** ESP-IDF promiscuous mode + channel hop + Beacon/NAN vendor-IE parsing, shared `odid_parser` module, log decoded messages over USB-CDC serial. This is the only hardware you need to order to get started. Medium effort — mechanical port of `wifi_feeder.py`'s frame-walking logic to C, no radio-contention risk since it's the only radio in play yet.

Once Phase 1 works, fork based on which mode you're building:

**Solo path:**

2. **Phase S2 — Add BLE scan on the same S3 chip.** This is where the coexistence question actually gets tested — measure Wi-Fi and BLE RID capture rate with both active vs. each running alone, with and without core-pinning, before deciding solo mode is good enough.
3. **Phase S3 — Add output.** No-solder display and/or USB-serial-to-Android, per what you decide to build.
4. **Phase S4 — Battery bring-up.**

**Dual-radio path:**

2. **Phase D2 — BLE decode on a second C3 board, plus the S3→C3 UART link.** NimBLE scanner reusing the shared parser; both boards still just log locally over serial at this point.
3. **Phase D3 — Tracker + BLE GATT peripheral on the C3.** Merge BLE + UART-forwarded Wi-Fi detections into one tracker; add the GATT peripheral role running concurrently with the NimBLE scan, and bench-check that concurrent scan+peripheral holds up. Define the GATT service/characteristics a future phone app will consume.
4. **Phase D4 — Battery bring-up.**

Phone app implementation is intentionally out of scope in this repo — once Phase D3's GATT protocol is stable, that becomes its own follow-on project.

## Verification

- Phase 1: confirm decoded Wi-Fi Beacon/NAN RID messages over serial match a known transmitter's broadcast (dump3411's existing `ArduPilot/ArduRemoteID`-on-ESP32-S3 bench transmitter setup described in its `TESTING.md` is directly reusable here), field-for-field against what `wifi_feeder.py` decodes from the same transmission on the Pi.
- Phase S2 / D2: same check for BLE-path decode correctness; for the dual-radio path, also confirm UART-forwarded Wi-Fi detections arrive intact on the C3.
- Phase S2 specifically: quantify Wi-Fi-only vs. BLE-only vs. combined capture rate on the solo board — this number is what decides whether solo mode is viable as-is or needs further tuning (or should be abandoned in favor of dual-radio mode).
- Phase D3: connect a BLE central (a phone's Bluetooth settings, or a generic BLE explorer app like nRF Connect) to confirm the GATT service is discoverable and streams detection notifications correctly while a scan is actively running; separately measure whether enabling the GATT peripheral role changes BLE RID capture rate versus scan-only.
- Phase S4 / D4: measure actual current draw with a USB power meter under continuous operation, compare against the power budget and runtime table above.
