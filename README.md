# dump3411-esp32

A battery-powered, handheld Remote ID drone detector — the firmware companion to [dump3411](https://github.com/ifnull/dump3411), reimplemented for standalone ESP32 hardware instead of a Raspberry Pi.

**Status: early planning / Phase 1 not yet started.** Nothing in this repo runs yet. See [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md) for the full design.

## What this is

dump3411 decodes the ASTM F3411/OpenDroneID Remote ID broadcasts that compliant drones transmit over Bluetooth LE and Wi-Fi, on a Raspberry Pi Zero W. This project reimplements that same decoding — BLE Basic ID / Location / System / Operator ID / Self-ID messages, and the Wi-Fi Beacon + NAN transports — as C firmware on ESP32 hardware, aiming for a pocketable, battery-powered unit instead of a Pi + USB Wi-Fi dongle + power bank.

It is **not** a literal port. The Python codebase's radio-integration layer (BlueZ/D-Bus, raw `AF_PACKET` sockets, `iw`/NetworkManager, systemd/journald) has no equivalent on an RTOS target, so this is a from-scratch rewrite against ESP-IDF. The message-parsing logic itself — small, dependency-free byte/bitfield unpacking, no SDR/DSP/ML anywhere — transfers over conceptually and is the part worth reusing as a reference.

## Why two boards

An ESP32 has a single 2.4 GHz radio shared between Wi-Fi and BLE. Running continuous Wi-Fi promiscuous capture (with channel hopping) *and* continuous BLE scanning on that one radio has no known-good precedent — Espressif's own coexistence docs note BLE scan windows can get truncated by Wi-Fi activity. Rather than build on that open question, this design uses **two small boards, each owning one radio**, closer to how dump3411 already runs two independent physical radios on the Pi:

- **Wi-Fi sensor board** (ESP32-S3) — promiscuous-mode capture, channel hop, Wi-Fi Beacon/NAN RID decode.
- **BLE + coordinator board** (ESP32-C3) — BLE RID scan, merges in the S3's detections over UART, and runs a BLE GATT peripheral so a phone can pull live detections without ever touching the busy Wi-Fi radio.

There's no onboard Wi-Fi dashboard, GPS, or compass by design — the Wi-Fi radio can't spare airtime to host an AP, and a phone's own GPS/compass/map already outclass anything embeddable. A companion phone app (separate, future repo) is the intended consumer of the BLE GATT feed.

## Hardware

See [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md#hardware--bom-handheld-build) for the full BOM. Short version: ESP32-S3 dev board + ESP32-C3 dev board + a single 1000-1500 mAh LiPo cell on whichever board has built-in USB-C charging, wired together over UART.

## Roadmap

1. **Phase 1** — Wi-Fi-only RID decode on the S3 board, logged over serial. *(current phase)*
2. **Phase 2** — BLE-only RID decode on the C3 board, plus the S3→C3 UART link.
3. **Phase 3** — Merged tracker + BLE GATT peripheral on the C3, with a bench check that concurrent BLE scan + GATT peripheral holds up.
4. **Phase 4** — Battery bring-up and real power-draw measurement.

Full detail, including the component-by-component mapping from dump3411's Python source to its ESP-IDF equivalent, the power budget, and verification steps per phase, is in [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md).

## Relationship to dump3411

This repo tracks [ifnull/dump3411](https://github.com/ifnull/dump3411) as the behavioral reference for message decoding — when the spec decoding logic changes there, it should change here too. Detection history, drone-registry enrichment, and the web dashboard stay out of scope for this device; a companion phone app and/or dump3411's own `ha-airspace` integration are the intended places for that.

## License

MIT — see [LICENSE](./LICENSE).
