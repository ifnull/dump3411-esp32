# dump3411-esp32

A battery-powered, handheld Remote ID drone detector — the firmware companion to [dump3411](https://github.com/ifnull/dump3411), reimplemented for standalone ESP32 hardware instead of a Raspberry Pi.

**Status: Phase 1 in progress.** On a XIAO ESP32-S3 the firmware captures Wi-Fi Remote ID (Beacon and NAN), decodes it and tracks drones, logging over USB serial. The decoder and tracker are tested against dump3411's. No display or Bluetooth yet. See [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md) for the full design.

## What this is

dump3411 decodes the ASTM F3411/OpenDroneID Remote ID broadcasts that compliant drones transmit over Bluetooth LE and Wi-Fi, on a Raspberry Pi Zero W. This project reimplements that same decoding — BLE Basic ID / Location / System / Operator ID / Self-ID messages, and the Wi-Fi Beacon + NAN transports — as C firmware on ESP32 hardware, aiming for a pocketable, battery-powered unit instead of a Pi + USB Wi-Fi dongle + power bank.

It is **not** a literal port. The Python codebase's radio-integration layer (BlueZ/D-Bus, raw `AF_PACKET` sockets, `iw`/NetworkManager, systemd/journald) has no equivalent on an RTOS target, so this is a from-scratch rewrite against ESP-IDF. The message-parsing logic itself — small, dependency-free byte/bitfield unpacking, no SDR/DSP/ML anywhere — transfers over conceptually and is the part worth reusing as a reference.

## Two build tiers

This project targets two modes rather than one fixed design — see [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md#operating-modes) for the full writeup:

- **Solo mode** — one ESP32-S3 does both Wi-Fi and BLE scanning, time-sliced on its single radio. Cheapest entry point (no second board), but this is the budget tier: sustained concurrent Wi-Fi+BLE capture on one radio has no known-good precedent, so reliability has to be bench-measured, not assumed. Output is a no-solder display and/or a USB-serial tether to Android (not BLE — that radio's already busy).
- **Dual-radio mode** — a second ESP32-C3 owns BLE outright (RID scan + a BLE GATT peripheral for phone connectivity), while the ESP32-S3 does nothing but Wi-Fi. Closer to how dump3411 already runs two independent radios on the Pi, and the mode with no open coexistence question. Output is BLE GATT to a companion phone app (iOS + Android).

External-antenna vs. onboard-antenna boards is an independent choice on top of either mode.

There's no onboard Wi-Fi dashboard, GPS, or compass by design in dual-radio mode — the Wi-Fi radio can't spare airtime to host an AP, and a phone's own GPS/compass/map already outclass anything embeddable. A companion phone app (separate, future repo) is the intended consumer of the BLE GATT feed.

## Hardware

See [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md#hardware--bom) for the full BOM per mode. Solo mode: one ESP32-S3 dev board + battery. Dual-radio mode: adds an ESP32-C3 board wired to the S3 over UART.

The reference hardware is the [Seeed XIAO build](./docs/ARCHITECTURE.md#reference-build--seeed-xiao-solo--dual-radio-upgrade-path): a XIAO ESP32-S3 in Seeed's ePaper driver board with a 2.9" mono e-ink panel, an optional L76K GNSS module, and a XIAO ESP32-C3 that takes over the socket for dual-radio mode. Any ESP32-S3 board can run the firmware; see [adding support for a new board](./docs/ARCHITECTURE.md#adding-support-for-a-new-board).

## Roadmap

**Phase 1** (build this first, regardless of mode) — Wi-Fi-only RID decode on an ESP32-S3, logged over serial. *(current phase: capture, decode and tracking work; see the [status](./docs/ARCHITECTURE.md#build-order))*

Then fork based on which mode you build:

- **Solo path:** add BLE scan on the same chip and measure the coexistence tradeoff → add display/USB-serial output → battery bring-up.
- **Dual-radio path:** add a C3 board for BLE + the UART link → merged tracker + BLE GATT peripheral on the C3 → battery bring-up.

Full detail, including the component-by-component mapping from dump3411's Python source to its ESP-IDF equivalent, the power budget per mode, and verification steps per phase, is in [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md).

## Building and testing

The tests run on an ordinary Linux or macOS machine, no ESP-IDF needed. They compare against a dump3411 checkout next to this repo (or wherever `DUMP3411_DIR` points):

```sh
cmake -S host -B build-host && cmake --build build-host
python3 tests/parity/run_parity.py            # decoder vs dump3411, and vs the reference encoder's values
python3 tests/tracker/run_tracker_parity.py   # tracker vs dump3411's tracker.py
```

The tracker test needs a dump3411 whose `Tracker` accepts an injected clock.

Firmware, with [ESP-IDF v5.5](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/):

```sh
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3     # once
idf.py -p /dev/ttyACM0 flash monitor
```

The S3 runs a decoder self-test at boot, then hops Wi-Fi channels 1-11 and logs one line per tracked drone every 5 s. `idf.py menuconfig` → "dump3411-esp32" sets a fixed channel or the hop dwell time.

For something to receive on the bench, [`tools/rid-transmitter`](./tools/rid-transmitter/README.md) turns an ESP32-C3 into a low-power test transmitter broadcasting a simulated drone on all four Remote ID transports. It needs the OpenDroneID library submodule: clone with `--recurse-submodules`, or run `git submodule update --init`.

## Relationship to dump3411

This repo tracks [ifnull/dump3411](https://github.com/ifnull/dump3411) as the behavioral reference for message decoding — when the spec decoding logic changes there, it should change here too. Detection history, drone-registry enrichment, and the web dashboard stay out of scope for this device; a companion phone app and/or dump3411's own `ha-airspace` integration are the intended places for that.

## License

MIT — see [LICENSE](./LICENSE).
