# Wi-Fi + BLE coexistence smoke test

An early data point for Phase S2: how well one ESP32-S3 captures Wi-Fi Remote ID while also scanning BLE. It's a smoke test, not the Phase S2 measurement. It's short, it uses one transmitter on one channel, and the setting is a house with ordinary Wi-Fi around.

## Method

The bench transmitter ([`tools/rid-transmitter`](../../tools/rid-transmitter/README.md), XIAO ESP32-C3, about 1 m away) sends one Wi-Fi Beacon and one NAN frame per second on channel 6. At the same time it sends BT4 legacy adverts every ~100 ms, rotating through the five message types, and one BT5 coded-PHY advert per second.

The test app (XIAO ESP32-S3) stays on channel 6 and runs each phase in turn, repeated over several rounds. Each phase settles for 2 s before counting starts.

| Phase | Wi-Fi capture | BLE scan |
|---|---|---|
| A | on | off |
| B100 / B50 / B25 / B10 | on | window = 100 / 50 / 25 / 10% of a 100 ms interval |
| C | off | window = interval |

BLE scanning is passive and covers both the 1M PHY (BT4) and the coded PHY (BT5). **The controller gives each PHY its own window per interval, so the radio spends twice the window on BLE:** B50 is effectively 100%, B25 about 50%, B10 about 20%.

For each phase the app counts:

- Remote ID Beacon and NAN frames;
- all Wi-Fi management frames on channel 6, a larger sample of ambient traffic;
- Remote ID BT4 and BT5 adverts;
- all BLE adverts.

## Results (2026-10-05)

Rates per second, averaged over the rounds. Run 1 used 3 rounds of 30 s and only phases A, B100 and C. Runs 2 and 3 used 2 rounds of 25 s. Run 3 pinned the BLE controller and NimBLE host to core 1; otherwise everything ran on core 0.

| Run | Phase | BLE share of radio | RID Beacon | RID NAN | All Wi-Fi mgmt | RID BT4 | RID BT5 |
|---|---|---|---|---|---|---|---|
| 1 | A | 0 | 0.56 | 0.72 | 10.5 | – | – |
| 1 | B100 | ~100% | 0 | 0 | 0.0 | 3.24 | 0.29 |
| 1 | C | ~100% | – | – | – | 3.11 | 0.16 |
| 2 | A | 0 | 0.40 | 0.38 | 7.5 | – | – |
| 2 | B100 | ~100% | 0 | 0 | 0.0 | 3.00 | 0.14 |
| 2 | B50 | ~100% | 0 | 0 | 0.0 | 3.14 | 0.10 |
| 2 | B25 | ~50% | 0.14 | 0.16 | 3.4 | 1.28 | 0.12 |
| 2 | B10 | ~20% | 0.26 | 0.16 | 3.4 | 0.52 | 0.04 |
| 2 | C | ~100% | – | – | – | 2.64 | 0.22 |
| 3 (core 1) | A | 0 | 0.14 | 0.20 | 4.8 | – | – |
| 3 (core 1) | B100 | ~100% | 0 | 0 | 0.0 | 3.04 | 0.22 |
| 3 (core 1) | B50 | ~100% | 0 | 0 | 0.0 | 2.88 | 0.14 |
| 3 (core 1) | B25 | ~50% | 0.08 | 0.10 | 2.6 | 1.34 | 0.10 |
| 3 (core 1) | B10 | ~20% | 0.06 | 0.18 | 7.6 | 0.58 | 0.02 |
| 3 (core 1) | C | ~100% | – | – | – | 3.08 | 0.24 |

## What it shows

- **Continuous BLE scanning stops promiscuous Wi-Fi capture completely.** Every round with BLE on the radio full time (B100, B50) captured zero Remote ID frames and at most one management frame, with either core layout. BLE itself was unaffected: B100 and C give the same BLE rates. The likely cause is ESP-IDF's coexistence scheduler, which allocates radio time by Wi-Fi state. A station that sniffs but never connects looks idle, so a full-time BLE scan gets the whole radio.
- **Leaving gaps in the BLE scan gives Wi-Fi some airtime back.** At ~50% and ~20% BLE share, both radios capture, and BLE rates drop roughly in proportion to its share. The Wi-Fi numbers are too noisy to say more than that.
- **Core pinning made no visible difference,** as expected when the radio, not the CPU, is what's shared.
- **The Wi-Fi baseline is noisy.** Phase A ranged from 0.14 to 0.72 Remote ID frames per second per transport across runs. The transmitter has the same problem on its side: the C3 also time-shares one radio between its Wi-Fi and BLE transmissions. A few frames per 25 s phase is too small a sample for fine comparisons.

## What this means for the design

Solo mode can't simply leave BLE scanning on and expect Wi-Fi capture to continue. It has to schedule the radio explicitly, either duty-cycling the BLE scan or alternating whole Wi-Fi and BLE periods, and it accepts slower detection on both. Dual-radio mode avoids this. That fits the architecture's framing of solo as the budget tier and dual-radio as the reliability tier.

For Phase S2, the useful measure for a handheld is time to first detection and how stale the track gets, not raw frame rate. Measure that against a few schedules, and use a higher transmit rate and longer runs to get past the noise.

## Running it

```sh
. ~/esp/esp-idf/export.sh
cd tests/coex
idf.py set-target esp32s3                     # once
idf.py -p /dev/ttyACM0 flash monitor          # results print as "COEX ..." lines, ending "COEX DONE"

# BLE controller and host pinned to core 1 instead:
idf.py -B build-core1 -D SDKCONFIG=build-core1/sdkconfig \
  -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.core1" -p /dev/ttyACM0 flash monitor
```

Run the bench transmitter nearby on channel 6 (its default).
