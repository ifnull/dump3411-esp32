# Hardware check

Firmware for the XIAO ESP32-S3 that checks the reference build's other parts after shipping: the L76K GNSS module, the ePaper driver board, and the 2.9" e-ink panel (SSD1680, per Seeed's own XIAO ePaper setup). It runs at every boot, so press the S3's **R** button to run it again.

```sh
. ~/esp/esp-idf/export.sh
cd tests/hwcheck
idf.py -p /dev/ttyACM0 flash monitor     # results are the "HWCHECK" lines, ending "HWCHECK DONE"
```

## Setups

Each part needs a solid electrical connection to the S3, so solder male headers onto the S3 first. For a one-off look at the display only, you can push header strips into the driver board's socket and press the S3 down onto them while the check runs. Expect flaky contact.

**Display and driver board:**
1. Slide the panel's ribbon fully into the driver board's FPC connector and close the latch.
2. Seat the S3 in the driver board's socket and set the board's switch to ON.
3. Power everything through the S3's USB-C.

The GNSS check reports `absent` in this setup, which is expected.

**GNSS:**
1. Stack the L76K on the S3, pin for pin. Check the install-direction photo on [Seeed's L76K page](https://wiki.seeedstudio.com/get_start_l76k_gnss/).
2. Attach the patch antenna, and put it near a window or outside, facing up.
3. Keep the S3 out of the driver board for this one.

The e-ink check fails in this setup, which is expected. The [bench rig](../../README.md#building-and-testing) wiring works too, with both connected at once.

## Reading the results

| Line | Means |
|---|---|
| `gnss RESULT=PASS (fix)` | Module, UART and antenna all work |
| `gnss RESULT=PASS (satellites in view, no fix yet)` | Module and antenna work; a cold start can take a minute or more to fix |
| `gnss RESULT=PASS (talking, no satellites ...)` | Module and UART work; check the antenna is attached and has sky |
| `gnss RESULT=WARN (many bad checksums ...)` | Data arrives but is corrupted; check the connection |
| `gnss RESULT=absent` | Nothing received in 5 s: not connected or not powered |
| `eink RESULT=PASS` | The panel refreshed twice (BUSY went high for each). Check that it shows a black frame, an X corner to corner, a solid square near one corner, and eight bars. Missing or faded areas mean panel damage. |
| `eink RESULT=FAIL (panel never went busy ...)` | The panel didn't respond: reseat the ribbon and latch, and check the S3 is fully in the socket |
| `eink RESULT=FAIL (BUSY stuck high ...)` | The panel or driver board is stuck: likely a fault |

The GNSS check prints progress every 10 s while it listens (90 s once the module is detected). The e-ink check takes about 10 s and leaves the pattern on screen.

The driver board's battery charger isn't covered: once a battery is in, plug USB-C into the driver board and check that its charge indicator lights.
