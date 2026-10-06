/*
 * Seeed XIAO ESP32-S3 in the Seeed ePaper driver board (2.9" mono panel),
 * with three buttons and an optional L76K GNSS module jumper-wired. Pins
 * are GPIO numbers; the XIAO labels are in the comments. See the solo-mode
 * row of "Pin budget" in docs/ARCHITECTURE.md.
 */
#ifndef BOARD_XIAO_ESP32S3_H
#define BOARD_XIAO_ESP32S3_H

#define BOARD_NAME "Seeed XIAO ESP32-S3 + ePaper driver board"

/* E-paper, fixed by the driver board's socket. */
#define BOARD_EPD_PIN_RST    1    /* D0 */
#define BOARD_EPD_PIN_CS     2    /* D1 */
#define BOARD_EPD_PIN_BUSY   3    /* D2 */
#define BOARD_EPD_PIN_DC     4    /* D3 */
#define BOARD_EPD_PIN_SCK    7    /* D8 */
#define BOARD_EPD_PIN_MOSI   9    /* D10 */

/* Buttons, wired to GND; use the internal pull-ups. */
#define BOARD_BUTTON_COUNT   3
#define BOARD_BUTTON_1_PIN   5    /* D4 */
#define BOARD_BUTTON_2_PIN   6    /* D5 */
#define BOARD_BUTTON_3_PIN   8    /* D9 */

/* L76K GNSS, NMEA at 9600 baud. Its D0/D2 control pads are tied to 3V3. */
#define BOARD_GNSS_UART_TX_PIN  43   /* D6: S3 TX -> L76K RX */
#define BOARD_GNSS_UART_RX_PIN  44   /* D7: L76K TX -> S3 RX */

#endif
