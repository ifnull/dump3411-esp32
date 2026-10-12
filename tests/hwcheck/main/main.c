/*
 * Hardware check for the reference build (XIAO ESP32-S3), run at every boot:
 *
 *   1. GNSS: listen for the L76K's NMEA output on D7 for a few seconds. If it
 *      talks, keep listening and report valid sentences, satellites in view
 *      and fix status. Satellites in view need the patch antenna and some
 *      sky; a fix can take a minute or more from cold.
 *   2. E-ink: reset and initialize the SSD1680 panel, then refresh it twice
 *      (all black, then a test pattern), timing the BUSY line. The panel
 *      holds BUSY high while it refreshes, so seeing that proves the driver
 *      board, the socket and the panel's FPC connection are all working.
 *
 * Results go to the USB console as "HWCHECK ..." lines. The test pattern
 * stays on the panel: a black frame, an X corner to corner, a solid square in
 * the top-left corner, and eight vertical bars.
 */
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hwcheck";

/* ---------------------------------------------------------------- GNSS -- */

#define GNSS_UART       UART_NUM_1
#define GNSS_BAUD       9600
#define GNSS_DETECT_MS  5000
#define GNSS_LISTEN_MS  90000

typedef struct {
    unsigned sentences, bad_checksum, gga, rmc, gsv;
    int sats_in_view;    /* summed across constellations, latest GSV of each */
    int fix_quality;     /* GGA field 6: 0 none, 1 GPS, 2 DGPS, ... */
    int sats_used;       /* GGA field 7 */
    char last_gga[96];
} gnss_stats_t;

/* Per-talker (GP, GL, GA, GB, GQ, GN) satellites in view from GSV. */
static int talker_view[8];

static int talker_index(const char *s)
{
    static const char *const talkers[] = {"GP", "GL", "GA", "GB", "GQ", "GN", "BD", "GI"};
    for (int i = 0; i < 8; i++) {
        if (s[1] == talkers[i][0] && s[2] == talkers[i][1]) {
            return i;
        }
    }
    return -1;
}

/* Return field n (0 = sentence id) of a comma-separated NMEA sentence. */
static const char *field(const char *s, int n)
{
    while (n > 0 && *s) {
        if (*s++ == ',') {
            n--;
        }
    }
    return n == 0 ? s : "";
}

static bool checksum_ok(const char *s)
{
    const char *star = strchr(s, '*');
    if (s[0] != '$' || star == NULL || strlen(star) < 3) {
        return false;
    }
    unsigned char sum = 0;
    for (const char *p = s + 1; p < star; p++) {
        sum ^= (unsigned char)*p;
    }
    return sum == (unsigned char)strtol(star + 1, NULL, 16);
}

static void handle_sentence(gnss_stats_t *g, const char *s)
{
    if (!checksum_ok(s)) {
        g->bad_checksum++;
        return;
    }
    g->sentences++;
    if (strlen(s) < 7) {
        return;
    }
    const char *type = s + 3;
    if (strncmp(type, "GGA", 3) == 0) {
        g->gga++;
        g->fix_quality = atoi(field(s, 6));
        g->sats_used = atoi(field(s, 7));
        strncpy(g->last_gga, s, sizeof(g->last_gga) - 1);
    } else if (strncmp(type, "RMC", 3) == 0) {
        g->rmc++;
    } else if (strncmp(type, "GSV", 3) == 0) {
        g->gsv++;
        int t = talker_index(s);
        if (t >= 0) {
            talker_view[t] = atoi(field(s, 3));
        }
        g->sats_in_view = 0;
        for (int i = 0; i < 8; i++) {
            g->sats_in_view += talker_view[i];
        }
    }
}

/* Read NMEA for up to ms milliseconds, or detect_ms if nothing arrives at all. */
static void gnss_listen(gnss_stats_t *g, uint32_t ms, uint32_t detect_ms)
{
    static char line[128];
    size_t n = 0;
    int64_t start = esp_timer_get_time();
    int64_t next_report = start + 10 * 1000000LL;
    for (;;) {
        int64_t now = esp_timer_get_time();
        uint32_t elapsed = (uint32_t)((now - start) / 1000);
        if (elapsed >= ms || (g->sentences == 0 && g->bad_checksum == 0 && elapsed >= detect_ms)) {
            return;
        }
        uint8_t c;
        if (uart_read_bytes(GNSS_UART, &c, 1, pdMS_TO_TICKS(50)) == 1) {
            if (c == '\n' || c == '\r') {
                if (n > 0) {
                    line[n] = '\0';
                    handle_sentence(g, line);
                    n = 0;
                }
            } else if (n < sizeof(line) - 1) {
                line[n++] = (char)c;
            } else {
                n = 0;    /* overlong: not NMEA, drop it */
            }
        }
        if (g->sentences > 0 && now >= next_report) {
            next_report += 10 * 1000000LL;
            ESP_LOGI(TAG, "HWCHECK gnss t=%lus sentences=%u bad=%u sats_in_view=%d fix=%d sats_used=%d",
                     (unsigned long)(elapsed / 1000), g->sentences, g->bad_checksum,
                     g->sats_in_view, g->fix_quality, g->sats_used);
        }
    }
}

static void check_gnss(void)
{
    /* The L76K's D0/D2 control inputs must be high. Stacked on the S3 they
     * meet the S3's D0 (driven high) and D2 (pulled up); on the bench rig
     * they're tied to 3V3 and these are the display's RST and BUSY, where
     * this is harmless. */
    gpio_set_direction(BOARD_EPD_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(BOARD_EPD_PIN_RST, 1);
    gpio_set_direction(BOARD_EPD_PIN_BUSY, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOARD_EPD_PIN_BUSY, GPIO_PULLUP_ONLY);

    uart_config_t cfg = {
        .baud_rate = GNSS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GNSS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART, BOARD_GNSS_UART_TX_PIN, BOARD_GNSS_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "HWCHECK gnss listening on RX GPIO%d at %d baud", BOARD_GNSS_UART_RX_PIN, GNSS_BAUD);
    static gnss_stats_t g;
    gnss_listen(&g, GNSS_LISTEN_MS, GNSS_DETECT_MS);
    uart_driver_delete(GNSS_UART);

    if (g.sentences == 0 && g.bad_checksum == 0) {
        ESP_LOGW(TAG, "HWCHECK gnss RESULT=absent: nothing received in %d s (not connected, or not powered)",
                 GNSS_DETECT_MS / 1000);
        return;
    }
    const char *result = g.sentences == 0                          ? "FAIL (data, but no valid sentence)"
                       : g.bad_checksum > g.sentences / 10          ? "WARN (many bad checksums: check wiring)"
                       : g.fix_quality > 0                          ? "PASS (fix)"
                       : g.sats_in_view > 0                         ? "PASS (satellites in view, no fix yet)"
                                                                    : "PASS (talking, no satellites: antenna or sky view?)";
    ESP_LOGI(TAG, "HWCHECK gnss RESULT=%s sentences=%u bad=%u gga=%u rmc=%u gsv=%u sats_in_view=%d fix=%d sats_used=%d",
             result, g.sentences, g.bad_checksum, g.gga, g.rmc, g.gsv, g.sats_in_view, g.fix_quality,
             g.sats_used);
    if (g.last_gga[0]) {
        ESP_LOGI(TAG, "HWCHECK gnss last %s", g.last_gga);
    }
}

/* ---------------------------------------------------------------- e-ink -- */

#define EPD_W        128            /* SSD1680, 128 x 296, 1 bit per pixel */
#define EPD_H        296
#define EPD_ROW      (EPD_W / 8)
#define EPD_BYTES    (EPD_ROW * EPD_H)
#define BUSY_TIMEOUT_MS 10000

static spi_device_handle_t spi;
static uint8_t frame[EPD_BYTES];

static void epd_send(bool data, const uint8_t *buf, size_t len)
{
    gpio_set_level(BOARD_EPD_PIN_DC, data);
    spi_transaction_t t = {.length = len * 8, .tx_buffer = buf};
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

static void epd_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    epd_send(false, &cmd, 1);
    if (len) {
        epd_send(true, data, len);
    }
}

#define CMD(c, ...)                                                     \
    do {                                                                \
        const uint8_t d_[] = {__VA_ARGS__};                             \
        epd_cmd((c), d_, sizeof(d_));                                   \
    } while (0)

/* Wait for BUSY to go low. Returns how long it stayed high, or -1 on timeout. */
static int wait_busy(void)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(BOARD_EPD_PIN_BUSY)) {
        if ((esp_timer_get_time() - start) / 1000 > BUSY_TIMEOUT_MS) {
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return (int)((esp_timer_get_time() - start) / 1000);
}

/* Full refresh; returns ms BUSY was high, 0 if it never rose, -1 on timeout. */
static int epd_refresh(void)
{
    CMD(0x4E, 0x00);                 /* RAM X counter */
    CMD(0x4F, 0x00, 0x00);           /* RAM Y counter */
    uint8_t c = 0x24;                /* write black/white RAM */
    epd_send(false, &c, 1);
    for (size_t off = 0; off < EPD_BYTES; off += 2048) {
        size_t n = EPD_BYTES - off < 2048 ? EPD_BYTES - off : 2048;
        epd_send(true, frame + off, n);
    }
    CMD(0x22, 0xF7);                 /* full update sequence */
    uint8_t act = 0x20;              /* master activation */
    epd_send(false, &act, 1);

    /* BUSY should rise almost at once; give it 100 ms. */
    int64_t start = esp_timer_get_time();
    while (!gpio_get_level(BOARD_EPD_PIN_BUSY)) {
        if ((esp_timer_get_time() - start) / 1000 > 100) {
            return 0;
        }
    }
    return wait_busy();
}

static void px(int x, int y, bool black)
{
    if (x < 0 || x >= EPD_W || y < 0 || y >= EPD_H) {
        return;
    }
    uint8_t mask = 0x80 >> (x % 8);
    uint8_t *b = &frame[y * EPD_ROW + x / 8];
    *b = black ? (uint8_t)(*b & ~mask) : (uint8_t)(*b | mask);   /* 1 = white */
}

static void draw_pattern(void)
{
    memset(frame, 0xFF, sizeof(frame));                  /* white */
    for (int i = 0; i < 4; i++) {                        /* frame */
        for (int x = 0; x < EPD_W; x++) { px(x, i, true); px(x, EPD_H - 1 - i, true); }
        for (int y = 0; y < EPD_H; y++) { px(i, y, true); px(EPD_W - 1 - i, y, true); }
    }
    for (int y = 0; y < EPD_H; y++) {                    /* X, 3 px wide */
        int x = y * (EPD_W - 1) / (EPD_H - 1);
        for (int w = -1; w <= 1; w++) { px(x + w, y, true); px(EPD_W - 1 - x + w, y, true); }
    }
    for (int y = 10; y < 34; y++) {                      /* top-left square */
        for (int x = 10; x < 34; x++) { px(x, y, true); }
    }
    for (int bar = 0; bar < 8; bar++) {                  /* 8 bars near the bottom */
        int x0 = 12 + bar * 14;
        for (int y = EPD_H - 70; y < EPD_H - 14; y++) {
            for (int x = x0; x < x0 + 7; x++) { px(x, y, true); }
        }
    }
}

static void check_eink(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << BOARD_EPD_PIN_RST) | (1ULL << BOARD_EPD_PIN_DC),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    /* Pulled down, so a missing panel reads "never busy" instead of floating. */
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << BOARD_EPD_PIN_BUSY,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    spi_bus_config_t bus = {
        .mosi_io_num = BOARD_EPD_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = BOARD_EPD_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 4 * 1000 * 1000,    /* slow on purpose: tolerant of loose contacts */
        .mode = 0,
        .spics_io_num = BOARD_EPD_PIN_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &spi));

    /* Hardware reset, software reset, then the SSD1680 setup sequence. */
    gpio_set_level(BOARD_EPD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BOARD_EPD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    int reset_busy = wait_busy();
    CMD(0x12);                                   /* software reset */
    vTaskDelay(pdMS_TO_TICKS(10));
    int swreset_busy = wait_busy();
    CMD(0x01, (EPD_H - 1) & 0xFF, (EPD_H - 1) >> 8, 0x00);   /* driver output control */
    CMD(0x11, 0x03);                             /* data entry: X then Y increment */
    CMD(0x44, 0x00, EPD_ROW - 1);                /* RAM X range */
    CMD(0x45, 0x00, 0x00, (EPD_H - 1) & 0xFF, (EPD_H - 1) >> 8);   /* RAM Y range */
    CMD(0x3C, 0x05);                             /* border waveform */
    CMD(0x18, 0x80);                             /* internal temperature sensor */
    ESP_LOGI(TAG, "HWCHECK eink reset busy_ms=%d swreset busy_ms=%d", reset_busy, swreset_busy);

    memset(frame, 0x00, sizeof(frame));          /* all black */
    int black_ms = epd_refresh();
    ESP_LOGI(TAG, "HWCHECK eink refresh=black busy_ms=%d", black_ms);
    draw_pattern();
    int pattern_ms = epd_refresh();
    ESP_LOGI(TAG, "HWCHECK eink refresh=pattern busy_ms=%d", pattern_ms);
    CMD(0x10, 0x01);                             /* deep sleep; the image stays */

    if (reset_busy < 0 || swreset_busy < 0 || black_ms < 0 || pattern_ms < 0) {
        ESP_LOGE(TAG, "HWCHECK eink RESULT=FAIL (BUSY stuck high: panel or driver board fault)");
    } else if (black_ms == 0 || pattern_ms == 0) {
        ESP_LOGE(TAG, "HWCHECK eink RESULT=FAIL (panel never went busy: check the FPC is fully "
                      "inserted and latched, and the XIAO is seated in the socket)");
    } else if (black_ms < 500 || pattern_ms < 500) {
        ESP_LOGW(TAG, "HWCHECK eink RESULT=WARN (refresh unusually short; check the panel shows the pattern)");
    } else {
        ESP_LOGI(TAG, "HWCHECK eink RESULT=PASS (panel refreshed twice; check it shows the test pattern)");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "HWCHECK START board=\"%s\"", BOARD_NAME);
    check_gnss();
    check_eink();
    ESP_LOGI(TAG, "HWCHECK DONE");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
