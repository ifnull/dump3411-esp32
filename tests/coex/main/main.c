/*
 * Wi-Fi + BLE coexistence smoke test (Phase 1 step 8, an early data point
 * for Phase S2).
 *
 * With the bench transmitter (tools/rid-transmitter) running nearby, this
 * cycles through three phases and counts what it hears in each:
 *
 *   A     Wi-Fi only: promiscuous capture on a fixed channel, BLE idle
 *   B100, B50, B25, B10
 *         Wi-Fi capture plus BLE scanning with a window of that share of
 *         each 100 ms interval
 *   C     BLE only: Wi-Fi stopped, BLE scanning (window = interval)
 *
 * The BLE scan covers two PHYs and the controller gives each its own window
 * per interval, so the radio's BLE share is twice the window: B50 keeps the
 * radio on BLE all the time, B25 about half, B10 about a fifth.
 *
 * Wi-Fi stays on one channel (the transmitter's) so hopping doesn't blur the
 * numbers. BLE scans both the 1M PHY (BT4 legacy adverts) and the coded PHY
 * (BT5 long range). Each phase settles for a moment before counting. Results go to the console as
 * "COEX ..." lines, ending with a per-phase summary and "COEX DONE".
 */
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "odid_wifi.h"

static const char *TAG = "coex";

#define WIFI_CHANNEL  6
#define ROUNDS        2
#define PHASE_MS      25000
#define SETTLE_MS     2000
#define SCAN_ITVL     160         /* 0.625 ms units: 100 ms */

#ifdef CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1
#define WIFI_TASK_CORE 1
#else
#define WIFI_TASK_CORE 0
#endif

typedef struct {
    uint32_t mgmt, beacon, nan, bt4, bt5, ble_all;
} counts_t;

static volatile bool counting;
static volatile counts_t c;
static volatile bool ble_synced;

/* -- Wi-Fi ----------------------------------------------------------------- */

static void on_packet(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!counting || type != WIFI_PKT_MGMT) {
        return;
    }
    const wifi_promiscuous_pkt_t *pkt = buf;
    c.mgmt++;
    if (pkt->rx_ctrl.sig_len <= 4) {
        return;
    }
    odid_wifi_frame_t f;
    if (odid_wifi_parse_frame(pkt->payload, pkt->rx_ctrl.sig_len - 4, &f)) {
        if (f.transport == ODID_WIFI_BEACON) c.beacon++;
        if (f.transport == ODID_WIFI_NAN)    c.nan++;
    }
}

static void wifi_init(void)
{
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(on_packet));
}

static void wifi_on(void)
{
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void wifi_off(void)
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
    ESP_ERROR_CHECK(esp_wifi_stop());
}

/* -- BLE ------------------------------------------------------------------- */

/* Service Data AD for UUID 0xFFFA with ASTM app code 0x0D. */
static bool has_rid(const uint8_t *d, size_t len)
{
    size_t i = 0;
    while (i + 1 < len) {
        uint8_t ad_len = d[i];
        if (ad_len == 0 || i + 1 + ad_len > len) {
            return false;
        }
        const uint8_t *ad = d + i + 1;
        if (ad_len >= 4 && ad[0] == 0x16 && ad[1] == 0xFA && ad[2] == 0xFF && ad[3] == 0x0D) {
            return true;
        }
        i += 1 + ad_len;
    }
    return false;
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_EXT_DISC || !counting) {
        return 0;
    }
    const struct ble_gap_ext_disc_desc *d = &event->ext_disc;
    c.ble_all++;
    if (has_rid(d->data, d->length_data)) {
        if (d->props & BLE_HCI_ADV_LEGACY_MASK) {
            c.bt4++;
        } else {
            c.bt5++;
        }
    }
    return 0;
}

static void on_sync(void) { ble_synced = true; }

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_scan_on(int duty_pct)
{
    uint16_t window = (uint16_t)(SCAN_ITVL * duty_pct / 100);
    struct ble_gap_ext_disc_params p = {.itvl = SCAN_ITVL, .window = window, .passive = 1};
    int rc = ble_gap_ext_disc(BLE_OWN_ADDR_PUBLIC, 0, 0, 0, 0, 0, &p, &p, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_ext_disc: %d", rc);
    }
}

static void ble_scan_off(void)
{
    ble_gap_disc_cancel();
}

/* -- test ------------------------------------------------------------------ */

typedef struct {
    const char *name;
    bool     wifi;
    int      ble_duty;            /* % of each scan interval spent scanning; 0 = BLE off */
    counts_t total;
    uint32_t ms;
} phase_t;

static void run_phase(phase_t *ph, int round)
{
    if (ph->wifi) wifi_on();
    if (ph->ble_duty) ble_scan_on(ph->ble_duty);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

    memset((void *)&c, 0, sizeof(c));
    int64_t t0 = esp_timer_get_time();
    counting = true;
    vTaskDelay(pdMS_TO_TICKS(PHASE_MS));
    counting = false;
    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    counts_t got = c;

    if (ph->ble_duty) ble_scan_off();
    if (ph->wifi) wifi_off();

    ESP_LOGI(TAG, "COEX round=%d phase=%s wifi=%d ble_duty=%d ms=%lu beacon=%lu nan=%lu mgmt=%lu bt4=%lu bt5=%lu ble_all=%lu",
             round, ph->name, ph->wifi, ph->ble_duty, (unsigned long)ms, (unsigned long)got.beacon,
             (unsigned long)got.nan, (unsigned long)got.mgmt, (unsigned long)got.bt4,
             (unsigned long)got.bt5, (unsigned long)got.ble_all);
    ph->ms += ms;
    ph->total.mgmt += got.mgmt;
    ph->total.beacon += got.beacon;
    ph->total.nan += got.nan;
    ph->total.bt4 += got.bt4;
    ph->total.bt5 += got.bt5;
    ph->total.ble_all += got.ble_all;
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init();
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = on_sync;
    nimble_port_freertos_init(host_task);
    while (!ble_synced) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "COEX START channel=%d rounds=%d phase_ms=%d nimble_core=%d ctrl_core=%d wifi_core=%d",
             WIFI_CHANNEL, ROUNDS, PHASE_MS, CONFIG_BT_NIMBLE_PINNED_TO_CORE,
             CONFIG_BT_CTRL_PINNED_TO_CORE, WIFI_TASK_CORE);

    phase_t phases[] = {
        {.name = "A",    .wifi = true,  .ble_duty = 0},
        {.name = "B100", .wifi = true,  .ble_duty = 100},
        {.name = "B50",  .wifi = true,  .ble_duty = 50},
        {.name = "B25",  .wifi = true,  .ble_duty = 25},
        {.name = "B10",  .wifi = true,  .ble_duty = 10},
        {.name = "C",    .wifi = false, .ble_duty = 100},
    };
    for (int r = 1; r <= ROUNDS; r++) {
        for (size_t i = 0; i < sizeof(phases) / sizeof(phases[0]); i++) {
            run_phase(&phases[i], r);
        }
    }

    for (size_t i = 0; i < sizeof(phases) / sizeof(phases[0]); i++) {
        const phase_t *ph = &phases[i];
        double s = ph->ms / 1000.0;
        ESP_LOGI(TAG, "COEX SUMMARY phase=%-4s beacon/s=%.3f nan/s=%.3f mgmt/s=%.1f bt4/s=%.2f bt5/s=%.3f ble_all/s=%.1f",
                 ph->name, ph->total.beacon / s, ph->total.nan / s, ph->total.mgmt / s,
                 ph->total.bt4 / s, ph->total.bt5 / s, ph->total.ble_all / s);
    }
    ESP_LOGI(TAG, "COEX DONE");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
