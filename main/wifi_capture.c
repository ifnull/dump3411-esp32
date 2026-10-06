/*
 * Wi-Fi Remote ID capture: the ESP-IDF counterpart of dump3411's
 * WiFiFeeder (wifi_feeder.py). Decoded messages feed the tracker, which this
 * task owns; Phase 1 output is a per-drone summary on the serial console.
 * Set the log level to Debug to also see every decoded message.
 */
#include "wifi_capture.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "odid_parser.h"
#include "odid_tracker.h"
#include "odid_wifi.h"

static const char *TAG = "rid_wifi";

#define FCS_LEN        4
#define MAX_FRAME_LEN  1024     /* RID beacons are ~200 bytes */
#define QUEUE_DEPTH    16
#define TTL_MS         60000    /* dump3411's default --ttl */
#define SUMMARY_MS     5000

typedef struct {
    uint16_t len;
    int8_t   rssi;
    uint8_t  channel;
    uint8_t  frame[MAX_FRAME_LEN];
} captured_t;

static QueueHandle_t queue;
static odid_tracker_t tracker;  /* owned by decode_task */

/* Counters, written by the Wi-Fi task and read by the stats log. */
static volatile uint32_t mgmt_frames, rid_frames, dropped, nan_unparsed;

/*
 * Runs in the Wi-Fi driver's task, so it must stay short. The ODID check is
 * a bounded IE walk (no decoding), done here so the queue only carries
 * Remote ID frames rather than every beacon in range.
 */
static void on_packet(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT) {
        return;
    }
    const wifi_promiscuous_pkt_t *pkt = buf;
    mgmt_frames++;
    if (pkt->rx_ctrl.sig_len <= FCS_LEN) {
        return;
    }
    size_t len = pkt->rx_ctrl.sig_len - FCS_LEN;
    if (len > MAX_FRAME_LEN) {
        return;
    }
    odid_wifi_frame_t f;
    if (!odid_wifi_parse_frame(pkt->payload, len, &f)) {
        return;
    }
    if (f.nan_unparsed) {
        nan_unparsed++;
    }
    if (f.transport == ODID_WIFI_NONE) {
        return;
    }
    rid_frames++;

    static captured_t item;     /* the Wi-Fi task is the only writer */
    item.len     = (uint16_t)len;
    item.rssi    = (int8_t)pkt->rx_ctrl.rssi;
    item.channel = (uint8_t)pkt->rx_ctrl.channel;
    memcpy(item.frame, pkt->payload, len);
    if (xQueueSend(queue, &item, 0) != pdTRUE) {
        dropped++;
    }
}

static void log_message(const char *transport, const uint8_t mac[6], int rssi, int ch,
                        const uint8_t *msg, size_t len)
{
    odid_msg_t m;
    if (!odid_decode(msg, len, &m)) {
        return;
    }
    const char *name = odid_msg_type_name(m.type);
    char head[64];
    snprintf(head, sizeof(head), "[%s] %02x:%02x:%02x:%02x:%02x:%02x %4d dBm ch%-2d",
             transport, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi, ch);

    if (!m.has_fields) {
        if (m.type == ODID_MSG_LOCATION) {
            return;      /* no GPS fix yet; dump3411 drops these too */
        }
        ESP_LOGD(TAG, "%s %s", head, name ? name : "unknown type");
        return;
    }
    switch (m.type) {
    case ODID_MSG_BASIC_ID:
        ESP_LOGD(TAG, "%s Basic ID     %.*s (%s)", head, m.u.basic_id.uas_id_len,
                 (const char *)m.u.basic_id.uas_id,
                 odid_ua_type_name(m.u.basic_id.ua_type) ? odid_ua_type_name(m.u.basic_id.ua_type) : "?");
        break;
    case ODID_MSG_LOCATION: {
        const odid_location_t *l = &m.u.location;
        ESP_LOGD(TAG, "%s Location     lat %.7f lon %.7f track %3.0f speed %6.2f vs %5.1f agl %s%.1f",
                 head, l->lat, l->lon, l->heading_deg, l->ground_speed_mps, l->vertical_speed_mps,
                 l->has_height ? "" : "? ", l->has_height ? l->height_m : 0.0);
        break;
    }
    case ODID_MSG_SELF_ID:
        ESP_LOGD(TAG, "%s Self ID      \"%.*s\"", head, m.u.self_id.description_len,
                 (const char *)m.u.self_id.description);
        break;
    case ODID_MSG_SYSTEM: {
        const odid_system_t *s = &m.u.system;
        if (s->has_operator) {
            ESP_LOGD(TAG, "%s System       operator %.7f %.7f (%s) alt %.1f", head, s->operator_lat,
                     s->operator_lon, odid_operator_location_type_name(s->operator_location_type),
                     s->has_alt_takeoff ? s->alt_takeoff_m : -1000.0);
        } else {
            ESP_LOGD(TAG, "%s System       operator position unknown", head);
        }
        break;
    }
    case ODID_MSG_OPERATOR_ID:
        ESP_LOGD(TAG, "%s Operator ID  %.*s", head, m.u.operator_id.operator_id_len,
                 (const char *)m.u.operator_id.operator_id);
        break;
    default:
        break;
    }
}

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void on_expire(void *ctx, const uint8_t *uas_id, size_t len)
{
    ESP_LOGI(TAG, "drone %.*s timed out", (int)len, (const char *)uas_id);
}

/* Log the payload's messages and feed them to the tracker. */
static void handle_payload(const odid_wifi_frame_t *f, int rssi, int channel)
{
    const char *name = f->transport == ODID_WIFI_BEACON ? "beacon" : "nan";
    odid_source_t src = f->transport == ODID_WIFI_BEACON ? ODID_SRC_WIFI_BEACON : ODID_SRC_WIFI_NAN;
    odid_msg_t top;
    if (!odid_decode(f->payload, f->payload_len, &top)) {
        return;
    }
    if (top.type != ODID_MSG_PACK) {
        log_message(name, f->addr2, rssi, channel, f->payload, f->payload_len);
        odid_tracker_update(&tracker, now_ms(), f->addr2, &top, (int16_t)rssi, src);
        return;
    }
    /* Two passes so the pack's Basic ID maps the MAC before the rest arrives. */
    for (int pass = 0; pass < 2; pass++) {
        odid_pack_iter_t it;
        const uint8_t *msg;
        size_t msg_len;
        if (!odid_pack_iter_init(&it, f->payload, f->payload_len)) {
            return;
        }
        while (odid_pack_next(&it, &msg, &msg_len)) {
            odid_msg_t m;
            if (!odid_decode(msg, msg_len, &m) || (m.type == ODID_MSG_BASIC_ID) != (pass == 0)) {
                continue;
            }
            log_message(name, f->addr2, rssi, channel, msg, msg_len);
            odid_tracker_update(&tracker, now_ms(), f->addr2, &m, (int16_t)rssi, src);
        }
    }
}

static void log_summary(void)
{
    uint32_t now = now_ms();
    const odid_drone_t *live[ODID_TRACKER_MAX_DRONES];
    size_t n = odid_tracker_list(&tracker, live, ODID_TRACKER_MAX_DRONES);
    ESP_LOGI(TAG, "%u drone(s); %lu Remote ID frames, %lu mgmt frames, %lu dropped, %lu NAN unparsed",
             (unsigned)n, (unsigned long)rid_frames, (unsigned long)mgmt_frames,
             (unsigned long)dropped, (unsigned long)nan_unparsed);
    for (size_t i = 0; i < n; i++) {
        const odid_drone_t *d = live[i];
        char pos[96] = "no position yet";
        if (d->has_position) {
            snprintf(pos, sizeof(pos), "%.6f %.6f agl %.0f m %.1f m/s track %.0f",
                     d->lat, d->lon, d->height_agl_m, d->ground_speed_mps, d->heading_deg);
        }
        const char *ua = odid_feed_ua_type(d->ua_type);
        ESP_LOGI(TAG, "  %-20.*s %-10s %s, %4d dBm, %lu msgs, last %.1f s ago", d->uas_id_len,
                 (const char *)d->uas_id, ua ? ua : "?", pos,
                 d->rssi_dbm == ODID_RSSI_NONE ? 0 : d->rssi_dbm, (unsigned long)d->message_count,
                 (now - d->last_seen_ms) / 1000.0);
    }
}

static void decode_task(void *arg)
{
    static captured_t item;
    odid_tracker_callbacks_t cb = {.on_expire = on_expire};
    odid_tracker_init(&tracker, TTL_MS, &cb);
    uint32_t last_sweep = now_ms(), last_summary = last_sweep;

    for (;;) {
        if (xQueueReceive(queue, &item, pdMS_TO_TICKS(250)) == pdTRUE) {
            odid_wifi_frame_t f;
            if (odid_wifi_parse_frame(item.frame, item.len, &f) && f.payload != NULL) {
                handle_payload(&f, item.rssi, item.channel);
            }
        }
        uint32_t now = now_ms();
        if (now - last_sweep >= 1000) {
            odid_tracker_sweep(&tracker, now);
            last_sweep = now;
        }
        if (now - last_summary >= SUMMARY_MS) {
            log_summary();
            last_summary = now;
        }
    }
}

static void hop_task(void *arg)
{
    uint8_t ch = 1;
    for (;;) {
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_DUMP3411_HOP_DWELL_MS));
        ch = ch % 11 + 1;
    }
}

esp_err_t wifi_capture_start(void)
{
    queue = xQueueCreate(QUEUE_DEPTH, sizeof(captured_t));
    if (queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));   /* never associates */
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(on_packet));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    xTaskCreate(decode_task, "odid_decode", 6144, NULL, 5, NULL);
    if (CONFIG_DUMP3411_WIFI_CHANNEL == 0) {
        xTaskCreate(hop_task, "wifi_hop", 2048, NULL, 4, NULL);
        ESP_LOGI(TAG, "capturing, hopping channels 1-11 every %d ms", CONFIG_DUMP3411_HOP_DWELL_MS);
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_DUMP3411_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
        ESP_LOGI(TAG, "capturing on channel %d", CONFIG_DUMP3411_WIFI_CHANNEL);
    }
    return ESP_OK;
}
