/*
 * Wi-Fi Remote ID capture: the ESP-IDF counterpart of dump3411's
 * WiFiFeeder (wifi_feeder.py). Phase 1 output is the serial log only; the
 * tracker and display consume this later.
 */
#include "wifi_capture.h"

#include <string.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "odid_parser.h"
#include "odid_wifi.h"

static const char *TAG = "rid_wifi";

#define FCS_LEN        4
#define MAX_FRAME_LEN  1024     /* RID beacons are ~200 bytes */
#define QUEUE_DEPTH    16

typedef struct {
    uint16_t len;
    int8_t   rssi;
    uint8_t  channel;
    uint8_t  frame[MAX_FRAME_LEN];
} captured_t;

static QueueHandle_t queue;

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
        ESP_LOGI(TAG, "%s %s", head, name ? name : "unknown type");
        return;
    }
    switch (m.type) {
    case ODID_MSG_BASIC_ID:
        ESP_LOGI(TAG, "%s Basic ID     %.*s (%s)", head, m.u.basic_id.uas_id_len,
                 (const char *)m.u.basic_id.uas_id,
                 odid_ua_type_name(m.u.basic_id.ua_type) ? odid_ua_type_name(m.u.basic_id.ua_type) : "?");
        break;
    case ODID_MSG_LOCATION: {
        const odid_location_t *l = &m.u.location;
        ESP_LOGI(TAG, "%s Location     lat %.7f lon %.7f track %3.0f speed %6.2f vs %5.1f agl %s%.1f",
                 head, l->lat, l->lon, l->heading_deg, l->ground_speed_mps, l->vertical_speed_mps,
                 l->has_height ? "" : "? ", l->has_height ? l->height_m : 0.0);
        break;
    }
    case ODID_MSG_SELF_ID:
        ESP_LOGI(TAG, "%s Self ID      \"%.*s\"", head, m.u.self_id.description_len,
                 (const char *)m.u.self_id.description);
        break;
    case ODID_MSG_SYSTEM: {
        const odid_system_t *s = &m.u.system;
        if (s->has_operator) {
            ESP_LOGI(TAG, "%s System       operator %.7f %.7f (%s) alt %.1f", head, s->operator_lat,
                     s->operator_lon, odid_operator_location_type_name(s->operator_location_type),
                     s->has_alt_takeoff ? s->alt_takeoff_m : -1000.0);
        } else {
            ESP_LOGI(TAG, "%s System       operator position unknown", head);
        }
        break;
    }
    case ODID_MSG_OPERATOR_ID:
        ESP_LOGI(TAG, "%s Operator ID  %.*s", head, m.u.operator_id.operator_id_len,
                 (const char *)m.u.operator_id.operator_id);
        break;
    default:
        break;
    }
}

static void decode_task(void *arg)
{
    static captured_t item;
    for (;;) {
        if (xQueueReceive(queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        odid_wifi_frame_t f;
        if (!odid_wifi_parse_frame(item.frame, item.len, &f) || f.payload == NULL) {
            continue;
        }
        const char *transport = f.transport == ODID_WIFI_BEACON ? "beacon" : "nan";
        odid_msg_t top;
        if (!odid_decode(f.payload, f.payload_len, &top)) {
            continue;
        }
        if (top.type != ODID_MSG_PACK) {
            log_message(transport, f.addr2, item.rssi, item.channel, f.payload, f.payload_len);
            continue;
        }
        odid_pack_iter_t it;
        const uint8_t *msg;
        size_t msg_len;
        if (odid_pack_iter_init(&it, f.payload, f.payload_len)) {
            while (odid_pack_next(&it, &msg, &msg_len)) {
                log_message(transport, f.addr2, item.rssi, item.channel, msg, msg_len);
            }
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

static void stats_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "stats: %lu mgmt frames, %lu Remote ID frames, %lu dropped, %lu NAN unparsed",
                 (unsigned long)mgmt_frames, (unsigned long)rid_frames,
                 (unsigned long)dropped, (unsigned long)nan_unparsed);
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

    xTaskCreate(decode_task, "odid_decode", 4096, NULL, 5, NULL);
    xTaskCreate(stats_task, "wifi_stats", 3072, NULL, 1, NULL);
    if (CONFIG_DUMP3411_WIFI_CHANNEL == 0) {
        xTaskCreate(hop_task, "wifi_hop", 2048, NULL, 4, NULL);
        ESP_LOGI(TAG, "capturing, hopping channels 1-11 every %d ms", CONFIG_DUMP3411_HOP_DWELL_MS);
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_DUMP3411_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
        ESP_LOGI(TAG, "capturing on channel %d", CONFIG_DUMP3411_WIFI_CHANNEL);
    }
    return ESP_OK;
}
