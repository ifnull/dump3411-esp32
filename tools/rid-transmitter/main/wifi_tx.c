/*
 * Wi-Fi transports. Frames are built entirely by opendroneid-core-c
 * (odid_wifi_build_message_pack_*_frame) and sent raw with
 * esp_wifi_80211_tx, so the bytes on the air are the reference library's.
 */
#include "wifi_tx.h"

#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "sdkconfig.h"

static const char *TAG = "wifi_tx";
static const char SSID[] = "RID-BENCH-TEST";

static uint8_t mac[6];
static uint8_t counter;

esp_err_t wifi_tx_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t ap = {
        .ap = {
            .ssid           = "RID-BENCH-TEST",
            .ssid_len       = sizeof(SSID) - 1,
            .ssid_hidden    = 1,
            .channel        = CONFIG_RID_TX_WIFI_CHANNEL,
            .authmode       = WIFI_AUTH_OPEN,
            .max_connection = 1,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(CONFIG_RID_TX_WIFI_POWER_QDBM));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, mac));
    ESP_LOGI(TAG, "Wi-Fi TX on channel %d, MAC %02x:%02x:%02x:%02x:%02x:%02x, max %.2f dBm",
             CONFIG_RID_TX_WIFI_CHANNEL, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             CONFIG_RID_TX_WIFI_POWER_QDBM / 4.0);
    return ESP_OK;
}

void wifi_tx_send(const ODID_UAS_Data *uas)
{
    static uint8_t frame[1024];
    counter++;

    int len = odid_wifi_build_message_pack_beacon_frame(uas, (const char *)mac, SSID, sizeof(SSID) - 1,
                                                        100, counter, frame, sizeof(frame));
    if (len > 0) {
        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_AP, frame, len, true);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "beacon tx: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG, "beacon build failed: %d", len);
    }

    len = odid_wifi_build_message_pack_nan_action_frame(uas, (const char *)mac, counter,
                                                        frame, sizeof(frame));
    if (len > 0) {
        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_AP, frame, len, true);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NAN tx: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG, "NAN build failed: %d", len);
    }
}
