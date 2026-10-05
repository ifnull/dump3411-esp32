/*
 * Bench Remote ID transmitter: broadcasts a simulated drone over Wi-Fi
 * Beacon, Wi-Fi NAN, Bluetooth 4 legacy and Bluetooth 5 long-range, all
 * encoded by the OpenDroneID reference library. For testing receivers on the
 * bench (dump3411, dump3411-esp32). Not for flight.
 */
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "ble_tx.h"
#include "sim.h"
#include "wifi_tx.h"

static const char *TAG = "rid_tx";

#define TICK_MS 200     /* legacy BLE rotation step: 5 types -> each at 1 Hz */

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    static ODID_UAS_Data uas;
    sim_init(&uas);
    ESP_ERROR_CHECK(wifi_tx_init());
    ESP_ERROR_CHECK(ble_tx_init());
    ESP_LOGI(TAG, "broadcasting UAS ID %s", uas.BasicID[0].UASID);

    unsigned tick = 0;
    for (;;) {
        sim_step(&uas, TICK_MS / 1000.0);
        ble_tx_legacy_next(&uas);
        if (tick % (1000 / TICK_MS) == 0) {
            wifi_tx_send(&uas);
            ble_tx_long_range(&uas);
            const ODID_Location_data *l = &uas.Location;
            ESP_LOGI(TAG, "lat %.7f lon %.7f track %3.0f speed %5.2f vs %5.2f agl %5.1f",
                     l->Latitude, l->Longitude, (double)l->Direction, (double)l->SpeedHorizontal,
                     (double)l->SpeedVertical, (double)l->Height);
        }
        tick++;
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}
