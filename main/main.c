/*
 * dump3411-esp32 firmware entry point.
 *
 * Phase 1 bring-up: report the chip and run the decoder self-test. Wi-Fi
 * capture lands here next (docs/ARCHITECTURE.md, Build order).
 */
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "selftest.h"

static const char *TAG = "dump3411";

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "dump3411-esp32 on %s rev %d.%d, %d cores, MAC %02x:%02x:%02x:%02x:%02x:%02x",
             CONFIG_IDF_TARGET, chip.revision / 100, chip.revision % 100, chip.cores,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (odid_selftest()) {
        ESP_LOGI(TAG, "decoder self-test passed");
    } else {
        ESP_LOGE(TAG, "decoder self-test FAILED");
    }

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
