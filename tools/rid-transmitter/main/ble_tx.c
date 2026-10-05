/*
 * Bluetooth transports, per ASTM F3411: each advertisement carries one
 * Service Data AD structure for UUID 0xFFFA:
 *
 *   len | 0x16 | 0xFA 0xFF | app code 0x0D | counter | ODID payload
 *
 * Legacy (BT4) adverts fit one 25-byte message, so the set rotates through
 * the message types. The BT5 set uses the coded (long-range) PHY and sends a
 * whole Message Pack per advert.
 */
#include "ble_tx.h"

#include <string.h>

#include "esp_bt.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

static const char *TAG = "ble_tx";

#define INSTANCE_LEGACY  0
#define INSTANCE_CODED   1

static const uint8_t SERVICE_HEADER[] = {0x16, 0xFA, 0xFF, 0x0D};

static volatile bool synced;
static bool started[2];
static uint8_t counters[16];       /* per message type */

static int gap_event(struct ble_gap_event *event, void *arg)
{
    return 0;
}

static void configure(uint8_t instance, bool legacy)
{
    struct ble_gap_ext_adv_params p = {0};
    p.legacy_pdu     = legacy;
    p.connectable    = 0;
    p.scannable      = 0;
    p.own_addr_type  = BLE_OWN_ADDR_RANDOM;
    p.primary_phy    = legacy ? BLE_HCI_LE_PHY_1M : BLE_HCI_LE_PHY_CODED;
    p.secondary_phy  = legacy ? BLE_HCI_LE_PHY_1M : BLE_HCI_LE_PHY_CODED;
    p.channel_map    = 0;              /* all three */
    p.tx_power       = 127;            /* no preference; see esp_ble_tx_power_set */
    p.sid            = instance;
    /* Units of 0.625 ms: ~100 ms legacy (several adverts per rotation step), ~1 s coded. */
    p.itvl_min       = legacy ? 0x00A0 : 0x0640;
    p.itvl_max       = legacy ? 0x00B0 : 0x0680;

    int rc = ble_gap_ext_adv_configure(instance, &p, NULL, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "configure instance %u: %d", instance, rc);
        return;
    }
    ble_addr_t addr;
    ble_hs_id_gen_rnd(0, &addr);           /* static random address */
    rc = ble_gap_ext_adv_set_addr(instance, &addr);
    if (rc != 0) {
        ESP_LOGE(TAG, "set addr instance %u: %d", instance, rc);
        return;
    }
    ESP_LOGI(TAG, "%s set uses address %02x:%02x:%02x:%02x:%02x:%02x",
             legacy ? "BT4 legacy" : "BT5 coded",
             addr.val[5], addr.val[4], addr.val[3], addr.val[2], addr.val[1], addr.val[0]);
}

static void on_sync(void)
{
    configure(INSTANCE_LEGACY, true);
    configure(INSTANCE_CODED, false);
    synced = true;
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset: %d", reason);
    synced = false;
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_tx_init(void)
{
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        return err;
    }
    /* Keep the bench transmitter's range modest. */
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12);

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(host_task);
    return ESP_OK;
}

static void advertise(uint8_t instance, uint8_t msg_type, const uint8_t *payload, size_t len)
{
    if (!synced) {
        return;
    }
    uint8_t ad[255];
    size_t n = 0;
    ad[n++] = (uint8_t)(sizeof(SERVICE_HEADER) + 1 + len);   /* AD length */
    memcpy(ad + n, SERVICE_HEADER, sizeof(SERVICE_HEADER));
    n += sizeof(SERVICE_HEADER);
    ad[n++] = counters[msg_type & 0x0F]++;
    memcpy(ad + n, payload, len);
    n += len;

    struct os_mbuf *om = os_msys_get_pkthdr(n, 0);
    if (om == NULL || os_mbuf_append(om, ad, n) != 0) {
        ESP_LOGW(TAG, "no mbuf for instance %u", instance);
        return;
    }
    int rc = ble_gap_ext_adv_set_data(instance, om);
    if (rc == BLE_HS_EBUSY) {
        /* Some stacks refuse data changes while running: stop, set, restart. */
        ble_gap_ext_adv_stop(instance);
        om = os_msys_get_pkthdr(n, 0);
        if (om == NULL || os_mbuf_append(om, ad, n) != 0) {
            return;
        }
        rc = ble_gap_ext_adv_set_data(instance, om);
        started[instance] = false;
    }
    if (rc != 0) {
        ESP_LOGW(TAG, "set data instance %u: %d", instance, rc);
        return;
    }
    if (!started[instance]) {
        rc = ble_gap_ext_adv_start(instance, 0, 0);
        if (rc != 0) {
            ESP_LOGW(TAG, "start instance %u: %d", instance, rc);
            return;
        }
        started[instance] = true;
    }
}

void ble_tx_legacy_next(const ODID_UAS_Data *uas)
{
    static int phase;
    uint8_t msg[ODID_MESSAGE_SIZE];
    int ok = ODID_FAIL;

    switch (phase) {
    case 0: ok = encodeBasicIDMessage((ODID_BasicID_encoded *)msg, &uas->BasicID[0]); break;
    case 1: ok = encodeLocationMessage((ODID_Location_encoded *)msg, &uas->Location); break;
    case 2: ok = encodeSelfIDMessage((ODID_SelfID_encoded *)msg, &uas->SelfID); break;
    case 3: ok = encodeSystemMessage((ODID_System_encoded *)msg, &uas->System); break;
    case 4: ok = encodeOperatorIDMessage((ODID_OperatorID_encoded *)msg, &uas->OperatorID); break;
    }
    phase = (phase + 1) % 5;
    if (ok == ODID_SUCCESS) {
        advertise(INSTANCE_LEGACY, msg[0] >> 4, msg, sizeof(msg));
    }
}

void ble_tx_long_range(const ODID_UAS_Data *uas)
{
    uint8_t pack[3 + ODID_PACK_MAX_MESSAGES * ODID_MESSAGE_SIZE];
    int len = odid_message_build_pack(uas, pack, sizeof(pack));
    if (len > 0) {
        advertise(INSTANCE_CODED, ODID_MESSAGETYPE_PACKED, pack, (size_t)len);
    }
}
