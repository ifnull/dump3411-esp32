#ifndef BLE_TX_H
#define BLE_TX_H

#include "esp_err.h"
#include "opendroneid.h"

/* Bring up NimBLE; advertising starts once the host has synced. */
esp_err_t ble_tx_init(void);

/* Legacy (BT4) set: advertise the next single message in rotation. */
void ble_tx_legacy_next(const ODID_UAS_Data *uas);

/* Extended (BT5, coded PHY) set: advertise a full Message Pack. */
void ble_tx_long_range(const ODID_UAS_Data *uas);

#endif
