#ifndef WIFI_TX_H
#define WIFI_TX_H

#include "esp_err.h"
#include "opendroneid.h"

/* Start Wi-Fi as a hidden soft-AP on the configured channel, used only for raw TX. */
esp_err_t wifi_tx_init(void);

/* Send one Beacon and one NAN action frame carrying a Message Pack. */
void wifi_tx_send(const ODID_UAS_Data *uas);

#endif
