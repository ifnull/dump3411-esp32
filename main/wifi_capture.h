#ifndef WIFI_CAPTURE_H
#define WIFI_CAPTURE_H

#include "esp_err.h"

/*
 * Start promiscuous Wi-Fi capture (and channel hopping, unless a fixed
 * channel is configured). Remote ID found in Beacon or NAN frames is
 * decoded and logged over the console.
 */
esp_err_t wifi_capture_start(void);

#endif
