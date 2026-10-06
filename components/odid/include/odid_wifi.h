/*
 * odid_wifi.h - find the ODID payload inside a raw 802.11 frame.
 *
 * Port of dump3411 wifi_feeder.py's _parse_dot11_mgmt, _extract_beacon_rid,
 * _is_nan_action and _extract_nan_odid. Input is the 802.11 frame starting at
 * the MAC header (no radiotap): the form ESP-IDF's promiscuous callback
 * delivers. The returned payload points into the caller's buffer and is what
 * odid_decode() takes.
 */
#ifndef ODID_WIFI_H
#define ODID_WIFI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ODID_WIFI_NONE = 0,
    ODID_WIFI_BEACON,     /* Beacon vendor IE, OUI FA:0B:BC type 0x0D */
    ODID_WIFI_NAN,        /* NAN Service Discovery Frame with the ODID Service ID */
} odid_wifi_transport_t;

typedef struct {
    odid_wifi_transport_t transport;
    uint8_t        subtype;      /* 802.11 management subtype */
    uint8_t        addr2[6];     /* transmitter MAC */
    const uint8_t *payload;      /* single message or Message Pack; NULL if none */
    size_t         payload_len;
    /*
     * NAN only: the frame carried the ODID NAN Service ID but no Service
     * Descriptor Attribute parsed to a payload. dump3411 logs these as
     * evidence of a parser/spec mismatch; callers should too.
     */
    bool           nan_unparsed;
} odid_wifi_frame_t;

/*
 * Returns false for anything that isn't a management frame (or is shorter
 * than the 24-byte header). Returns true with transport == ODID_WIFI_NONE for
 * management frames that carry no ODID payload.
 */
bool odid_wifi_parse_frame(const uint8_t *frame, size_t len, odid_wifi_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ODID_WIFI_H */
