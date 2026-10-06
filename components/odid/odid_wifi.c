/*
 * odid_wifi.c - see odid_wifi.h.
 */
#include "odid_wifi.h"

#include <string.h>

#define DOT11_MGMT_HDR_LEN   24
#define SUBTYPE_BEACON       8
#define SUBTYPE_ACTION       13

#define IE_VENDOR_SPECIFIC   221
#define NAN_ATTR_SDA         0x03

static const uint8_t ASTM_OUI[3] = {0xFA, 0x0B, 0xBC};
#define ASTM_OUI_TYPE 0x0D

/*
 * Parrot SA's OUI. Some Remote ID beacons carry the ASTM layout under it:
 * OUI, type, send counter, Message Pack. The layout comes from Sky-Spy's
 * receiver, not from a Parrot capture. Parrot uses the OUI for other vendor
 * elements too, so it only counts when a valid Message Pack follows.
 * Unrelated to the French scheme's own OUI.
 */
static const uint8_t PARROT_OUI[3] = {0x90, 0x3A, 0xE6};

static const uint8_t NAN_OUI[3] = {0x50, 0x6F, 0x9A};
#define NAN_OUI_TYPE 0x13

/* First 6 bytes of SHA-256("org.opendroneid.remoteid"). */
static const uint8_t ODID_NAN_SERVICE_ID[6] = {0x88, 0x69, 0x19, 0x9D, 0x92, 0x09};

/* _is_message_pack: type 0xF, 25-byte messages, 1-9 of them, all present. */
static bool is_message_pack(const uint8_t *p, size_t len)
{
    if (len < 3 || (p[0] >> 4) != 0xF || p[1] != 25) {
        return false;
    }
    return p[2] >= 1 && p[2] <= 9 && len >= 3 + (size_t)p[2] * 25;
}

/* Walk the beacon IE chain for the Remote ID vendor IE (_extract_beacon_rid). */
static bool extract_beacon(const uint8_t *body, size_t len,
                           const uint8_t **payload, size_t *payload_len)
{
    size_t offset = 12;  /* timestamp(8) + beacon interval(2) + capability(2) */
    while (offset + 2 <= len) {
        uint8_t tag_id  = body[offset];
        uint8_t tag_len = body[offset + 1];
        size_t  end     = offset + 2 + tag_len;
        if (end > len) {
            break;
        }
        if (tag_id == IE_VENDOR_SPECIFIC) {
            const uint8_t *info = body + offset + 2;
            /* OUI(3) + vendor type(1) + send counter(1), then the message. */
            if (tag_len >= 6 && memcmp(info, ASTM_OUI, 3) == 0 && info[3] == ASTM_OUI_TYPE) {
                *payload     = info + 5;
                *payload_len = tag_len - 5;
                return true;
            }
            if (tag_len >= 6 && memcmp(info, PARROT_OUI, 3) == 0 &&
                is_message_pack(info + 5, tag_len - 5)) {
                *payload     = info + 5;
                *payload_len = tag_len - 5;
                return true;
            }
        }
        offset = end;
    }
    return false;
}

/* NAN public action frame carrying the ODID Service ID (_is_nan_action). */
static bool is_nan_action(const uint8_t *body, size_t len)
{
    if (len < 6 || body[0] != 4 || memcmp(body + 2, NAN_OUI, 3) != 0 ||
        body[5] != NAN_OUI_TYPE) {
        return false;
    }
    /* The Service ID must appear in the first 64 bytes (body[:64] in Python). */
    size_t window = len < 64 ? len : 64;
    for (size_t i = 0; i + sizeof(ODID_NAN_SERVICE_ID) <= window; i++) {
        if (memcmp(body + i, ODID_NAN_SERVICE_ID, sizeof(ODID_NAN_SERVICE_ID)) == 0) {
            return true;
        }
    }
    return false;
}

/* Service Descriptor Attribute body -> ODID message (_parse_nan_sda). */
static bool parse_nan_sda(const uint8_t *sda, size_t len,
                          const uint8_t **payload, size_t *payload_len)
{
    if (len < 9 || memcmp(sda, ODID_NAN_SERVICE_ID, 6) != 0) {
        return false;
    }
    uint8_t service_control = sda[8];
    size_t  offset = 9;
    if (service_control & (1 << 6)) {           /* Binding Bitmap */
        offset += 2;
    }
    if (service_control & (1 << 2)) {           /* Matching Filter */
        if (offset >= len) {
            return false;
        }
        offset += 1 + sda[offset];
    }
    if (service_control & (1 << 3)) {           /* Service Response Filter */
        if (offset >= len) {
            return false;
        }
        offset += 1 + sda[offset];
    }
    if (!(service_control & (1 << 4)) || offset >= len) {   /* Service Info */
        return false;
    }
    uint8_t sil = sda[offset];
    offset += 1;
    if (sil < 2 || offset + sil > len) {
        return false;
    }
    /* Service Info byte 0 is the send counter; the rest is the message. */
    *payload     = sda + offset + 1;
    *payload_len = (size_t)sil - 1;
    return true;
}

static bool walk_nan_attrs(const uint8_t *body, size_t len, size_t offset,
                           const uint8_t **payload, size_t *payload_len)
{
    while (offset + 3 <= len) {
        uint8_t  attr_id  = body[offset];
        uint16_t attr_len = (uint16_t)(body[offset + 1] | (body[offset + 2] << 8));
        size_t   attr_end = offset + 3 + attr_len;
        if (attr_end > len) {
            return false;
        }
        if (attr_id == NAN_ATTR_SDA &&
            parse_nan_sda(body + offset + 3, attr_len, payload, payload_len)) {
            return true;
        }
        offset = attr_end;
    }
    return false;
}

bool odid_wifi_parse_frame(const uint8_t *frame, size_t len, odid_wifi_frame_t *out)
{
    memset(out, 0, sizeof(*out));
    if (frame == NULL || len < DOT11_MGMT_HDR_LEN) {
        return false;
    }
    uint8_t fc0 = frame[0];
    if (((fc0 >> 2) & 0x3) != 0) {              /* 0 = management */
        return false;
    }
    out->subtype = (fc0 >> 4) & 0xF;
    memcpy(out->addr2, frame + 10, 6);

    const uint8_t *body     = frame + DOT11_MGMT_HDR_LEN;
    size_t         body_len = len - DOT11_MGMT_HDR_LEN;

    if (out->subtype == SUBTYPE_BEACON) {
        if (extract_beacon(body, body_len, &out->payload, &out->payload_len)) {
            out->transport = ODID_WIFI_BEACON;
        }
    } else if (out->subtype == SUBTYPE_ACTION && is_nan_action(body, body_len)) {
        /* Some senders put an extra OUI-subtype byte before the attributes. */
        if (walk_nan_attrs(body, body_len, 6, &out->payload, &out->payload_len) ||
            walk_nan_attrs(body, body_len, 7, &out->payload, &out->payload_len)) {
            out->transport = ODID_WIFI_NAN;
        } else {
            out->nan_unparsed = true;
        }
    }
    return true;
}
