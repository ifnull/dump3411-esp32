/*
 * Boot-time decoder check: one Wi-Fi Beacon frame built by the OpenDroneID
 * reference encoder (tests/parity/fixtures/encoder.txt, "multirotor-serial"),
 * decoded on the target and compared with the values the encoder was given.
 * The host parity tests cover far more; this proves the same code behaves
 * the same once cross-compiled for the chip.
 */
#include "selftest.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "odid_parser.h"
#include "odid_wifi.h"

static const char *TAG = "selftest";

static const uint8_t BEACON[] = {
    0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x60, 0x60,
    0x1F, 0x12, 0x34, 0x56, 0x60, 0x60, 0x1F, 0x12, 0x34, 0x56, 0x00, 0x00,
    0x4B, 0x70, 0xC2, 0xFD, 0x79, 0x00, 0x00, 0x00, 0x64, 0x00, 0x20, 0x04,
    0x00, 0x08, 0x52, 0x49, 0x44, 0x2D, 0x54, 0x45, 0x53, 0x54, 0x01, 0x01,
    0x8C, 0xDD, 0x85, 0xFA, 0x0B, 0xBC, 0x0D, 0x07, 0xF2, 0x19, 0x05, 0x02,
    0x12, 0x31, 0x35, 0x38, 0x31, 0x46, 0x35, 0x46, 0x4B, 0x44, 0x32, 0x32,
    0x39, 0x34, 0x30, 0x30, 0x42, 0x30, 0x33, 0x4E, 0x37, 0x00, 0x00, 0x00,
    0x12, 0x26, 0x5B, 0x32, 0xFD, 0xA7, 0xF8, 0x4E, 0x18, 0x51, 0xC8, 0xE9,
    0xD3, 0x8E, 0x08, 0xB1, 0x08, 0x2A, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x32, 0x00, 0x50, 0x68, 0x6F, 0x74, 0x6F, 0x67, 0x72, 0x61, 0x70,
    0x68, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x42, 0x01, 0x67, 0xB8, 0x4E, 0x18, 0x0E, 0x9B, 0xE9, 0xD3,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x52, 0x00, 0x46, 0x49, 0x4E, 0x38, 0x37, 0x61, 0x73,
    0x74, 0x72, 0x64, 0x67, 0x65, 0x31, 0x32, 0x6B, 0x38, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

static int failures;

static void check(bool ok, const char *what)
{
    if (!ok) {
        ESP_LOGE(TAG, "FAIL: %s", what);
        failures++;
    }
}

static bool near(double got, double want, double tol) { return fabs(got - want) <= tol; }

bool odid_selftest(void)
{
    failures = 0;
    odid_wifi_frame_t frame;
    check(odid_wifi_parse_frame(BEACON, sizeof(BEACON), &frame), "frame is a management frame");
    check(frame.transport == ODID_WIFI_BEACON, "ODID beacon vendor IE found");
    if (failures) {
        return false;
    }

    odid_pack_iter_t it;
    const uint8_t *msg;
    size_t msg_len;
    int seen = 0;
    check(odid_pack_iter_init(&it, frame.payload, frame.payload_len), "payload is a message pack");
    while (odid_pack_next(&it, &msg, &msg_len)) {
        odid_msg_t m;
        if (!odid_decode(msg, msg_len, &m) || !m.has_fields) {
            continue;
        }
        seen |= 1 << m.type;
        switch (m.type) {
        case ODID_MSG_BASIC_ID:
            check(m.u.basic_id.uas_id_len == 20 &&
                  memcmp(m.u.basic_id.uas_id, "1581F5FKD229400B03N7", 20) == 0, "UAS ID");
            check(m.u.basic_id.ua_type == 2, "UA type multirotor");
            break;
        case ODID_MSG_LOCATION: {
            const odid_location_t *l = &m.u.location;
            check(near(l->lat, 40.7828647, 1e-7) && near(l->lon, -73.9653551, 1e-7), "position");
            check(near(l->ground_speed_mps, 12.5, 0.125), "ground speed 12.5 m/s");
            check(near(l->heading_deg, 271.0, 0.5), "heading 271");
            check(near(l->vertical_speed_mps, -1.5, 0.25), "vertical speed -1.5 m/s");
            check(l->height_type == 1 && l->has_height && near(l->height_m, 45.0, 0.25), "height 45 m AGL");
            check(l->has_alt_geo && near(l->alt_geo_m, 112.5, 0.25), "altitude 112.5 m");
            break;
        }
        case ODID_MSG_SYSTEM: {
            const odid_system_t *s = &m.u.system;
            check(s->has_operator && near(s->operator_lat, 40.7812199, 1e-7), "operator position");
            check(s->has_alt_takeoff && near(s->alt_takeoff_m, 68.5, 0.25), "operator altitude 68.5 m");
            check(s->area_count == 1, "area count");
            break;
        }
        default:
            break;
        }
    }
    check(seen == ((1 << ODID_MSG_BASIC_ID) | (1 << ODID_MSG_LOCATION) | (1 << ODID_MSG_SELF_ID) |
                   (1 << ODID_MSG_SYSTEM) | (1 << ODID_MSG_OPERATOR_ID)), "all five message types");
    return failures == 0;
}
