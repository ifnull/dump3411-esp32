/*
 * odid_parser.h - ASTM F3411 / OpenDroneID message decoder.
 *
 * A C port of dump3411's decode_rid_message() and its parse_* helpers
 * (dump3411 wifi_feeder.py; ble_feeder.py carries an identical copy). The
 * behaviour, including which fields are present or absent, follows that
 * Python code rather than the spec text, so the two can be diffed
 * field-for-field by tests/parity/run_parity.py.
 *
 * No allocation, no globals, no platform headers: builds unchanged for the
 * host (tests) and for ESP-IDF.
 */
#ifndef ODID_PARSER_H
#define ODID_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ODID_MSG_SIZE 25

enum {
    ODID_MSG_BASIC_ID    = 0x0,
    ODID_MSG_LOCATION    = 0x1,
    ODID_MSG_AUTH        = 0x2,
    ODID_MSG_SELF_ID     = 0x3,
    ODID_MSG_SYSTEM      = 0x4,
    ODID_MSG_OPERATOR_ID = 0x5,
    ODID_MSG_PACK        = 0xF,
};

/*
 * Text fields keep the raw bytes with trailing NULs stripped, like Python's
 * data[a:b].rstrip(b'\x00'). Embedded NULs and non-ASCII bytes are kept as-is;
 * rendering them is the caller's job.
 */
typedef struct {
    uint8_t id_type;          /* 4-bit enum, see odid_id_type_name() */
    uint8_t ua_type;          /* 4-bit enum, see odid_ua_type_name() */
    uint8_t uas_id[20];
    uint8_t uas_id_len;
} odid_basic_id_t;

typedef struct {
    uint8_t height_type;      /* 0 = above takeoff, 1 = AGL */
    double  heading_deg;      /* 0-359 */
    double  ground_speed_mps;
    double  vertical_speed_mps;
    double  lat;
    double  lon;
    bool    has_alt_geo;      /* false when the wire value is the -1000 m "unknown" */
    double  alt_geo_m;
    bool    has_height;
    double  height_m;
} odid_location_t;

typedef struct {
    uint8_t description_type;
    uint8_t description[23];
    uint8_t description_len;
} odid_self_id_t;

typedef struct {
    uint16_t area_count;
    uint16_t area_radius_m;
    bool     has_operator;    /* false when operator lat/lon are out of range */
    double   operator_lat;
    double   operator_lon;
    uint8_t  operator_location_type;  /* see odid_operator_location_type_name() */
    bool     has_alt_takeoff;
    double   alt_takeoff_m;
} odid_system_t;

typedef struct {
    uint8_t operator_id_type;
    uint8_t operator_id[20];
    uint8_t operator_id_len;
} odid_operator_id_t;

typedef struct {
    uint8_t type;             /* high nibble of byte 0 */
    /*
     * true when the type-specific fields below are valid. false for types
     * with no decoder (Authentication, Message Pack, unknown), for messages
     * too short for their type, and for a Location whose lat/lon are out of
     * range (a pre-GPS-lock placeholder). Mirrors the Python parse_* helpers
     * returning {}.
     */
    bool    has_fields;
    union {
        odid_basic_id_t    basic_id;
        odid_location_t    location;
        odid_self_id_t     self_id;
        odid_system_t      system;
        odid_operator_id_t operator_id;
    } u;
} odid_msg_t;

/*
 * Decode one ODID message. Returns false only when len < 2 (Python returns
 * None); every other input decodes to at least a type. A Message Pack
 * decodes with has_fields == false: walk its sub-messages with
 * odid_pack_iter_init() / odid_pack_next().
 */
bool odid_decode(const uint8_t *data, size_t len, odid_msg_t *out);

typedef struct {
    const uint8_t *data;
    size_t         len;
    uint8_t        msg_size;
    uint8_t        msg_count;
    uint8_t        index;
} odid_pack_iter_t;

/* Returns false when the pack is shorter than its 3-byte header. */
bool odid_pack_iter_init(odid_pack_iter_t *it, const uint8_t *pack, size_t len);

/*
 * Yields each sub-message slice in order; stops early at the first one that
 * would run past the end of the buffer. A slice may be shorter than 2 bytes
 * (msg_size 0 or 1), in which case odid_decode() rejects it, matching
 * Python's filtering of None results.
 */
bool odid_pack_next(odid_pack_iter_t *it, const uint8_t **msg, size_t *msg_len);

/* Display names, identical to dump3411's tables. NULL when unknown. */
const char *odid_msg_type_name(uint8_t type);
const char *odid_id_type_name(uint8_t id_type);
const char *odid_ua_type_name(uint8_t ua_type);
const char *odid_operator_location_type_name(uint8_t loc_type);  /* "reserved" for 3 */

#ifdef __cplusplus
}
#endif

#endif /* ODID_PARSER_H */
