/*
 * odid_tracker.h - per-drone state aggregated from decoded ODID messages.
 *
 * A C port of dump3411's tracker.Tracker. The rules are the same:
 *
 *   - Drones are keyed by UAS ID. Only Basic ID carries it, so a short-lived
 *     transmitter-MAC -> UAS ID map ties Location, System, Self ID and
 *     Operator ID messages to their drone. A non-Basic-ID message from an
 *     unmapped MAC is dropped.
 *   - Identity is write-once: id_type is fixed when the drone is created,
 *     ua_type is filled in the first time a known value arrives.
 *   - Everything else is most-recent-wins, except that System fields only
 *     overwrite when the decoder produced a usable value, and empty Self ID
 *     and Operator ID strings are ignored.
 *   - A drone not heard from for ttl_ms is removed by odid_tracker_sweep(),
 *     along with any MAC mappings that pointed at it.
 *
 * Differences from the Python, both forced by running without malloc:
 *
 *   - Time is passed in by the caller (now_ms, any monotonic millisecond
 *     clock) instead of read inside. Wraparound after ~49 days is handled.
 *   - Capacity is fixed. When the drone table is full, a new drone replaces
 *     the least recently heard one (reported through on_expire); when the
 *     MAC table is full, the least recently used mapping is dropped.
 *
 * Not thread-safe: own a tracker from one task and hand other tasks copies.
 */
#ifndef ODID_TRACKER_H
#define ODID_TRACKER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "odid_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ODID_TRACKER_MAX_DRONES
#define ODID_TRACKER_MAX_DRONES 32
#endif
#ifndef ODID_TRACKER_MAX_MACS
#define ODID_TRACKER_MAX_MACS   64
#endif

/* Values match the rid_source strings in dump3411's feed (FEED.md). */
typedef enum {
    ODID_SRC_BLE = 0,        /* "ble": Bluetooth 4 legacy */
    ODID_SRC_BLE5,           /* "ble5": Bluetooth 5 long range */
    ODID_SRC_WIFI_BEACON,    /* "wifi_beacon" */
    ODID_SRC_WIFI_NAN,       /* "wifi_nan" */
    ODID_SRC_COUNT
} odid_source_t;

#define ODID_RSSI_NONE INT16_MIN

typedef struct {
    bool     in_use;
    uint32_t seq;                 /* creation order; snapshots list drones in this order */

    uint8_t  uas_id[20];
    uint8_t  uas_id_len;
    uint8_t  id_type;             /* raw enum, fixed at creation */
    int16_t  ua_type;             /* raw enum, -1 until a known value arrives */

    /* Position and velocity, SI units. NAN means not known. */
    double   lat, lon;
    double   alt_geo_m, height_agl_m;
    double   ground_speed_mps, heading_deg, vertical_speed_mps;
    bool     has_position;

    int16_t  rssi_dbm;            /* ODID_RSSI_NONE when not known */
    int8_t   rid_source;          /* odid_source_t of the latest message, -1 none yet */
    uint8_t  sources[ODID_SRC_COUNT];   /* every transport heard, most recent first */
    uint8_t  source_count;

    /* Operator block. NAN / 0xFF means not known. */
    double   operator_lat, operator_lon, operator_alt_takeoff_m;
    uint8_t  operator_location_type;
    uint8_t  operator_id[20];
    uint8_t  operator_id_len;
    bool     has_operator_id;

    uint8_t  self_id[23];
    uint8_t  self_id_len;
    bool     has_self_id;

    uint32_t message_count;
    uint32_t last_seen_ms;
    bool     has_last_pos;
    uint32_t last_pos_ms;
    bool     has_last_operator;
    uint32_t last_operator_ms;
    bool     has_last_self_id;
    uint32_t last_self_id_ms;
} odid_drone_t;

typedef struct {
    /* Called after every accepted update. Keep it short (e.g. post to a queue). */
    void (*on_change)(void *ctx, const odid_drone_t *drone, uint32_t now_ms);
    /* Called for each drone removed by a sweep or evicted for space. */
    void (*on_expire)(void *ctx, const uint8_t *uas_id, size_t uas_id_len);
    void *ctx;
} odid_tracker_callbacks_t;

typedef struct {
    bool     in_use;
    uint8_t  mac[6];
    uint8_t  uas_id[20];
    uint8_t  uas_id_len;
    uint32_t last_used_ms;
} odid_mac_entry_t;

typedef struct {
    uint32_t messages;
    bool     heard;
    uint32_t last_seen_ms;
} odid_source_stats_t;

typedef struct {
    uint32_t                 ttl_ms;
    odid_tracker_callbacks_t cb;
    uint32_t                 next_seq;
    uint32_t                 messages_total;
    odid_source_stats_t      by_source[ODID_SRC_COUNT];
    odid_drone_t             drones[ODID_TRACKER_MAX_DRONES];
    odid_mac_entry_t         macs[ODID_TRACKER_MAX_MACS];
} odid_tracker_t;

/* cb may be NULL. */
void odid_tracker_init(odid_tracker_t *t, uint32_t ttl_ms, const odid_tracker_callbacks_t *cb);

void odid_tracker_basic_id(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                           const odid_basic_id_t *msg, int16_t rssi, odid_source_t src);
void odid_tracker_location(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                           const odid_location_t *msg, int16_t rssi, odid_source_t src);
void odid_tracker_system(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                         const odid_system_t *msg, int16_t rssi, odid_source_t src);
void odid_tracker_self_id(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                          const odid_self_id_t *msg, int16_t rssi, odid_source_t src);
void odid_tracker_operator_id(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                              const odid_operator_id_t *msg, int16_t rssi, odid_source_t src);

/*
 * Route one decoded message to the matching update above. Message types
 * with no tracker state (Authentication, packs, unknown) are ignored. For a
 * Message Pack, feed the Basic ID sub-message first so the MAC is mapped
 * before the rest of the pack arrives (as dump3411 does).
 */
void odid_tracker_update(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                         const odid_msg_t *msg, int16_t rssi, odid_source_t src);

/* Remove drones not heard from for ttl_ms. Call about once a second. */
void odid_tracker_sweep(odid_tracker_t *t, uint32_t now_ms);

/*
 * Fill out[] with pointers to the live drones in creation order (the order
 * dump3411's snapshot lists them). Returns how many were written.
 */
size_t odid_tracker_list(const odid_tracker_t *t, const odid_drone_t **out, size_t max);

/* Display strings matching dump3411's feed values. */
const char *odid_source_name(odid_source_t src);
const char *odid_feed_id_type(uint8_t id_type);      /* "unknown" for unassigned */
const char *odid_feed_ua_type(int16_t ua_type);      /* NULL when not known */

#ifdef __cplusplus
}
#endif

#endif /* ODID_TRACKER_H */
