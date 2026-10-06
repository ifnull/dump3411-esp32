/*
 * odid_tracker.c - see odid_tracker.h. Comments name the dump3411
 * tracker.py method each function mirrors.
 */
#include "odid_tracker.h"

#include <math.h>
#include <string.h>

/* -- helpers --------------------------------------------------------------- */

/* Milliseconds since `then`, correct across uint32 wraparound. */
static uint32_t age_ms(uint32_t now, uint32_t then) { return now - then; }

static bool same_id(const uint8_t *a, size_t a_len, const uint8_t *b, size_t b_len)
{
    return a_len == b_len && memcmp(a, b, a_len) == 0;
}

static odid_drone_t *find_drone(odid_tracker_t *t, const uint8_t *uas_id, size_t len)
{
    for (size_t i = 0; i < ODID_TRACKER_MAX_DRONES; i++) {
        odid_drone_t *d = &t->drones[i];
        if (d->in_use && same_id(d->uas_id, d->uas_id_len, uas_id, len)) {
            return d;
        }
    }
    return NULL;
}

static odid_mac_entry_t *find_mac(odid_tracker_t *t, const uint8_t mac[6])
{
    for (size_t i = 0; i < ODID_TRACKER_MAX_MACS; i++) {
        if (t->macs[i].in_use && memcmp(t->macs[i].mac, mac, 6) == 0) {
            return &t->macs[i];
        }
    }
    return NULL;
}

/* Drop MAC mappings whose drone no longer exists (end of _sweep_once). */
static void drop_dead_macs(odid_tracker_t *t)
{
    for (size_t i = 0; i < ODID_TRACKER_MAX_MACS; i++) {
        odid_mac_entry_t *m = &t->macs[i];
        if (m->in_use && find_drone(t, m->uas_id, m->uas_id_len) == NULL) {
            m->in_use = false;
        }
    }
}

static void fire_expire(odid_tracker_t *t, const uint8_t *uas_id, size_t len)
{
    if (t->cb.on_expire != NULL) {
        t->cb.on_expire(t->cb.ctx, uas_id, len);
    }
}

static void fire_change(odid_tracker_t *t, const odid_drone_t *d, uint32_t now_ms)
{
    if (t->cb.on_change != NULL) {
        t->cb.on_change(t->cb.ctx, d, now_ms);
    }
}

/* _drone_for_mac */
static odid_drone_t *drone_for_mac(odid_tracker_t *t, const uint8_t mac[6], uint32_t now_ms)
{
    odid_mac_entry_t *m = find_mac(t, mac);
    if (m == NULL) {
        return NULL;
    }
    m->last_used_ms = now_ms;
    return find_drone(t, m->uas_id, m->uas_id_len);
}

static void map_mac(odid_tracker_t *t, const uint8_t mac[6], const odid_drone_t *d, uint32_t now_ms)
{
    odid_mac_entry_t *m = find_mac(t, mac);
    if (m == NULL) {
        for (size_t i = 0; i < ODID_TRACKER_MAX_MACS && m == NULL; i++) {
            if (!t->macs[i].in_use) {
                m = &t->macs[i];
            }
        }
    }
    if (m == NULL) {
        /* Full: reuse the least recently used mapping. */
        m = &t->macs[0];
        for (size_t i = 1; i < ODID_TRACKER_MAX_MACS; i++) {
            if (age_ms(now_ms, t->macs[i].last_used_ms) > age_ms(now_ms, m->last_used_ms)) {
                m = &t->macs[i];
            }
        }
    }
    m->in_use = true;
    memcpy(m->mac, mac, 6);
    memcpy(m->uas_id, d->uas_id, d->uas_id_len);
    m->uas_id_len   = d->uas_id_len;
    m->last_used_ms = now_ms;
}

/* _bump_source */
static void bump_source(odid_tracker_t *t, odid_source_t src, uint32_t now_ms)
{
    t->by_source[src].messages++;
    t->by_source[src].heard        = true;
    t->by_source[src].last_seen_ms = now_ms;
}

/* _touch_sources: move src to the front of the heard-from list. */
static void touch_sources(odid_drone_t *d, odid_source_t src)
{
    size_t i = 0;
    while (i < d->source_count && d->sources[i] != src) {
        i++;
    }
    if (i == d->source_count) {
        d->source_count++;            /* new transport; list grows by one */
    }
    memmove(&d->sources[1], &d->sources[0], i);
    d->sources[0] = (uint8_t)src;
}

/* The per-message bookkeeping every update_* in tracker.py repeats. */
static void heard(odid_drone_t *d, uint32_t now_ms, int16_t rssi, odid_source_t src)
{
    d->rssi_dbm   = rssi;
    d->rid_source = (int8_t)src;
    touch_sources(d, src);
    d->message_count++;
    d->last_seen_ms = now_ms;
}

static odid_drone_t *new_drone(odid_tracker_t *t, uint32_t now_ms)
{
    odid_drone_t *d = NULL;
    for (size_t i = 0; i < ODID_TRACKER_MAX_DRONES && d == NULL; i++) {
        if (!t->drones[i].in_use) {
            d = &t->drones[i];
        }
    }
    if (d == NULL) {
        /* Full: evict the drone heard from least recently. */
        d = &t->drones[0];
        for (size_t i = 1; i < ODID_TRACKER_MAX_DRONES; i++) {
            if (age_ms(now_ms, t->drones[i].last_seen_ms) > age_ms(now_ms, d->last_seen_ms)) {
                d = &t->drones[i];
            }
        }
        uint8_t gone[20];
        size_t gone_len = d->uas_id_len;
        memcpy(gone, d->uas_id, gone_len);
        d->in_use = false;
        drop_dead_macs(t);
        fire_expire(t, gone, gone_len);
    }
    memset(d, 0, sizeof(*d));
    d->in_use       = true;
    d->seq          = t->next_seq++;
    d->ua_type      = -1;
    d->lat = d->lon = NAN;
    d->alt_geo_m = d->height_agl_m = NAN;
    d->ground_speed_mps = d->heading_deg = d->vertical_speed_mps = NAN;
    d->rssi_dbm     = ODID_RSSI_NONE;
    d->rid_source   = -1;
    d->operator_lat = d->operator_lon = d->operator_alt_takeoff_m = NAN;
    d->operator_location_type = 0xFF;
    return d;
}

/* -- public API ------------------------------------------------------------ */

void odid_tracker_init(odid_tracker_t *t, uint32_t ttl_ms, const odid_tracker_callbacks_t *cb)
{
    memset(t, 0, sizeof(*t));
    t->ttl_ms = ttl_ms;
    if (cb != NULL) {
        t->cb = *cb;
    }
}

/* update_basic_id: creates the drone if new. */
void odid_tracker_basic_id(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                           const odid_basic_id_t *msg, int16_t rssi, odid_source_t src)
{
    t->messages_total++;
    bump_source(t, src, now_ms);

    bool ua_known = odid_feed_ua_type(msg->ua_type) != NULL;
    odid_drone_t *d = find_drone(t, msg->uas_id, msg->uas_id_len);
    if (d == NULL) {
        d = new_drone(t, now_ms);
        memcpy(d->uas_id, msg->uas_id, msg->uas_id_len);
        d->uas_id_len = msg->uas_id_len;
        d->id_type    = msg->id_type;
        d->ua_type    = ua_known ? msg->ua_type : -1;
    } else if (d->ua_type < 0 && ua_known) {
        d->ua_type = msg->ua_type;          /* identity is write-once */
    }
    heard(d, now_ms, rssi, src);
    map_mac(t, mac, d, now_ms);
    fire_change(t, d, now_ms);
}

/* update_location: dropped if the MAC isn't mapped yet. */
void odid_tracker_location(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                           const odid_location_t *msg, int16_t rssi, odid_source_t src)
{
    odid_drone_t *d = drone_for_mac(t, mac, now_ms);
    if (d == NULL) {
        return;
    }
    t->messages_total++;
    bump_source(t, src, now_ms);
    d->lat                = msg->lat;
    d->lon                = msg->lon;
    d->alt_geo_m          = msg->has_alt_geo ? msg->alt_geo_m : (double)NAN;
    d->height_agl_m       = msg->has_height ? msg->height_m : (double)NAN;
    d->ground_speed_mps   = msg->ground_speed_mps;
    d->heading_deg        = msg->heading_deg;
    d->vertical_speed_mps = msg->vertical_speed_mps;
    d->has_position       = true;
    heard(d, now_ms, rssi, src);
    d->has_last_pos = true;
    d->last_pos_ms  = now_ms;
    fire_change(t, d, now_ms);
}

/* update_system: only usable values overwrite. */
void odid_tracker_system(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                         const odid_system_t *msg, int16_t rssi, odid_source_t src)
{
    odid_drone_t *d = drone_for_mac(t, mac, now_ms);
    if (d == NULL) {
        return;
    }
    t->messages_total++;
    bump_source(t, src, now_ms);
    if (msg->has_operator) {
        d->operator_lat           = msg->operator_lat;
        d->operator_lon           = msg->operator_lon;
        d->operator_location_type = msg->operator_location_type;
    }
    if (msg->has_alt_takeoff) {
        d->operator_alt_takeoff_m = msg->alt_takeoff_m;
    }
    heard(d, now_ms, rssi, src);
    d->has_last_operator = true;
    d->last_operator_ms  = now_ms;
    fire_change(t, d, now_ms);
}

/* update_self_id: an empty description doesn't overwrite. */
void odid_tracker_self_id(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                          const odid_self_id_t *msg, int16_t rssi, odid_source_t src)
{
    odid_drone_t *d = drone_for_mac(t, mac, now_ms);
    if (d == NULL) {
        return;
    }
    t->messages_total++;
    bump_source(t, src, now_ms);
    if (msg->description_len > 0) {
        memcpy(d->self_id, msg->description, msg->description_len);
        d->self_id_len = msg->description_len;
        d->has_self_id = true;
    }
    heard(d, now_ms, rssi, src);
    d->has_last_self_id = true;
    d->last_self_id_ms  = now_ms;
    fire_change(t, d, now_ms);
}

/* update_operator_id: an empty ID doesn't overwrite. */
void odid_tracker_operator_id(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                              const odid_operator_id_t *msg, int16_t rssi, odid_source_t src)
{
    odid_drone_t *d = drone_for_mac(t, mac, now_ms);
    if (d == NULL) {
        return;
    }
    t->messages_total++;
    bump_source(t, src, now_ms);
    if (msg->operator_id_len > 0) {
        memcpy(d->operator_id, msg->operator_id, msg->operator_id_len);
        d->operator_id_len = msg->operator_id_len;
        d->has_operator_id = true;
    }
    heard(d, now_ms, rssi, src);
    d->has_last_operator = true;
    d->last_operator_ms  = now_ms;
    fire_change(t, d, now_ms);
}

void odid_tracker_update(odid_tracker_t *t, uint32_t now_ms, const uint8_t mac[6],
                         const odid_msg_t *msg, int16_t rssi, odid_source_t src)
{
    if (!msg->has_fields) {
        return;      /* includes Locations with no GPS fix, which dump3411 skips */
    }
    switch (msg->type) {
    case ODID_MSG_BASIC_ID:    odid_tracker_basic_id(t, now_ms, mac, &msg->u.basic_id, rssi, src); break;
    case ODID_MSG_LOCATION:    odid_tracker_location(t, now_ms, mac, &msg->u.location, rssi, src); break;
    case ODID_MSG_SYSTEM:      odid_tracker_system(t, now_ms, mac, &msg->u.system, rssi, src); break;
    case ODID_MSG_SELF_ID:     odid_tracker_self_id(t, now_ms, mac, &msg->u.self_id, rssi, src); break;
    case ODID_MSG_OPERATOR_ID: odid_tracker_operator_id(t, now_ms, mac, &msg->u.operator_id, rssi, src); break;
    default: break;
    }
}

/* _sweep_once */
void odid_tracker_sweep(odid_tracker_t *t, uint32_t now_ms)
{
    const odid_drone_t *live[ODID_TRACKER_MAX_DRONES];
    size_t n = odid_tracker_list(t, live, ODID_TRACKER_MAX_DRONES);

    /* Collect first, in creation order, so expiry callbacks fire in the same
     * order as dump3411's (dict order) and after all removals. */
    uint8_t gone[ODID_TRACKER_MAX_DRONES][20];
    uint8_t gone_len[ODID_TRACKER_MAX_DRONES];
    size_t  n_gone = 0;
    for (size_t i = 0; i < n; i++) {
        odid_drone_t *d = (odid_drone_t *)live[i];
        if (age_ms(now_ms, d->last_seen_ms) > t->ttl_ms) {
            memcpy(gone[n_gone], d->uas_id, d->uas_id_len);
            gone_len[n_gone++] = d->uas_id_len;
            d->in_use = false;
        }
    }
    if (n_gone > 0) {
        drop_dead_macs(t);
    }
    for (size_t i = 0; i < n_gone; i++) {
        fire_expire(t, gone[i], gone_len[i]);
    }
}

size_t odid_tracker_list(const odid_tracker_t *t, const odid_drone_t **out, size_t max)
{
    size_t n = 0;
    for (size_t i = 0; i < ODID_TRACKER_MAX_DRONES && n < max; i++) {
        if (t->drones[i].in_use) {
            out[n++] = &t->drones[i];
        }
    }
    /* Insertion sort by creation order; n is small. */
    for (size_t i = 1; i < n; i++) {
        const odid_drone_t *d = out[i];
        size_t j = i;
        while (j > 0 && out[j - 1]->seq > d->seq) {
            out[j] = out[j - 1];
            j--;
        }
        out[j] = d;
    }
    return n;
}

/* -- feed strings (tracker.py _ID_TYPE_STRINGS / _UA_TYPE_STRINGS) --------- */

const char *odid_source_name(odid_source_t src)
{
    switch (src) {
    case ODID_SRC_BLE:         return "ble";
    case ODID_SRC_BLE5:        return "ble5";
    case ODID_SRC_WIFI_BEACON: return "wifi_beacon";
    case ODID_SRC_WIFI_NAN:    return "wifi_nan";
    default:                   return "?";
    }
}

const char *odid_feed_id_type(uint8_t id_type)
{
    static const char *const names[] = {"unknown", "serial", "caa_reg", "utm_uuid", "session"};
    return id_type < sizeof(names) / sizeof(names[0]) ? names[id_type] : "unknown";
}

const char *odid_feed_ua_type(int16_t ua_type)
{
    static const char *const names[] = {
        "none", "aeroplane", "multirotor", "gyroplane", "hybrid", "ornithopter",
        "glider", "kite", "free_balloon", "captive_balloon", "airship", "parachute",
        "rocket", "tethered", "ground_obstacle",
    };
    if (ua_type == 255) {
        return "other";
    }
    if (ua_type >= 0 && ua_type < (int16_t)(sizeof(names) / sizeof(names[0]))) {
        return names[ua_type];
    }
    return NULL;
}
