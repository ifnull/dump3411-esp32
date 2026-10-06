/*
 * tracker_sim - replay a timed script through the C tracker and print the
 * snapshots, for comparison with dump3411's tracker.py on the same script
 * (tests/tracker/run_tracker_parity.py).
 *
 * Usage: tracker_sim TTL_MS < script
 *
 * Script lines, '#' comments allowed; '-' means "not present":
 *
 *   <t_ms> B <mac> <src> <rssi> <id_type> <ua_type> <uas_id_hex>
 *   <t_ms> L <mac> <src> <rssi> <lat> <lon> <alt_geo> <height> <speed> <heading> <vspeed>
 *   <t_ms> S <mac> <src> <rssi> <op_lat> <op_lon> <loc_type> <alt_takeoff>
 *   <t_ms> I <mac> <src> <rssi> <self_id_hex>
 *   <t_ms> O <mac> <src> <rssi> <operator_id_hex>
 *   <t_ms> W                       sweep
 *   <t_ms> P                       print a snapshot
 *
 * mac is 12 hex digits; src is ble | ble5 | wifi_beacon | wifi_nan.
 *
 * Each snapshot is one JSON line shaped like Tracker.snapshot()'s "messages"
 * and "drones", plus the UAS IDs reported to on_change / on_expire since the
 * previous snapshot. Values are unrounded; the comparison allows for
 * tracker.py's rounding.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "json_out.h"
#include "odid_tracker.h"

#define M_TO_FT      3.28084
#define MPS_TO_KT    1.943844
#define MPS_TO_FTPM  196.8503937

#define MAX_EVENTS 4096

typedef struct {
    uint8_t id[20];
    uint8_t len;
} id_t_;

static id_t_  changes[MAX_EVENTS], expired[MAX_EVENTS];
static size_t n_changes, n_expired;

static void on_change(void *ctx, const odid_drone_t *d, uint32_t now_ms)
{
    (void)ctx;
    (void)now_ms;
    if (n_changes < MAX_EVENTS) {
        memcpy(changes[n_changes].id, d->uas_id, d->uas_id_len);
        changes[n_changes++].len = d->uas_id_len;
    }
}

static void on_expire(void *ctx, const uint8_t *uas_id, size_t len)
{
    (void)ctx;
    if (n_expired < MAX_EVENTS) {
        memcpy(expired[n_expired].id, uas_id, len);
        expired[n_expired++].len = (uint8_t)len;
    }
}

static void die(unsigned long lineno, const char *what)
{
    fprintf(stderr, "line %lu: %s\n", lineno, what);
    exit(2);
}

static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Hex string ('-' = empty) into buf; returns the length. */
static size_t parse_hex(const char *s, uint8_t *buf, size_t max, unsigned long lineno)
{
    if (strcmp(s, "-") == 0) {
        return 0;
    }
    size_t n = strlen(s);
    if (n % 2 != 0 || n / 2 > max) {
        die(lineno, "bad hex field");
    }
    for (size_t i = 0; i < n / 2; i++) {
        int hi = hexval(s[2 * i]), lo = hexval(s[2 * i + 1]);
        if (hi < 0 || lo < 0) {
            die(lineno, "bad hex digit");
        }
        buf[i] = (uint8_t)(hi << 4 | lo);
    }
    return n / 2;
}

static bool opt_num(const char *s, double *out)
{
    if (strcmp(s, "-") == 0) {
        return false;
    }
    *out = strtod(s, NULL);
    return true;
}

static odid_source_t parse_src(const char *s, unsigned long lineno)
{
    for (int i = 0; i < ODID_SRC_COUNT; i++) {
        if (strcmp(s, odid_source_name((odid_source_t)i)) == 0) {
            return (odid_source_t)i;
        }
    }
    die(lineno, "unknown source");
    return ODID_SRC_BLE;
}

static void print_ids(const char *key, const id_t_ *ids, size_t n)
{
    printf(",\"%s\":[", key);
    for (size_t i = 0; i < n; i++) {
        if (i) putchar(',');
        json_ascii(ids[i].id, ids[i].len);
    }
    putchar(']');
}

/* Tracker._row_for, without the rounding. */
static void print_row(const odid_drone_t *d, uint32_t now_ms)
{
    printf("{\"id\":");
    json_ascii(d->uas_id, d->uas_id_len);
    printf(",\"id_type\":\"%s\",\"message_count\":%u,\"seen\":%.17g",
           odid_feed_id_type(d->id_type), d->message_count, (now_ms - d->last_seen_ms) / 1000.0);
    const char *ua = odid_feed_ua_type(d->ua_type);
    if (ua != NULL)                    printf(",\"ua_type\":\"%s\"", ua);
    if (!isnan(d->lat))                printf(",\"lat\":%.17g", d->lat);
    if (!isnan(d->lon))                printf(",\"lon\":%.17g", d->lon);
    if (!isnan(d->alt_geo_m))          printf(",\"alt_geom_ft\":%.17g", d->alt_geo_m * M_TO_FT);
    if (!isnan(d->height_agl_m))       printf(",\"agl_ft\":%.17g", d->height_agl_m * M_TO_FT);
    if (!isnan(d->ground_speed_mps))   printf(",\"gs\":%.17g", d->ground_speed_mps * MPS_TO_KT);
    if (!isnan(d->heading_deg))        printf(",\"track\":%.17g", d->heading_deg);
    if (!isnan(d->vertical_speed_mps)) printf(",\"geom_rate\":%.17g", d->vertical_speed_mps * MPS_TO_FTPM);
    if (d->rssi_dbm != ODID_RSSI_NONE) printf(",\"rssi\":%d", d->rssi_dbm);
    if (d->has_last_pos)               printf(",\"seen_pos\":%.17g", (now_ms - d->last_pos_ms) / 1000.0);
    if (d->rid_source >= 0)            printf(",\"rid_source\":\"%s\"", odid_source_name((odid_source_t)d->rid_source));
    if (d->source_count > 0) {
        printf(",\"rid_sources\":[");
        for (size_t i = 0; i < d->source_count; i++) {
            printf("%s\"%s\"", i ? "," : "", odid_source_name((odid_source_t)d->sources[i]));
        }
        putchar(']');
    }
    if (d->has_self_id) {
        printf(",\"self_id\":");
        json_ascii(d->self_id, d->self_id_len);
    }
    if (d->has_last_self_id)           printf(",\"self_id_seen\":%.17g", (now_ms - d->last_self_id_ms) / 1000.0);

    /* operator block, only if any of it is known */
    bool any = !isnan(d->operator_lat) || !isnan(d->operator_lon) || d->operator_location_type != 0xFF ||
               d->has_operator_id || !isnan(d->operator_alt_takeoff_m) || d->has_last_operator;
    if (any) {
        const char *sep = "";
        printf(",\"operator\":{");
        if (!isnan(d->operator_lat)) { printf("%s\"lat\":%.17g", sep, d->operator_lat); sep = ","; }
        if (!isnan(d->operator_lon)) { printf("%s\"lon\":%.17g", sep, d->operator_lon); sep = ","; }
        if (d->operator_location_type != 0xFF) {
            printf("%s\"location_type\":\"%s\"", sep, odid_operator_location_type_name(d->operator_location_type));
            sep = ",";
        }
        if (d->has_operator_id) {
            printf("%s\"id\":", sep);
            json_ascii(d->operator_id, d->operator_id_len);
            sep = ",";
        }
        if (!isnan(d->operator_alt_takeoff_m)) {
            printf("%s\"alt_takeoff_ft\":%.17g", sep, d->operator_alt_takeoff_m * M_TO_FT);
            sep = ",";
        }
        if (d->has_last_operator) {
            printf("%s\"seen\":%.17g", sep, (now_ms - d->last_operator_ms) / 1000.0);
        }
        putchar('}');
    }
    putchar('}');
}

static void snapshot(const odid_tracker_t *t, uint32_t now_ms)
{
    const odid_drone_t *live[ODID_TRACKER_MAX_DRONES];
    size_t n = odid_tracker_list(t, live, ODID_TRACKER_MAX_DRONES);
    printf("{\"t\":%u,\"messages\":%u,\"drones\":[", now_ms, t->messages_total);
    for (size_t i = 0; i < n; i++) {
        if (i) putchar(',');
        print_row(live[i], now_ms);
    }
    putchar(']');
    print_ids("changes", changes, n_changes);
    print_ids("expired", expired, n_expired);
    printf("}\n");
    n_changes = n_expired = 0;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: tracker_sim TTL_MS < script\n");
        return 2;
    }
    static odid_tracker_t t;
    odid_tracker_callbacks_t cb = {.on_change = on_change, .on_expire = on_expire};
    odid_tracker_init(&t, (uint32_t)strtoul(argv[1], NULL, 10), &cb);

    char line[1024];
    unsigned long lineno = 0;
    while (fgets(line, sizeof(line), stdin) != NULL) {
        lineno++;
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';
        char *f[16];
        size_t nf = 0;
        for (char *tok = strtok(line, " \t\r\n"); tok && nf < 16; tok = strtok(NULL, " \t\r\n")) {
            f[nf++] = tok;
        }
        if (nf == 0) continue;
        if (nf < 2) die(lineno, "missing op");

        uint32_t now = (uint32_t)strtoul(f[0], NULL, 10);
        char op = f[1][0];
        if (op == 'W') { odid_tracker_sweep(&t, now); continue; }
        if (op == 'P') { snapshot(&t, now); continue; }
        if (nf < 5) die(lineno, "missing fields");

        uint8_t mac[6];
        if (parse_hex(f[2], mac, 6, lineno) != 6) die(lineno, "bad mac");
        odid_source_t src = parse_src(f[3], lineno);
        double r;
        int16_t rssi = opt_num(f[4], &r) ? (int16_t)r : ODID_RSSI_NONE;

        switch (op) {
        case 'B': {
            if (nf != 8) die(lineno, "B needs 8 fields");
            odid_basic_id_t b = {0};
            b.id_type    = (uint8_t)atoi(f[5]);
            b.ua_type    = (uint8_t)atoi(f[6]);
            b.uas_id_len = (uint8_t)parse_hex(f[7], b.uas_id, sizeof(b.uas_id), lineno);
            odid_tracker_basic_id(&t, now, mac, &b, rssi, src);
            break;
        }
        case 'L': {
            if (nf != 12) die(lineno, "L needs 12 fields");
            odid_location_t l = {0};
            l.lat = strtod(f[5], NULL);
            l.lon = strtod(f[6], NULL);
            l.has_alt_geo = opt_num(f[7], &l.alt_geo_m);
            l.has_height  = opt_num(f[8], &l.height_m);
            l.ground_speed_mps   = strtod(f[9], NULL);
            l.heading_deg        = strtod(f[10], NULL);
            l.vertical_speed_mps = strtod(f[11], NULL);
            odid_tracker_location(&t, now, mac, &l, rssi, src);
            break;
        }
        case 'S': {
            if (nf != 9) die(lineno, "S needs 9 fields");
            odid_system_t s = {0};
            s.has_operator = opt_num(f[5], &s.operator_lat);
            opt_num(f[6], &s.operator_lon);
            double lt = 0;
            opt_num(f[7], &lt);
            s.operator_location_type = (uint8_t)lt;
            s.has_alt_takeoff = opt_num(f[8], &s.alt_takeoff_m);
            odid_tracker_system(&t, now, mac, &s, rssi, src);
            break;
        }
        case 'I': {
            odid_self_id_t s = {0};
            s.description_len = (uint8_t)parse_hex(f[5], s.description, sizeof(s.description), lineno);
            odid_tracker_self_id(&t, now, mac, &s, rssi, src);
            break;
        }
        case 'O': {
            odid_operator_id_t o = {0};
            o.operator_id_len = (uint8_t)parse_hex(f[5], o.operator_id, sizeof(o.operator_id), lineno);
            odid_tracker_operator_id(&t, now, mac, &o, rssi, src);
            break;
        }
        default:
            die(lineno, "unknown op");
        }
    }
    return 0;
}
