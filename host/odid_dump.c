/*
 * odid_dump - decode fixture lines with the C parser and print JSON.
 *
 * Input, one per line on stdin (blank lines and '#' comments ignored, and
 * anything after a '#' on a line is a comment):
 *
 *   m <hex>    one ODID message or Message Pack
 *   f <hex>    one raw 802.11 frame, starting at the MAC header
 *
 * Output: one JSON value per input line, shaped like the dict dump3411 builds
 * for the same bytes (decode_rid_message for 'm', the wifi_feeder frame path
 * for 'f'), so tests/parity/run_parity.py can compare them field by field.
 */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "json_out.h"
#include "odid_parser.h"
#include "odid_wifi.h"

#define MAX_LINE 8192

static void put_hex(const uint8_t *b, size_t n)
{
    putchar('"');
    for (size_t i = 0; i < n; i++) {
        printf("%02X", b[i]);
    }
    putchar('"');
}

static void put_name(const char *name, const char *unknown_fmt, unsigned value)
{
    if (name != NULL) {
        printf("\"%s\"", name);
    } else {
        putchar('"');
        printf(unknown_fmt, value);
        putchar('"');
    }
}

static void put_message(const uint8_t *data, size_t len);

static void put_fields(const odid_msg_t *m)
{
    switch (m->type) {
    case ODID_MSG_BASIC_ID: {
        const odid_basic_id_t *b = &m->u.basic_id;
        fputs(",\"id_type\":", stdout);
        put_name(odid_id_type_name(b->id_type), "Unknown(%u)", b->id_type);
        fputs(",\"ua_type\":", stdout);
        put_name(odid_ua_type_name(b->ua_type), "Unknown(%u)", b->ua_type);
        printf(",\"id_type_raw\":%u,\"ua_type_raw\":%u,\"uas_id\":", b->id_type, b->ua_type);
        json_ascii(b->uas_id, b->uas_id_len);
        break;
    }
    case ODID_MSG_LOCATION: {
        const odid_location_t *l = &m->u.location;
        printf(",\"latitude\":%.17g,\"longitude\":%.17g", l->lat, l->lon);
        printf(",\"ground_speed\":%.17g,\"vertical_speed\":%.17g,\"heading\":%.17g",
               l->ground_speed_mps, l->vertical_speed_mps, l->heading_deg);
        printf(",\"height_type\":\"%s\"", l->height_type ? "AGL" : "Above Takeoff");
        if (l->has_alt_geo) {
            printf(",\"altitude_geo\":%.17g", l->alt_geo_m);
        }
        if (l->has_height) {
            printf(",\"height_agl\":%.17g", l->height_m);
        }
        break;
    }
    case ODID_MSG_SELF_ID: {
        const odid_self_id_t *s = &m->u.self_id;
        printf(",\"description_type\":%u,\"description\":", s->description_type);
        json_ascii(s->description, s->description_len);
        break;
    }
    case ODID_MSG_SYSTEM: {
        const odid_system_t *s = &m->u.system;
        printf(",\"area_count\":%u,\"area_radius_m\":%u", s->area_count, s->area_radius_m);
        if (s->has_operator) {
            printf(",\"operator_lat\":%.17g,\"operator_lon\":%.17g,\"operator_location_type\":\"%s\"",
                   s->operator_lat, s->operator_lon,
                   odid_operator_location_type_name(s->operator_location_type));
        }
        if (s->has_alt_takeoff) {
            printf(",\"alt_takeoff_geo\":%.17g", s->alt_takeoff_m);
        }
        break;
    }
    case ODID_MSG_OPERATOR_ID: {
        const odid_operator_id_t *o = &m->u.operator_id;
        printf(",\"operator_id_type\":%u,\"operator_id\":", o->operator_id_type);
        json_ascii(o->operator_id, o->operator_id_len);
        break;
    }
    default:
        break;
    }
}

/* decode_rid_message(): null for < 2 bytes, packs recurse into "messages". */
static void put_message(const uint8_t *data, size_t len)
{
    odid_msg_t m;
    if (!odid_decode(data, len, &m)) {
        fputs("null", stdout);
        return;
    }
    fputs("{\"message_type\":", stdout);
    put_name(odid_msg_type_name(m.type), "Unknown(0x%X)", m.type);
    fputs(",\"raw_hex\":", stdout);
    put_hex(data, len);
    if (m.has_fields) {
        put_fields(&m);
    }
    if (m.type == ODID_MSG_PACK) {
        fputs(",\"messages\":[", stdout);
        odid_pack_iter_t it;
        const uint8_t *sub;
        size_t sub_len;
        bool first = true;
        if (odid_pack_iter_init(&it, data, len)) {
            while (odid_pack_next(&it, &sub, &sub_len)) {
                odid_msg_t probe;
                if (!odid_decode(sub, sub_len, &probe)) {
                    continue;          /* Python drops None sub-messages */
                }
                if (!first) {
                    putchar(',');
                }
                first = false;
                put_message(sub, sub_len);
            }
        }
        putchar(']');
    }
    putchar('}');
}

static void put_frame(const uint8_t *data, size_t len)
{
    odid_wifi_frame_t f;
    if (!odid_wifi_parse_frame(data, len, &f)) {
        fputs("{\"mgmt\":false}", stdout);
        return;
    }
    printf("{\"mgmt\":true,\"subtype\":%u,\"addr2\":\"%02x:%02x:%02x:%02x:%02x:%02x\"",
           f.subtype, f.addr2[0], f.addr2[1], f.addr2[2], f.addr2[3], f.addr2[4], f.addr2[5]);
    const char *transport = f.transport == ODID_WIFI_BEACON ? "\"beacon\""
                          : f.transport == ODID_WIFI_NAN    ? "\"nan\""
                          : "null";
    printf(",\"transport\":%s,\"nan_unparsed\":%s,\"decoded\":",
           transport, f.nan_unparsed ? "true" : "false");
    if (f.payload != NULL) {
        put_message(f.payload, f.payload_len);
    } else {
        fputs("null", stdout);
    }
    putchar('}');
}

static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = tolower(c);
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

int main(void)
{
    static char    line[MAX_LINE];
    static uint8_t buf[MAX_LINE / 2];
    unsigned long  lineno = 0;

    while (fgets(line, sizeof(line), stdin) != NULL) {
        lineno++;
        char *hash = strchr(line, '#');
        if (hash != NULL) {
            *hash = '\0';
        }
        char *p = line;
        while (isspace((unsigned char)*p)) p++;
        if (*p == '\0') {
            continue;
        }
        char mode = *p++;
        if ((mode != 'm' && mode != 'f') || !isspace((unsigned char)*p)) {
            fprintf(stderr, "line %lu: expected 'm <hex>' or 'f <hex>'\n", lineno);
            return 2;
        }
        size_t n = 0;
        for (;;) {
            while (isspace((unsigned char)*p)) p++;
            if (*p == '\0') break;
            int hi = hexval((unsigned char)p[0]);
            int lo = p[1] ? hexval((unsigned char)p[1]) : -1;
            if (hi < 0 || lo < 0) {
                fprintf(stderr, "line %lu: bad hex\n", lineno);
                return 2;
            }
            buf[n++] = (uint8_t)(hi << 4 | lo);
            p += 2;
        }
        /*
         * Copy into an exact-size heap block so ASan flags any read past the
         * end of the input, which a static buffer would hide.
         */
        uint8_t *exact = malloc(n ? n : 1);
        if (exact == NULL) {
            return 3;
        }
        memcpy(exact, buf, n);
        if (mode == 'm') {
            put_message(exact, n);
        } else {
            put_frame(exact, n);
        }
        putchar('\n');
        free(exact);
    }
    return 0;
}
