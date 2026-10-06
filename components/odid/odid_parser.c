/*
 * odid_parser.c - see odid_parser.h.
 */
#include "odid_parser.h"

#include <math.h>
#include <string.h>

/* -- little-endian readers ------------------------------------------------- */

static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* uint16 * 0.5 - 1000 m; 0 encodes -1000 m, the ODID "unknown" sentinel. */
static double altitude(const uint8_t *p) { return rd_u16(p) * 0.5 - 1000.0; }

/* data[a:a+n].rstrip(b'\x00') */
static uint8_t copy_rstrip_nul(uint8_t *dst, const uint8_t *src, uint8_t n)
{
    while (n > 0 && src[n - 1] == 0) {
        n--;
    }
    memcpy(dst, src, n);
    return n;
}

/* -- per-type parsers (each mirrors a Python parse_* helper) --------------- */

static bool parse_basic_id(const uint8_t *d, size_t len, odid_basic_id_t *o)
{
    if (len < 25) {
        return false;
    }
    o->id_type    = (d[1] >> 4) & 0x0F;
    o->ua_type    = d[1] & 0x0F;
    o->uas_id_len = copy_rstrip_nul(o->uas_id, d + 2, 20);
    return true;
}

static bool parse_location(const uint8_t *d, size_t len, odid_location_t *o)
{
    if (len < 25) {
        return false;
    }
    /* Byte 1: bit 0 speed multiplier, bit 1 E/W half, bit 2 height type,
     * bits 4-7 operational status. */
    uint8_t speed_mult  = d[1] & 0x01;
    uint8_t dir_segment = (d[1] >> 1) & 0x01;
    uint8_t height_type = (d[1] >> 2) & 0x01;

    double lat = rd_i32(d + 5) * 1e-7;
    double lon = rd_i32(d + 9) * 1e-7;
    /* Pre-lock placeholder positions (e.g. DJI's lat > 90) are dropped. */
    if (fabs(lat) > 90.0 || fabs(lon) > 180.0) {
        return false;
    }

    o->height_type        = height_type;
    o->heading_deg        = (double)d[2] + (dir_segment ? 180.0 : 0.0);
    /* Above 63.75 m/s the encoding is 0.75 m/s steps offset by 63.75. */
    o->ground_speed_mps   = speed_mult ? d[3] * 0.75 + 63.75 : d[3] * 0.25;
    o->vertical_speed_mps = (int8_t)d[4] * 0.5;
    o->lat                = lat;
    o->lon                = lon;

    o->alt_geo_m   = altitude(d + 15);
    o->has_alt_geo = o->alt_geo_m > -1000.0;
    o->height_m    = altitude(d + 17);
    o->has_height  = o->height_m > -1000.0;
    return true;
}

static bool parse_self_id(const uint8_t *d, size_t len, odid_self_id_t *o)
{
    if (len < 25) {
        return false;
    }
    o->description_type = d[1];
    o->description_len  = copy_rstrip_nul(o->description, d + 2, 23);
    return true;
}

static bool parse_system(const uint8_t *d, size_t len, odid_system_t *o)
{
    if (len < 20) {
        return false;
    }
    double op_lat = rd_i32(d + 2) * 1e-7;
    double op_lon = rd_i32(d + 6) * 1e-7;

    o->area_count    = rd_u16(d + 10);
    o->area_radius_m = (uint16_t)(d[12] * 10);

    o->has_operator = fabs(op_lat) <= 90.0 && fabs(op_lon) <= 180.0;
    o->operator_lat = op_lat;
    o->operator_lon = op_lon;
    o->operator_location_type = d[1] & 0x03;

    o->alt_takeoff_m   = altitude(d + 18);
    o->has_alt_takeoff = o->alt_takeoff_m > -1000.0;
    return true;
}

static bool parse_operator_id(const uint8_t *d, size_t len, odid_operator_id_t *o)
{
    if (len < 22) {
        return false;
    }
    o->operator_id_type = d[1];
    o->operator_id_len  = copy_rstrip_nul(o->operator_id, d + 2, 20);
    return true;
}

/* -- public API ------------------------------------------------------------ */

bool odid_decode(const uint8_t *data, size_t len, odid_msg_t *out)
{
    if (data == NULL || out == NULL || len < 2) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->type = (data[0] >> 4) & 0x0F;

    switch (out->type) {
    case ODID_MSG_BASIC_ID:
        out->has_fields = parse_basic_id(data, len, &out->u.basic_id);
        break;
    case ODID_MSG_LOCATION:
        out->has_fields = parse_location(data, len, &out->u.location);
        break;
    case ODID_MSG_SELF_ID:
        out->has_fields = parse_self_id(data, len, &out->u.self_id);
        break;
    case ODID_MSG_SYSTEM:
        out->has_fields = parse_system(data, len, &out->u.system);
        break;
    case ODID_MSG_OPERATOR_ID:
        out->has_fields = parse_operator_id(data, len, &out->u.operator_id);
        break;
    default:
        /* Authentication, Message Pack and unassigned types: type only. */
        break;
    }
    return true;
}

bool odid_pack_iter_init(odid_pack_iter_t *it, const uint8_t *pack, size_t len)
{
    memset(it, 0, sizeof(*it));
    if (pack == NULL || len < 3) {
        return false;
    }
    it->data      = pack;
    it->len       = len;
    it->msg_size  = pack[1];
    it->msg_count = pack[2];
    return true;
}

bool odid_pack_next(odid_pack_iter_t *it, const uint8_t **msg, size_t *msg_len)
{
    if (it->data == NULL || it->index >= it->msg_count) {
        return false;
    }
    size_t offset = 3 + (size_t)it->index * it->msg_size;
    if (offset + it->msg_size > it->len) {
        it->index = it->msg_count;      /* Python breaks out of the loop here */
        return false;
    }
    it->index++;
    *msg     = it->data + offset;
    *msg_len = it->msg_size;
    return true;
}

/* -- display names (wifi_feeder.py MSG_TYPE / ID_TYPE / UA_TYPE / ...) ----- */

const char *odid_msg_type_name(uint8_t type)
{
    switch (type) {
    case 0x0: return "Basic ID";
    case 0x1: return "Location/Vector";
    case 0x2: return "Authentication";
    case 0x3: return "Self ID";
    case 0x4: return "System";
    case 0x5: return "Operator ID";
    case 0xF: return "Message Pack";
    default:  return NULL;
    }
}

const char *odid_id_type_name(uint8_t id_type)
{
    static const char *const names[] = {
        "None",
        "Serial Number (ANSI/CTA-2063-A)",
        "CAA Assigned",
        "UTM Assigned",
        "Specific Session ID",
    };
    return id_type < sizeof(names) / sizeof(names[0]) ? names[id_type] : NULL;
}

const char *odid_ua_type_name(uint8_t ua_type)
{
    static const char *const names[] = {
        "None",
        "Aeroplane",
        "Helicopter/Multirotor",
        "Gyroplane",
        "Hybrid Lift",
        "Ornithopter",
        "Glider",
        "Kite",
        "Free Balloon",
        "Captive Balloon",
        "Airship",
        "Free Fall/Parachute",
        "Rocket",
        "Tethered Powered Aircraft",
        "Ground Obstacle",
    };
    if (ua_type == 255) {
        return "Other";
    }
    return ua_type < sizeof(names) / sizeof(names[0]) ? names[ua_type] : NULL;
}

const char *odid_operator_location_type_name(uint8_t loc_type)
{
    switch (loc_type) {
    case 0:  return "takeoff";
    case 1:  return "live_gnss";
    case 2:  return "fixed";
    default: return "reserved";
    }
}
