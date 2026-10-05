/*
 * gen_fixtures - write parity fixtures with the OpenDroneID reference encoder.
 *
 * The decoder under test and the oracle (dump3411) were both written from
 * reading the spec. Fixtures built by a third, independent encoder, the one
 * ArduRemoteID transmits with, keep a shared misreading of the spec from
 * passing as parity. Hand-built edge cases live in make_malformed.py instead.
 *
 * Output (stdout) is the fixture format odid_dump reads: 'm <hex>' for an ODID
 * message or pack, 'f <hex>' for an 802.11 frame. Regenerate with:
 *
 *   cmake -S host -B build-host -DODID_FIXTURE_GEN=ON
 *   cmake --build build-host --target gen_fixtures
 *   build-host/gen_fixtures tests/parity/fixtures/encoder_expected.jsonl \
 *       > tests/parity/fixtures/encoder.txt
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "opendroneid.h"

typedef struct {
    const char *name;
    uint8_t     mac[6];
    ODID_UAS_Data uas;
} scenario_t;

static void put_line(char mode, const void *data, size_t len, const char *comment)
{
    const uint8_t *b = data;
    printf("%c ", mode);
    for (size_t i = 0; i < len; i++) {
        printf("%02X", b[i]);
    }
    printf("  # %s\n", comment);
}

/*
 * Ground truth for each single message: the values the encoder was given,
 * in dump3411's field names and units. null means the field must be absent
 * (the -1000 m "unknown" altitude). Checked against both decoders, so a bug
 * the C port faithfully copies from dump3411 still fails.
 */
static FILE *expected_out;

static void put_expected(const void *encoded, const char *fields_json)
{
    const uint8_t *b = encoded;
    fputs("{\"hex\":\"", expected_out);
    for (size_t i = 0; i < ODID_MESSAGE_SIZE; i++) {
        fprintf(expected_out, "%02X", b[i]);
    }
    fprintf(expected_out, "\",\"expect\":{%s}}\n", fields_json);
}

static void put_alt(char *dst, size_t n, const char *key, float m)
{
    if (m <= -1000.0f) {
        snprintf(dst, n, ",\"%s\":null", key);
    } else {
        snprintf(dst, n, ",\"%s\":%.1f", key, (double)m);
    }
}

static void emit_expected(const scenario_t *s, const ODID_BasicID_encoded *bid,
                          const ODID_Location_encoded *loc, const ODID_SelfID_encoded *sid,
                          const ODID_System_encoded *sys, const ODID_OperatorID_encoded *opid)
{
    char j[512], a[64], h[64];
    const ODID_UAS_Data *u = &s->uas;

    snprintf(j, sizeof(j), "\"id_type_raw\":%d,\"ua_type_raw\":%d,\"uas_id\":\"%s\"",
             u->BasicID[0].IDType, u->BasicID[0].UAType, u->BasicID[0].UASID);
    put_expected(bid, j);

    const ODID_Location_data *l = &u->Location;
    put_alt(a, sizeof(a), "altitude_geo", l->AltitudeGeo);
    put_alt(h, sizeof(h), "height_agl", l->Height);
    snprintf(j, sizeof(j),
             "\"latitude\":%.7f,\"longitude\":%.7f,\"ground_speed\":%.2f,"
             "\"vertical_speed\":%.1f,\"heading\":%.1f,\"height_type\":\"%s\"%s%s",
             l->Latitude, l->Longitude, (double)l->SpeedHorizontal, (double)l->SpeedVertical,
             (double)l->Direction, l->HeightType == ODID_HEIGHT_REF_OVER_GROUND ? "AGL" : "Above Takeoff",
             a, h);
    put_expected(loc, j);

    snprintf(j, sizeof(j), "\"description_type\":%d,\"description\":\"%s\"",
             u->SelfID.DescType, u->SelfID.Desc);
    put_expected(sid, j);

    static const char *const loc_names[] = {"takeoff", "live_gnss", "fixed"};
    const ODID_System_data *y = &u->System;
    put_alt(a, sizeof(a), "alt_takeoff_geo", y->OperatorAltitudeGeo);
    snprintf(j, sizeof(j),
             "\"area_count\":%u,\"area_radius_m\":%u,\"operator_lat\":%.7f,"
             "\"operator_lon\":%.7f,\"operator_location_type\":\"%s\"%s",
             y->AreaCount, y->AreaRadius, y->OperatorLatitude, y->OperatorLongitude,
             loc_names[y->OperatorLocationType], a);
    put_expected(sys, j);

    snprintf(j, sizeof(j), "\"operator_id_type\":%d,\"operator_id\":\"%s\"",
             u->OperatorID.OperatorIdType, u->OperatorID.OperatorId);
    put_expected(opid, j);
}

static void emit(const scenario_t *s)
{
    char note[128];
    uint8_t buf[1024];

    printf("\n# --- %s ---\n", s->name);

    ODID_BasicID_encoded bid;
    if (encodeBasicIDMessage(&bid, &s->uas.BasicID[0]) != ODID_SUCCESS) {
        fprintf(stderr, "%s: encodeBasicIDMessage failed\n", s->name);
        exit(1);
    }
    snprintf(note, sizeof(note), "%s: Basic ID", s->name);
    put_line('m', &bid, sizeof(bid), note);
    ODID_Location_encoded loc;
    if (encodeLocationMessage(&loc, &s->uas.Location) != ODID_SUCCESS) {
        fprintf(stderr, "%s: encodeLocationMessage failed\n", s->name);
        exit(1);
    }
    snprintf(note, sizeof(note), "%s: Location/Vector", s->name);
    put_line('m', &loc, sizeof(loc), note);
    ODID_SelfID_encoded sid;
    if (encodeSelfIDMessage(&sid, &s->uas.SelfID) != ODID_SUCCESS) {
        fprintf(stderr, "%s: encodeSelfIDMessage failed\n", s->name);
        exit(1);
    }
    snprintf(note, sizeof(note), "%s: Self ID", s->name);
    put_line('m', &sid, sizeof(sid), note);
    ODID_System_encoded sys;
    if (encodeSystemMessage(&sys, &s->uas.System) != ODID_SUCCESS) {
        fprintf(stderr, "%s: encodeSystemMessage failed\n", s->name);
        exit(1);
    }
    snprintf(note, sizeof(note), "%s: System", s->name);
    put_line('m', &sys, sizeof(sys), note);
    ODID_OperatorID_encoded opid;
    if (encodeOperatorIDMessage(&opid, &s->uas.OperatorID) != ODID_SUCCESS) {
        fprintf(stderr, "%s: encodeOperatorIDMessage failed\n", s->name);
        exit(1);
    }
    snprintf(note, sizeof(note), "%s: Operator ID", s->name);
    put_line('m', &opid, sizeof(opid), note);

    emit_expected(s, &bid, &loc, &sid, &sys, &opid);

    int n = odid_message_build_pack(&s->uas, buf, sizeof(buf));
    if (n > 0) {
        snprintf(note, sizeof(note), "%s: Message Pack", s->name);
        put_line('m', buf, (size_t)n, note);
    }
    n = odid_wifi_build_message_pack_beacon_frame(&s->uas, (const char *)s->mac,
                                                  "RID-TEST", 8, 100, 7, buf, sizeof(buf));
    if (n > 0) {
        snprintf(note, sizeof(note), "%s: Wi-Fi Beacon frame", s->name);
        put_line('f', buf, (size_t)n, note);
    }
    n = odid_wifi_build_message_pack_nan_action_frame(&s->uas, (const char *)s->mac,
                                                      9, buf, sizeof(buf));
    if (n > 0) {
        snprintf(note, sizeof(note), "%s: Wi-Fi NAN action frame", s->name);
        put_line('f', buf, (size_t)n, note);
    }
}

static void base(scenario_t *s)
{
    odid_initUasData(&s->uas);
    s->uas.BasicIDValid[0] = 1;
    s->uas.LocationValid   = 1;
    s->uas.SelfIDValid     = 1;
    s->uas.SystemValid     = 1;
    s->uas.OperatorIDValid = 1;
}

int main(int argc, char **argv)
{
    scenario_t s;

    if (argc != 2 || (expected_out = fopen(argv[1], "w")) == NULL) {
        fprintf(stderr, "usage: gen_fixtures EXPECTED.jsonl > fixtures.txt\n");
        return 2;
    }

    printf("# Generated by host/gen_fixtures.c with opendroneid-core-c %s.\n",
           OPENDRONEID_CORE_C_COMMIT);
    printf("# Do not edit by hand; regenerate (see gen_fixtures.c).\n");

    /* A consumer multirotor over a public park, everything populated. */
    memset(&s, 0, sizeof(s));
    s.name = "multirotor-serial";
    memcpy(s.mac, (uint8_t[]){0x60, 0x60, 0x1f, 0x12, 0x34, 0x56}, 6);
    base(&s);
    s.uas.BasicID[0].IDType = ODID_IDTYPE_SERIAL_NUMBER;
    s.uas.BasicID[0].UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
    strcpy(s.uas.BasicID[0].UASID, "1581F5FKD229400B03N7");
    s.uas.Location.Status          = ODID_STATUS_AIRBORNE;
    s.uas.Location.Direction       = 271.0f;
    s.uas.Location.SpeedHorizontal = 12.5f;
    s.uas.Location.SpeedVertical   = -1.5f;
    s.uas.Location.Latitude        = 40.7828647;
    s.uas.Location.Longitude       = -73.9653551;
    s.uas.Location.AltitudeBaro    = 95.0f;
    s.uas.Location.AltitudeGeo     = 112.5f;
    s.uas.Location.HeightType      = ODID_HEIGHT_REF_OVER_GROUND;
    s.uas.Location.Height          = 45.0f;
    s.uas.SelfID.DescType = ODID_DESC_TYPE_TEXT;
    strcpy(s.uas.SelfID.Desc, "Photography");
    s.uas.System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS;
    s.uas.System.OperatorLatitude     = 40.7812199;
    s.uas.System.OperatorLongitude    = -73.9665138;
    s.uas.System.AreaCount            = 1;
    s.uas.System.AreaRadius           = 0;
    s.uas.System.OperatorAltitudeGeo  = 68.5f;
    s.uas.OperatorID.OperatorIdType = ODID_OPERATOR_ID;
    strcpy(s.uas.OperatorID.OperatorId, "FIN87astrdge12k8");
    emit(&s);

    /* Fast fixed-wing: speed above 63.75 m/s needs the 0.75 multiplier. */
    memset(&s, 0, sizeof(s));
    s.name = "aeroplane-fast-southern";
    memcpy(s.mac, (uint8_t[]){0x02, 0x00, 0x5e, 0x10, 0x20, 0x30}, 6);
    base(&s);
    s.uas.BasicID[0].IDType = ODID_IDTYPE_CAA_REGISTRATION_ID;
    s.uas.BasicID[0].UAType = ODID_UATYPE_AEROPLANE;
    strcpy(s.uas.BasicID[0].UASID, "CASA-UAS-00421");
    s.uas.Location.Status          = ODID_STATUS_AIRBORNE;
    s.uas.Location.Direction       = 45.0f;
    s.uas.Location.SpeedHorizontal = 80.25f;
    s.uas.Location.SpeedVertical   = 10.0f;
    s.uas.Location.Latitude        = -33.8567844;
    s.uas.Location.Longitude       = 151.2152967;
    s.uas.Location.AltitudeGeo     = 610.0f;
    s.uas.Location.HeightType      = ODID_HEIGHT_REF_OVER_TAKEOFF;
    s.uas.Location.Height          = 580.5f;
    s.uas.SelfID.DescType = ODID_DESC_TYPE_TEXT;
    strcpy(s.uas.SelfID.Desc, "Pipeline Survey");
    s.uas.System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    s.uas.System.OperatorLatitude     = -33.8600000;
    s.uas.System.OperatorLongitude    = 151.2100000;
    s.uas.System.AreaCount            = 1;
    s.uas.System.OperatorAltitudeGeo  = 29.5f;
    s.uas.OperatorID.OperatorIdType = ODID_OPERATOR_ID;
    strcpy(s.uas.OperatorID.OperatorId, "AUS-OP-7781");
    emit(&s);

    /* Unknown/invalid values the encoder writes as spec sentinels. */
    memset(&s, 0, sizeof(s));
    s.name = "unknown-altitudes";
    memcpy(s.mac, (uint8_t[]){0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f}, 6);
    base(&s);
    s.uas.BasicID[0].IDType = ODID_IDTYPE_SPECIFIC_SESSION_ID;
    s.uas.BasicID[0].UAType = ODID_UATYPE_NONE;
    strcpy(s.uas.BasicID[0].UASID, "SESSION-ABC");
    s.uas.Location.Status          = ODID_STATUS_GROUND;
    s.uas.Location.Direction       = 0.0f;
    s.uas.Location.SpeedHorizontal = 0.0f;
    s.uas.Location.SpeedVertical   = 0.0f;
    s.uas.Location.Latitude        = 0.0;
    s.uas.Location.Longitude       = 0.0;
    s.uas.Location.AltitudeGeo     = -1000.0f;
    s.uas.Location.Height          = -1000.0f;
    s.uas.SelfID.DescType = ODID_DESC_TYPE_TEXT;
    s.uas.System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_FIXED;
    s.uas.System.AreaCount            = 1;
    s.uas.System.OperatorAltitudeGeo  = -1000.0f;
    s.uas.OperatorID.OperatorIdType = ODID_OPERATOR_ID;
    emit(&s);

    /* Range edges: heading 359, max vertical speed, near-pole position. */
    memset(&s, 0, sizeof(s));
    s.name = "range-edges";
    memcpy(s.mac, (uint8_t[]){0xde, 0xad, 0xbe, 0xef, 0x00, 0x01}, 6);
    base(&s);
    s.uas.BasicID[0].IDType = ODID_IDTYPE_UTM_ASSIGNED_UUID;
    s.uas.BasicID[0].UAType = ODID_UATYPE_OTHER;
    strcpy(s.uas.BasicID[0].UASID, "0123456789ABCDEFGHIJ");
    s.uas.Location.Status          = ODID_STATUS_EMERGENCY;
    s.uas.Location.Direction       = 359.0f;
    s.uas.Location.SpeedHorizontal = 254.25f;
    s.uas.Location.SpeedVertical   = -62.0f;
    s.uas.Location.Latitude        = 89.9999999;
    s.uas.Location.Longitude       = -179.9999999;
    s.uas.Location.AltitudeGeo     = 31767.5f;
    s.uas.Location.HeightType      = ODID_HEIGHT_REF_OVER_GROUND;
    s.uas.Location.Height          = -999.5f;
    s.uas.SelfID.DescType = ODID_DESC_TYPE_EMERGENCY;
    strcpy(s.uas.SelfID.Desc, "Lost link, RTH active");
    s.uas.System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS;
    s.uas.System.OperatorLatitude     = -89.9999999;
    s.uas.System.OperatorLongitude    = 179.9999999;
    s.uas.System.AreaCount            = 255;
    s.uas.System.AreaRadius           = 2550;
    s.uas.System.OperatorAltitudeGeo  = 31767.5f;
    s.uas.OperatorID.OperatorIdType = ODID_OPERATOR_ID;
    strcpy(s.uas.OperatorID.OperatorId, "OP-ID-TWENTY-CHARS-X");
    emit(&s);

    /* Authentication (type 0x2) has no dump3411 decoder: type-only output. */
    ODID_Auth_data auth;
    odid_initAuthData(&auth);
    auth.AuthType      = ODID_AUTH_UAS_ID_SIGNATURE;
    auth.DataPage      = 0;
    auth.LastPageIndex = 0;
    auth.Length        = 17;
    auth.Timestamp     = 28000000;
    memcpy(auth.AuthData, "0123456789abcdefg", 17);
    ODID_Auth_encoded auth_enc;
    if (encodeAuthMessage(&auth_enc, &auth) == ODID_SUCCESS) {
        printf("\n# --- authentication ---\n");
        put_line('m', &auth_enc, sizeof(auth_enc), "Authentication page 0");
    }
    fclose(expected_out);
    return 0;
}
