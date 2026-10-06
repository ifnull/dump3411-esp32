#include "sim.h"

#include <math.h>
#include <string.h>

#include "sdkconfig.h"

#define RADIUS_M        150.0
#define GROUND_ALT_M    250.0
#define CRUISE_AGL_M    60.0
#define M_PER_DEG_LAT   111320.0
#define DEG             (M_PI / 180.0)

static double t_s;       /* time since start */
static double theta;     /* bearing of the drone from the center, radians */

static double center_lat(void) { return CONFIG_RID_TX_CENTER_LAT_E7 * 1e-7; }
static double center_lon(void) { return CONFIG_RID_TX_CENTER_LON_E7 * 1e-7; }

static double speed_at(double t) { return fmod(t, 60.0) < 40.0 ? 12.0 : 70.0; }

void sim_init(ODID_UAS_Data *uas)
{
    odid_initUasData(uas);
    t_s = 0.0;
    theta = 0.0;

    uas->BasicID[0].IDType = ODID_IDTYPE_SERIAL_NUMBER;
    uas->BasicID[0].UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
    strcpy(uas->BasicID[0].UASID, "DUMP3411-BENCH-TX-01");
    uas->BasicIDValid[0] = 1;

    uas->SelfID.DescType = ODID_DESC_TYPE_TEXT;
    strcpy(uas->SelfID.Desc, "BENCH TEST TRANSMITTER");
    uas->SelfIDValid = 1;

    uas->System.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS;
    uas->System.ClassificationType   = ODID_CLASSIFICATION_TYPE_UNDECLARED;
    uas->System.OperatorLatitude     = center_lat();
    uas->System.OperatorLongitude    = center_lon();
    uas->System.AreaCount            = 1;
    uas->System.AreaRadius           = 0;
    uas->System.AreaCeiling          = -1000.0f;
    uas->System.AreaFloor            = -1000.0f;
    uas->System.OperatorAltitudeGeo  = (float)GROUND_ALT_M;
    uas->SystemValid = 1;

    uas->OperatorID.OperatorIdType = ODID_OPERATOR_ID;
    strcpy(uas->OperatorID.OperatorId, "TEST-OPERATOR-0001");
    uas->OperatorIDValid = 1;

    uas->LocationValid = 1;
    sim_step(uas, 0.0);
}

void sim_step(ODID_UAS_Data *uas, double dt)
{
    double v = speed_at(t_s);
    theta = fmod(theta + v * dt / RADIUS_M, 2 * M_PI);
    t_s += dt;

    double north = RADIUS_M * cos(theta);
    double east  = RADIUS_M * sin(theta);
    double agl   = CRUISE_AGL_M + 20.0 * sin(t_s / 20.0);

    ODID_Location_data *l = &uas->Location;
    l->Status          = ODID_STATUS_AIRBORNE;
    /* Flying clockwise as seen from above, so the track is 90 deg past the bearing. */
    l->Direction       = (float)fmod(theta / DEG + 90.0, 360.0);
    l->SpeedHorizontal = (float)v;
    l->SpeedVertical   = (float)cos(t_s / 20.0);            /* d(agl)/dt */
    l->Latitude        = center_lat() + north / M_PER_DEG_LAT;
    l->Longitude       = center_lon() + east / (M_PER_DEG_LAT * cos(center_lat() * DEG));
    l->AltitudeBaro    = (float)(GROUND_ALT_M + agl);
    l->AltitudeGeo     = (float)(GROUND_ALT_M + agl);
    l->HeightType      = ODID_HEIGHT_REF_OVER_GROUND;
    l->Height          = (float)agl;
    l->HorizAccuracy   = ODID_HOR_ACC_3_METER;
    l->VertAccuracy    = ODID_VER_ACC_3_METER;
    l->BaroAccuracy    = ODID_VER_ACC_3_METER;
    l->SpeedAccuracy   = ODID_SPEED_ACC_1_METERS_PER_SECOND;
    l->TSAccuracy      = ODID_TIME_ACC_0_1_SECOND;
    l->TimeStamp       = (float)fmod(t_s, 3600.0);
}
