#ifndef SIM_H
#define SIM_H

#include "opendroneid.h"

/*
 * Simulated flight: 150 m circles around the configured center, 60 m AGL
 * with a slow +/-20 m climb and descent. Each minute is 40 s at 12 m/s then
 * 20 s at 70 m/s, so the 0.75 m/s high-speed encoding gets exercised.
 */
void sim_init(ODID_UAS_Data *uas);

/* Advance the flight by dt seconds and refresh uas->Location. */
void sim_step(ODID_UAS_Data *uas, double dt);

#endif
