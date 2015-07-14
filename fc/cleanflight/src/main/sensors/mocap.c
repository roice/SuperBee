/* This file is part of SuperBee.
 *
 * Motion Capture related.
 *
 * Author       Date        Changelog
 * Roice Luo    2015.07.02  Create
 */
#include <stdbool.h>

#include "platform.h"

#include "drivers/system.h"
#include "sensors/mocap.h"

#ifdef MOCAP

// Motion Capture Altitude Ready Flag
static bool mocapAltReady = false;
// Motion Capture Altitude data
static uint32_t mocapAlt = 0;    // in cm

/* Global parameters */
struct mocap_enu_t mocap_enu = {0,0,0,0,false};

/* Functions -- R/W/Compute... */
void updateMocap(int32_t e, int32_t n, int32_t u)
{
    mocap_enu.east = e;  // in 0.1mm
    mocap_enu.north = n; // in 0.1mm
    mocap_enu.up = u;    // in 0.1mm
    mocap_enu.time = millis(); // save current time
    mocap_enu.fresh = true;
}

bool mocapUpdatePos(void)
{
    if (mocap_enu.fresh != true)
    {// no new data
       clearMocapAltReadyFlag(); 
       return false;
    }

    mocap_enu.fresh = false;
    mocapAlt = mocap_enu.up / 100;  // cm
    setMocapAltReadyFlag();

    return true;
}

uint32_t mocapReadAltitude(void) {
    return mocapAlt;
}

/* Functions -- Flags */
bool isMocapAltReady(void) {
    return mocapAltReady;
}

void setMocapAltReadyFlag(void) {
    mocapAltReady = true;
}

void clearMocapAltReadyFlag(void) {
    mocapAltReady = false;
}

#endif
