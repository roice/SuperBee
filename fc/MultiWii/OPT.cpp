#include "Arduino.h"
#include "OPT.h"

/* OptiTrack arena boundary */
#define OPT_X_MIN   -10*1000*10     // 10 meters
#define OPT_X_MAX   10*1000*10     // 10 meters
#define OPT_Y_MIN   -10*1000*10     // 10 meters
#define OPT_Y_MAX   10*1000*10     // 10 meters
#define OPT_Z_MIN   -10*1000*10     // 10 meters
#define OPT_Z_MAX   10*1000*10     // 10 meters

/* Global parameters */
struct pos_t pos_enu;

uint8_t OPT_NewData(void)
{
    // check if position data is valid
    if (    (pos_enu.north < OPT_X_MIN) || (pos_enu.north > OPT_X_MAX) || \
            (pos_enu.east < OPT_Y_MIN) || (pos_enu.east > OPT_Y_MAX) || \
            (pos_enu.up < OPT_Z_MIN) || (pos_enu.up > OPT_Z_MAX))
        return 0;

    /* convert to and fill GPS and Sonar data */
    pos_enu.north =0;
    pos_enu.east=0;
    pos_enu.up=0;
}











