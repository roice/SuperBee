#include "Arduino.h"
#include "OPT.h"    // for pos_t struct type and some defs

/* Global parameters */
struct pos_t pos_enu;
int32_t pos_llh[3];

static void enu2llh(const double *e, double *pos);

uint8_t OPT_NewData(void)
{
    double position_e[3], converted_pos[3];
    // check if position data is valid
    if (    (pos_enu.north < OPT_X_MIN) || (pos_enu.north > OPT_X_MAX) || \
            (pos_enu.east < OPT_Y_MIN) || (pos_enu.east > OPT_Y_MAX) || \
            (pos_enu.up < OPT_Z_MIN) || (pos_enu.up > OPT_Z_MAX))
        return 0;

    /* convert to and fill GPS and Sonar data */

    /* Local ENU to LLH */
    // Note: as the type of GPS_coord is int32_t (supreme 2.1*10^9), it's
    // not enough for the accuracy of 10^(-8) degree (approx. 1 mm), so
    // the Local LLH coord should limited below 20, in this case, I choose
    // 0.00000000N 0.00000000E 0.000H, which actually lies on the
    // Guinea Bay (oil rich), Africa.
    // So imagine we are flying above the Guinea Bay~
    // Note: As coordinates conversion is a tough task (matrix, LAPACK,...),
    // so here simply uses linearized function
    /* save opt pos to temp array */
    position_e[0] = double(pos_enu.east) / 10000;   // see type def of pos_t
    position_e[1] = double(pos_enu.north) / 10000;
    position_e[2] = double(pos_enu.up) / 10000;
    /* convert local ENU to LLH
     * This function is linearized and the position to be converted is limited
     * to no more than 100 m to the original pos llh 0.0 N 0.0E */
    enu2llh(position_e, converted_pos);
    pos_llh[0] = int32_t(converted_pos[0] * 100000000); // 0.00000001 degree
    pos_llh[1] = int32_t(converted_pos[1] * 100000000);
    pos_llh[2] = int32_t(converted_pos[2] * 10000);     // 0.0001 m

    return 1;
}

/* convert Local ENU to LLH, use with caution! */
static void enu2llh(const double *e, double *pos)
{// original position is 0.00000000N 0.00000000E 0.000m

    /* convert LAT */
    pos[0] = e[0]/111.3194*0.001;
    /* convert LON */
    pos[1] = e[1]/110.5741*0.001;
    /* convert HEI */
    pos[2] = e[2];
}

