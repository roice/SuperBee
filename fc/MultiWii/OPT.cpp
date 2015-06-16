#include "Arduino.h"
#include "OPT.h"    // for pos_t struct type and some defs

/* OptiTrack arena boundary */
#define OPT_X_MIN   -100000     // 10 meters
#define OPT_X_MAX   10*1000*10     // 10 meters
#define OPT_Y_MIN   -100000     // 10 meters
#define OPT_Y_MAX   10*1000*10     // 10 meters
#define OPT_Z_MIN   -100000     // 10 meters
#define OPT_Z_MAX   10*1000*10     // 10 meters

/* these struct is strange, or Arduino is strange!
 * I included "MultiWii.h" but the compiler still
 * can't find the prototype of this struct
 * as if the header files are not included at all
 * So I pasted them here, just to get them working*/
#define GPS 1
#define MAG 1
typedef struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t ANGLE_MODE :1 ;
  uint8_t HORIZON_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
#ifdef HEADFREE
  uint8_t HEADFREE_MODE :1 ;
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  uint8_t PASSTHRU_MODE :1 ;
#endif
  uint8_t SMALL_ANGLES_25 :1 ;
#if MAG
  uint8_t CALIBRATE_MAG :1 ;
#endif
#ifdef VARIOMETER
  uint8_t VARIO_MODE :1;
#endif
  uint8_t GPS_mode: 2;               // 0-3 NONE,HOLD, HOME, NAV (see GPS_MODE_* defines
#if BARO || GPS
  uint8_t THROTTLE_IGNORED : 1;      // If it is 1 then ignore throttle stick movements in baro mode;
#endif
#if GPS
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t GPS_BARO_MODE : 1;         // This flag is used when GPS controls baro mode instead of user (it will replace rcOptions[BARO]
  uint8_t GPS_head_set: 1;           // it is 1 if the navigation engine got commands to control heading (SET_POI or SET_HEAD) CLEAR_HEAD will zero it
  uint8_t LAND_COMPLETED: 1;
  uint8_t LAND_IN_PROGRESS: 1;
#endif
} flags_struct_t;

typedef struct {
  int32_t  EstAlt;
  int16_t  vario;
} alt_t;

/* Global parameters */
struct pos_t pos_enu;
struct pos_flag_t
{
    // gps and alt new data flag
    uint8_t gps;
    uint8_t alt;
};
struct opt_flag_t opt_flag; // new data flag

/* extern parameters */
extern int32_t  GPS_coord[2];
extern flags_struct_t f;
extern uint8_t  GPS_update;
extern uint8_t  GPS_numSat;
extern alt_t alt;

static void enu2llh(const double *e, double *pos);

uint8_t OPT_NewData(void)
{
    double position_e[3], converted_pos[3];
    // check if position data is valid

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

    /* Refresh GPS state */
    //save to global parameter for GPS.cpp use
    GPS_coord[0] = int32_t(converted_pos[0] * 100000000); // 0.00000001 degree
    GPS_coord[1] = int32_t(converted_pos[1] * 100000000);
    f.GPS_FIX = 1;  // have a good GPS 3D FIX
    //Mark that a new GPS frame is available for GPS_Compute()
    //GPS_Frame = 1;
    //Blink GPS update
    if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;
    GPS_numSat = 8; // >5 indicates good GPS signal

    /* Refresh Altitude */
    alt.EstAlt = int32_t(converted_pos[2] * 1000);     // 1 mm
    opt_flag.alt = 1;    // indicates a new EstAlt data is available

    return 1;
}

uint8_t OPT_Alt_Filter(void)
{
//    opt_flag[1] = 0;    //
    return 1;
}

uint8_t OPT_Alt_Compute(void)
{
 //   opt_flag[1] = 0;    //
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

