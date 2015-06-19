#include "Arduino.h"
#include "OPT.h"    // for pos_t struct type and some defs

/* OptiTrack arena boundary */
#define OPT_X_MIN   -100000     // 10 meters
#define OPT_X_MAX   10*1000*10     // 10 meters
#define OPT_Y_MIN   -100000     // 10 meters
#define OPT_Y_MAX   10*1000*10     // 10 meters
#define OPT_Z_MIN   -100000     // 10 meters
#define OPT_Z_MAX   10*1000*10     // 10 meters

/* copied from Sensors.h */
#define ACC_1G 512  // MPU6050
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

/* copied from IMU */
#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define ACC_Z_DEADBAND (ACC_1G>>5) // was 40 instead of 32 now
#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }
#define MultiS16X16to32(longRes, intIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %A2 \n\t" \
"movw %A0, r0 \n\t" \
"muls %B1, %B2 \n\t" \
"movw %C0, r0 \n\t" \
"mulsu %B2, %A1 \n\t" \
"sbc %D0, r26 \n\t" \
"add %B0, r0 \n\t" \
"adc %C0, r1 \n\t" \
"adc %D0, r26 \n\t" \
"mulsu %B1, %A2 \n\t" \
"sbc %D0, r26 \n\t" \
"add %B0, r0 \n\t" \
"adc %C0, r1 \n\t" \
"adc %D0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (longRes) \
: \
"a" (intIn1), \
"a" (intIn2) \
: \
"r26" \
)

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
enum box {
  BOXARM,
  #if ACC
    BOXANGLE,
    BOXHORIZON,
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    BOXBARO,
  #endif
  #ifdef VARIOMETER
    BOXVARIO,
  #endif
  BOXMAG,
  #if defined(HEADFREE)
    BOXHEADFREE,
    BOXHEADADJ, // acquire heading for HEADFREE mode
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)  || defined(SERVO_MIX_TILT)
    BOXCAMSTAB,
  #endif
  #if defined(CAMTRIG)
    BOXCAMTRIG,
  #endif
  #if GPS
    BOXGPSHOME,
    BOXGPSHOLD,
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    BOXPASSTHRU,
  #endif
  #if defined(BUZZER)
    BOXBEEPERON,
  #endif
  #if defined(LED_FLASHER)
    BOXLEDMAX, // we want maximum illumination
    BOXLEDLOW, // low/no lights
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    BOXLLIGHTS, // enable landing lights at any altitude
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    BOXCALIB,
  #endif
  #ifdef GOVERNOR_P
    BOXGOV,
  #endif
  #ifdef OSD_SWITCH
    BOXOSD,
  #endif
  #if GPS
    BOXGPSNAV,
    BOXLAND,
  #endif
  CHECKBOXITEMS
};
enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL,     // not used currently
  PIDITEMS
};
struct pid_ {
  uint8_t P8;
  uint8_t I8;
  uint8_t D8;
};
struct servo_conf_ {  // this is a generic way to configure a servo, every multi type with a servo should use it
  int16_t min;        // minimum value, must be more than 1020 with the current implementation
  int16_t max;        // maximum value, must be less than 2000 with the current implementation
  int16_t middle;     // default should be 1500
  int8_t  rate;       // range [-100;+100] ; can be used to ajust a rate 0-100% and a direction
};
typedef struct {
  pid_    pid[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t angleTrim[2]; 
  #if defined(EXTENDED_AUX_STATES)
   uint32_t activate[CHECKBOXITEMS];  //Extended aux states define six different aux state for each aux channel
  #else
   uint16_t activate[CHECKBOXITEMS];
  #endif 
  uint8_t powerTrigger1;
  #if MAG
    int16_t mag_declination;
  #endif
  servo_conf_ servoConf[8];
  #if defined(GYRO_SMOOTHING)
    uint8_t Smoothing[3];
  #endif
  #if defined (FAILSAFE)
    int16_t failsafe_throttle;
  #endif
  #ifdef VBAT
    uint8_t vbatscale;
    uint8_t vbatlevel_warn1;
    uint8_t vbatlevel_warn2;
    uint8_t vbatlevel_crit;
  #endif
  #ifdef POWERMETER
    uint8_t pint2ma;
  #endif
  #ifdef POWERMETER_HARD
    uint16_t psensornull;
  #endif
  #ifdef MMGYRO
    uint8_t mmgyro;
  #endif
  #ifdef ARMEDTIMEWARNING
    uint16_t armedtimewarning;
  #endif
  int16_t minthrottle;
  #ifdef GOVERNOR_P
   int16_t governorP;
   int16_t governorD;
  #endif
  #ifdef YAW_COLL_PRECOMP
   uint8_t yawCollPrecomp;
   uint16_t yawCollPrecompDeadband;
  #endif
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} conf_t;

/* Global parameters */
struct opt_pos_enu_t pos_enu;
struct opt_flag_t opt_flag; // new data flag
alt_t    opt_alt;    // altitude provided by OPT

/* extern parameters */
extern int32_t  GPS_coord[2];
extern flags_struct_t f;
extern uint8_t  GPS_update;
extern uint8_t  GPS_numSat;
extern alt_t alt;
extern int32_t AltHold; // actually in mm
extern int16_t AltPID;
extern conf_t   conf;
extern int16_t  errorAltitudeI;
extern int16_t accZ;
extern uint8_t GPS_Frame;
extern int32_t  __attribute__ ((noinline)) mul(int16_t a, int16_t b);
extern uint32_t currentTime;
extern int8_t  OPTregainFlag;

/* local parameters */

static void enu2llh(const double *e, double *pos);

uint8_t OPT_GPS_NewData(void)
{
    if (opt_flag.opt == 0)
    {
        if (millis() - pos_enu.time >= 2000) // if not received pos for 2s
            f.GPS_FIX = 0;  // signal lost for 2 second
        return 0;
    }
    else 
    {
        opt_flag.opt = 0;   // clear opt new data flag

        /* check if this is the first frame that OPT signal regained */
        if (f.GPS_FIX == 0) // last time the OPT signal is in lost state
            OPTregainFlag = 1;  // then should notify the main loop to init Alt
    }

    // debug
    //f.GPS_FIX = 1;

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
    GPS_Frame = 1;
    //Blink GPS update
    if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;
    GPS_numSat = 8; // >5 indicates good GPS signal
    
    opt_flag.gps = 1;   // set gps new data flag

    /* Refresh Altitude */
    /* opt_alt.EstAlt will be filted in OPT_Alt_Filter and then
     * trans to global alt.EstAlt in OPT_Alt_Compute*/
    opt_alt.EstAlt = pos_enu.up;

    opt_flag.alt = 1;    // indicates a new EstAlt data is available

    return 1;
}

uint8_t OPT_Alt_Filter(void)
{
    if (opt_flag.alt == 0) return 0;

    /* Smooth Altitude data */
    // opt_alt.EstAlt = opt_alt.EstAlt;

    return 1;
}

uint8_t OPT_Alt_Compute(void)
{
    static float vel = 0.0f;
    static uint16_t previousT;
    uint16_t currentT = micros();
    uint16_t    dTime;

    dTime = currentT - previousT;
    if (dTime < UPDATE_INTERVAL) return 0;
    previousT = currentT;

    if (opt_flag.alt == 0) return 0;

    alt.EstAlt = opt_alt.EstAlt;

    /* compute PID */
    //P
    //int16_t error16 = constrain(AltHold - alt.EstAlt, -300, 300);//300mm
    int16_t error16 = constrain(AltHold - alt.EstAlt, -300, 300) * 10;
    applyDeadband(error16, 10); //remove small P parametr to reduce noise near zero position, deadband = 10mm
    AltPID = constrain((conf.pid[PIDALT].P8 * error16 >>7), -150, +150);

    //I
    errorAltitudeI += conf.pid[PIDALT].I8 * error16 >>6;
    errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
    AltPID += errorAltitudeI>>9; //I in range +/-60
 
    applyDeadband(accZ, ACC_Z_DEADBAND);

    //static int32_t lastAlt;
    // could only overflow with a difference of 32m, which is highly improbable here
    //int16_t AltVel = mul((alt.EstAlt - lastAlt) , (1000000 / UPDATE_INTERVAL));

    //lastAlt = alt.EstAlt;

    //AltVel = constrain(AltVel, -300, 300); // constrain baro velocity +/- 30cm/s
    //applyDeadband(AltVel, 10); // to reduce noise near zero

    // Integrator - velocity, cm/sec
    vel += accZ * ACC_VelScale * dTime;

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    //vel = vel * 0.985f + AltVel * 0.015f;

    //D
    alt.vario = vel;
    applyDeadband(alt.vario, 5);
    AltPID -= constrain(conf.pid[PIDALT].D8 * alt.vario >>4, -150, 150);

    opt_flag.alt = 0;   // clear alt new data flag

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
    // ...
}

