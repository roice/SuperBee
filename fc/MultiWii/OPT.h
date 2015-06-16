/* OptiTrack Motion Capture position sense header file */

#ifndef OPT_H_
#define OPT_H_

/* OptiTrack arena boundary */
#define OPT_X_MIN   -10*1000*10     // 10 meters
#define OPT_X_MAX   10*1000*10     // 10 meters
#define OPT_Y_MIN   -10*1000*10     // 10 meters
#define OPT_Y_MAX   10*1000*10     // 10 meters
#define OPT_Z_MIN   -10*1000*10     // 10 meters
#define OPT_Z_MAX   10*1000*10     // 10 meters

/* Added by Roice, 20150615 */
/* position struct, refreshed by SBSP messages */
/* processed in OPT_NewData function */
struct pos_t
{
    // local ENU, accuracy: 0.1 milimeters
    // example: if up coordinate is -330.6 mm
    //          up == -3306
    int32_t east;
    int32_t north;
    int32_t up;
};

extern int32_t pos_llh[3];

uint8_t OPT_NewData(void);

#endif /* OPT_H_ */


































