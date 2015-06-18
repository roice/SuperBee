/* OptiTrack Motion Capture position sense header file */

#ifndef OPT_H_
#define OPT_H_

/* Added by Roice, 20150615 */
/* position struct, refreshed by SBSP messages */
/* processed in OPT_NewData function */
struct opt_pos_enu_t
{
    // local ENU, accuracy: 0.1 milimeters
    // example: if up coordinate is -330.6 mm
    //          up == -3306
    int32_t east;
    int32_t north;
    int32_t up;
    // data time
    // this value will be checked in OPT_GPS_New_Data
    // if it exceeds current time a predefined period, the GPS signal
    // must be lost, the reason is probably the plane moves out of
    // OptiTrack range...
    uint32_t time;  // ms
};

struct opt_flag_t
{
    // new data flag
    uint8_t opt;
    uint8_t gps;
    uint8_t alt;
};

uint8_t OPT_GPS_NewData(void);
uint8_t OPT_Alt_Filter(void);
uint8_t OPT_Alt_Compute(void);

#endif /* OPT_H_ */


































