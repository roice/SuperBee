/* OptiTrack Motion Capture position sense header file */

#ifndef OPT_H_
#define OPT_H_

/* Added by Roice, 20150615 */
/* position struct, refreshed by SBSP messages */
/* processed in OPT_NewData function */
struct pos_t
{
    // local NEU, accuracy: 0.1 milimeters
    // example: if up coordinate is -330.6 mm
    //          up == -3306
    int32_t north;
    int32_t east;
    int32_t up;
};

uint8_t OPT_NewData(void);

#endif /* OPT_H_ */

































