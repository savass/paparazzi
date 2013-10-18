/*
*
*
*huloooooogggggg!!!  modul lan boru mu?
*
*
*
*/

#ifndef ANT_TRACKER2_H
#define ANT_TRACKER2_H

#include "std.h"
#include "paparazzi.h"




/**************************************************************************************
--DECLARE DEFAULT VALUES OF VARIABLES WHICH SHOULD BE TAKEN FROM AIRFAME FILE------- */

#ifndef PAN_SERVO_ID
#define PAN_SERVO_ID PAN_SERVO
#endif

#ifndef PAN_SERVO_MAX
#define PAN_SERVO_MAX 2000
#endif

#ifndef PAN_SERVO_MIN
#define PAN_SERVO_MIN 1000
#endif

#ifndef PAN_SERVO_NOM
#define PAN_SERVO_NOM 1500
#endif

#ifndef TILT_SERVO_ID
#define TILT_SERVO_ID TILT_SERVO
#endif

#ifndef TILT_SERVO_MAX
#define TILT_SERVO_MAX 1800
#endif

#ifndef TILT_SERVO_MIN
#define TILT_SERVO_MIN 1200
#endif

#ifndef TILT_SERVO_NOM
#define TILT_SERVO_NOM 1500
#endif


/*---END OF DEFAULT VALUES DECLARATION!!--------------------------------------------------
*****************************************************************************************/

/*****************************************************************************************
--DECLARE VARIABLES AND FUNCTIONS TO BE USED IN C FILE ---------------------------------*/
extern void ant_tracker_init(void);
extern void ant_tracker_track_target(void);
extern void ant_tracker_get_new_aircraft_data(void);
/*---END OF VARIABLES AND FUNCTIONS DECLARATION!!-----------------------------------------
*****************************************************************************************/



#endif 

