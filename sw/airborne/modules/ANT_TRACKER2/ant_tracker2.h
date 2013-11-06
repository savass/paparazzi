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



// For Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

/**************************************************************************************
--DECLARE DEFAULT VALUES OF VARIABLES WHICH SHOULD BE TAKEN FROM AIRFAME FILE------- */

#ifdef USE_SERVO_PAN_TILT 

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
#define TILT_SERVO_MAX 2000
//#warning heyooo!
#endif

#ifndef TILT_SERVO_MIN
#define TILT_SERVO_MIN 1000
#endif

#ifndef TILT_SERVO_NOM
#define TILT_SERVO_NOM 1500
#endif 

#ifndef KP_TRACKER
#define KP_TRACKER 0.2 
#endif 

#ifndef PAN_SERVO_MAX_ANG
#error Please define PAN_SERVO_MAX_ANG in your airframe file. 
#endif

#ifndef PAN_SERVO_MIN_ANG
#error Please define PAN_SERVO_MIN_ANG in your airframe file. 
#endif

#ifndef TILT_SERVO_MAX_ANG
#error Please define TILT_SERVO_MAX_ANG in your airframe file.
#endif

#ifndef TILT_SERVO_MIN_ANG
#error Please define TILT_SERVO_MIN_ANG in your airframe file.

#endif

//error handling if servo max-min angles are nor defined

#endif //USE_SERVO_PAN_TILT

/*---END OF DEFAULT VALUES DECLARATION!!--------------------------------------------------
*****************************************************************************************/

/*****************************************************************************************
--DECLARE VARIABLES AND FUNCTIONS TO BE USED IN C FILE ---------------------------------*/
extern void ant_tracker_inform_ground(void);
extern void ant_tracker_init(void);
extern void ant_tracker_track_target(void);
extern void ant_tracker_get_new_aircraft_data(void);
void vPoint(float fPlaneEast, float fPlaneNorth, float fPlaneAltitude,
            float fRollAngle, float fPitchAngle, float fYawAngle,
            float fObjectEast, float fObjectNorth, float fAltitude,
            float *fPan, float *fTilt);
/*---END OF VARIABLES AND FUNCTIONS DECLARATION!!-----------------------------------------
*****************************************************************************************/



#endif 

