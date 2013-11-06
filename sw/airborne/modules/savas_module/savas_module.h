/*
*
*
*huloooooogggggg!!!  modul lan boru mu?
*
*
*
*/


#ifndef SAVAS_MODULE_H
#define SAVAS_MODULE_H

#include "std.h"
#include "paparazzi.h"

/**************************************************************************************
--DECLARE DEFAULT VALUES OF VARIABLES WHICH SHOULD BE TAKEN FROM AIRFAME FILE------- */

#ifndef SAVAS_SERVO_ID
#define SAVAS_SERVO_ID SAVAS_SERVO
#endif

#ifndef SAVAS_SERVO_MAX
#define SAVAS_SERVO_MAX 1800
#endif

#ifndef SAVAS_SERVO_MIN
#define SAVAS_SERVO_MIN 1200
#endif

#ifndef SAVAS_SERVO_NOM
#define SAVAS_SERVO_NOM 1500
#endif

#ifndef SSAAVVAASS_pitch_acisi
#define SSAAVVAASS_pitch_acisi SSAAVVAASS_pitch_acisi
#endif

/*---END OF DEFAULT VALUES DECLARATION!!--------------------------------------------------
*****************************************************************************************/

/*****************************************************************************************
--DECLARE VARIABLES AND FUNCTIONS TO BE USED IN C FILE ---------------------------------*/
extern bool_t sol_sag,devam;
extern int16_t servo_acisi;
extern void isinma_hareketleri(void);
extern void hareket_frekansi(void);
extern bool_t oldu_mu(void);
extern float pic_acisi;
/*---END OF VARIABLES AND FUNCTIONS DECLARATION!!-----------------------------------------
*****************************************************************************************/



#endif 

