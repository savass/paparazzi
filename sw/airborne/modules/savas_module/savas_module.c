/*
*
*Lan capulsuzzz!!!!
*
*
*/
#include <math.h>
#include "generated/airframe.h"
#include "subsystems/actuators.h"
#include "savas_module.h"
#include "state.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"

// For Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

bool_t sol_sag,devam;
int16_t servo_acisi;
float pic_acisi;

void isinma_hareketleri(void)  //Initiation function
{
//ActuatorSet(SAVAS_SERVO_ID, 1800);
sol_sag = TRUE;
devam=FALSE;
//hareket_frekansi();
}

void hareket_frekansi(void) {  //Function to be run after 'Initiation function'

     	
		if (oldu_mu()){
		  
			  
				 
				 if (sol_sag) {
				  servo_acisi=SAVAS_SERVO_MAX;
				  sol_sag = FALSE;
				   }
				  else{
				  servo_acisi=SAVAS_SERVO_MIN;
				  sol_sag = TRUE;
				  } 
				 ActuatorSet(SAVAS_SERVO_ID, servo_acisi); 
				}
		  

}
  

bool_t oldu_mu(void) {

struct FloatEulers* acilar = stateGetNedToBodyEulers_f();

pic_acisi=fabs(acilar->theta);

//for demostration this line adds stg 
//DOWNLINK_SEND_SSAAVVAASS(DefaultChannel, DefaultDevice, &pic_acisi);

//pitch_acisi=pic_acisi;

//pitch_acisi=pic_acisi;


bool_t dooru_mu;
	if (pic_acisi > 0.174532925 ){
	 //devam=TRUE;
	 dooru_mu= TRUE;
	 
	}
	else{
	 dooru_mu= FALSE;
	 //devam=FALSE;
	}
return dooru_mu;
}


