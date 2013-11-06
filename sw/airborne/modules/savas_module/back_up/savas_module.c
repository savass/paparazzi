/*
*
*Lan capulsuzzz!!!!
*
*
*/

#include "generated/airframe.h"
#include "subsystems/actuators.h"
#include "savas_module.h"

bool_t sol_sag;
int16_t servo_acisi;


void isinma_hareketleri(void)  //Initiation function
{
//ActuatorSet(SAVAS_SERVO_ID, 1800);
sol_sag = TRUE;
//hareket_frekansi();
}



void hareket_frekansi(void)   //Function to be run after 'Initiation function'
{
  if (sol_sag == TRUE) {
  servo_acisi=SAVAS_SERVO_MAX;
  sol_sag = FALSE;
  }
  else{
  servo_acisi=SAVAS_SERVO_MIN;
  sol_sag = TRUE;
  }
 ActuatorSet(SAVAS_SERVO_ID, servo_acisi);
}


