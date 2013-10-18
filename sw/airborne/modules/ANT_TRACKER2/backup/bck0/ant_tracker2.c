/*
*
*Lan capulsuzzz!!!!
*
*
*/
#include <math.h>
#include "generated/airframe.h"
#include "subsystems/actuators.h"
#include "ant_tracker2.h"
#include "state.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include "math/pprz_geodetic_float.h"

// For Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif


#define MANUAL 0
#define AUTO 1

void calculate_angles(void);
void process_angles(void);


//Tracker and Aircraft status structures
typedef struct {
	int id;
	float roll_ang;
	float pitch_ang;
	float ang_to_N;
	int x;
	int y;
	int z;
}device_status ;

//structures defined to use in conversations;
struct LlaCoor_f inc_aircraft_data_lla;
struct EcefCoor_f inc_aircraft_data_ecef ;


//EcefCoor_f inc_aircraft_data_ecef;

device_status tracker_status;
device_status aircraft_status;

float man_tilt_angle;
float man_pan_angle;
int8_t tracker_mode;

static uint8_t mode;
static int tracker_gps_fix;
static int aircraft_gps_fix;

float tilt_angle;
float pan_angle;
float aircraft_distance;



void ant_tracker_init(void)  						//Initiation function
{
/*ActuatorSet(PAN_SERVO_ID, PAN_SERVO_MIN);
sol_sag = TRUE;
devam=FALSE;
//hareket_frekansi(); */
ant_tracker_track_target();
}

void ant_tracker_track_target(void) 					//Function to be run after 'Initiation function'
{
    //Checking gps fix will be needed.. 
    
    fill_device_status();
    
	calculate_angles();
		
	process_angles();	  

}

void ant_tracker_get_new_aircraft_data(void)			//Parsing 'Ant_Tracker_Data' messages
{



	uint8_t aircraf_no = DL_ANT_TRACKER_DATA_ac_id(dl_buffer);
      
      
            
      if (aircraf_no  == AC_ID) {   //bu koda iyi bak gözüm ileride havada haberleşme için kullanılabilir gibi geliyor....
		  
		
		mode = DL_ANT_TRACKER_DATA_tracker_mode(dl_buffer);  
		inc_aircraft_data_lla.lat = RadOfDeg(DL_ANT_TRACKER_DATA_aircraft_lat(dl_buffer));	//Converting incoming values to radians 
		inc_aircraft_data_lla.lon = RadOfDeg(DL_ANT_TRACKER_DATA_aircraft_lon(dl_buffer));	//Converting incoming values to radians
		inc_aircraft_data_lla.alt= DL_ANT_TRACKER_DATA_aircraft_alt(dl_buffer); 
		man_tilt_angle = DL_ANT_TRACKER_DATA_tracker_pan_ang(dl_buffer);
		man_pan_angle = DL_ANT_TRACKER_DATA_tracker_tilt_ang(dl_buffer);
		
		ecef_of_lla_f(&inc_aircraft_data_ecef, &inc_aircraft_data_lla); 					//Converting incoming values to ecef_f
		
		
		
		
		
		if ( mode == MANUAL ){
			//Tracker in Manual mod.. Do Manual stg.. May be this should be in main func
			//ActuatorSet(PAN_SERVO_ID,(((PAN_SERVO_MAX-PAN_SERVO_MIN)/180)*man_pan_angle)+PAN_SERVO_MIN);
			}
			
		if (mode == AUTO) {
			//Tracker in Manual mod.. Do Manual stg
			//ActuatorSet(PAN_SERVO_ID,PAN_SERVO_MAX);
			}
			
			
		
			
		}
		
		//for debugging purposes
		fill_device_status();
    
		calculate_angles();
		
		process_angles();	



}

void calculate_angles(void)					//Calculate joint angles and write them to variables
{
	//check tracker_gps_fix and aircraft_gps_fix (both needs to be 1)  , (self note: tracker gps fix değilse ve daha önce fix olmuşsa kullanılmaya devam edilebilinir.
	// 
	//check magnetometer status and data.. if no magnetometers attacked tracker needs to be orianted to Noth-South direction. (Pan angle nominal to North)
	//This function now calculates degree (not radian) see *180 /M_PI
	
	pan_angle=atan2((aircraft_status.y-tracker_status.y),(aircraft_status.x-tracker_status.x)) *180 /M_PI;
		
		
	aircraft_distance=sqrt( pow((aircraft_status.y-tracker_status.y),2) + pow((aircraft_status.x-tracker_status.x),2)   );
		
		
	tilt_angle=atan2(((aircraft_status.z-tracker_status.z)),aircraft_distance) *180 /M_PI;
		
		if (tilt_angle<0) tilt_angle += 360.;
		
		if (pan_angle<0) pan_angle += 360.;
		
	//Tracker attitude (pitch and roll angles needs to be taken into account
	
	
		
	
}

void fill_device_status(void)				// Just to tidy things 
{
	
	tracker_status.x = stateGetPositionEcef_f()->x;
	tracker_status.y = stateGetPositionEcef_f()->y;
	tracker_status.z = stateGetPositionEcef_f()->z;	 //Aircraft altitute can be taken from status
	
	aircraft_status.x=inc_aircraft_data_ecef.x;
	aircraft_status.y=inc_aircraft_data_ecef.y;
	aircraft_status.z=inc_aircraft_data_ecef.z;
	
	
	
}

void process_angles(void)					//Send calculated angles and additional data to tracker over ivybus.. 	
{

//mode değişkenine göre işlemler burada olabilir.

//Send ivy to see values
float pp=0;	
DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &mode, &pan_angle, &tilt_angle, &pp);	
	
	
}




/*YARDIMCI OLABILECEK ÇÖP
 //For debug process >>>
		float z1,z2,z3;
		z1=pan_angle;
		z2=tilt_angle;
		z3=0;
		DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &mode, &z1, &z2, &z3);	
		//For debug process <<<<	 
 * 
 * //for demostration this line adds stg 
//DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &inc_aircraft_data_ecef.z, stateGetPositionEcef_f()->z;, NULL);

//pitch_acisi=pic_acisi;

//pitch_acisi=pic_acisi;
* float x,y,z;	
		x = inc_aircraft_data_lla.lat;
		y = inc_aircraft_data_lla.lon;
		z = inc_aircraft_data_lla.alt;
		
		DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &mode, &x, &y, &z);
		* 
bool_t oldu_mu(void) {

struct FloatEulers* acilar = stateGetNedToBodyEulers_f();

pic_acisi=fabs(acilar->theta);




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
* 
* 

		//struct FloatEulers* acilar = stateGetNedToBodyEulers_f();	
		
	
		//DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &tracker_status.x, &tracker_status.y, &tracker_status.z);
		
		if (oldu_mu()){
		  
			ActuatorSet(PAN_SERVO_ID, PAN_SERVO_MIN);
				/* 
				 if (sol_sag) {
				  servo_acisi=PAN_SERVO_MAX;
				  sol_sag = FALSE;
				   }
				  else{
				  servo_acisi=PAN_SERVO_MIN;
				  sol_sag = TRUE;
				  } 
				 ActuatorSet(PAN_SERVO_ID, servo_acisi);
				}
 
 ActuatorSet(PAN_SERVO_ID, PAN_SERVO_MIN);
 * 
 * 
 * 
 * 
 * 
 * */



	


