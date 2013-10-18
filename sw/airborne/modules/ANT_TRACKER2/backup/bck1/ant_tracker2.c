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

#include "inter_mcu.h"
#include "subsystems/navigation/common_nav.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "subsystems/navigation/traffic_info.h"


// For Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif


#define MANUAL 0
#define AUTO 1

//This will be deleted >>
#define pan_servo_max_angle 120
#define tilt_servo_max_angle 100
//<<This will be deleted

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
struct EnuCoor_f inc_aircraft_data_enu ;
struct LtpDef_f aircraft_ref;


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
ActuatorSet(PAN_SERVO_ID, PAN_SERVO_MIN);
ActuatorSet(TILT_SERVO_ID, TILT_SERVO_MIN);
/*sol_sag = TRUE;
devam=FALSE;
//hareket_frekansi(); */
//ant_tracker_track_target();
}

void ant_tracker_track_target(void) 					//Function to be run after 'Initiation function'
{
    //Checking gps fix will be needed.. 
   //ActuatorSet(PAN_SERVO_ID, PAN_SERVO_MIN);
//ActuatorSet(TILT_SERVO_ID, TILT_SERVO_MIN); 
    
    
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
		man_pan_angle = DL_ANT_TRACKER_DATA_tracker_pan_ang(dl_buffer);
		man_tilt_angle = DL_ANT_TRACKER_DATA_tracker_tilt_ang(dl_buffer);
		tracker_status.z= DL_ANT_TRACKER_DATA_tracker_alt(dl_buffer);
		//ecef_of_lla_f(&inc_aircraft_data_enu, &inc_aircraft_data_lla); 					//Converting incoming values to ecef_f
		
		//aircraft_ref=state.ned_origin_f;
		//aircraft_ref=ltp_def_from_lla_f(aircraft_ref,inc_aircraft_data_lla);
		
		
		
		
		
		//enu_of_lla_point_f(&inc_aircraft_data_enu,&aircraft_ref,&inc_aircraft_data_lla);
	
		
		
		
		
		
		
		if ( mode == MANUAL ){
			//Tracker in Manual mod.. Do Manual stg.. May be this should be in main func
			//ActuatorSet(PAN_SERVO_ID,(((PAN_SERVO_MAX-PAN_SERVO_MIN)/180)*man_pan_angle)+PAN_SERVO_MIN);
			//process_angles();
			}
			
		if (mode == AUTO) {
			//Tracker in Manual mod.. Do Manual stg
			//ActuatorSet(PAN_SERVO_ID,PAN_SERVO_MAX);
			}
			
			
		
			
		}
		
		//for debugging purposes
		fill_device_status();
    
	



}

void calculate_angles(void)					//Calculate joint angles and write them to variables
{
	//check tracker_gps_fix and aircraft_gps_fix (both needs to be 1)  , (self note: tracker gps fix değilse ve daha önce fix olmuşsa kullanılmaya devam edilebilinir.
	// 
	//check magnetometer status and data.. if no magnetometers attacked tracker needs to be orianted to Noth-South direction. (Pan angle nominal to North)
	//This function now calculates degree (not radian) see *180 /M_PI
	
	pan_angle=atan2((aircraft_status.y-tracker_status.y),(aircraft_status.x-tracker_status.x)) *180 /M_PI;
		
		
	aircraft_distance=sqrt( pow((aircraft_status.y-tracker_status.y),2) + pow((aircraft_status.x-tracker_status.x),2)   );
		
		
	tilt_angle=(atan2(((aircraft_status.z-tracker_status.z)),aircraft_distance)- stateGetNedToBodyEulers_f()->theta  ) *180 /M_PI;
	
		
	if (tilt_angle<0) tilt_angle += 360.;
		if (tilt_angle>360) tilt_angle -= 360.;
		
		
		if (pan_angle<0) pan_angle += 360.;
		if (pan_angle>360) pan_angle -= 360.;
		
	//Tracker attitude (pitch and roll angles needs to be taken into account
	/*float x,y;
	x=aircraft_status.z;
	y=tracker_status.z;
	DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &mode, &x, &y, &aircraft_distance);
	*/
		
	
}

void fill_device_status(void)				// Just to tidy things 
{
	
	
							
							
							//struct LlaCoor_f inc_aircraft_data_lla;
							struct EnuCoor_f buffer_enu ;
							struct LtpDef_f aircraft_ref2;
							struct EcefCoor_f tracker_ecef_coor;
							

							//Initialize aircraft ecef coordinates
							tracker_ecef_coor.x	=stateGetPositionEcef_f()->x;
							tracker_ecef_coor.y	=stateGetPositionEcef_f()->y;	
							tracker_ecef_coor.z	=stateGetPositionEcef_f()->z;
							
							
							//get LtpDef_f from aircraft ecef coordinates						
							ltp_def_from_ecef_f(&aircraft_ref2,&tracker_ecef_coor);
							
							//use this LtpDef_f in conversation function
							enu_of_lla_point_f(&buffer_enu,&aircraft_ref2,&inc_aircraft_data_lla);
							
							
							
							aircraft_status.x=buffer_enu.x;
							aircraft_status.y=buffer_enu.y;
							aircraft_status.z=inc_aircraft_data_lla.alt;  //This is send from ground station
	
							tracker_status.x = stateGetPositionEnu_f()->x;
							tracker_status.y = stateGetPositionEnu_f()->y;
							//tracker_status.z = stateGetPositionEnu_f()->z;  //This is send from ground station
	
							/*float x,y,z;
							x=stateGetPositionLla_f()->alt;
							y=6;
							z=5;
							DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &mode, &x, &y, &z);
	
	
	tracker_status.x = stateGetPositionEnu_f()->x;
	tracker_status.y = stateGetPositionEnu_f()->y;
	tracker_status.z = stateGetPositionEnu_f()->z;*/	 //Aircraft altitute can be taken from status
	
	/*aircraft_status.x=inc_aircraft_data_enu.x;
	aircraft_status.y=inc_aircraft_data_enu.y;
	aircraft_status.z=inc_aircraft_data_lla.alt;
	
	float x,y,z;
							x=aircraft_status.z;
							y=tracker_status.z;
							z=0;
							DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &mode, &x, &y, &z);*/
	
}

void process_angles(void)					//Send calculated angles and additional data to tracker over ivybus.. 	
{

//mode değişkenine göre işlemler burada olabilir.




drive_servos();


//Send ivy to see values
//DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &mode, &pan_angle, &tilt_angle, &aircraft_distance);	

float z1,z2,z3;
		z1=(stateGetNedToBodyEulers_f()->phi) *180/M_PI ;
		z2=(stateGetNedToBodyEulers_f()->theta) *180/M_PI;
		z3=(stateGetNedToBodyEulers_f()->psi) *180/M_PI;
		DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &mode, &z1, &z2, &z3);	


	
	
}

void drive_servos(void)
{
	/*
if (mode==MANUAL) {ActuatorSet(PAN_SERVO_ID, PAN_SERVO_MAX);}
else {ActuatorSet(PAN_SERVO_ID, PAN_SERVO_MIN);}
*/


	if (mode==MANUAL) {
		pan_angle=man_pan_angle;
		tilt_angle=man_tilt_angle;
		}
	

	if (pan_angle <pan_servo_max_angle && pan_angle >0) {

					//first move pan servo
					int pan_durt;
					pan_durt=PAN_SERVO_MAX-(((PAN_SERVO_MAX-PAN_SERVO_MIN)*pan_angle)/pan_servo_max_angle);
					ActuatorSet(PAN_SERVO_ID, pan_durt);
					
					
					
					}


		if (tilt_angle <tilt_servo_max_angle && tilt_angle >0) {
					//move tilt servo
					int tilt_durt;
					tilt_durt=TILT_SERVO_MAX-(((TILT_SERVO_MAX-TILT_SERVO_MIN)/tilt_servo_max_angle)*tilt_angle);
					ActuatorSet(TILT_SERVO_ID, tilt_durt);
					}
		
	
	
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



	


