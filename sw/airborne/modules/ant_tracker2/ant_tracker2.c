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
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include "math/pprz_geodetic_float.h"
#include "subsystems/navigation/common_nav.h" 

//For Fixed Wing Configurations
#include "generated/flight_plan.h"





#define MANUAL 0
#define AUTO 1

void calculate_angles(void);
void process_angles(void);
void fill_device_status(void);

#ifdef USE_SERVO_PAN_TILT 
void drive_servos(void);
int pan_target_in_range(float desired_pan_value);
int tilt_target_in_range(float desired_tilt_value);
#endif


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
device_status tracker_status;
device_status aircraft_status;

struct LlaCoor_f inc_aircraft_data_lla;
struct EnuCoor_f inc_aircraft_data_enu ;
struct LtpDef_f aircraft_ref;

float man_tilt_angle;
float man_pan_angle;

static uint8_t mode;

float tilt_angle;
float pan_angle;

void ant_tracker_inform_ground(void)					//Function to send status of joints
{
//Send ivy to see values
float debug_par = KP_TRACKER;

DOWNLINK_SEND_ANT_TRACKER(DefaultChannel, DefaultDevice, &mode, &pan_angle, &tilt_angle, &debug_par);	
	
}

void ant_tracker_init(void)  							//Initiation function
{
//Center the control operators


	#ifdef USE_SERVO_PAN_TILT  //if servo controls defined:
	ActuatorSet(TILT_SERVO_ID, ((TILT_SERVO_MAX+TILT_SERVO_MIN)/2)); 
	ActuatorSet(PAN_SERVO_ID, ((PAN_SERVO_MAX+PAN_SERVO_MIN)/2));
	#endif
}

void ant_tracker_track_target(void) 					//Function to be run after 'Initiation function'
{
    
    
	calculate_angles();
		
	process_angles();	  
							

}

void ant_tracker_get_new_aircraft_data(void)			//Parse 'Ant_Tracker_Data' messages
{



	uint8_t aircraf_no = DL_ANT_TRACKER_DATA_ac_id(dl_buffer);
      
      
            
      if (aircraf_no  == AC_ID) {   
		  
		
		mode = DL_ANT_TRACKER_DATA_tracker_mode(dl_buffer);  
		inc_aircraft_data_lla.lat = RadOfDeg(DL_ANT_TRACKER_DATA_aircraft_lat(dl_buffer));	//Converting incoming values to radians 
		inc_aircraft_data_lla.lon = RadOfDeg(DL_ANT_TRACKER_DATA_aircraft_lon(dl_buffer));	//Converting incoming values to radians
		inc_aircraft_data_lla.alt= DL_ANT_TRACKER_DATA_aircraft_alt(dl_buffer);				//Get aircraft altitude 
		man_pan_angle = DL_ANT_TRACKER_DATA_tracker_pan_ang(dl_buffer);						//Get pan_angle from ground
		man_tilt_angle = DL_ANT_TRACKER_DATA_tracker_tilt_ang(dl_buffer);
		tracker_status.z= DL_ANT_TRACKER_DATA_tracker_alt(dl_buffer);
		
		
		if ( mode == MANUAL ){
			
			}
			
		if (mode == AUTO) {
			
			}
			
			
		
			
		}
		
		
		fill_device_status();
    
	



}

void calculate_angles(void)								//Calculate joint angles and write them to variables
{
	//check tracker_gps_fix and aircraft_gps_fix (both needs to be 1)  , (self note: tracker gps fix değilse ve daha önce fix olmuşsa kullanılmaya devam edilebilinir.
	// 
	//check magnetometer status and data.. if no magnetometers attacked tracker needs to be orianted to Noth-South direction. (Pan angle nominal to North)
	//This function now calculates degree (not radian) see *180 /M_PI
	
	
	vPoint( tracker_status.x, tracker_status.y, tracker_status.z, 
			stateGetNedToBodyEulers_f()->phi, stateGetNedToBodyEulers_f()->theta, stateGetNedToBodyEulers_f()->psi, 
			aircraft_status.x, aircraft_status.y, aircraft_status.z, 
			&pan_angle, &tilt_angle );	
	

	pan_angle=pan_angle*180/M_PI;
	tilt_angle=tilt_angle*180/M_PI;
	
	if (pan_angle<0) pan_angle += 360.;
	if (tilt_angle<0) tilt_angle += 360.;
	
	
}

void fill_device_status(void)							// Fill Aircraft and tracker structures.
{
	
	
							//struct LlaCoor_f inc_aircraft_data_lla;
							struct EnuCoor_f buffer_enu ;
							struct LtpDef_f aircraft_ref2;
							struct EcefCoor_f tracker_ecef_coor;
							

							//Initialize tracker ecef coordinates
							tracker_ecef_coor.x	=stateGetPositionEcef_f()->x;
							tracker_ecef_coor.y	=stateGetPositionEcef_f()->y;	
							tracker_ecef_coor.z	=stateGetPositionEcef_f()->z;
							
							
							//get LtpDef_f from tracker ecef coordinates						
							ltp_def_from_ecef_f(&aircraft_ref2,&tracker_ecef_coor);
							
							//use this LtpDef_f in conversation function
							enu_of_lla_point_f(&buffer_enu,&aircraft_ref2,&inc_aircraft_data_lla);
													
							
							aircraft_status.x=buffer_enu.x;
							aircraft_status.y=buffer_enu.y;
							aircraft_status.z=inc_aircraft_data_lla.alt;  //This is send from ground station
	
							tracker_status.x = stateGetPositionEnu_f()->x;
							tracker_status.y = stateGetPositionEnu_f()->y;
							//tracker_status.z = stateGetPositionEnu_f()->z;  //This is send from ground station
		
}

void process_angles(void)								//Send calculated angles and additional data to tracker over ivybus.. 	
{



#ifdef USE_SERVO_PAN_TILT
drive_servos();
#endif



}


#ifdef USE_SERVO_PAN_TILT

int pan_applied, tilt_applied;
void drive_servos(void)   												//Drive servos within its limits..
{
		
	
	float servo_pan, servo_tilt;	
			
	servo_pan=pan_angle;
	servo_tilt=tilt_angle;
			
	
	

	if (mode==MANUAL) {
		servo_pan=man_pan_angle;
		servo_tilt=man_tilt_angle;
		}
	
	int pan_pwm_val,tilt_pwm_val;

					
	if ( pan_target_in_range(servo_pan) ==  0 ) {   //target in range				
		
					
					float pan_servo_ang_delta;
					
					if (PAN_SERVO_MIN_ANG > PAN_SERVO_MAX_ANG ) {  		//0 degree is between min and max angles
							
							pan_servo_ang_delta= 360 - PAN_SERVO_MIN_ANG + PAN_SERVO_MAX_ANG;
						
							if (PAN_SERVO_MAX > PAN_SERVO_MIN) {
								
									if ( (servo_pan-PAN_SERVO_MIN_ANG) < 0 ){   //travelling over 0 degree 
									
									pan_pwm_val=(servo_pan + 360 - PAN_SERVO_MIN_ANG)*(abs(PAN_SERVO_MAX-PAN_SERVO_MIN)/pan_servo_ang_delta) + PAN_SERVO_MIN;	
									
									
									}
									else
									{									
									pan_pwm_val=(servo_pan-PAN_SERVO_MIN_ANG)*(abs(PAN_SERVO_MAX-PAN_SERVO_MIN)/pan_servo_ang_delta) + PAN_SERVO_MIN;	
									
								
									}
						
							
							
							}
							else {
								
									if ((servo_pan-PAN_SERVO_MIN_ANG)<0){
									
									pan_pwm_val=PAN_SERVO_MIN - (servo_pan + 360 - PAN_SERVO_MIN_ANG)*(abs(PAN_SERVO_MAX-PAN_SERVO_MIN)/(pan_servo_ang_delta));	
									}
									else
									{
									pan_pwm_val=PAN_SERVO_MIN - (servo_pan-PAN_SERVO_MIN_ANG)*(abs(PAN_SERVO_MAX-PAN_SERVO_MIN)/(pan_servo_ang_delta));	
									}	
								
							
							}
							
							
						}
					else{  												//if (PAN_SERVO_MIN_ANG > PAN_SERVO_MAX_ANG )
						pan_servo_ang_delta=PAN_SERVO_MAX_ANG-PAN_SERVO_MIN_ANG;
						if (PAN_SERVO_MAX > PAN_SERVO_MIN) {
						pan_pwm_val=(servo_pan-PAN_SERVO_MIN_ANG)*(abs(PAN_SERVO_MAX-PAN_SERVO_MIN)/(pan_servo_ang_delta)) + PAN_SERVO_MIN;
						}
						else {
						pan_pwm_val=PAN_SERVO_MIN - (servo_pan-PAN_SERVO_MIN_ANG)*(abs(PAN_SERVO_MAX-PAN_SERVO_MIN)/(pan_servo_ang_delta));	
						}
					}					
					
					
					
			
	}
	else { pan_pwm_val = pan_target_in_range(servo_pan); }	

	pan_applied += ((pan_pwm_val - pan_applied) * KP_TRACKER);				
					
	ActuatorSet(PAN_SERVO_ID, pan_applied);	



	if (tilt_target_in_range(servo_tilt) == 0)  {	//Move tilt servo
					
			
					
					float tilt_servo_ang_delta;
					
					if (TILT_SERVO_MIN_ANG > TILT_SERVO_MAX_ANG ) {  		//0 degree is between min and max angles
							
							tilt_servo_ang_delta= 360 - TILT_SERVO_MIN_ANG + TILT_SERVO_MAX_ANG;
						
							if (TILT_SERVO_MAX > TILT_SERVO_MIN) {
								
									if ( (servo_tilt-TILT_SERVO_MIN_ANG) < 0 ){   //travelling over 0 degree 
									
									tilt_pwm_val=(servo_tilt + 360 - TILT_SERVO_MIN_ANG)*(abs(TILT_SERVO_MAX-TILT_SERVO_MIN)/tilt_servo_ang_delta) + TILT_SERVO_MIN;	
									
									
									}
									else
									{									
									tilt_pwm_val=(servo_tilt-TILT_SERVO_MIN_ANG)*(abs(TILT_SERVO_MAX-TILT_SERVO_MIN)/tilt_servo_ang_delta) + TILT_SERVO_MIN;	
									
								
									}
						
							
							
							}
							else {
								
									if ((servo_tilt-TILT_SERVO_MIN_ANG)<0){
									
									tilt_pwm_val=TILT_SERVO_MIN - (servo_tilt + 360 - TILT_SERVO_MIN_ANG)*(abs(TILT_SERVO_MAX-TILT_SERVO_MIN)/(tilt_servo_ang_delta));	
									}
									else
									{
									tilt_pwm_val=TILT_SERVO_MIN - (servo_tilt-TILT_SERVO_MIN_ANG)*(abs(TILT_SERVO_MAX-TILT_SERVO_MIN)/(tilt_servo_ang_delta));	
									}	
								
							
							}
							
							
						}
					else{  												//if (PAN_SERVO_MIN_ANG > PAN_SERVO_MAX_ANG )
						tilt_servo_ang_delta=TILT_SERVO_MAX_ANG-TILT_SERVO_MIN_ANG;
						if (TILT_SERVO_MAX > TILT_SERVO_MIN) {
						tilt_pwm_val=(servo_tilt-TILT_SERVO_MIN_ANG)*(abs(TILT_SERVO_MAX-TILT_SERVO_MIN)/(tilt_servo_ang_delta)) + TILT_SERVO_MIN;
						}
						else {
						tilt_pwm_val=TILT_SERVO_MIN - (servo_tilt-TILT_SERVO_MIN_ANG)*(abs(TILT_SERVO_MAX-TILT_SERVO_MIN)/(tilt_servo_ang_delta));	
						}
					}
					
					
					
					
					

	}
	else { tilt_pwm_val = tilt_target_in_range(servo_tilt); }	

	tilt_applied += ((tilt_pwm_val - tilt_applied) * KP_TRACKER);				
					
	ActuatorSet(TILT_SERVO_ID, tilt_applied);	
	
	
	
	
	
}

int pan_target_in_range(float desired_pan_value) {						//returns 0 within servo limits , returns pwm of edge -which target is close to-
	
	if  (PAN_SERVO_MAX_ANG > PAN_SERVO_MIN_ANG) {
	
		if ((desired_pan_value > PAN_SERVO_MIN_ANG) && (desired_pan_value < PAN_SERVO_MAX_ANG) ) return 0;		//value in range send 0 for calculation
		
		//Determine & send close edge pwm value;
		if (desired_pan_value >PAN_SERVO_MAX_ANG) {
			if ( (desired_pan_value - PAN_SERVO_MAX_ANG) < (360 - desired_pan_value + PAN_SERVO_MIN_ANG)) return PAN_SERVO_MAX;
			else return PAN_SERVO_MIN;
		}else {
		if ( (PAN_SERVO_MIN_ANG - desired_pan_value) < (360 - PAN_SERVO_MAX_ANG + desired_pan_value)) return PAN_SERVO_MIN;
			else return PAN_SERVO_MAX;		
		}
		
	}
	else { 		//0 point is within defined travel area of pan servo
	
	if ( (desired_pan_value < PAN_SERVO_MAX_ANG) ) return 0;
	if ( (desired_pan_value > PAN_SERVO_MIN_ANG) ) return 0;
	if ( (PAN_SERVO_MIN_ANG - desired_pan_value) < (desired_pan_value-PAN_SERVO_MAX_ANG) ) return PAN_SERVO_MIN;
	else return PAN_SERVO_MAX;

	}
	
	
}


int tilt_target_in_range(float desired_tilt_value) {					//returns 0 within servo limits , returns pwm of edge -which target is close to-
	
	if  (TILT_SERVO_MAX_ANG > TILT_SERVO_MIN_ANG) {
	
		if ((desired_tilt_value > TILT_SERVO_MIN_ANG) && (desired_tilt_value < TILT_SERVO_MAX_ANG) ) return 0;		//value in range send 0 for calculation
		
		//Determine & send close edge pwm value;
		if (desired_tilt_value >TILT_SERVO_MAX_ANG) {
			if ( (desired_tilt_value - TILT_SERVO_MAX_ANG) < (360 - desired_tilt_value + TILT_SERVO_MIN_ANG)) return TILT_SERVO_MAX;
			else return TILT_SERVO_MIN;
		}else {
		if ( (TILT_SERVO_MIN_ANG - desired_tilt_value) < (360 - TILT_SERVO_MAX_ANG + desired_tilt_value)) return TILT_SERVO_MIN;
			else return TILT_SERVO_MAX;		
		}
		
	}
	else { 		//0 point is within defined travel area of tilt servo
	
	if ( (desired_tilt_value < TILT_SERVO_MAX_ANG) ) return 0;
	if ( (desired_tilt_value > TILT_SERVO_MIN_ANG) ) return 0;
	if ( (TILT_SERVO_MIN_ANG - desired_tilt_value) < (desired_tilt_value-TILT_SERVO_MAX_ANG) ) return TILT_SERVO_MIN;
	else return TILT_SERVO_MAX;

	}
	
}




#endif //USE_SERVO_PAN_TILT

/*From now on, code is taken from modules/cam_control/point.c file. Read that for documentation. Some modifications are made for this module. Arnold Schroeter & Chris Efstathiou
*did great job on code & calculations & documentation.
*/

typedef struct {
         float fx;
         float fy;
         float fz;} VECTOR;

typedef struct {
         float fx1; float fx2; float fx3;
         float fy1; float fy2; float fy3;
         float fz1; float fz2; float fz3;} MATRIX;

float cam_theta;
float cam_phi;
bool_t heading_positive = 0;
float  memory_x, memory_y, memory_z;

void vSubtractVectors(VECTOR* svA, VECTOR svB, VECTOR svC);
void vMultiplyMatrixByVector(VECTOR* svA, MATRIX smB, VECTOR svC);

/*******************************************************************
; function name:   vSubtractVectors
; description:     subtracts two vectors a = b - c
; parameters:
;*******************************************************************/
void vSubtractVectors(VECTOR* svA, VECTOR svB, VECTOR svC)
{
  svA->fx = svB.fx - svC.fx;
  svA->fy = svB.fy - svC.fy;
  svA->fz = svB.fz - svC.fz;
}

/*******************************************************************
; function name:   vMultiplyMatrixByVector
; description:     multiplies matrix by vector svA = smB * svC
; parameters:
;*******************************************************************/
void vMultiplyMatrixByVector(VECTOR* svA, MATRIX smB, VECTOR svC)
{
  svA->fx = smB.fx1 * svC.fx  +  smB.fx2 * svC.fy  +  smB.fx3 * svC.fz;
  svA->fy = smB.fy1 * svC.fx  +  smB.fy2 * svC.fy  +  smB.fy3 * svC.fz;
  svA->fz = smB.fz1 * svC.fx  +  smB.fz2 * svC.fy  +  smB.fz3 * svC.fz;
}

/*******************************************************************
; function name:   vPoint
; description:     Transforms ground coordinate system into
;                  plane's coordinate system via three rotations
;                  and determines positions of camera servos.
; parameters:      fPlaneNorth, fPlaneEast, fPlaneAltitude  plane's
;                           position with respect to ground
;                           in m (actually the units do not matter as
;                           long as they are the same as for the object's
;                           position)
;                  fRollAngle  level=0; right wing down = positive values
;                  fPitchAngle level=0; nose up = positive values
;                           plane's pitch and roll angles
;                           with respect to ground in radians
;                  fYawAngle   north=0; right= positive values in radians
;                           plane's yaw angle with respect to north
;                  fObjectNorth, fObjectEast, fAltitude object's
;                           position with respect to ground
;                           in m (actually the units do not matter as
;                           long as they are the same for the plane's
;                           position)
;                  fPan, fTilt angles for camera servos in radians,
;                           pan is turn/left-right and tilt is down-up
;                           in reference to the camera picture
; camera mount:    The way the camera is mounted is given through a
;                  define POINT_CAM_a_[_b] where a gives the mount
;                  angle within the aircraft and b the angle when
;                  viewing the direction of the first servo.
;*******************************************************************/
void vPoint(float fPlaneEast, float fPlaneNorth, float fPlaneAltitude,
            float fRollAngle, float fPitchAngle, float fYawAngle,
            float fObjectEast, float fObjectNorth, float fAltitude,
            float *fPan, float *fTilt)
{
  static VECTOR svPlanePosition,
                svObjectPosition,
                svObjectPositionForPlane,
                svObjectPositionForPlane2;

  static VECTOR sv_cam_projection,
                sv_cam_projection_buf;

  static MATRIX smRotation;

/***        BELOW IS THE CODE THAT READS THE RC PAN AND TILT CHANNELS AND CONVERTS THEM TO ANGLES (RADIANS)   ***/
/***        IT IS USED FOR CALCULATING THE COORDINATES OF THE POINT WHERE THE CAMERA IS LOOKING AT            ***/
/***        THIS IS DONE ONLY FOR THE CAM_MODE_STABILIZED OR CAM_MODE_RC.                                     ***/
/***        IN OTHER MODES ONLY THE CAM_POINT WAYPOINT AND THE DISTANCE FROM TARGET IS UPDATED                ***/


	#ifdef DISABLE_AUTO_HEADING
	fYawAngle=0;
	#endif
	
	#ifdef DISABLE_AUTO_ATTITUDE
	fPitchAngle=0;
	fRollAngle=0;
	#endif



   svPlanePosition.fx = fPlaneEast;
   svPlanePosition.fy = fPlaneNorth;
   svPlanePosition.fz = fPlaneAltitude;

// FOR TESTING ANGLES IN THE SIMULATOR ONLY CODE UNCOMMENT THE TWO BELOW LINES
//cam_phi = RadOfDeg(90); // LOOK 45 DEGREES TO THE LEFT, -X IS TO THE LEFT AND +X IS TO THE RIGHT
//cam_theta = RadOfDeg(70);     //  LOOK 45 DEGREES DOWN, 0 IS STRAIGHT DOWN 90 IS STRAIGHT IN FRONT

  
    sv_cam_projection_buf.fx = svPlanePosition.fx + (tanf(cam_theta)*(fPlaneAltitude-ground_alt));
    sv_cam_projection_buf.fy = svPlanePosition.fy;
   


   /* distance between plane and camera projection */
   vSubtractVectors(&sv_cam_projection, sv_cam_projection_buf, svPlanePosition);

   float heading_radians = RadOfDeg(90) - fYawAngle; //Convert the gps heading (radians) to standard mathematical notation.
   if(heading_radians > RadOfDeg(180)){ heading_radians -= RadOfDeg(360); }
   if(heading_radians < RadOfDeg(-180)){ heading_radians += RadOfDeg(360); }
   //heading_radians += cam_theta;

   /* camera pan angle correction, using a clockwise rotation */
   smRotation.fx1 = (float)(cos(cam_phi));
   smRotation.fx2 = (float)(sin(cam_phi));
   smRotation.fx3 = 0.;
   smRotation.fy1 = -smRotation.fx2;
   smRotation.fy2 = smRotation.fx1;
   smRotation.fy3 = 0.;
   smRotation.fz1 = 0.;
   smRotation.fz2 = 0.;
   smRotation.fz3 = 1.;

   vMultiplyMatrixByVector(&sv_cam_projection_buf, smRotation, sv_cam_projection);

   /* yaw correction using a counter clockwise rotation*/
   smRotation.fx1 = (float)(cos(heading_radians));
   smRotation.fx2 = -(float)(sin(heading_radians));
   smRotation.fx3 = 0.;
   smRotation.fy1 = -smRotation.fx2;
   smRotation.fy2 = smRotation.fx1;
   smRotation.fy3 = 0.;
   smRotation.fz1 = 0.;
   smRotation.fz2 = 0.;
   smRotation.fz3 = 1.;

   vMultiplyMatrixByVector(&sv_cam_projection, smRotation, sv_cam_projection_buf);



//************************************************************************************************
//************************************************************************************************
//************************************************************************************************
//************************************************************************************************

/*
By swapping coordinates (fx=fPlaneNorth, fy=fPlaneEast) we make the the circle angle go from 0 (0 is to the top of the circle)
to 360 degrees or from 0 radians to 2 PI radians in a clockwise rotation. This way the GPS reported angle can be directly
applied to the rotation matrices (in radians).
In standard mathematical notation 0 is to the right (East) of the circle, -90 is to the bottom, +-180 is to the left
and +90 is to the top (counterclockwise rotation).
When reading back the actual rotated coordinates sv_cam_projection.fx has the y coordinate and sv_cam_projection.fy has the x
represented on a circle in standard mathematical notation.
*/
  svPlanePosition.fx = fPlaneNorth;
  svPlanePosition.fy = fPlaneEast;
  svPlanePosition.fz = fPlaneAltitude;

  svObjectPosition.fx = fObjectNorth;
  svObjectPosition.fy = fObjectEast;
  svObjectPosition.fz = fAltitude;

  /* distance between plane and object */
  vSubtractVectors(&svObjectPositionForPlane, svObjectPosition, svPlanePosition);

  /* yaw */
  smRotation.fx1 = (float)(cos(fYawAngle));
  smRotation.fx2 = (float)(sin(fYawAngle));
  smRotation.fx3 = 0.;
  smRotation.fy1 = -smRotation.fx2;
  smRotation.fy2 = smRotation.fx1;
  smRotation.fy3 = 0.;
  smRotation.fz1 = 0.;
  smRotation.fz2 = 0.;
  smRotation.fz3 = 1.;

  vMultiplyMatrixByVector(&svObjectPositionForPlane2, smRotation, svObjectPositionForPlane);

  /* pitch */
  smRotation.fx1 = (float)(cos(fPitchAngle));
  smRotation.fx2 = 0.;
  smRotation.fx3 = (float)(sin(fPitchAngle)); //bu kitapta -sin
  smRotation.fy1 = 0.;
  smRotation.fy2 = 1.;
  smRotation.fy3 = 0.;
  smRotation.fz1 = -smRotation.fx3;
  smRotation.fz2 = 0.;
  smRotation.fz3 = smRotation.fx1;

  vMultiplyMatrixByVector(&svObjectPositionForPlane, smRotation, svObjectPositionForPlane2);

  /* roll */
  smRotation.fx1 = 1.;
  smRotation.fx2 = 0.;
  smRotation.fx3 = 0.;
  smRotation.fy1 = 0.;
  smRotation.fy2 = (float)(cos(fRollAngle));
  smRotation.fy3 = (float)(-sin(fRollAngle)); //bu da kitapta +sin
  smRotation.fz1 = 0.;
  smRotation.fz2 = -smRotation.fy3;
  smRotation.fz3 = smRotation.fy2;

  vMultiplyMatrixByVector(&svObjectPositionForPlane2, smRotation, svObjectPositionForPlane);





	*fPan = (float)(atan2( svObjectPositionForPlane2.fy, svObjectPositionForPlane2.fx));

	*fTilt = (float)(atan2(svObjectPositionForPlane2.fz,
                          sqrt(  svObjectPositionForPlane2.fy * svObjectPositionForPlane2.fy
                               + svObjectPositionForPlane2.fx * svObjectPositionForPlane2.fx )
                        ));

	//Roll control can be added


}
