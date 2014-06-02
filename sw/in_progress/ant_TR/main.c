/*Main loop takes input start_mod,tracker_id and aircraft_id
 * ant_TR [start_mod aircraft_id tracker_id ]
 * start_mod :		a for automatic m for manuel
 * aircraft_id:		id of aircraft which is monitored by tracker_id (int)
 * tracker_id: 		id of tracker (int)
 * Paparazzi server sent  'ground NAV_STATUS 10 0 2 43 0 43.564118 1.481279 0.000000 147.000846 0.000000 0.000000'

*/


#include <glib.h>
#include <glib/gprintf.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <sys/types.h>
#include <unistd.h>


#include "main.h"

#ifdef USE_ORION
  #include "orion.h"
  #define ORIONSTEP 2
#endif

#define MANUAL 0
#define AUTO 1
#define GUI_PRECISION (8)

#define MaxNumDevices 25
#define MaxNumConfNames 500
#define MaxNumConfNameLength 100
#define BUFLENx 2048

#ifdef __APPLE__
char defaultIvyBus[] = "224.255.255.255:2010";
#else
char defaultIvyBus[] = "127.255.255.255:2010";
#endif
char* IvyBus;

// verbose flag
int verbose = 0;

int StartParamUsed = 0;
int tracker_mode;

float tilt_angle;
float pan_angle;

int SelTrackerID = 0;		//by Combobox'
int SelAircraftID = 0;		//by Combobox'
int tty_fd;


void Send_ContList_Msg (int ContListNumb);

void Refresh_Tracker_List (void);
void Refresh_Device_List (void);
void Refresh_Tracker_Labels (void);
void Refresh_Aircraft_Labels (void);
void Refresh_Tracker_Mode (void);
void Refresh_Sliders (void);
void print_help();

int Check_Controlled_Aircraft_List (int Aircraft_Id);
int Check_Controlled_Tracker_List (int Tracker_Id);
int Get_Dev_Ind(int device_id);
void Fill_Dev_Name(int dev_index);
int Get_Trac_Ind (int tracker_id);
void Get_Device_Name( int dev_status_index );
void Load_Device_Names (void);
int check_device_name(int dev_id);
void request_ac_config(int ac_id_req);
void save_device_name(int dev_id, char *dev_name);


typedef struct {
	int used;
	int device_id;
	float latitude;
	float longitude;
	float altitude;
	char *name;
}DevStatus_s ;
DevStatus_s DevStatus [2 * MaxNumDevices];  //Holds all status of devices

typedef struct{
	int used;
	int mode;
	int tracker_id;
	int aircraft_id;
	float pan_angle;
	float tilt_angle;
}TrackContList_s;
TrackContList_s TrackContList [MaxNumDevices];

typedef struct {
	int device_id;
	char name[MaxNumConfNameLength];
}DevNames_s ;
DevNames_s DevNames [MaxNumConfNames];



int program_started = 0;
//ERRORS
int MaxNumDevicesReached = 1 ;   //will be -1 if error flag raised


void on_window_destroy (GtkWidget *object, gpointer user_data)  {

#ifdef USE_ORION
close(tty_fd);
#endif

IvyStop();

gtk_main_quit();

}

void on_window_ant_track_show(GtkWidget *object, gpointer user_data)  {

		if(StartParamUsed>0) {
		Refresh_Device_List();
		Refresh_Tracker_List();
		Refresh_Tracker_Mode();
		}
		program_started=1;

}

void on_combobox_aircrafts_changed (GtkComboBox *widget, gpointer user_data){}

void on_combobox_tracker_changed (GtkComboBox *widget, gpointer user_data) {


//!!!!!!get id of selected device

gtk_combo_box_set_active( GTK_COMBO_BOX(myGUI->combobox_devices), Get_Dev_Ind ( TrackContList[gtk_combo_box_get_active(myGUI->combobox_trackers)].aircraft_id ) );

SelTrackerID = TrackContList[gtk_combo_box_get_active(myGUI->combobox_trackers)].tracker_id;
SelAircraftID = TrackContList[gtk_combo_box_get_active(myGUI->combobox_trackers)].aircraft_id;

Refresh_Aircraft_Labels();
Refresh_Tracker_Labels();
Refresh_Tracker_Mode();


/*int selected = gtk_combo_box_get_active(myGUI->combobox_aircrafts);

char str[15];
sprintf(str, "%d", selected);

show_err_msg(str);
*/

}

void on_set_button_clicked (GtkWidget *object, gpointer user_data) {

SelTrackerID = TrackContList[gtk_combo_box_get_active(myGUI->combobox_trackers)].tracker_id  ;
SelAircraftID = DevStatus[gtk_combo_box_get_active(myGUI->combobox_devices)].device_id;

TrackContList[gtk_combo_box_get_active(myGUI->combobox_trackers)].aircraft_id=SelAircraftID;

Refresh_Aircraft_Labels();


}

int orion_pan_buf= 1;
int orion_tilt_buf = 1;

extern void on_pan_angle_changed (GtkRange *object, gpointer user_data);
void on_pan_angle_changed (GtkRange *object, gpointer user_data) {

        if (TrackContList[Get_Trac_Ind(SelTrackerID)].mode == MANUAL) {

        TrackContList[Get_Trac_Ind(SelTrackerID)].pan_angle=gtk_range_get_value(myGUI->GtkSCale_pan_angle);

        Send_ContList_Msg ( Get_Trac_Ind(SelTrackerID) );

        #ifdef USE_ORION
        int GotoAng_pan;
        GotoAng_pan=gtk_range_get_value(myGUI->GtkSCale_pan_angle);
        if (abs(orion_pan_buf-GotoAng_pan)>ORIONSTEP){
          if (GotoAng_pan>= 0 && GotoAng_pan<=180){
		    go_to(1, GotoAng_pan , tty_fd);
		  }
		  else {
		    go_to(1, (GotoAng_pan-360) , tty_fd);
		  }

                  orion_pan_buf=GotoAng_pan;
                  //printf("pan (auto):>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%d\n",GotoAng_pan);
                  }
        #endif

	}


}


void on_tilt_angle_changed (GtkRange *object, gpointer user_data) {

	if (TrackContList[Get_Trac_Ind(SelTrackerID)].mode == MANUAL) {

	TrackContList[Get_Trac_Ind(SelTrackerID)].tilt_angle=gtk_range_get_value(myGUI->GtkSCale_tilt_angle);

	Send_ContList_Msg ( Get_Trac_Ind(SelTrackerID) );

        #ifdef USE_ORION
        int GotoAng;
        GotoAng=gtk_range_get_value(myGUI->GtkSCale_tilt_angle);
        if ( abs(orion_tilt_buf-GotoAng) > ORIONSTEP ){
        if (GotoAng>= 0 && GotoAng<=180){
		    go_to(2, GotoAng , tty_fd);
		  }
		  else {
		    go_to(2, (GotoAng-360) , tty_fd);
		  }
        orion_tilt_buf=GotoAng;
        printf("tilt:>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%d\n",GotoAng);
        }
        #endif

	}



}

void on_manuel_mod_changed(GtkSwitch *widget, gpointer user_data)
{
	if (program_started > 0 ) {

	if (gtk_switch_get_active(myGUI->GtkSwitch_manuel_mod)) TrackContList[Get_Trac_Ind(SelTrackerID)].mode=1;
	else TrackContList[Get_Trac_Ind(SelTrackerID)].mode=0;
	Refresh_Tracker_Mode();

	}
}

void on_FLIGHT_PARAM_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){

  //Paparazzi server sent  'ground NAV_STATUS 10 0 2 43 0 43.564118 1.481279 0.000000 147.000846 0.000000 0.000000'
  //Paparazzi server sent  'ground FLIGHT_PARAM 10 4.434268 -2.951516 346.557313 43.564118 1.481279 0.031059 309.8 136.961784 0.008162 -10.038216 1380791276.770251 378491500'

	//check if device id is in controlled aircrafts. If not add it to combobox and return -1 ; If yes return the place of it in structure of the structure
	int IncDevId = Get_Dev_Ind ( atoi(argv[0]) );

	if (IncDevId >= 0 ) {

			//Fill device status values
			DevStatus[IncDevId].altitude = atof(argv[8]);
			DevStatus[IncDevId].longitude = atof(argv[5]);
			DevStatus[IncDevId].latitude = atof(argv[4]);

			//If device is in control list send data over ivybus

			if ( Check_Controlled_Aircraft_List(atoi(argv[0])) >= 0 ){
				//show_err_msg("bilo",2);
			Send_ContList_Msg ( Check_Controlled_Aircraft_List(atoi(argv[0])) ); }

			if ( SelAircraftID == atoi(argv[0]) ) Refresh_Aircraft_Labels();
			if ( SelTrackerID == atoi(argv[0]) ) Refresh_Tracker_Labels();

	}
	else return;

}

void on_tracker_ANT_TRACK(IvyClientPtr app, void *user_data, int argc, char *argv[]) {

  int TrackerId= atoi(argv[0]);

  if (check_device_name(TrackerId) < 0) return;


	int IncDevId = Check_Controlled_Tracker_List (TrackerId );


	//int IncDevId = Get_Dev_Id ( atoi(argv[0]) );

	if (IncDevId >= 0 ) {

		if (TrackContList[IncDevId].mode == AUTO) {
		TrackContList[IncDevId].pan_angle= atof(argv[2]);
		TrackContList[IncDevId].tilt_angle = atof(argv[3]);
		Refresh_Sliders();

		#ifdef USE_ORION
		if (atoi(argv[0])==SelTrackerID) {
                int GotoAng_pan ;
                GotoAng_pan=TrackContList[IncDevId].pan_angle;

                  if ( abs(orion_pan_buf-GotoAng_pan) > ORIONSTEP ){

		  if (GotoAng_pan>= 0 && GotoAng_pan<=180){
		    go_to(1, GotoAng_pan , tty_fd);
		  }
		  else {
		    go_to(1, (GotoAng_pan-360) , tty_fd);
		  }

                  orion_pan_buf=GotoAng_pan;
                  //printf("pan (auto):>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%d\n",GotoAng_pan);
                  }

                int GotoAng_tilt;
                GotoAng_tilt=TrackContList[IncDevId].tilt_angle;
                  if ( abs(orion_pan_buf-GotoAng_tilt) > ORIONSTEP ){
                  go_to(2, GotoAng_tilt , tty_fd);
                  orion_tilt_buf=GotoAng_tilt;
                  //printf("tilt (auto):>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%d\n",GotoAng_tilt);
                  }
		}
                #endif

		}
	}
	else return;

	/*add_tracker_if_new(atoi(argv[0]));
	//tepki_ver();
	 if (atoi(argv[0]) == atoi(tracker_id) ) {

		if (tracker_mode==AUTO){
		pan_angle= atof(argv[2]);
		tilt_angle = atof(argv[3]);
		}
		refresh_gui();

	 }*/

}

int ProcessID;
int RequestID;
void on_tracker_NEW_AC(IvyClientPtr app, void *user_data, int argc, char *argv[]) {

 //Request config
 request_ac_config(atoi(argv[0]));

}

void request_ac_config(int ac_id_req) {

  RequestID++;
  IvySendMsg("anttrUI %d_%d CONFIG_REQ %d" ,ProcessID, RequestID ,ac_id_req );

  if (verbose) {
  printf("AC(id= %d) config requested.\n",ac_id_req);
  }
}

void on_tracker_GET_CONFIG(IvyClientPtr app, void *user_data, int argc, char *argv[]) {

  //Check request_id
  int i=0;

  int RmId[2];
  /*
   * RmId[0] >> ProcessID of incoming MsgProcessId
   * RmId[1] >> RequestID of incoming MsgProcessId
   */

  //Split arg0 to get process id and request id
  char * mtok;
  mtok = strtok (argv[0],"_");
  while (mtok != NULL || i>2)
  {
    RmId[i]= atoi(mtok);
    i++;
    mtok = strtok (NULL, "_");
  }

  //Check whether process id and request id matches or not
  if ( RmId[0]== ProcessID && RmId[1]==RequestID) {

    save_device_name(atoi(argv[1]),argv[7]);

  }

}

void save_device_name(int dev_id, char *dev_name) {

  if (verbose) {
  printf("Saving device name id= %d name=%s\n",dev_id,dev_name);
  }


  //Search DevNames
  int i=0;
  while ( (i< MaxNumConfNames) && (DevNames[i].device_id > 0) ) {
        if ( DevNames[i].device_id == dev_id ) {
	  //Rewrite device name
	  strcpy(DevNames[i].name, dev_name);
	  if (verbose) {
	  printf("Device saved DevNames (RW). id= %d name=%s\n",dev_id,dev_name);
	  }
	  return;
        }
      i++;
      //if (DevNames[i].device_id > 0 ) break;
      }
  //i=0;

  //save new item
  while ( i< MaxNumConfNames) {
        if ( DevNames[i].device_id == 0 ) {
	  //save device name
	  strcpy(DevNames[i].name, dev_name);
	    if (verbose) {
	    printf("Device saved DevNames. (new) id= %d name=%s\n",dev_id,dev_name);
	    }
	  DevNames[i].device_id=dev_id;
	  return;
        }
      i++;
      }
  //No country for new device!!
  if (verbose) {
  printf("Device cannot be saved. MaxNumConfNames reached!\n");
  }

 return;
}

// Print help message
void print_help() {
  printf("Usage: ant_tracker [options]\n");
  printf(" Options :\n");
  printf("   -m <Tracker mode>\tauto(a) or manual(m) (default: m)\n");
  printf("   -t <Tracker id>\tid of tracker: (default: none))\n");
  printf("   -a <Aircraft id>\tid of aircraft to be tracked.(default: none)\n");
  printf("   -b <Ivy bus>\tdefault is %s\n", defaultIvyBus);
  printf("   -v\tverbose\n");
  printf("   -h --help show this help\n");
}


int main(int argc, char **argv) {

  //Get process id
  ProcessID= getpid();

  // try environment variable first, set to default if failed
  IvyBus = getenv("IVYBUS");
  if (IvyBus == NULL) IvyBus = defaultIvyBus;

  // Parse options
  int i;
  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_help();
      exit(0);
    }
    else if (strcmp(argv[i], "-m") == 0) {
      char StartMode=argv[++i][0];
      if (StartMode=='m' || StartMode=='M') {
	tracker_mode=MANUAL;
	TrackContList[0].mode=0;
      }
      else if (StartMode=='a' || StartMode=='A') {
	tracker_mode=AUTO;
	TrackContList[0].mode=1;
      }
      else print_help();
      //tracker_mode = argv[++i][0];
    }
    else if (strcmp(argv[i], "-t") == 0) {
      SelTrackerID = atoi(argv[++i]);
    }
    else if (strcmp(argv[i], "-a") == 0) {
      SelAircraftID = atoi(argv[++i]);
    }
    else if (strcmp(argv[i], "-b") == 0) {
      IvyBus = argv[++i];
    }
    else if (strcmp(argv[i], "-v") == 0) {
      verbose = 1;
    }
    else {
      printf("App Server: Unknown option\n");
      print_help();
      exit(0);
    }
  }

  //Load_Device_Names();

#ifdef USE_ORION
tty_fd = init_serial_port();
init_orion(tty_fd);
if (verbose) {
printf("Using Orion Tracker\n");
}
#endif

  myGUI = g_slice_new (myGTK);
  gtk_init (&argc, &argv);
  bound_ui_items();
  gtk_widget_show (myGUI->window);


  if (StartParamUsed > 0) {
    gtk_combo_box_set_active(GTK_COMBO_BOX(myGUI->combobox_devices),1);
    gtk_combo_box_set_active(GTK_COMBO_BOX(myGUI->combobox_trackers),1);
  }

  IvyInit ("ant_TR", "Anthenna Tracker READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_FLIGHT_PARAM_STATUS, NULL, "ground FLIGHT_PARAM (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_tracker_ANT_TRACK, NULL, "(\\S*) ANT_TRACKER (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_tracker_NEW_AC, NULL, "ground NEW_AIRCRAFT (\\S*)");
  IvyBindMsg(on_tracker_GET_CONFIG, NULL, "(\\S*) ground CONFIG (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");

  IvyStart(IvyBus);
  //IvyStart("10.31.6.17");

  gtk_main ();

  return 0;
}

int Get_Trac_Ind (int tracker_id) {
  return Check_Controlled_Tracker_List(tracker_id);
}

int check_device_name(int dev_id) {

  //Search DevNames struct
  int i=0;
  while ( (i< MaxNumConfNames) && (DevNames[i].device_id > 0) ) {
        if ( DevNames[i].device_id == dev_id ) {

	  return 1;
        }
      i++;
      //if () break;
      }

  //no device found request for device name

  if (dev_id>0) request_ac_config(dev_id);
  return -1;

}

// Returns DevStatus array index of send device id (if device exists).
// If device not found, adds it to the list and then send its new array index. If MaxNumDevices reached returns -1
int Get_Dev_Ind(int device_id) {

  //check whether device is recorded in DevNames
  if (check_device_name(device_id) < 0) {
    //device not in list
    return -1;
  }

  int i=0;
  //Search list
  while ( (DevStatus[i].used != 0) && (i < (2*MaxNumDevices)) ) {
    if ( DevStatus[i].device_id == device_id ) return i;
  i +=1;
  }

  if (i >= (2*MaxNumDevices) ) { MaxNumDevicesReached =-1; return -1;}	//MaxNumDevices reached return -1
  //Add new entry to list and return entry index;
  DevStatus[i].device_id = device_id;
  //raise .used flag
  DevStatus[i].used=1;
  //Refresh UI
  Refresh_Device_List();
  Refresh_Tracker_List();
  //Return index
  return i;
}

//Returns TrackContList array index of send device id if device exist in controlled aircrafts. If MaxNumDevices reached returns -1
int Check_Controlled_Aircraft_List (int Aircraft_Id) {

  int i=0;
  //Search tracker control list
  while ( (TrackContList[i].used != 0) && (i < MaxNumDevices) ) {
    if ( TrackContList[i].aircraft_id == Aircraft_Id ) return i;
  i +=1;
  }
  //return -1 if max numb of devices reached
  if ( i >= (MaxNumDevices) )  MaxNumDevicesReached =-1;
  return -1;
}

//Sends ivy msg for given index of TrackContList
void Send_ContList_Msg (int ContListNumb) {
  //get aircraft id
  int AircraftID = TrackContList[ContListNumb].aircraft_id;
  //get tracker id
  int TrackerID  = TrackContList[ContListNumb].tracker_id;
  //cancel if max numb of device error raised
  if ( TrackerID <0 || AircraftID <0 ) return;
  //send ivy msg
  IvySendMsg("ground ANT_TRACKER_DATA %d %d %f %f %f %f %f %f" ,
        TrackerID, TrackContList[ContListNumb].mode ,
        TrackContList[ContListNumb].pan_angle, TrackContList[ContListNumb].tilt_angle,
        DevStatus[Get_Dev_Ind(AircraftID)].latitude, DevStatus[Get_Dev_Ind(AircraftID)].longitude, DevStatus[Get_Dev_Ind(AircraftID)].altitude,
        DevStatus[Get_Dev_Ind(TrackerID)].altitude );

}

//Returns TrackContList array index of send device id, if device exist in trackers.
//If MaxNumDevices reached returns -1
//If device is not at tracker list, adds it to list and send new id,
int Check_Controlled_Tracker_List (int Tracker_Id) {
  int i=0;
  //check if device is already in control list
  while ( (TrackContList[i].used != 0) && (i < MaxNumDevices) ) {
    if ( TrackContList[i].tracker_id == Tracker_Id ) return i;
      i +=1;
    }
  //if MaxNumDevices reached return -1
  if ( i >= (MaxNumDevices) ) { MaxNumDevicesReached =-1; return -1; }
  //Add it to control list & return index if it
  TrackContList[i].used = 1;
  TrackContList[i].tracker_id = Tracker_Id;
  TrackContList[i].aircraft_id = -1;
  TrackContList[i].mode = 0;
  Refresh_Tracker_List();
  //Debug purposes
  if (verbose) {
  printf("New tracker added.. (ID=%d)\n",Tracker_Id);
  }

  return i;
}

//Fill device name in device control list
void Fill_Dev_Name(int dev_index) {

  if (DevStatus[dev_index].device_id >=0 ) {
    int i=1;  //0th place is n/a
    while ( i< MaxNumConfNames) {
        if ( DevNames[i-1].device_id == DevStatus[dev_index].device_id ) {
          DevStatus[dev_index].name=DevNames[i-1].name;
        }
      i+=1;
      }
  }
  else DevStatus[dev_index].name="N/A";
}

//Refreshs devicelist/aircraftlist combobox in GUI
void Refresh_Device_List(void){

  int selected = gtk_combo_box_get_active(myGUI->combobox_devices);
  GtkTreeIter iter_dev;
  GtkListStore *store_dev = gtk_list_store_new(1,G_TYPE_STRING);
  GtkCellRenderer *cell_dev = gtk_cell_renderer_text_new();
  gtk_list_store_clear(store_dev);
  gtk_cell_layout_clear(GTK_CELL_LAYOUT(myGUI->combobox_devices));

  int i=0;
  while ( (DevStatus[i].used == 1) && ( i < (2*MaxNumDevices) )  ){
    Fill_Dev_Name(i);
    gtk_list_store_append(store_dev,&iter_dev);
    gtk_list_store_set(store_dev,&iter_dev,0,DevStatus[i].name,-1);
    i +=1;
  }

  gtk_combo_box_set_model((myGUI->combobox_devices), GTK_TREE_MODEL(store_dev));
  gtk_cell_layout_pack_start(GTK_CELL_LAYOUT(myGUI->combobox_devices), cell_dev, TRUE);
  gtk_cell_layout_set_attributes( GTK_CELL_LAYOUT( myGUI->combobox_devices ), cell_dev, "text", 0, NULL );
  gtk_combo_box_set_active(GTK_COMBO_BOX(myGUI->combobox_devices),selected);

}

//Refreshs Trackerlist combobox in GUI
void Refresh_Tracker_List(void){

  int selected = gtk_combo_box_get_active(myGUI->combobox_trackers);

  GtkTreeIter iter;
  GtkListStore *store = gtk_list_store_new(1,G_TYPE_STRING);
  GtkCellRenderer *cell = gtk_cell_renderer_text_new();
  gtk_list_store_clear(store);
  gtk_cell_layout_clear(GTK_CELL_LAYOUT(myGUI->combobox_trackers));

  int i=0;
  while ( (TrackContList[i].used == 1) && (i < MaxNumDevices) ){
    Fill_Dev_Name(i);
    gtk_list_store_append(store,&iter);

    gtk_list_store_set(store,&iter,0,DevStatus[Get_Dev_Ind(TrackContList[i].tracker_id)].name,-1);
    i +=1;
  }

  gtk_combo_box_set_model((myGUI->combobox_trackers), GTK_TREE_MODEL(store));
  gtk_cell_layout_pack_start(GTK_CELL_LAYOUT(myGUI->combobox_trackers), cell, TRUE);
  gtk_cell_layout_set_attributes( GTK_CELL_LAYOUT( myGUI->combobox_trackers ), cell, "text", 0, NULL );
  gtk_combo_box_set_active(GTK_COMBO_BOX(myGUI->combobox_trackers),selected);

return;
}

//Refreshs Tracker Labels in GUI
void Refresh_Tracker_Labels (void) {

  char buffer[10];
  sprintf(buffer, "%d", SelTrackerID);
  gtk_label_set_text( myGUI->label_tracker_id,buffer);
  char buf[G_ASCII_DTOSTR_BUF_SIZE];
  gtk_label_set_text(myGUI->label_tracker_alt,(g_ascii_dtostr(buf,sizeof (buf),(DevStatus[Get_Dev_Ind(SelTrackerID)].altitude))));
  gtk_label_set_text(myGUI->label_tracker_east,(g_ascii_dtostr(buf,sizeof (buf),(DevStatus[Get_Dev_Ind(SelTrackerID)].longitude))));
  gtk_label_set_text(myGUI->label_tracker_north,(g_ascii_dtostr(buf,sizeof (buf),(DevStatus[Get_Dev_Ind(SelTrackerID)].latitude))));


}

//Refresh Aircraft Labels in GUI
void Refresh_Aircraft_Labels (void) {

  char buffer[10];
  sprintf(buffer, "%d", SelAircraftID);
  gtk_label_set_text( myGUI->label_aircraft_id,buffer);
  char buf[G_ASCII_DTOSTR_BUF_SIZE];

  //Get_Dev_Ind(SelAircraftID)
  gtk_label_set_text(myGUI->label_aircraft_alt,(g_ascii_dtostr(buf,sizeof (buf),(DevStatus[Get_Dev_Ind(SelAircraftID)].altitude))));
  gtk_label_set_text(myGUI->label_aircraft_east,(g_ascii_dtostr(buf,sizeof (buf),(DevStatus[Get_Dev_Ind(SelAircraftID)].longitude))));
  gtk_label_set_text(myGUI->label_aircraft_north,(g_ascii_dtostr(buf,sizeof (buf),(DevStatus[Get_Dev_Ind(SelAircraftID)].latitude))));

}

//Refreshs mode indicators in GUI
void Refresh_Tracker_Mode (void) {

  if ( TrackContList[Get_Trac_Ind(SelTrackerID)].mode == 0 ) {
    gtk_image_set_from_file(myGUI->app_logo,LOGO);
    gtk_switch_set_active(myGUI->GtkSwitch_manuel_mod,FALSE);
  }
  else{
    gtk_image_set_from_file(myGUI->app_logo,MOVING_LOGO);
    gtk_switch_set_active(myGUI->GtkSwitch_manuel_mod,TRUE);
  }

}


void Refresh_Sliders (void) {

	gtk_range_set_value(myGUI->GtkSCale_pan_angle,TrackContList[Get_Trac_Ind(SelTrackerID)].pan_angle);
	gtk_range_set_value(myGUI->GtkSCale_tilt_angle,TrackContList[Get_Trac_Ind(SelTrackerID)].tilt_angle);

}


