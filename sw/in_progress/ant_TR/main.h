#include <gtk/gtk.h>

#define BUILDER_XML_FILE "ant_TR.xml"

static char *start_mod;

typedef struct
{
	GtkBuilder			*builder;
        GtkWidget			*window;
        GtkSpinner			*spinner;
        GtkRange			*GtkSCale_pan_angle;
        GtkRange			*GtkSCale_tilt_angle;
        GtkSwitch			*GtkSwitch_manuel_mod;
        GtkStatusbar			*statusbar;

        GtkComboBox			*combobox_devices;
        GtkComboBox			*combobox_trackers;

        GtkLabel			*label;
	GtkLabel			*label_aircraft_east;
	GtkLabel			*label_aircraft_north;
	GtkLabel			*label_aircraft_alt;
	GtkLabel			*label_tracker_east;
	GtkLabel			*label_tracker_north;
	GtkLabel			*label_tracker_alt;
	GtkLabel			*label_aircraft_id;
	GtkLabel			*label_tracker_id;
	GtkLabel			*label_tracker_pitch_angle;
	GtkLabel			*label_tracker_roll_angle;
	GtkImage			*app_logo;
        guint				statusbar_context_id;
        gchar				*filename;
}myGTK;

myGTK *myGUI;



void show_err_msg (char *err_msg_text,int rr);


// bound_ui_items() function bounds myGTK object items before entering gtk loop
void bound_ui_items(void) {

	//Pre functions
	(myGUI->builder) = gtk_builder_new ();
        gtk_builder_add_from_file (myGUI->builder, BUILDER_XML_FILE , NULL);

		//Bounding ui items
        myGUI->window = GTK_WIDGET (gtk_builder_get_object (myGUI->builder, "window_ant_track"));

        myGUI->spinner = GTK_SPINNER (gtk_builder_get_object (myGUI->builder, "spinner_processing"));

        myGUI->label = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label2"));
        myGUI->label_aircraft_east = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_aircraft_east"));
        myGUI->label_aircraft_north = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_aircraft_north"));
        myGUI->label_aircraft_alt = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_aircraft_alt"));
        myGUI->label_tracker_east = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_tracker_east"));
        myGUI->label_tracker_north = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_tracker_north"));
        myGUI->label_tracker_alt = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_tracker_alt"));
        myGUI->label_tracker_id = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_tracker_id"));
        myGUI->label_aircraft_id = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_aircraft_id"));
        myGUI->label_tracker_pitch_angle = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_tracker_pitch_angle"));
        myGUI->label_tracker_roll_angle = GTK_LABEL (gtk_builder_get_object (myGUI->builder, "label_tracker_roll_angle"));

        myGUI->GtkSCale_pan_angle = GTK_RANGE (gtk_builder_get_object (myGUI->builder, "GtkSCale_pan_angle"));
        myGUI->GtkSCale_tilt_angle = GTK_RANGE (gtk_builder_get_object (myGUI->builder, "GtkSCale_tilt_angle"));

        myGUI->GtkSwitch_manuel_mod = GTK_SWITCH (gtk_builder_get_object (myGUI->builder, "switch_manuel_mod"));

        myGUI->statusbar = GTK_STATUSBAR(gtk_builder_get_object (myGUI->builder, "statusbar_ant_TR"));

        //combobox_aircrafts
        myGUI->combobox_devices = GTK_COMBO_BOX (gtk_builder_get_object (myGUI->builder, "combobox_devices"));
        myGUI->combobox_trackers = GTK_COMBO_BOX (gtk_builder_get_object (myGUI->builder, "combobox_trackers"));
        //myGUI->aircrafts_liststore= GTK_LIST_STORE (gtk_builder_get_object (myGUI->builder, "liststore_aircrafts"));
	myGUI->app_logo=GTK_IMAGE (gtk_builder_get_object (myGUI->builder, "image_logo"));


        //Connecting UI signals
        gtk_builder_connect_signals (myGUI->builder, NULL);

        g_object_unref (G_OBJECT (myGUI->builder));
}

//Function to check system variables & dependancy..vs
int system_check(void) {
		if (gtk_check_version (3, 0, 0) != NULL)
        {
                g_warning ("You need to install GTK+ 3.00 or newer!");
                return 1;
        }
        return 0;
}

void tepki_ver (void){
//
		gtk_spinner_start (myGUI->spinner);
		gtk_label_set_text(myGUI->label,"Tepkiliyim!");
		IvySendMsg("Ivyde Tepkiliyim!");
		gtk_range_set_value(myGUI->GtkSCale_pan_angle,99);
		gtk_statusbar_push(myGUI->statusbar,0,"<<<<<< Tepkiliyim! >>>>>>");
		//show_err_msg("Pop up mesajdan da tepkiliyim!! ");

}

void show_err_msg (char *err_msg_text, int rr){

//Function to ease error message showing in GUI

		GtkWidget               *dialog;
		dialog = gtk_message_dialog_new (NULL,
                GTK_DIALOG_DESTROY_WITH_PARENT,
                                  GTK_MESSAGE_INFO,
                                  GTK_BUTTONS_CLOSE,
                                  "%s %d",err_msg_text,rr);
		gtk_dialog_run (GTK_DIALOG (dialog));
		gtk_widget_destroy (dialog);

}








