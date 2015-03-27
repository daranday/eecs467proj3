#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>

#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/math_util.h"

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "eecs467_util.h"    // This is where a lot of the internals live

#define NUM_SERVOS 6

typedef struct state state_t;
struct state
{
    getopt_t *gopt;

    // vx_stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events

    // LCM
    lcm_t *lcm;
    const char *command_channel;
    const char *status_channel;

    pthread_t status_thread;
    pthread_t command_thread;
};

static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{

    return 0;
}

static int
key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
    //state_t *state = vxeh->impl;
    return 0;
}

static int
touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0; // Does nothing
}


static void
status_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const dynamixel_status_list_t *msg,
                void *user)
{
    state_t *state = user;
    char plot_channel[5] = "bar1";
    // Print out servo positions
    for (int id = 0; id < msg->len; id++) {
        dynamixel_status_t stat = msg->statuses[id];
        printf ("[id %d]=%6.3f ",id, stat.position_radians);
		float bar_scale = 0.125;

		vx_object_t *vxo_square;
		if(msg->statuses[id].position_radians > 0){
		  vxo_square = vxo_chain(vxo_mat_translate3(bar_scale * msg->statuses[id].position_radians - 0.4, bar_scale * id - 0.3, 0),
					 vxo_mat_scale3(bar_scale * msg->statuses[id].position_radians*2.0, bar_scale * 1, 0),
					 vxo_mat_rotate_z(M_PI/2.0),
					 vxo_box(vxo_mesh_style(vx_blue),
						 vxo_lines_style(vx_white,2.0f)));
		}
		else{
		vxo_square = vxo_chain(vxo_mat_translate3(bar_scale * msg->statuses[id].position_radians/2.0 - 0.4, bar_scale * id - 0.3, 0),
				       vxo_mat_scale3(bar_scale * msg->statuses[id].position_radians, bar_scale * 1, 0),
				       vxo_mat_rotate_z(M_PI/2.0),
				       vxo_box(vxo_mesh_style(vx_red),
					       vxo_lines_style(vx_white,2.0f)));
		}			       
					       
        // We add this object to a different buffer so it may be rendered
        // separately if desired
        plot_channel[3] = '0' + id;
	vx_buffer_t *buf = vx_world_get_buffer (state->vxworld, plot_channel);
        vx_buffer_add_back (buf, vxo_square);
	
	char str1[64];
	sprintf(str1, "<<left,#ffffff, serif>>Base\t%f", msg->statuses[0].position_radians);
	vx_object_t *text_base = vxo_chain(vxo_mat_translate3(0.27, -0.3, 0),
				      vxo_mat_scale3(bar_scale*0.025, bar_scale*0.025, 0),
				      vxo_text_create(VXO_TEXT_ANCHOR_CENTER,
						      str1));
	vx_buffer_add_back(buf, text_base);

	char str2[64];
	sprintf(str2, "<<left,#ffffff, serif>>Shoulder\t%f", msg->statuses[1].position_radians);
	vx_object_t *text_shoulder = vxo_chain(vxo_mat_translate3(0.27, -0.175, 0),
				      vxo_mat_scale3(bar_scale*0.025, bar_scale*0.025, 0),
				      vxo_text_create(VXO_TEXT_ANCHOR_CENTER,
						      str2));
	vx_buffer_add_back(buf, text_shoulder);

	char str3[64];
	sprintf(str3, "<<left,#ffffff, serif>>Elbow\t%f", msg->statuses[2].position_radians);
	vx_object_t *text_elbow = vxo_chain(vxo_mat_translate3(0.27, -0.05, 0),
				      vxo_mat_scale3(bar_scale*0.025, bar_scale*0.025, 0),
				      vxo_text_create(VXO_TEXT_ANCHOR_CENTER,
						      str3));
	vx_buffer_add_back(buf, text_elbow);

	char str4[64];
	sprintf(str4, "<<left,#ffffff, serif>>Wrist bend\t%f", msg->statuses[3].position_radians);
	vx_object_t *text_wrist1 = vxo_chain(vxo_mat_translate3(0.27, 0.075, 0),
				      vxo_mat_scale3(bar_scale*0.025, bar_scale*0.025, 0),
				      vxo_text_create(VXO_TEXT_ANCHOR_CENTER,
						      str4));
	vx_buffer_add_back(buf, text_wrist1);

	char str5[64];
	sprintf(str5, "<<left,#ffffff, serif>>Wrist rotate\t%f", msg->statuses[4].position_radians);
	vx_object_t *text_wrist2 = vxo_chain(vxo_mat_translate3(0.27, 0.20, 0),
				      vxo_mat_scale3(bar_scale*0.025, bar_scale*0.025, 0),
				      vxo_text_create(VXO_TEXT_ANCHOR_CENTER,
						      str5));
	vx_buffer_add_back(buf, text_wrist2);

	char str6[64];
	sprintf(str6, "<<left,#ffffff, serif>>Fingers\t%f", msg->statuses[5].position_radians);
	vx_object_t *text_fingers = vxo_chain(vxo_mat_translate3(0.27, 0.32, 0),
				      vxo_mat_scale3(bar_scale*0.025, bar_scale*0.025, 0),
				      vxo_text_create(VXO_TEXT_ANCHOR_CENTER,
						      str6));
	vx_buffer_add_back(buf, text_fingers);
	
	
        vx_buffer_swap (buf);
    }
    printf ("\n");
}

void *
status_loop (void *data)
{
    state_t *state = data;
    dynamixel_status_list_t_subscribe (state->lcm,
                                       state->status_channel,
                                       status_handler,
                                       state);
    const int hz = 15;
    while (1) {
        // Set up the LCM file descriptor for waiting. This lets us monitor it
        // until something is "ready" to happen. In this case, we are ready to
        // receive a message.
        int status = lcm_handle_timeout (state->lcm, 1000/hz);
        if (status <= 0)
            continue;

        // LCM has events ready to be processed
    }

    return NULL;
}

void *
command_loop (void *user)
{
/*    state_t *state = user;
    const int hz = 30;

    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = calloc (NUM_SERVOS, sizeof(dynamixel_command_t));

    while (1) {
        // Send LCM commands to arm. Normally, you would update positions, etc,
        // but here, we will just home the arm.
        for (int id = 0; id < NUM_SERVOS; id++) {
            if (getopt_get_bool (state->gopt, "idle")) {
                cmds.commands[id].utime = utime_now ();
                cmds.commands[id].position_radians = 0.0;
                cmds.commands[id].speed = 0.0;
                cmds.commands[id].max_torque = 0.0;
            }
            else {
                // home servos slowly
                cmds.commands[id].utime = utime_now ();
                cmds.commands[id].position_radians = 0.0;
                cmds.commands[id].speed = 0.05;
                cmds.commands[id].max_torque = 0.35;
            }
        }
        dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);

        usleep (1000000/hz);
    }

    free (cmds.commands);
*/
    return NULL;
}

void
run_gui (vx_application_t *app, int w, int h)
{
    // Creates a GTK window to wrap around our vx display canvas. The vx world
    // is rendered to the canvas widget, which acts as a viewport into your
    // virtual world.
    vx_gtk_display_source_t *appwrap = vx_gtk_display_source_create (app);
    GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget *canvas = vx_gtk_display_source_get_widget (appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window), w, h);

    // Pack a parameter gui and canvas into a vertical box
    GtkWidget *vbox = gtk_vbox_new (0, 0);
    gtk_box_pack_start (GTK_BOX (vbox), canvas, 1, 1, 0);
    gtk_widget_show (canvas);    // XXX Show all causes errors!

    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show (window);
    gtk_widget_show (vbox);

    g_signal_connect_swapped (G_OBJECT (window), "destroy", G_CALLBACK (gtk_main_quit), NULL);

    gtk_main (); // Blocks as long as GTK window is open
    gdk_threads_leave ();

    // destroy function was causing segfault, so comment it out for now
    //vx_gtk_display_source_destroy (appwrap);
}

// This subscribes to the status messages sent out by the arm, displaying servo
// state in the terminal. It also sends messages to the arm ordering it to the
// "home" position (all servos at 0 radians).
int
main (int argc, char *argv[])
{
    eecs467_init (argc, argv);


    getopt_t *gopt = getopt_create ();
    getopt_add_bool (gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_bool (gopt, 'i', "idle", 0, "Command all servos to idle");
    getopt_add_string (gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");
    getopt_add_string (gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");

    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt);
        exit (EXIT_FAILURE);
    }

    
    state_t *state = calloc (1, sizeof(*state));
    state->gopt = gopt;
    state->lcm = lcm_create (NULL);
    state->vxworld = vx_world_create ();
    state->vxeh = calloc (1, sizeof(*state->vxeh));
    state->vxeh->key_event = key_event;
    state->vxeh->mouse_event = mouse_event;
    state->vxeh->touch_event = touch_event;
    state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);
    //    vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "grid"), vxo_grid());
    vx_buffer_swap (vx_world_get_buffer (state->vxworld, "grid"));

    state->command_channel = getopt_get_string (gopt, "command-channel");
    state->status_channel = getopt_get_string (gopt, "status-channel");

    pthread_create (&state->status_thread, NULL, status_loop, state);
    pthread_create (&state->command_thread, NULL, command_loop, state);

    // This is the main loop
    run_gui (&state->vxapp, 1024, 768);

    // Probably not needed, given how this operates
    pthread_join (state->status_thread, NULL);
    pthread_join (state->command_thread, NULL);

    lcm_destroy (state->lcm);
    free (state);
    getopt_destroy (gopt);
}
