#ifndef __A3_PICASSO__
#define __A3_PICASSO__

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>


// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// opencv
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "apps/eecs467_util.h"    // This is where a lot of the internals live

// a2 sources
#include "a3_blob_detector.h"
#include "a3_inverse_kinematics.h"
#include "a3_image_to_arm_coord.h"
#include "a3_ai.h"

//lcm
#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/ttt_turn_t.hpp"

static void my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name);
void convert_vx_to_image_coords(double vx_x, double vx_y, double& image_x, double& image_y);
void convert_image_to_vx_coords(double image_x, double image_y, double& vx_x, double& vx_y);
void make_move(double ball_x, double ball_y, int idx_i, int idx_j);
static int mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse);
static int key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key);
static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse);
void * animate_thread (void *data);
void* start_vx(void* user);
void* start_inverse_kinematics(void* user);
void get_coordinates_from_joints(double& x, double& y);
void get_camera_to_arm(double camera_x, double camera_y, double& x, double& y);
void render_blob(string shape, double x, double y, const float* color);
void draw_axes();
void thresh_callback(int, void*);
void draw(int threshold);

struct state_t {
	bool running;

	getopt_t        *gopt;
	parameter_gui_t *pg;

	// image stuff
	char *img_url;
	int   img_height;
	int   img_width;

	// vx stuff
	vx_application_t    vxapp;
	vx_world_t         *vxworld;      // where vx objects are live
	vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
	vx_mouse_event_t    last_mouse_event;

	// threads
	pthread_t animate_thread;

	// for accessing the arrays
	pthread_mutex_t mutex;
	int argc;
	char **argv;

	bool use_cached_bbox_colors;
	bool use_cached_calibration;
	image_u32_t* current_image;

	coord_convert converter;
	bool converter_initialized;

	double origin_x;
	double origin_y;
	double interval_x;
	double interval_y;

	bool opencv_initialized;

	state_t() : current_image(NULL), converter_initialized(false), opencv_initialized(false) {
		use_cached_bbox_colors = (ifstream("Bbox_Colors.txt")) ? true : false;
		use_cached_calibration = (ifstream("ConversionMatrices.txt")) ? true : false;
		interval_x = 0.06; interval_y = 0.06;
		origin_x = 0; origin_y = 0.13;
		vxworld = vx_world_create ();
		vxeh = (vx_event_handler_t*)calloc (1, sizeof(*vxeh));
		vxeh->key_event = key_event;
		vxeh->mouse_event = mouse_event;
		vxeh->touch_event = touch_event;
		vxeh->dispatch_order = 100;
		vxapp.display_started = eecs467_default_display_started;
		vxapp.display_finished = eecs467_default_display_finished;
		vxapp.impl = eecs467_default_implementation_create (vxworld, vxeh);

		running = 1;
	}

	~state_t() {
		if (vxeh)
			free (vxeh);

		if (gopt)
			getopt_destroy (gopt);

		if (pg)
			pg_destroy (pg);
		}
};

struct DrawBot {

	// double drawH = 0.11;
	double drawH;
	double hoverH;



	void axes();
	void draw();
	void basic_shape(std::string& shape);
};

extern DrawBot Picasso;

#endif