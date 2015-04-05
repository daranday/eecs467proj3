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

#endif