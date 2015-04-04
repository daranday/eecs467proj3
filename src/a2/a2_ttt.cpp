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
#include "a2_blob_detector.h"
#include "a2_inverse_kinematics.h"
#include "a2_image_to_arm_coord.h"
#include "a2_ai.h"

//lcm
#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/ttt_turn_t.hpp"

using namespace std;

//LCM setup
pthread_mutex_t md;
lcm::LCM lcm_inst;

void *lcm_comm(void *input){
  while(true){
    lcm_inst.handle();
  }
  return NULL;
}

struct move_t{
  int64_t utime;
  int32_t turn;

  move_t(){
    turn = 0;
    utime = 0;
  }
  
};


struct comms{
  move_t current;
  bool c_move;
  bool is_red;
  int our_turn;

  comms(){
    c_move = false;
    our_turn = 0;
  }

  void ttt_turn_handler(const lcm::ReceiveBuffer* rbuf, const string &channel, const ttt_turn_t* msg){
    pthread_mutex_lock(&md);
    current.utime = msg->utime;
    current.turn = msg->turn;
    pthread_mutex_unlock(&md);
    can_move();
  }

  void can_move(){
    pthread_mutex_lock(&md);
    if(is_red){
        if(our_turn == current.turn){
            c_move = true;
        }
        else if(our_turn > current.turn)
            c_move= false;
    }
    else{
        if(our_turn == current.turn)
            c_move = false;
        else if(our_turn < current.turn){
            c_move = true;
        }
    }
    pthread_mutex_unlock(&md);
  }

  bool wait_turn(){
    bool ret;
    pthread_mutex_lock(&md);
    ret = c_move;
    c_move = false;
    pthread_mutex_unlock(&md);

    return ret;

  }
  
};


// void wait_turn() {
//     usleep(10000000);
// }

// It's good form for every application to keep its state in a struct.
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

    int balls_placed;

    double origin_x;
    double origin_y;
    double interval_x;
    double interval_y;

    state_t() : current_image(NULL), converter_initialized(false), balls_placed(0) {
        use_cached_bbox_colors = (ifstream("Bbox_Colors.txt")) ? true : false;
        use_cached_calibration = (ifstream("ConversionMatrices.txt")) ? true : false;
        interval_x = 0.06; interval_y = 0.06;
        origin_x = 0; origin_y = 0.15;
    }
} state_obj;

state_t *state = &state_obj;

// === Parameter listener =================================================
// This function is handed to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// state, etc if need be.
static void
my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    if (0==strcmp ("sl1", name))
        printf ("sl1 = %f\n", pg_gd (pg, name));
    else if (0==strcmp ("sl2", name))
        printf ("sl2 = %d\n", pg_gi (pg, name));
    else if (0==strcmp ("cb1", name) || 0==strcmp ("cb2", name))
        printf ("%s = %d\n", name, pg_gb (pg, name));
    else
        printf ("%s changed\n", name);
}

void convert_vx_to_image_coords(double vx_x, double vx_y, double& image_x, double& image_y) {
    image_x = round((vx_x + 1) * state->img_width / 2. + 0.5);
    image_y = state->img_height - round((vx_y + state->img_height / (state->img_width * 1.)) * state->img_width / 2. + 0.5);
    // printf("Input vx_x = %g, vx_y = %g, Coords vx_xx = %d, y = %d\n", vx_x, vx_y, (int)image_x, (int)image_y);
}

void convert_image_to_vx_coords(double image_x, double image_y, double& vx_x, double& vx_y) {
    vx_x = ((image_x - 0.5)*2/state->img_width) - 1;
    vx_y = 2*(state->img_height - image_y - 0.5)/state->img_width - state->img_height/state->img_width;
    // cout << "----------" << vx_x << ", " << vx_y << endl;
}

void make_move(double ball_x, double ball_y, int idx_i, int idx_j) {
    printf("Moving piece to Position (%d, %d)\n", idx_i, idx_j);
    move_to(ball_x, ball_y, 0.13);
    arm_fetch();
    move_to(-(idx_i) * state->interval_x + state->origin_x, (idx_j) * state->interval_y + state->origin_y, 0.13);
    arm_drop();
    stand_arm();
}

static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    // vx_camera_pos_t contains camera location, field of view, etc
    // vx_mouse_event_t contains scroll, x/y, and button click events

    if ((mouse->button_mask & VX_BUTTON1_MASK) &&
        !(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {

        vx_ray3_t ray;
        vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

        double ground[3];
        vx_ray3_intersect_xy (&ray, 0, ground);

        double camera_vector[3] = {0,0,1};
        convert_vx_to_image_coords(ground[0], ground[1], camera_vector[0], camera_vector[1]);

        if (state->converter_initialized) {
            double arm_vector[3] = {0,0,1};
            state->converter.camera_to_arm(arm_vector, camera_vector);
            printf("Camera coords: %g, %g\nArm coords: %g, %g", camera_vector[0], camera_vector[1], arm_vector[0], arm_vector[1]);

            //int a;

            make_move(arm_vector[0], arm_vector[1], state->balls_placed % 3, state->balls_placed / 3);
            state->balls_placed++;
        }
        // printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Camera coordinates clicked at: [%6.3f, %6.3f]\n",
        //         mouse->x, mouse->y, camera_vector[0], camera_vector[1]);
        // printf("Height: %d, Width: %d\n", state->img_height, state->img_width);
    }

    // store previous mouse event to see if the user *just* clicked or released
    state->last_mouse_event = *mouse;

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

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by state
void *
animate_thread (void *data)
{
    const int fps = 60;

    // Set up the imagesource
    image_source_t *isrc = image_source_open (state->img_url);

    if (isrc == NULL)
        printf ("Error opening device.\n");
    else {
        // Print out possible formats. If no format was specified in the
        // url, then format 0 is picked by default.
        // e.g. of setting the format parameter to format 2:
        //
        // --url=dc1394://bd91098db0as9?fidx=2
        for (int i = 0; i < isrc->num_formats (isrc); i++) {
            image_source_format_t ifmt;
            isrc->get_format (isrc, i, &ifmt);
            printf ("%3d: %4d x %4d (%s)\n",
                    i, ifmt.width, ifmt.height, ifmt.format);
        }
        isrc->start (isrc);
    }

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) {

        // Get the most recent camera frame and render it to screen.
        if (isrc != NULL) {
            image_source_data_t *frmd = (image_source_data_t *)calloc (1, sizeof(*frmd));
            int res = isrc->get_frame (isrc, frmd);
            if (res < 0)
                printf ("get_frame fail: %d\n", res);
            else {
                // Handle frame
                pthread_mutex_lock(&state->mutex);    
                if (state->current_image != NULL) {
                    image_u32_destroy (state->current_image);
                    state->current_image = NULL;
                }
                state->current_image = image_convert_u32 (frmd);
                image_u32_t *im = state->current_image;
                if (im != NULL) {
                    vx_object_t *vim = vxo_image_from_u32(im,
                                                          VXO_IMAGE_FLIPY,
                                                          VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

                    // render the image centered at the origin and at a normalized scale of +/-1 unit in x-dir
                    const double scale = 2./im->width;
                    vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "image"),
                                        vxo_chain (vxo_mat_scale3 (scale, scale, 1.0),
                                                   vxo_mat_translate3 (-im->width/2., -im->height/2., 0.),
                                                   vim));
                    vx_buffer_swap (vx_world_get_buffer (state->vxworld, "image"));
                    state->img_height = im->height;
                    state->img_width = im->width;
                    // image_u32_destroy (im);
                }
                pthread_mutex_unlock(&state->mutex);                    
            }
            fflush (stdout);
            isrc->release_frame (isrc, frmd);
        }


        // Now, we update both buffers
        // vx_buffer_swap (vx_world_get_buffer (state->vxworld, "rot-sphere"));
        // vx_buffer_swap (vx_world_get_buffer (state->vxworld, "osc-square"));
        // vx_buffer_swap (vx_world_get_buffer (state->vxworld, "axes"));

        usleep (1000000/fps);
    }

    if (isrc != NULL)
        isrc->stop (isrc);

    return NULL;
}

state_t *
state_create (void)
{
    state->vxworld = vx_world_create ();
    state->vxeh = (vx_event_handler_t*)calloc (1, sizeof(*state->vxeh));
    state->vxeh->key_event = key_event;
    state->vxeh->mouse_event = mouse_event;
    state->vxeh->touch_event = touch_event;
    state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);

    state->running = 1;

    return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

    if (state->vxeh)
        free (state->vxeh);

    if (state->gopt)
        getopt_destroy (state->gopt);

    if (state->pg)
        pg_destroy (state->pg);
}

// This is intended to give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.

void* start_vx(void* user) {
    int argc = state->argc;
    char **argv = state->argv;

    eecs467_init (argc, argv);
    state_t *state = state_create ();

    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_string (state->gopt, '\0', "url", "", "Camera URL");
    getopt_add_bool   (state->gopt,  'l', "list", 0, "Lists available camera URLs and exit");
    getopt_add_bool   (state->gopt, 'r', "isred", 0, "red balls"); 

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }

    // Set up the imagesource. This looks for a camera url specified on
    // the command line and, if none is found, enumerates a list of all
    // cameras imagesource can find and picks the first url it finds.
    if (strncmp (getopt_get_string (state->gopt, "url"), "", 1)) {
        state->img_url = strdup (getopt_get_string (state->gopt, "url"));
        printf ("URL: %s\n", state->img_url);
    }
    else {
        // No URL specified. Show all available and then use the first
        zarray_t *urls = image_source_enumerate ();
        printf ("Cameras:\n");
        for (int i = 0; i < zarray_size (urls); i++) {
            char *url;
            zarray_get (urls, i, &url);
            printf ("  %3d: %s\n", i, url);
        }

        if (0==zarray_size (urls)) {
            printf ("Found no cameras.\n");
            return NULL;
        }

        zarray_get (urls, 0, &state->img_url);
    }

    if (getopt_get_bool (state->gopt, "list")) {
        state_destroy (state);
        exit (EXIT_SUCCESS);
    }

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    // Initialize a parameter gui
    state->pg = pg_create ();
    pg_add_double_slider (state->pg, "sl1", "Slider 1", 0, 100, 50);
    pg_add_int_slider    (state->pg, "sl2", "Slider 2", 0, 100, 25);
    pg_add_check_boxes (state->pg,
                        "cb1", "Check Box 1", 0,
                        "cb2", "Check Box 2", 1,
                        NULL);
    pg_add_buttons (state->pg,
                    "but1", "Button 1",
                    "but2", "Button 2",
                    "but3", "Button 3",
                    NULL);

    parameter_listener_t *my_listener = (parameter_listener_t*)calloc (1, sizeof(*my_listener));
    my_listener->impl = state;
    my_listener->param_changed =    my_param_changed;
    pg_add_listener (state->pg, my_listener);

    // Launch our worker threads
    pthread_create (&state->animate_thread, NULL, animate_thread, state);

    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, 1024, 768);

    // Quit when GTK closes
    state->running = 0;
    pthread_join (state->animate_thread, NULL);

    // Cleanup
    free (my_listener);
    state_destroy (state);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();

    return NULL;
}

void read_bbox_and_colors(vector<int>& bbox, vector<vector<double> >& hsv_ranges) {
    // use cached values
    if (state->use_cached_bbox_colors) {
        ifstream fin("Bbox_Colors.txt");
        for (int i = 0; i < 4; ++i) {
            fin >> bbox[i];
        }
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                fin >> hsv_ranges[i][j * 2] >> hsv_ranges[i][j * 2 + 1];
            }
        }
        fin.close();
    } else {
        // save these values to cache
        ofstream fout("Bbox_Colors.txt");

        //read mask
        system("./bin/a2_mask");
        ifstream fmask("Mask.txt");
        for (int i = 0; i < 4; ++i) {
            fmask >> bbox[i];
            fout << bbox[i] << " ";
        }
        fmask.close();
        cout << endl;
        fout << endl;

        // read colors for blue corner squares, our color, their color
        for (int i = 0; i < 3; ++i) {
            system("./bin/a2_color_picker");
            ifstream fhsv("HsvRange.txt");
            for (int j = 0; j < 3; ++j) {
                fhsv >> hsv_ranges[i][j * 2] >> hsv_ranges[i][j * 2 + 1];
                fout << hsv_ranges[i][j * 2] << " " << hsv_ranges[i][j * 2 + 1] << " ";
            }
            fout << endl;
            fhsv.close();
        }
        fout.close();
    }

    //output input values
    cout << "\n\nBBox Coordinates:" << endl;
    for (int i = 0; i < 4; ++i)
        cout << bbox[i] << " ";
    cout << "\nHSV colors:" << endl;
    vector<string> color_types = {"Blue Corner Squares", "Our Color", "Opponent Color"};
    for (int i = 0; i < 3; ++i) {
        cout << color_types[i] << ": " << endl;
        for (int j = 0; j < 3; ++j) {
            cout << hsv_ranges[i][j * 2] << " " << hsv_ranges[i][j * 2 + 1] << " ";
        }
        cout << endl;
    }
    cout << "\n\n" << endl;
}

bool is_color(r_data& hsv, vector<double>& color_range) {
    if (color_range[0] < hsv.H && hsv.H < color_range[1]
        && color_range[2] < hsv.S && hsv.S < color_range[3]
        && color_range[4] < hsv.V && hsv.V < color_range[5])
        return true;
    return false;
}

void* start_inverse_kinematics(void* user) {
    kin_main(state->argc, state->argv);
    return NULL;
}

void get_coordinates_from_joints(double& x, double& y) {
    double R = 0;
    vector<double> arm_length = {0.118, 0.1, 0.0985, 0.099}; // change this accordingly
    
    R = arm_length[1] * sin(kin_state->arm_joints[1]) + arm_length[2] * sin(kin_state->arm_joints[1] + kin_state->arm_joints[2]);
    R += arm_length[3] * sin(kin_state->arm_joints[1] + kin_state->arm_joints[2] + kin_state->arm_joints[3]);

    x = R * sin(kin_state->arm_joints[0]);
    y = - R * cos(kin_state->arm_joints[0]);
}

void read_matrices(double Amat[], double Cmat[]) {
    ifstream fin("ConversionMatrices.txt");
    for (int i = 0; i < 9; ++i) {
        fin >> Cmat[i];
    }
    for (int i = 0; i < 9; ++i) {
        fin >> Amat[i];
    }
    fin.close();
}

void save_matrices(double Amat[], double Cmat[]) {
    ofstream fout("ConversionMatrices.txt");
    for (int i = 0; i < 9; ++i) {
        fout << Cmat[i] << " ";
        if ((i - 2) % 3 == 0)
            fout << endl;
    }
    for (int i = 0; i < 9; ++i) {
        fout << Amat[i] << " ";
        if ((i - 2) % 3 == 0)
            fout << endl;
    }
    fout.close();
}

void get_camera_to_arm(double camera_x, double camera_y, double& x, double& y) {
    double camera_vector[3] = {camera_x, camera_y, 1};
    double arm_vector[3] = {-69, -69, -69};
    state->converter.camera_to_arm(arm_vector, camera_vector);
    x = arm_vector[0];
    y = arm_vector[1];
}

void get_arm_to_camera(double x, double y, double& camera_x, double& camera_y) {
    double camera_vector[3] = {0, 0, 1};
    double arm_vector[3] = {x, y, 1};
    state->converter.arm_to_camera(camera_vector, arm_vector);
    camera_x = camera_vector[0];
    camera_y = camera_vector[1];
}

// void calibrate_ttt_grid(vector<double>& blue_squares) {
//     double x_origin, y_origin, x_interval, y_interval;
//     double blue_side = 0;
//     for (int i = 0; i < 4; ++i)
//         blue_side += blue_squares[i * 3 + 2];

//     blue_side = sqrt(blue_side / 4.0);

//     double dx = blue_squares[4] - blue_squares[0], dy = blue_squares[5] - blue_squares[1];
//     double grid_side = sqrt(pow(dx, 2) + pow(dy, 2));
//     dx /= grid_side; dy /= grid_side;

//     double image_origin_x = blue_squares[0] + blue_side / 2.0  * dx;
//     double image_origin_y = blue_squares[1] + blue_side / 2.0  * dy;

//     cout << "Before Calibrate Origins: " << state->origin_x << ", " << state->origin_y << endl;
//     printf("Blue side = %g, grid_side = %g\n", blue_side, grid_side);
//     get_camera_to_arm(image_origin_x, image_origin_y, state->origin_x, state->origin_y);
//     cout << "After Calibrate Origins: " << state->origin_x << ", " << state->origin_y << endl;
//     exit(0);
// }

void calibrate_coordinate_converter(blob_detect &B, vector<int>& bbox, vector<vector<double> >& hsv_ranges) {
    cout << "Calibrating..." << endl;

    pthread_mutex_lock(&state->mutex);
    B.run( state->current_image);
    pthread_mutex_unlock(&state->mutex);


    double Cmat[9] = {  0, 0, 0,
                        0, 0, 0,
                        1, 1, 1};
    double Amat[9] = {  0.12, -0.12, 0.12,
                        0, 0, 0.24,
                        1, 1, 1};

    vector<double> blue_squares(12);

    if (state->use_cached_calibration) {
        cout << "Using cached calibration" << endl;
        read_matrices(Amat, Cmat);
    } else {
        cout << "Reading Camera Points..." << endl;
        printf("Detected %d blobs\n", int(B.region_data.size()));
        printf("Blue color HSV range is: [%g, %g, %g, %g, %g, %g]\n", hsv_ranges[0][0], hsv_ranges[0][1], hsv_ranges[0][2], hsv_ranges[0][3], hsv_ranges[0][4], hsv_ranges[0][5]);
        vector<int> blue_blobs_order = {0, 0, 0, 0};
        // showing blue square coordinates
        for(size_t i = 0; i < B.region_data.size(); i++){
            if(B.region_data[i].area > 100 && is_color(B.region_data[i], hsv_ranges[0])){
                printf("Blue square %d -- x: %g, y: %g, area: %d\n", int(i), B.region_data[i].x, B.region_data[i].y, B.region_data[i].area);
            }
        }
        cout << "Please indicate the blue blobs order in letter Z order, -1 for blob not used: " << endl;
        cin >> blue_blobs_order[0] >> blue_blobs_order[1] >> blue_blobs_order[2] >> blue_blobs_order[3];


        for(size_t i = 0, j = 0; i < B.region_data.size(); i++){
            // printf("Blob %d, color: [%g, %g, %g], x: %g, y: %g\n", i, 
            //                             B.region_data[i].H, B.region_data[i].S, B.region_data[i].V,
            //                             B.region_data[i].x, B.region_data[i].y);
            if(B.region_data[i].area > 100 && is_color(B.region_data[i], hsv_ranges[0])){
                if (blue_blobs_order[j] != -1) {
                    Cmat[blue_blobs_order[j]] = B.region_data[i].x;
                    Cmat[blue_blobs_order[j]+3] = B.region_data[i].y;
                }
                blue_squares[blue_blobs_order[j == -1 ? 3 : j] * 3] = B.region_data[i].x;
                blue_squares[blue_blobs_order[j == -1 ? 3 : j] * 3 + 1] = B.region_data[i].y;
                blue_squares[blue_blobs_order[j == -1 ? 3 : j] * 3 + 2] = B.region_data[i].area;

                j++;
                printf("Blue square %d -- x: %g, y: %g, area: %d\n", int(i), B.region_data[i].x, B.region_data[i].y, B.region_data[i].area);
            }
        }

        cout << "Reading Arm Points..." << endl;
        cout << "Please move arm tip to blue blobs" << endl;
        relax_arm();
        for (int i = 0; i < 3; ++i) {
            string garbage;
            cout << "Enter anything: ";
            cin >> garbage;
            cout << "Joint angles: ";
            for (int j = 0; j < kin_state->arm_joints.size(); ++j) {
                cout << kin_state->arm_joints[j] << ", ";
            }
            cout << endl;

            double x, y;
            get_coordinates_from_joints(x, y);
            printf("X: %g, Y: %g\n", x, y);
            // Amat[i] = x;
            // Amat[i+3] = y;
        }
        save_matrices(Amat, Cmat);
    }
    
    state->converter.c2a_get_factors(Amat, Cmat);

    // test if conversion works:
    double test_c[3] = {Cmat[0], Cmat[3], 1};
    double test_a[3] = {-69, -69, -69};
    state->converter.camera_to_arm(test_a, test_c);
    printf("The first arm coordinate should be %g, %g\n", test_a[0], test_a[1]);
    state->converter_initialized = true;
    // calibrate_ttt_grid(blue_squares);
}


void render_blob(string shape, double x, double y, const float* color) {
    char buffer_name[20] = "Debug_Buffer";

    vx_buffer_add_back (vx_world_get_buffer(state->vxworld, buffer_name),
                                vxo_chain(
                                         vxo_mat_translate3(x, y - (1.0 * state->img_height)/state->img_width, 0.00001),
                                         vxo_mat_scale(0.01),
                                         (shape == "circle" ? vxo_circle(vxo_mesh_style(color))
                                                            : vxo_box(vxo_mesh_style(color)))
                                
                                 ));
}

void get_board_state(string& board_state, blob_detect& B, vector<vector<double> >& hsv_ranges) {
    board_state = ".........";
    pthread_mutex_lock(&state->mutex);
    B.run( state->current_image);
    pthread_mutex_unlock(&state->mutex);


    double xx, yy, xxx, yyy;
    get_arm_to_camera( state->origin_x + 0.5 * state->interval_x, state->origin_y - 0.5 * state->interval_y, xx, yy);
    convert_image_to_vx_coords(xx, yy, xxx, yyy);
    render_blob("box", xxx, yyy, vx_purple);
    get_arm_to_camera( state->origin_x - 2.5 * state->interval_x, state->origin_y - 0.5 * state->interval_y, xx, yy);
    convert_image_to_vx_coords(xx, yy, xxx, yyy);
    render_blob("box", xxx, yyy, vx_yellow);
    get_arm_to_camera( state->origin_x + 0.5 * state->interval_x, state->origin_y + 2.5 * state->interval_y, xx, yy);
    convert_image_to_vx_coords(xx, yy, xxx, yyy);
    render_blob("box", xxx, yyy, vx_red);
    get_arm_to_camera( state->origin_x - 2.5 * state->interval_x, state->origin_y + 2.5 * state->interval_y, xx, yy);
    convert_image_to_vx_coords(xx, yy, xxx, yyy);
    render_blob("box", xxx, yyy, vx_green);


    for(size_t i = 0, j = 0; i < B.region_data.size(); i++){
        // my pieces
        double x, y;
        double vx_x, vx_y;
        get_camera_to_arm(B.region_data[i].x, B.region_data[i].y, x, y);
        convert_image_to_vx_coords(B.region_data[i].x, B.region_data[i].y, vx_x, vx_y);
        
        if(B.region_data[i].area < 100)
            continue; 
        // printf("%d, Area: %d, Label: %d, Image x = %g, y = %g. x = %g, y = %g\n", i, B.region_data[i].area, B.region_data[i].label, B.region_data[i].x, B.region_data[i].y, x, y);


        if (state->origin_x - 2.5 * state->interval_x <= x && x <= state->origin_x + 0.5 * state->interval_x && 
            state->origin_y - 0.5 * state->interval_y <= y && y <= state->origin_y + 2.5 * state->interval_y) {
            int idx_i = int((state->origin_x + 0.5 * state->interval_x - x) / state->interval_x);
            int idx_j = int((y - (state->origin_y - 0.5 * state->interval_y)) / state->interval_y);
            printf("---%d, Area: %d, Label: %d, Image x = %g, y = %g. Index: x = %d, y = %d ", i, B.region_data[i].area, B.region_data[i].label, B.region_data[i].x, B.region_data[i].y, idx_i, idx_j);

            if (B.region_data[i].label == 1) {
                cout << "Is R" << endl;
                board_state[idx_i + 3 * idx_j] = 'R';
                render_blob("circle", vx_x, vx_y, vx_orange);
            } else if (B.region_data[i].label == 2) {
                cout << "Is G" << endl;
                board_state[idx_i + 3 * idx_j] = 'G';
                render_blob("circle", vx_x, vx_y, vx_green);
            }
        }
    }
    vx_buffer_swap (vx_world_get_buffer (state->vxworld, "Debug_Buffer"));
}

int find_free_piece(blob_detect& B, double& ball_x, double& ball_y) {
    int ret = -1;
    for(size_t i = 0, j = 0; i < B.region_data.size(); i++){
        // my pieces
        double x, y;
        double vx_x, vx_y;
        get_camera_to_arm(B.region_data[i].x, B.region_data[i].y, x, y);
        convert_image_to_vx_coords(B.region_data[i].x, B.region_data[i].y, vx_x, vx_y);
        
        if(B.region_data[i].area < 100)
            continue; 
        // printf("%d, Area: %d, Label: %d, Image x = %g, y = %g. x = %g, y = %g\n", i, B.region_data[i].area, B.region_data[i].label, B.region_data[i].x, B.region_data[i].y, x, y);


        if (!(state->origin_x - 2.5 * state->interval_x <= x && x <= state->origin_x + 0.5 * state->interval_x && 
            state->origin_y - 0.5 * state->interval_y <= y && y <= state->origin_y + 2.5 * state->interval_y)) {
            if (B.region_data[i].label == 1) {
                // cout << "Is R" << endl;
                // board_state[idx_i + 3 * idx_j] = 'R';
                render_blob("circle", vx_x, vx_y, vx_orange);
                ball_x = x;
                ball_y = y;
                ret = 0;
            } else if (B.region_data[i].label == 2) {
                // cout << "Is G" << endl;
                // board_state[idx_i + 3 * idx_j] = 'G';
                render_blob("circle", vx_x, vx_y, vx_green);
            }
        }
    }
    return ret;
}

bool manual_wait_turn() {
    string y;
    cout << "Our turn? ";
    cin >> y;
    cin.ignore(1);
    return true;
}

void draw_axes() {
    // draw x axis
    double x0 = state->origin_x, y0 = state->origin_y;
    double h = 0.1025;
    int iters = 50, wait_t = 50000;

    double x = x0 - state->interval_x, y = y0;
    move_to(x, y, h + 0.02);

    double dx = 0.1 / iters, dy = 0.1 / iters;
    for (int i = 0; i < iters; ++i) {
        x += dx;
        move_to(x, y, h);
        // usleep(wait_t);
    }
    
    move_to(x, y, h + 0.02);
    x = x0; y = y0 - state->interval_y / 2;
    move_to(x, y, h + 0.02);

    for (int i = 0; i < iters; ++i) {
        y += dy;
        move_to(x, y, h);
        // usleep(wait_t);
    }

    move_to(x0, y0, h + 0.02);
}

int main (int argc, char *argv[])
{
    //read bounding box and color hsv ranges
    //hsv_ranges example: [*blue squares color: [h_min, h_max, s_min, s_max, v_min, v_max], 
    //                      *our color [...], 
    //                      *opponent color [...]]
    vector<int> bbox(4);
    vector<vector<double> > hsv_ranges(3, vector<double>(6));
    string player_type;
    string player_color;
    
    cout << "\n============ PROGRAM BEGIN =============\n";

    read_bbox_and_colors(bbox, hsv_ranges);
    //start vx
    state->argc = argc;
    state->argv = argv;
    pthread_t vx_thread, kinematics_thread;
    pthread_create (&kinematics_thread, NULL, start_inverse_kinematics, (void*)NULL);


    usleep(1000000);

    cout << "\n============ MAIN PROGRAM RUNNING =============\n";

    double pi = 3.1415926;

    double x0 = state->origin_x, y0 = state->origin_y;
    double x = x0, y = y0;
    double h = 0.1025;
    int wait_t = 100000;
    
    // draw_axes();
    
    while(1) {
        double a, b, c;
        string cmd;
        
        cout << "command > ";
        cin >> cmd;


        if (cmd == "mv") {
            cin >> a >> b >> c;
            // move_to(-(a) * state->interval_x + state->origin_x, (b) * state->interval_y + state->origin_y, c);
            move_to(a, b, c);
        } else if (cmd == "square") {
            int iters = 50;
            x = x0; y = y0;

            move_to(x, y, h + 0.02);

            double dx = 0.05 / iters, dy = 0.05 / iters;
            for (int i = 0; i < iters; ++i) {
                y += dy;
                move_to(x, y, h);
                usleep(wait_t);
            }
            for (int i = 0; i < iters; ++i) {
                x -= dx;
                move_to(x, y, h);
                usleep(wait_t);
            }
            for (int i = 0; i < iters; ++i) {
                y -= dy;
                move_to(x, y, h);
                usleep(wait_t);
            }
            for (int i = 0; i < iters; ++i) {
                x += dx;
                move_to(x, y, h);
                usleep(wait_t);
            }
            stand_arm();

        } else if (cmd == "circle") {
            double R = 0.03;
            int iters = 20;
            cin >> iters;
            double dtheta = 2 * pi / iters, theta = 0;

            move_to(x0 + R, y0, h + 0.02);

            for (int i = 0; i < iters; ++i) {
                x = x0 + R * cos(theta + i * dtheta);
                y = y0 + R * sin(theta + i * dtheta);
                move_to(x, y, h);
                // usleep(wait_t);
            }
            stand_arm();

        // } else if (cmd == "")
        } else if (cmd == "sine") {
            double R = 0.03;
            int iters = 20;
            cin >> iters;
            double dtheta = 4 * pi / iters, theta = 0;

            move_to(x0, y0, h + 0.02);

            for (int i = 0; i < iters; ++i) {
                x = x0 + 0.08 / (4 * pi) * (i * dtheta);
                y = y0 + 0.045 * sin(i * dtheta);
                move_to(x, y, h);
                usleep(wait_t);
            }
            stand_arm();
        // } else if (cmd == "")
        } else if (cmd == "quit") {
            kin_state->running = false;
            break;
        }
    }

    pthread_join (vx_thread, NULL);
    return 0;
}
