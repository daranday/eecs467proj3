#include "a2_mask.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <string>
#include <fstream>

// gtk
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

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
#include "imagesource/image_u8x3.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "apps/eecs467_util.h"

using namespace std;

static int mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse);
static int key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key);
static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse);
void update_image(image_u32_t* im);

struct state_t {
    // take a pic required
    char     *url; // image_source url
    image_source_t *isrc;
    int fidx;
    pthread_mutex_t mutex;
    int fmt_x, fmt_y;

    // vx_world required
    bool running;

    getopt_t        *gopt;
    parameter_gui_t *pg;

    // image stuff
    char *img_url;
    int   img_height;
    int   img_width;
    image_u32_t* image;
    image_u32_t* layered_image;
    int   p1_x;
    int   p1_y;
    int   p2_x;
    int   p2_y;


    // selection state
    int selection_state;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;

    // threads
    pthread_t animate_thread;

    // for accessing the arrays
    // pthread_mutex_t mutex;
    state_t() {
        vxworld = vx_world_create ();
        vxeh = (vx_event_handler_t*)calloc (1, sizeof(*vxeh));
        vxeh->key_event = key_event;
        vxeh->mouse_event = mouse_event;
        vxeh->touch_event = touch_event;
        vxeh->dispatch_order = 100;
        vxeh->impl = this; // this gets passed to events, so store useful struct here!

        vxapp.display_started = eecs467_default_display_started;
        vxapp.display_finished = eecs467_default_display_finished;
        vxapp.impl = eecs467_default_implementation_create (vxworld, vxeh);

        running = 1;

        // bbox selection
        selection_state = 0;
        layered_image = NULL;
    }

    //clean up
    ~state_t() {
        if (vxeh)
            free (vxeh);

        if (gopt)
            getopt_destroy (gopt);

        if (pg)
            pg_destroy (pg);

        image_u32_destroy (image);
        if (layered_image != NULL)
            image_u32_destroy (layered_image);
    }
} state_obj;

state_t *state = &state_obj;

// takes a picture and return it
// can also outputs the image to a specified path, filename always is: BeforeMask.ppm
// if don't want to, leave the path empty
image_u32_t* take_a_pic (char* output_image_path)
{
    //instantiate a state variable
    // state_t *state = &state_obj;

    //get camera addresses
    zarray_t *urls = image_source_enumerate();

    printf("Cameras:\n");
    for (int i = 0; i < zarray_size(urls); i++) {
        char *url;
        zarray_get(urls, i, &url);
        printf("  %3d: %s\n", i, url);
    }
    // printf("haha\n");
    if (zarray_size(urls) == 0) {
        printf("No cameras found.\n");
        exit(0);
    }
    // printf("haha\n");
    // Choose first image source
    zarray_get(urls, 0, &state->url);
    // printf("haha\n");


    pthread_mutex_init(&state->mutex, NULL);

    //////////////////////////////////////////////////////////
    state->isrc = image_source_open(state->url);
    // printf("haha\n");
    if (state->isrc == NULL) {
        printf("Unable to open device %s\n", state->url);
        exit(-1);
    }
    // printf("haha\n");

    image_source_t *isrc = state->isrc;
    // printf("haha\n");

    if (isrc->start(isrc))
        exit(-1);
    // printf("haha\n");

    state->fidx = isrc->get_current_format(isrc);

    // printf("Image source formats:\n");
    // for (int i = 0; i < isrc->num_formats(isrc); i++) {
    //     image_source_format_t ifmt;
    //     isrc->get_format(isrc, i, &ifmt);
    //     printf("\t%d\t%4d x %4d (%s)\n", i, ifmt.width, ifmt.height, ifmt.format);
    // }


    // // ask which resolution to take picture in
    // while (true) {
    //     printf("Please select graphics: ");
    //     isrc->stop(isrc);
    //     scanf("%d", &(state->fidx));
    //     if (isrc->start(isrc))
    //         exit(-1);
    //     if (state->fidx >= 0 && state->fidx < isrc->num_formats(isrc)) {
    //         image_source_format_t ifmt;
    //         isrc->get_format(isrc, state->fidx, &ifmt);
    //         state->fmt_x = ifmt.width;
    //         state->fmt_y = ifmt.height;
    //         isrc->set_format (isrc, state->fidx);
    //         break;
    //     }
    // }

    if (state->fidx >= 0 && state->fidx < isrc->num_formats(isrc)) {
        image_source_format_t ifmt;
        isrc->get_format(isrc, state->fidx, &ifmt);
        state->fmt_x = ifmt.width;
        state->fmt_y = ifmt.height;
        // isrc->set_format (isrc, state->fidx);
        // break;
    }

    // printf("Image source features:\n");
    // for (int i = 0; i < isrc->num_features(isrc); i++) {
    //     const char *feature_name = isrc->get_feature_name(isrc, i);
    //     char *feature_type = isrc->get_feature_type(isrc, i);
    //     double v = isrc->get_feature_value(isrc, i);

    //     printf("\t%-20s %10f     %s\n", feature_name, v, feature_type);
    //     free(feature_type);
    // }
       
    //variables to store the pic 
    image_source_data_t isdata;

    pthread_mutex_lock(&state->mutex);
    //take pic
    int res;
    while((res = isrc->get_frame(isrc, &isdata)) != 0);

    //convert pic to proper format and output
    image_u32_t* img = image_convert_u32(&isdata);
    
    //write image to outputpath in pnm format
    //only if output image path is not empty
    if (strcmp(output_image_path, "") != 0) {
        string dest = string(output_image_path) + "BeforeMask.ppm";
        image_u32_write_pnm(img, dest.c_str());
    }
    
    isrc->release_frame(isrc, &isdata);
    isrc->stop(isrc);
    
    pthread_mutex_unlock(&state->mutex);

    return img;
}

// === Parameter listener =================================================
// This function is handed to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// state, etc if need be.
static void my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    if (0==strcmp ("sl1", name))
        printf ("sl1 = %f\n", pg_gd (pg, name));
    else if (0==strcmp ("sl2", name))
        printf ("sl2 = %d\n", pg_gi (pg, name));
    else if (0==strcmp ("cb1", name) || 0==strcmp ("cb2", name))
        printf ("%s = %d\n", name, pg_gb (pg, name));
    else
        printf ("%s changed\n", name);


    // button clicks
    if (0==strcmp("button_clear", name)) {

    } else if (0==strcmp("button_capture", name)) {
        state->image = take_a_pic("");

    } else if (0==strcmp("button_save", name)) {
        if (state->selection_state < 2) {
            printf("Error: Please select your bounding box vertices\n");
            return;
        }
        ofstream fout("Mask.txt");
        fout << state->p1_x << " " << state->p1_y << endl;
        fout << state->p2_x << " " << state->p2_y << endl;
        fout.close();
        printf("File saved to Mask.txt\n");
    } 

    //for any button click, clear the selection
    if (0==strncmp("button", name, 6)) {
        state->selection_state = 0;
        image_u32_destroy (state->layered_image);
        state->layered_image = NULL;
        update_image(state->image);
    }
    
}

void get_image_coordinates(double x, double y, int& coord_x, int& coord_y) {
    coord_x = round((x + 1) * state->fmt_x / 2. + 0.5);
    coord_y = state->fmt_y - round((y + state->fmt_y / (state->fmt_x * 1.)) * state->fmt_x / 2. + 0.5);
    printf("Coords x = %d, y = %d\n", coord_x, coord_y);
}

void mask_point(image_u32_t* im, int i, int j) {
    uint32_t rr = max((im->buf[j*im->stride+i] & 0xFF000000) - 0x1F000000, (uint32_t)0);
    uint32_t gg = max((im->buf[j*im->stride+i] & 0x00FF0000) - 0x001F0000, (uint32_t)0);
    uint32_t bb = max((im->buf[j*im->stride+i] & 0x0000FF00) - 0x00001F00, (uint32_t)0);
    uint32_t aa = im->buf[j*im->stride+i] & 0x000000FF;
    im->buf[j*im->stride+i] = rr + gg + bb + aa;
}

static int mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    // state_t *state = (state_t*)(vxeh->impl);

    // vx_camera_pos_t contains camera location, field of view, etc
    // vx_mouse_event_t contains scroll, x/y, and button click events

    if ((mouse->button_mask & VX_BUTTON1_MASK) &&
        !(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {

        vx_ray3_t ray;
        vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

        double ground[3];
        vx_ray3_intersect_xy (&ray, 0, ground);

        printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Ground clicked at coords: [%6.3f, %6.3f]\n",
                mouse->x, mouse->y, ground[0], ground[1]);

        // selecting bounding box
        if (state->selection_state == 0) {
            state->layered_image = image_u32_copy(state->image);
            image_u32_t* im = state->layered_image;
            int h = im->height, w = im->width, current_width;
            get_image_coordinates(ground[0], ground[1], state->p1_x, state->p1_y);
            for (int j = 0; j < h; ++j) {
                if (j < state->p1_y)
                    current_width = w;
                else
                    current_width = state->p1_x;

                for (int i = 0; i < current_width; ++i)
                    mask_point(im, i, j);
            }
            update_image(im);
        } else if (state->selection_state == 1) {
            image_u32_t* im = state->layered_image;
            int h = im->height, w = im->width, start_width;
            get_image_coordinates(ground[0], ground[1], state->p2_x, state->p2_y);
            for (int j = state->p1_y; j < h; ++j) {
                if (j < state->p2_y)
                    start_width = state->p2_x;
                else
                    start_width = state->p1_x;

                for (int i = start_width; i < w; ++i)
                    mask_point(im, i, j);
            }
            update_image(im);
        }
        state->selection_state++;
    }

    // store previous mouse event to see if the user *just* clicked or released
    state->last_mouse_event = *mouse;

    return 0;
}

static int key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
    //state_t *state = vxeh->impl;
    return 0;
}

static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0; // Does nothing
}

void update_image(image_u32_t* im) {
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
}

void *animate_thread (void *data)
{
    image_u32_t *im = state->image;
    if (im != NULL) {
        update_image(state->image);
    }
    return NULL;
}

void init_getopt(int argc, char** argv) {
    // state_t *state = &state_obj;
    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_string (state->gopt, 'f', "file", "", "Image file to be masked");

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--file=FILE_PATH] or an image will be cropped from your camera.\n\n", argv[0]);
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }
}

void run_graphics(int argc, char** argv) {
    // state_t *state = &state_obj;
    eecs467_init (argc, argv);

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    // Initialize a parameter gui
    state->pg = pg_create ();
    // pg_add_double_slider (state->pg, "sl1", "Slider 1", 0, 100, 50);
    // pg_add_int_slider    (state->pg, "sl2", "Slider 2", 0, 100, 25);
    // pg_add_check_boxes (state->pg,
    //                     "cb1", "Check Box 1", 0,
    //                     "cb2", "Check Box 2", 1,
    //                     NULL);
    pg_add_buttons (state->pg,
                    "button_capture", "Capture",
                    "button_clear", "Clear Selection",
                    "button_save", "Save",
                    NULL);

    parameter_listener_t *my_listener = (parameter_listener_t*)calloc(1, sizeof(*my_listener));
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener (state->pg, my_listener);

    // Launch our worker threads
    pthread_create(&state->animate_thread, NULL, animate_thread, state);

    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, state->fmt_x, state->fmt_y);

    // Quit when GTK closes
    state->running = 0;
    pthread_join (state->animate_thread, NULL);

    // Cleanup
    free (my_listener);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();
}


int main(int argc, char** argv) {
    char output_path[100] = "./";

    init_getopt(argc, argv);
    
    if (strcmp(getopt_get_string (state->gopt, "file"), "") == 0)
        state->image = take_a_pic(output_path);
    else {
        state->image = image_u32_create_from_pnm(getopt_get_string(state->gopt, "file"));
        state->fmt_x = state->image->width;
        state->fmt_y = state->image->height;
    }

    run_graphics(argc, argv);
}
