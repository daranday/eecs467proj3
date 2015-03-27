#include "a2_color_picker.h"

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
void show_image(image_u32_t* im);
void rgb_to_hsv(uint32_t rgba, double& h, double& s, double& v);

struct color_state_t {
    // take a pic required
    char     *url; // image_source url
    image_source_t *isrc;
    int fidx;
    pthread_mutex_t mutex;
    pthread_mutex_t cmd_mutex;
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
    
    // color picker color_state
    double h_min_last;
    double s_min_last;
    double v_min_last;
    double h_max_last;
    double s_max_last;
    double v_max_last;

    double h_min;
    double s_min;
    double v_min;
    double h_max;
    double s_max;
    double v_max;
    int hsv_state;



    // selection color_state
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
    color_state_t() {
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

        // hsv_init
        hsv_state = 0;
        h_min = 1; s_min = 1; v_min = 1; 
        h_max = 0; s_max = 0; v_max = 0; 
    }

    //clean up
    ~color_state_t() {
        if (vxeh)
            free (vxeh);

        if (gopt)
            getopt_destroy (gopt);

        if (pg)
            pg_destroy (pg);

        image_u32_destroy (image);
        if (layered_image != NULL) {
            image_u32_destroy (layered_image);
            layered_image = NULL;
        }
    }

    void update_layered_image() {
        int count = 0;
        for (int j = 0; j < layered_image->height; ++j) {
            for (int i = 0; i < layered_image->width; ++i) {
                double h, s, v;
                rgb_to_hsv(image->buf[j*image->stride+i], h, s, v);
                if (h_min <= h && h <= h_max && s_min <= s && s <= s_max && v_min <= v && v <= v_max) {
                    layered_image->buf[j*layered_image->stride+i] = 0x800080FF; // mark purple
                    count ++;
                }
            }
        }
        printf("%d updated\n", count);
        show_image(layered_image);
    }

    void update_hsv(double h, double s, double v) {
        h_min_last = h_min;
        s_min_last = s_min;
        v_min_last = v_min;
        h_max_last = h_max;
        s_max_last = s_max;
        v_max_last = v_max;

        h_min = min(h_min, h);
        s_min = min(s_min, s);
        v_min = min(v_min, v);
        h_max = max(h_max, h);
        s_max = max(s_max, s);
        v_max = max(v_max, v);

        // create layer if not already
        if (hsv_state == 0) {
            layered_image = image_u32_copy(image);
        }

        printf("HSV MIN: %g, %g, %g\n", h_min, s_min, v_min);
        printf("HSV MAX: %g, %g, %g\n", h_max, s_max, v_max);
        
        update_layered_image();
        hsv_state++;

    }

    void undo_hsv() {
        if (hsv_state == 0) {
            printf("Error: Already at original, cannot undo\n");
            return;
        }
        h_min = h_min_last;
        s_min = s_min_last;
        v_min = v_min_last;
        h_max = h_max_last;
        s_max = s_max_last;
        v_max = v_max_last;

        hsv_state--;
        image_u32_destroy (layered_image);
        if (hsv_state == 0) {
            layered_image = NULL;
            show_image(image);
        } else {
            layered_image = image_u32_copy(image);
            update_layered_image();
        }
        printf("Undid last color selection\n");
    }
} color_state_obj;

color_state_t *color_state = &color_state_obj;

// takes a picture and return it
// can also outputs the image to a specified path, filename always is: BeforeMask.ppm
// if don't want to, leave the path empty
image_u32_t* take_a_pic (char* output_image_path)
{
    // color_state_t *color_state = &state_obj;

    //get camera addresses
    zarray_t *urls = image_source_enumerate();

    // printf("Cameras:\n");
    // for (int i = 0; i < zarray_size(urls); i++) {
    //     char *url;
    //     zarray_get(urls, i, &url);
    //     printf("  %3d: %s\n", i, url);
    // }
    // printf("haha\n");
    if (zarray_size(urls) == 0) {
        printf("No cameras found.\n");
        exit(0);
    }
    // printf("haha\n");
    // Choose first image source
    zarray_get(urls, 0, &color_state->url);
    // printf("haha\n");


    pthread_mutex_init(&color_state->mutex, NULL);

    //////////////////////////////////////////////////////////
    color_state->isrc = image_source_open(color_state->url);
    // printf("haha\n");
    if (color_state->isrc == NULL) {
        printf("Unable to open device %s\n", color_state->url);
        exit(-1);
    }
    // printf("haha\n");

    image_source_t *isrc = color_state->isrc;
    // printf("haha\n");

    if (isrc->start(isrc))
        exit(-1);
    // printf("haha\n");

    color_state->fidx = isrc->get_current_format(isrc);

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
    //     scanf("%d", &(color_state->fidx));
    //     if (isrc->start(isrc))
    //         exit(-1);
    //     if (color_state->fidx >= 0 && color_state->fidx < isrc->num_formats(isrc)) {
    //         image_source_format_t ifmt;
    //         isrc->get_format(isrc, color_state->fidx, &ifmt);
    //         color_state->fmt_x = ifmt.width;
    //         color_state->fmt_y = ifmt.height;
    //         isrc->set_format (isrc, color_state->fidx);
    //         break;
    //     }
    // }

    if (color_state->fidx >= 0 && color_state->fidx < isrc->num_formats(isrc)) {
        image_source_format_t ifmt;
        isrc->get_format(isrc, color_state->fidx, &ifmt);
        color_state->fmt_x = ifmt.width;
        color_state->fmt_y = ifmt.height;
        // isrc->set_format (isrc, color_state->fidx);
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

    pthread_mutex_lock(&color_state->mutex);
    //take pic
    int res;
    while((res = isrc->get_frame(isrc, &isdata)) != 0);

    //convert pic to proper format and output
    image_u32_t* img = image_convert_u32(&isdata);
    
    //write image to outputpath in pnm format
    //only if output image path is not empty
    if (strcmp(output_image_path, "") != 0) {
        string dest = string(output_image_path) + "BeforeColorSelect.ppm";
        image_u32_write_pnm(img, dest.c_str());
    }
    
    isrc->release_frame(isrc, &isdata);
    isrc->stop(isrc);
    
    pthread_mutex_unlock(&color_state->mutex);

    return img;
}

// === Parameter listener =================================================
// This function is handed to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// color_state, etc if need be.
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

    // } else if (0==strcmp("button_capture", name)) {
    //     color_state->image = take_a_pic("");

    } else if (0==strcmp("button_save", name)) {
        if (color_state->hsv_state == 0) {
            printf("Error: No hsv ranges to save\n");
            return;
        }
        ofstream fout("HsvRange.txt");
        fout << color_state->h_min << " " << color_state->h_max << " "
             << color_state->s_min << " " << color_state->s_max << " "
             << color_state->v_min << " " << color_state->v_max << endl;
        fout.close();
        printf("File saved to HsvRange.txt\n");
    } 

    //for any button click, clear the selection
    if (0==strncmp("button", name, 6)) {
        color_state->hsv_state = 0;
        color_state->h_min = 1; color_state->s_min = 1; color_state->v_min = 1; 
        color_state->h_max = 0; color_state->s_max = 0; color_state->v_max = 0; 

        image_u32_destroy (color_state->layered_image);
        color_state->layered_image = NULL;
        show_image(color_state->image);
    }
    
}

void get_image_coordinates(double x, double y, int& coord_x, int& coord_y) {
    coord_x = round((x + 1) * color_state->fmt_x / 2. + 0.5);
    coord_y = color_state->fmt_y - round((y + color_state->fmt_y / (color_state->fmt_x * 1.)) * color_state->fmt_x / 2. + 0.5);
    printf("Coords x = %d, y = %d\n", coord_x, coord_y);
}

void rgb_to_hsv(uint32_t abgr, double& h, double& s, double& v){
    double b = ((abgr & 0x00FF0000) >> 16) / 255.;
    double g = ((abgr & 0x0000FF00) >> 8) / 255.;
    double r = ((abgr & 0x000000FF) >> 0) / 255.;
    double rgb_max = std::max(r, std::max(g, b));
    double rgb_min = std::min(r, std::min(g, b));
    double delta = rgb_max - rgb_min;
    s = delta / (rgb_max + 1e-20f);
    v = rgb_max;

    double hue;
    if (r == rgb_max)
        hue = (g - b) / (delta + 1e-20f);
    else if (g == rgb_max)
        hue = 2 + (b - r) / (delta + 1e-20f);
    else
        hue = 4 + (r - g) / (delta + 1e-20f);
    if (hue < 0)
        hue += 6.f;
    h = hue * (1.f / 6.f);
}

static int mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    // color_state_t *color_state = (color_state_t*)(vxeh->impl);

    // vx_camera_pos_t contains camera location, field of view, etc
    // vx_mouse_event_t contains scroll, x/y, and button click events

    if ((mouse->button_mask & VX_BUTTON1_MASK) &&
        !(color_state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {

        vx_ray3_t ray;
        vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

        double ground[3];
        vx_ray3_intersect_xy (&ray, 0, ground);

        // printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Ground clicked at coords: [%6.3f, %6.3f]\n", mouse->x, mouse->y, ground[0], ground[1]);

        int image_x, image_y;
        get_image_coordinates(ground[0], ground[1], image_x, image_y);
        if (image_x < 0 || image_x > color_state->fmt_x || image_y < 0 || image_y > color_state->fmt_y) {
            printf("Error: mouse click outside of image\n");
            return 1;
        }

        double h, s, v;

        // update new hsv range
        uint32_t rgb = color_state->image->buf[image_y * color_state->image->stride + image_x];
        rgb_to_hsv(rgb, h, s, v);
        printf("ARGB: %X\n", rgb);
        printf("H: %g, S: %g, V: %g\n", h, s, v);
        // printf("haha\n");
        color_state->update_hsv(h, s, v);
        // printf("haha\n");
    }

    // store previous mouse event to see if the user *just* clicked or released
    color_state->last_mouse_event = *mouse;

    return 0;
}

static int key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
    pthread_mutex_lock(&color_state->cmd_mutex);
    if (key->released) {
        if (key->key_code == 's' || key->key_code == 'S') {
            if (color_state->hsv_state == 0) {
                printf("Error: No hsv ranges to save\n");
                return 1;
            }
            ofstream fout("HsvRange.txt");
            fout << color_state->h_min << " " << color_state->h_max << " "
                 << color_state->s_min << " " << color_state->s_max << " "
                 << color_state->v_min << " " << color_state->v_max << endl;
            fout.close();
            printf("File saved to HsvRange.txt\n");
        } else if (key->key_code == VX_KEY_DEL) {
            color_state->undo_hsv();
        }
    }
    pthread_mutex_unlock(&color_state->cmd_mutex);

    return 0;
}

static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0; // Does nothing
}

void show_image(image_u32_t* im) {
    vx_object_t *vim = vxo_image_from_u32(im,
                                      VXO_IMAGE_FLIPY,
                                      VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

    // render the image centered at the origin and at a normalized scale of +/-1 unit in x-dir
    const double scale = 2./im->width;
    vx_buffer_add_back (vx_world_get_buffer (color_state->vxworld, "image"),
                        vxo_chain (vxo_mat_scale3 (scale, scale, 1.0),
                                   vxo_mat_translate3 (-im->width/2., -im->height/2., 0.),
                                   vim));
    vx_buffer_swap (vx_world_get_buffer (color_state->vxworld, "image"));
}

void *animate_thread (void *data)
{
    image_u32_t *im = color_state->image;
    if (im != NULL) {
        show_image(color_state->image);
    }
    return NULL;
}

void init_getopt(int argc, char** argv) {
    // color_state_t *color_state = &state_obj;
    // Parse arguments from the command line, showing the help
    // screen if required
    color_state->gopt = getopt_create ();
    getopt_add_bool   (color_state->gopt,  'h', "help", 0, "Show help");
    getopt_add_string (color_state->gopt, 'f', "file", "", "Image file to be masked");

    if (!getopt_parse (color_state->gopt, argc, argv, 1) || getopt_get_bool (color_state->gopt, "help")) {
        printf ("Usage: %s [--file=FILE_PATH] or an image will be cropped from your camera.\n\n", argv[0]);
        getopt_do_usage (color_state->gopt);
        exit (EXIT_FAILURE);
    }
}

void run_graphics(int argc, char** argv) {
    // color_state_t *color_state = &state_obj;
    eecs467_init (argc, argv);

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&color_state->vxapp);

    // Initialize a parameter gui
    color_state->pg = pg_create ();
    // pg_add_double_slider (color_state->pg, "sl1", "Slider 1", 0, 100, 50);
    // pg_add_int_slider    (color_state->pg, "sl2", "Slider 2", 0, 100, 25);
    // pg_add_check_boxes (color_state->pg,
    //                     "cb1", "Check Box 1", 0,
    //                     "cb2", "Check Box 2", 1,
    //                     NULL);
    pg_add_buttons (color_state->pg,
                    // "button_capture", "Capture",
                    "button_clear", "Clear Selection",
                    "button_save", "Save",
                    NULL);

    parameter_listener_t *my_listener = (parameter_listener_t*)calloc(1, sizeof(*my_listener));
    my_listener->impl = color_state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener (color_state->pg, my_listener);

    // Launch our worker threads
    pthread_create(&color_state->animate_thread, NULL, animate_thread, color_state);

    // This is the main loop
    eecs467_gui_run (&color_state->vxapp, color_state->pg, color_state->fmt_x, color_state->fmt_y);

    // Quit when GTK closes
    color_state->running = 0;
    pthread_join (color_state->animate_thread, NULL);

    // Cleanup
    free (my_listener);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();
}

int main(int argc, char** argv) {
    char output_path[100] = "./";

    init_getopt(argc, argv);
    
    if (strcmp(getopt_get_string (color_state->gopt, "file"), "") == 0)
        color_state->image = take_a_pic(output_path);
    else {
        color_state->image = image_u32_create_from_pnm(getopt_get_string(color_state->gopt, "file"));
        color_state->fmt_x = color_state->image->width;
        color_state->fmt_y = color_state->image->height;
    }

    run_graphics(argc, argv);
}
