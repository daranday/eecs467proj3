#include "a3_picasso.h"


using namespace std;
using namespace cv;

Mat src, erosion_dst, dilation_dst, dst; 
Mat src_gray;
int thresh = 60;
int max_thresh = 255;
RNG rng(12345);
int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int canny = 0;
int curContour = 99;
int maxContour = 100; 
int const max_elem = 2;
int const max_kernel_size =1;
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
} state_obj;

state_t *state = &state_obj;

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


static int mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
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
		}
		// printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Camera coordinates clicked at: [%6.3f, %6.3f]\n",
		//         mouse->x, mouse->y, camera_vector[0], camera_vector[1]);
		// printf("Height: %d, Width: %d\n", state->img_height, state->img_width);
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

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by state
void * animate_thread (void *data)
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
				// if (state->current_image != NULL) {
				//     image_u32_destroy (state->current_image);
				//     state->current_image = NULL;
				// }
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

// This is intended animate_thread
// animate_threadto give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.

void* start_vx(void* user) {
	/*
	int argc = state->argc;
	char **argv = state->argv;

	eecs467_init (argc, argv);

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
	vx_remote_display_source_destroy (cxn);
	vx_global_destroy ();
	*/
	return NULL;
	
}

void* start_inverse_kinematics(void* user) {
	kin_main(state->argc, state->argv);
	return NULL;
}

void* start_opencv(void * arg) {
	double imageSize = 600;
	Mat expanded_src;
	cvtColor( src, src_gray, CV_BGR2GRAY );
	//Canny( src_gray, src_gray, thresh, thresh*2, 3 );
	blur( src_gray, src_gray, Size(3,3) );
	blur( src_gray, src_gray, Size(3,3) );



	
	int erosion_type = 0;
	if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
	else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
	else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement( erosion_type,
	                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
	                                       Point( erosion_size, erosion_size ) );

	/// Apply the erosion operation
	erode( src_gray, src_gray, element );
	

	/// Create Window
	char* source_window = "Source";
	namedWindow( source_window, CV_WINDOW_NORMAL );
	imshow( source_window, src_gray );


	createTrackbar( " Thresh:", "Source", &thresh, max_thresh, thresh_callback );
	/*
	createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Source",
                  &dilation_elem, max_elem,
                  thresh_callback );
	*/
    createTrackbar( "Canny On:\n", "Source",
                  &canny, max_kernel_size,
                  thresh_callback );

    createTrackbar("Contour", "Source",
                  &curContour, maxContour,
                  thresh_callback );

	thresh_callback( 0, 0 );
	waitKey(0);
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

void draw_axes() {
	// draw x axis
	double x0 = state->origin_x, y0 = state->origin_y;
	double h = 0.1025;
	int iters = 50, wait_t = 50000;

	double x = x0 - state->interval_x, y = y0;
	move_to(x, y, h + 0.02);
	usleep(wait_t);

	double dx = 0.1 / iters, dy = 0.1 / iters;
	for (int i = 0; i < iters; ++i) {
		x += dx;
		move_to(x, y, h);
		usleep(wait_t);
		// usleep(wait_t);
	}
	
	move_to(x, y, h + 0.02);
	usleep(wait_t);
	x = x0; y = y0 - state->interval_y / 2;
	move_to(x, y, h + 0.02);
	usleep(wait_t);

	for (int i = 0; i < iters; ++i) {
		y += dy;
		move_to(x, y, h);
		usleep(wait_t);
		// usleep(wait_t);
	}

	move_to(x0, y0, h + 0.02);
	usleep(wait_t);
}


void thresh_callback(int, void* )
{
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	double imageSize = 100;
	resize(src_gray,src_gray,Size(),imageSize/src_gray.cols,imageSize/src_gray.rows,INTER_AREA);

	if (canny == 1){
		cout << "Using Canny" << endl;
		Canny( src_gray, dst, thresh, thresh*2, 3 );
	}
	else{
		cout << "Using Threshold" << endl;	
		threshold( src_gray, dst, thresh, thresh*2,3);
	}

	/// Find contours
	findContours( dst, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	maxContour = contours.size() - 1;
	cout << "Number of Contours " << contours.size() << endl;

		/// Draw contours
	Mat drawing = Mat::zeros( dst.size(), CV_8UC3 );

	if (curContour > contours.size() -1 ){
		for( unsigned int i = 0; i< contours.size(); i++ ){
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		}
	} else {
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, curContour, color, 2, 8, hierarchy, 0, Point() );
	}


	/// Show in a window
	namedWindow( "Contours", CV_WINDOW_NORMAL );
	imshow( "Contours", drawing );
}

void draw(int, void* ){

	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/// Detect edges using canny
	//Canny( src_gray, canny_output, thresh, thresh*2, 3 );
	/// Find contours
	cout << "Drawing with threshold " << thresh << endl;

	if (canny == 1){
		cout << "Using Canny" << endl;
		Canny( src_gray, dst, thresh, thresh*2, 3 );
	}
	else{
		cout << "Using Threshold" << endl;	
		threshold( src_gray, dst, thresh, thresh*2,3);
	}

	findContours( dst, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	/// Draw contours
	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	cout << contours.size() << " ";
	if (contours.size() != 0) cout << contours[0].size() << endl;


	double h = 0.1145; //.1125
	double dropH = .13;
	double wait_t = 150000;//150000
	int totalPoints = 0;
	Point prev;
	Point cur;
	Point diff;
	vector<float> points;
	double stepSize, stepX, stepY;

  	stand_arm();

	usleep(wait_t);
	cout << "Ready" << endl;
	for (unsigned int i = 0; i < contours.size(); i++){
		//if(hierarchy[i][3] >= 0){
			prev.x = contours[i][0].x;
			prev.y = contours[i][0].y;
			contours[i].push_back(prev);
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

			// lift arm to start point of current contour
			double start_x = ((double)contours[i][0].x - 50)/1000.;
			double start_y = ((double)contours[i][0].y + 60)/1000.;
			move_to(start_x, start_y, dropH);
			usleep(wait_t*4);

			move_to(start_x, start_y, h);
			usleep(wait_t);

			for (unsigned int j = 0; j < contours[i].size(); j++){

				cout << contours[i][j] << " " << endl;
				cur.x = contours[i][j].x;
				cur.y = contours[i][j].y;
				diff.x = cur.x - prev.x;
				diff.y = cur.y - prev.y; 
				vector<float> cur_points = {(float)(prev.x / 1000.), (float)(prev.y / 1000.), (float)0.001, (float)(cur.x / 1000.), (float)(cur.y / 1000.), (float)0.001};
				points.insert(points.end(), cur_points.begin(), cur_points.end());
				vx_resc_t *verts = vx_resc_copyf(points.data(), points.size());
				vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "drawing_vector"), 
									vxo_lines(verts, points.size() / 3, GL_LINES, vxo_points_style(vx_red, 2.0f)));
				vx_buffer_swap(vx_world_get_buffer(state->vxworld, "drawing_vector"));
				
				totalPoints++;

				if (abs(diff.x) > abs(diff.y))stepSize = abs(diff.x);
				else stepSize = abs(diff.y);

				stepX = ((double)diff.x)/stepSize;
				stepY = ((double)diff.y)/stepSize;
				for( double k = 1; k < stepSize; k++){
					cout << " midpoint with stepY= " << stepY*k << " ";
					move_to(((double)prev.x - 50 + k*stepX)/1000., ((double)prev.y  + 50 + k*stepY)/1000., h  );
					usleep(wait_t);
				}

				cout << ((double)cur.x-50)/1000. << " " << ((double)cur.y + 60)/1000. << endl;
				move_to(((double)cur.x-50)/1000., ((double)cur.y + 60)/1000., h  );
				usleep(wait_t);
				prev = cur;
			}
			cout << endl;
			cout << contours[i].size() << endl;
			move_to(((double)cur.x-50)/1000., ((double)cur.y + 50)/1000., dropH  );
			usleep(wait_t);
		//}
	}
	cout << contours.size() << " " << totalPoints << endl;
}

int main (int argc, char *argv[])
{
	cout << "\n============ PROGRAM BEGIN =============\n";

	//start vx
	state->argc = argc;
	state->argv = argv;
	pthread_t vx_thread, kinematics_thread, opencv_thread;
	pthread_create (&kinematics_thread, NULL, start_inverse_kinematics, (void*)NULL);
	pthread_create (&vx_thread, NULL, start_vx, (void*)NULL);

	if (argc == 1) src = imread("sud2.jpg");
	else src = imread( argv[1] ); 
	pthread_create (&opencv_thread, NULL, start_opencv, (void*) NULL);
	usleep(1000000);

	cout << "\n============ MAIN PROGRAM RUNNING =============\n";

	double pi = 3.1415926;

	double x0 = state->origin_x, y0 = state->origin_y;
	double x = x0, y = y0;
	double h = 0.1025;
	int wait_t = 100000;
	
	int my_init = 0;

	while(1) {
	    double a, b, c;
	    string cmd;
		
	    cout << "command > ";

	    if (my_init) {
	    	cmd = "draw";
	    	my_init = 0;
	    } else {
	    	cin >> cmd;
	    	
	    }


	    if (cmd == "draw") {
	        draw(0,0);
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
	        int iters = 180;
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
