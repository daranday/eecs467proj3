#include "a3_picasso.h"


using namespace std;
using namespace cv;

// OpenCV Constant 

double pi = 3.1415926;

Mat src, erosion_dst, dilation_dst, dst;
Mat src_gray;
vector<Vec4i> hierarchy;
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



// Application State
state_t state_obj;
state_t *state = &state_obj;

// Initiaize draw bot
DrawBot Picasso;

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

// This is intended animate_thread
// animate_threadto give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.

void* start_vx(void* user) {
	
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
	// pthread_create (&state->animate_thread, NULL, animate_thread, state);

	// This is the main loop
	eecs467_gui_run (&state->vxapp, state->pg, 1024, 768);

	// Quit when GTK closes
	state->running = 0;
	pthread_join (state->animate_thread, NULL);

	// Cleanup
	free (my_listener);
	vx_remote_display_source_destroy (cxn);
	vx_global_destroy ();
	
	return NULL;
}

void* start_inverse_kinematics(void* user) {
	Arm.start(state->argc, state->argv);
	return NULL;
}

void* start_opencv(void * arg) {
	// double imageSize = 600;
	Mat expanded_src;
	cvtColor( src, src_gray, CV_BGR2GRAY );
	//Canny( src_gray, src_gray, thresh, thresh*2, 3 );
	blur( src_gray, src_gray, Size(3,3) );
	// blur( src_gray, src_gray, Size(3,3) );

	
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
	const char* source_window = "Source";
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

    createTrackbar( "Contour", "Source",
                  &curContour, maxContour,
                  thresh_callback );

	thresh_callback( 0, 0 );

	return NULL;
}

void get_coordinates_from_joints(double& x, double& y) {
	double R = 0;
	vector<double> arm_length = {0.118, 0.1, 0.0985, 0.099}; // change this accordingly
	
	R = arm_length[1] * sin(Arm.real_angles[1]) + arm_length[2] * sin(Arm.real_angles[1] + Arm.real_angles[2]);
	R += arm_length[3] * sin(Arm.real_angles[1] + Arm.real_angles[2] + Arm.real_angles[3]);

	x = R * sin(Arm.real_angles[0]);
	y = - R * cos(Arm.real_angles[0]);
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

void DrawBot::axes() {
	// draw x axis
	double x0 = state->origin_x, y0 = state->origin_y;
	double h = 0.1025;
	int iters = 50, wait_t = 50000;

	double x = x0 - state->interval_x, y = y0;
	Arm.move_to(x, y, h + 0.02);
	usleep(wait_t);

	double dx = 0.1 / iters, dy = 0.1 / iters;
	for (int i = 0; i < iters; ++i) {
		x += dx;
		Arm.move_to(x, y, h);
		usleep(wait_t);
		// usleep(wait_t);
	}
	
	Arm.move_to(x, y, h + 0.02);
	usleep(wait_t);
	x = x0; y = y0 - state->interval_y / 2;
	Arm.move_to(x, y, h + 0.02);
	usleep(wait_t);

	for (int i = 0; i < iters; ++i) {
		y += dy;
		Arm.move_to(x, y, h);
		usleep(wait_t);
		// usleep(wait_t);
	}

	Arm.move_to(x0, y0, h + 0.02);
	usleep(wait_t);
}

vector<vector<Point> > GetContours(Mat& src) {
	// Mat canny_output;
	vector<vector<Point> > contours;

	if (canny == 1){
		cout << "Using Canny" << endl;
		Canny( src, dst, thresh, thresh*2, 3 );
	}
	else{
		cout << "Using Threshold" << endl;	
		threshold( src, dst, thresh, thresh*2,3);
	}

	/// Find contours
	findContours( dst, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	/// Output contours
	cout << "Number of Contours " << contours.size() << endl;
	return contours;
}


void thresh_callback(int, void* )
{
	double imageSize = 100;
	resize(src_gray,src_gray,Size(),imageSize/src_gray.cols,imageSize/src_gray.rows,INTER_AREA);
	
	vector<vector<Point> > contours = GetContours(src_gray);

	maxContour = contours.size() - 1;

	/// Draw contours
	Mat drawing = Mat::zeros( dst.size(), CV_8UC3 );

	if (curContour > contours.size() -1 ){
		for( unsigned int i = 0; i< contours.size(); i++ ){
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
		}
	} else {
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, curContour, color, 1, 8, hierarchy, 0, Point() );
	}

	// cout << "drawing " << drawing << endl;

	/// Show in a window
	namedWindow( "Contours", CV_WINDOW_NORMAL );
	imshow( "Contours", drawing );
	state->opencv_initialized = true;
	waitKey(0);
}

void DrawBot::draw(){
	Mat fliped_src;
	flip(src_gray, fliped_src, 1);
	vector<vector<Point> > contours = GetContours(fliped_src);

	double wait_t = 150000;//150000
	double stepSize, stepX, stepY;
	Point prev, cur, diff;
	vector<float> points;
	int totalPoints = 0;

  	Arm.stand();

	cout << "Ready" << endl;
	for (unsigned int i = 0; i < contours.size(); i++){
		//if(hierarchy[i][3] >= 0){
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

			prev = contours[i][0];
			contours[i].push_back(prev);

			// lift arm to start point of current contour
			double start_x = ((double)contours[i][0].x - 50)/1000.;
			double start_y = ((double)contours[i][0].y + 60)/1000.;
			Arm.move_to(start_x, start_y, hoverH);
			usleep(wait_t);
			Arm.move_to(start_x, start_y, drawH);
			usleep(wait_t);

			for (unsigned int j = 0; j < contours[i].size(); j++){
				cout << contours[i][j] << " " << endl;

				cur = contours[i][j];
				diff = cur - prev; 
				// vector<float> cur_points = {(float)(prev.x / 1000.), (float)(prev.y / 1000.), (float)0.001, (float)(cur.x / 1000.), (float)(cur.y / 1000.), (float)0.001};
				// points.insert(points.end(), cur_points.begin(), cur_points.end());
				// vx_resc_t *verts = vx_resc_copyf(points.data(), points.size());
				// vx_buffer_add_back(vx_world_get_buffer(state->vxworld, "drawing_vector"), 
				// 					vxo_lines(verts, points.size() / 3, GL_LINES, vxo_points_style(vx_red, 2.0f)));
				// vx_buffer_swap(vx_world_get_buffer(state->vxworld, "drawing_vector"));
				

				if (abs(diff.x) > abs(diff.y))stepSize = abs(diff.x);
				else stepSize = abs(diff.y);

				stepX = ((double)diff.x)/stepSize;
				stepY = ((double)diff.y)/stepSize;
				for( double k = 1; k < stepSize; k++){
					cout << " midpoint with stepY= " << stepY*k << " ";
					Arm.move_to(((double)prev.x - 50 + k*stepX)/1000., ((double)prev.y  + 50 + k*stepY)/1000., drawH  );
					usleep(wait_t);
				}

				cout << ((double)cur.x-50)/1000. << " " << ((double)cur.y + 60)/1000. << endl;
				Arm.move_to(((double)cur.x-50)/1000., ((double)cur.y + 60)/1000., drawH);
				usleep(wait_t);
				prev = cur;
				totalPoints++;
			}
			cout << endl;
			cout << "Contour " << i << " length is " << contours[i].size() << endl;
			Arm.move_to(((double)cur.x-50)/1000., ((double)cur.y + 50)/1000., hoverH  );
			usleep(wait_t);
		//}
	}
	cout << "Total points drawn: " << totalPoints << endl;
}

void DrawBot::basic_shape(string& shape) {
	double x0 = state->origin_x, y0 = state->origin_y;
	double x = x0, y = y0;
	int wait_t = 100000;
	int second = 1000000;

	Arm.move_to(x0, y0, hoverH);
	usleep(second);

	if (shape == "square") {
		int iters = 50;
        x = x0; y = y0;

        double dx = 0.05 / iters, dy = 0.05 / iters;
        for (int i = 0; i < iters; ++i) {
            y += dy;
            Arm.move_to(x, y, drawH);
            usleep(wait_t);
        }
        for (int i = 0; i < iters; ++i) {
            x -= dx;
            Arm.move_to(x, y, drawH);
            usleep(wait_t);
        }
        for (int i = 0; i < iters; ++i) {
            y -= dy;
            Arm.move_to(x, y, drawH);
            usleep(wait_t);
        }
        for (int i = 0; i < iters; ++i) {
            x += dx;
            Arm.move_to(x, y, drawH);
            usleep(wait_t);
        }
	} else if (shape == "circle") {
		double R = 0.03;
        int iters = 180;
        cout << "Radius: ";
        cin >> R;
        double dtheta = 2 * pi / iters, theta = 0;

        // cout << "stop1" << endl;
        Arm.move_to(x0 + R, y0, hoverH);

        for (int i = 0; i <= iters; ++i) {
            x = x0 + R * cos(theta + i * dtheta);
            y = y0 + R * sin(theta + i * dtheta);
            Arm.move_to(x, y, drawH);
        	// cout << "stop1" << endl;
            usleep(wait_t);
        }
	} else if (shape == "sine") {
		double R = 0.03;
        int iters = 20;
        cin >> iters;
        double dtheta = 4 * pi / iters, theta = 0;

        Arm.move_to(x0, y0, hoverH);

        for (int i = 0; i < iters; ++i) {
            x = x0 + 0.08 / (4 * pi) * (i * dtheta);
            y = y0 + 0.045 * sin(i * dtheta);
            Arm.move_to(x, y, drawH);
            usleep(wait_t);
        }
	}

    Arm.stand();
}

int main (int argc, char *argv[])
{
	cout << "\n============ INITIALIZING =============\n";


	Picasso.drawH = 0.122;
	Picasso.hoverH = 0.13;
	//start vx
	state->argc = argc;
	state->argv = argv;
	pthread_t vx_thread, kinematics_thread, opencv_thread;
	pthread_create (&kinematics_thread, NULL, start_inverse_kinematics, (void*)NULL);
	// pthread_create (&vx_thread, NULL, start_vx, (void*)NULL);

	if (argc == 1) src = imread("sud2.jpg");
	else src = imread( argv[1] ); 
	pthread_create (&opencv_thread, NULL, start_opencv, (void*) NULL);
	// Wait till opencv is initialized. 
	while (!state->opencv_initialized);

	cout << "\n============ MAIN PROGRAM RUNNING =============\n";
	
	// Command loop
	while(true) {
	    double a, b, c;
	    string cmd;
		
	    cout << "command > ";
    	cin >> cmd;

	    if (cmd == "draw") {
	        cout << "drawing figure" << endl;
	        Picasso.draw();

	    } else if (cmd == "square" || cmd == "circle" || cmd == "sine") {
	        cout << "drawing " << cmd << endl;
	    	Picasso.basic_shape(cmd);

	    } else if (cmd == "quit") {
	        Arm.running = false;
	        break;
	    }
	}

	// pthread_join (vx_thread, NULL);
	return 0;
}
