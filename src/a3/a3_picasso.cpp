#include "a3_picasso.h"
#include <queue>


using namespace std;
using namespace cv;

// OpenCV Constant 

double pi = 3.1415926;

Mat src, erosion_dst, dilation_dst, dst;
Mat src_gray;
vector<vector<Point> > image_contours;
vector<vector<Point> > skeletons;
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

DrawBot::DrawBot() {
	drawH = 0.12;
	hoverH = 0.14;
}

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

void get_gradient(vector<Point>& pts, double& slope, double& intercept) {
	int count = 3;
	int sumX = 0, sumY = 0, sumX2, sumXY, xMean = 0, yMean = 0;
	for (int i = 0; i < pts.size(); ++i) {
		sumX += pts[i].x;
		sumY += pts[i].y;
		sumX2 += pts[i].x * pts[i].x;
		sumXY += pts[i].x * pts[i].y;
	}
	xMean = sumX / count;
	yMean = sumY / count;
	slope = (sumXY - sumX * yMean) / (sumX2 - sumX * xMean);
	intercept = yMean - slope * xMean;
}

vector<Point> image_explore(Mat& img, unsigned char * input, vector<vector<int>>& group_map, int current_group, int a, int b) {
	queue<Point> frontier;
	frontier.push(Point(a,b));
	vector<Point> nodes;
	nodes.push_back(Point(a,b));
	group_map[a][b] = current_group;


	cout << "current_group: " << current_group << endl;
	waitKey(0);
	while (!frontier.empty()) {
		Point p = frontier.front();
		frontier.pop();

		for (int i = p.x-1; i <= p.x+1; ++i) {
			for (int j = p.y-1; j <= p.y+1; ++j) {
				if (!(i == p.x && j == p.y)
					&& (i >= 0 && i < img.cols)
					&& (j >= 0 && j < img.rows)) {
					unsigned char b = input[img.step * j + i ];
					if (b == 255 && group_map[i][j] == 0) {
	        			Point cur = Point(i, j);

	     //    			double slope, intercept;
						// get_gradient(vector<int>(frontier.begin(), frontier.end()), slope, intercept));
						// if (j - x * slope - intercept < 0)
						// 	continue;
						frontier.push(cur);
						group_map[i][j] = current_group;
						cout << "New frontier: " << cur << endl;
					}
				}
			}
		}
		if (p.x == a && p.y == b 
				|| (sqrt((p.x - nodes.back().x)*(p.x - nodes.back().x)+(p.y - nodes.back().y)*(p.y - nodes.back().y)) > 13
					&& (nodes.size() <= 1 || sqrt((p.x - nodes.end()[-2].x)*(p.x - nodes.end()[-2].x)+(p.y - nodes.end()[-2].y)*(p.y - nodes.end()[-2].y)) > 13))) {
			nodes.push_back(p);
		} else {
			input[img.step * p.y + p.x ] = 0;
		}
	}
	imshow("Exploration", img);
	waitKey();
	return nodes;
}

void* start_opencv(void * arg) {
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



	// Skeletonization
	Mat img = src_gray.clone();
	double imageSize = 100;
	resize(img,img,Size(),imageSize/img.cols,imageSize/img.rows,INTER_AREA);
	bitwise_not(img, img);
	threshold(img, img, 127, 255, THRESH_BINARY);
	Mat skel(img.size(), CV_8UC1, Scalar(0));
	Mat temp(img.size(), CV_8UC1);
	Mat element2 = getStructuringElement(MORPH_CROSS, Size(3, 3));
	namedWindow("Obama", CV_WINDOW_NORMAL);
	imshow("Obama", img);
	waitKey(0);
	bool done;
	do
	{
		morphologyEx(img, temp, MORPH_OPEN, element2);
		bitwise_not(temp, temp);
		bitwise_and(img, temp, temp);
		bitwise_or(skel, temp, skel);
		erode(img, img, element2);

		double max;
		minMaxLoc(img, 0, &max);
		done = (max == 0);
	} while (!done);
	namedWindow("Skeleton", CV_WINDOW_NORMAL);
	dilate(skel, skel, element2);
	imshow("Skeleton", skel);
	waitKey(0);
	// dilate(skel, skel, element2);
	// imshow("Skeleton", skel);
	// waitKey(0);

	int groups = 0;
	vector<vector<int>> group_map(skel.cols, vector<int>(skel.rows, 0));
	unsigned char *input = (unsigned char*)(skel.data);
	namedWindow( "Exploration", CV_WINDOW_NORMAL ); 	
	for(int j = 0;j < skel.rows;j++){
	    for(int i = 0;i < skel.cols;i++){
	        unsigned char b = input[skel.step * j + i ];
	        if (b == 255) {
	        	Point cur = Point(i, j);
	        	groups++;
		        if (group_map[i][j] == 0) {
		        	group_map[i][j] = groups;
		        	vector<Point> nodes = image_explore(skel, input, group_map, groups, i, j);
		        	skeletons.push_back(nodes);
		        }
	        }
	    }
	}

	cout << "There are " << skeletons.size() << " letters." << endl;

	for (int i = 0; i < skeletons.size(); ++i) {
		for (int j = 0; j < skeletons.size(); ++j) {
			cout << skeletons[i][j] << " ";
		}
		cout << endl;
	}
	

	const char* source_window = "Source"; 	
	namedWindow( source_window, CV_WINDOW_NORMAL ); 	
	imshow( source_window, src_gray );
	state->opencv_initialized = true;

	createTrackbar( " Thresh:", "Source", &thresh, max_thresh, thresh_callback );
	
    createTrackbar( "Canny On:\n", "Source",
                  &canny, max_kernel_size,
                  thresh_callback );

    createTrackbar( "Contour", "Source",
                  &curContour, maxContour,
                  thresh_callback );

	thresh_callback( 0, 0 );

	waitKey(0);
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
	// medianBlur(dst, dst, 7);

	/// Find contours
	findContours( dst, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0) );
	// vector<vector<Point> > contours_poly( contours.size() );
	// for (int i = 0; i < contours.size(); ++i) {
	// 	approxPolyDP( Mat(contours[i]), contours[i], 3, true );
	// }

	// Delete the image frame contour
	if (canny == 0)
		for (int i = 0; i < contours.size(); ++i) {
			if (src.rows * 4 * 0.9 <= contours[i].size() && contours[i].size() <= src.rows * 4 * 1.1) {
				contours.erase(contours.begin() + i);
				break;
			}
		}

	/// Output contours
	cout << "Number of contours " << contours.size() << endl;
	return contours;
}


void thresh_callback(int, void* )
{
	double imageSize = 80;
	resize(src_gray,src_gray,Size(),imageSize/src_gray.cols,imageSize/src_gray.rows,INTER_AREA);
	
	vector<vector<Point> > contours = GetContours(src_gray);

	maxContour = contours.size() - 1;

	/// Draw contours
	Mat drawing = Mat::zeros( dst.size(), CV_8UC3 );

	if (curContour > contours.size() -1 ){
		for( unsigned int i = 0; i< contours.size(); i++ ){
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			//cout << contours[i].size() << endl;
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

}

Point adapt_angle(Point p) {
	Point new_p;    
    new_p.x = -p.x;
    new_p.y = -(p.y - 0.1 * p.x);
    cout << "Point: " << p << endl;
    cout << "Modified point: " << new_p << endl;
    return new_p;
}

double adjustHeight(double y){
	double height;
	double low = .113;
	double med = .117;//.119
	double high = .12; //.123
	height = low;
	if (y >60){
		height = med;
	}
	else if (y >120){
		height = high;
	}
	Picasso.drawH = height;
	return height;
}

void DrawBot::draw(bool use_skeleton){
	vector<vector<Point> >& contours = use_skeleton ? image_contours : skeletons;
	// if (use_skeleton)
	// 	contours = image_contours;
	// else
	// 	contours = skeletons;

	double imageSize = 80;

	double wait_t = 150000;//150000
	double stepSize, stepX, stepY;
	Point prev, cur, diff;
	vector<float> points;
	int totalPoints = 0;
	double height = .123;
  	Arm.stand();

	cout << "Ready" << endl;
	for (unsigned int i = 0; i < contours.size(); i++){
		//if(hierarchy[i][3] >= 0){
			height = .12;
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

			prev = contours[i][0];
			contours[i].push_back(prev);
			prev.x = prev.x*imageSize/src_gray.cols;
			prev.y = prev.y*imageSize/src_gray.rows;
			

			// lift arm to start point of current contour
			double x_dist = -25;
			double y_dist = 63;
			double scale = 1000;
			double start_x = ((double)contours[i][0].x*imageSize/src_gray.cols+ x_dist)/scale;
			double start_y = ((double)contours[i][0].y*imageSize/src_gray.rows + y_dist)/scale;


			Arm.move_to(start_x, start_y, hoverH);
			usleep(wait_t*5);
			Arm.move_to(start_x, start_y, height);
			usleep(wait_t);

			for (unsigned int j = 0; j < contours[i].size(); j++){
				cout << contours[i][j] << " " << endl;
				cur = contours[i][j];
				cur.x = cur.x*imageSize/src_gray.cols;
				cur.y = cur.y*imageSize/src_gray.rows;
				diff = cur - prev; 

				if (abs(diff.x) > abs(diff.y))stepSize = abs(diff.x);
				else stepSize = abs(diff.y);

				stepX = ((double)diff.x)/stepSize;
				stepY = ((double)diff.y)/stepSize;


				for( double k = 1; k < stepSize; k++){
					height = adjustHeight((double)prev.y + y_dist + k*stepY);
					cout << " midpoint with stepY= " << stepY*k << " ";
					Arm.move_to(((double)prev.x  + x_dist + k*stepX)/scale, ((double)prev.y + y_dist + k*stepY)/scale, height  );
					usleep(wait_t);
				}
				height = adjustHeight((double)cur.y + y_dist);		

				cout << ((double)cur.x + x_dist)/scale << " " << ((double)cur.y + y_dist)/scale << endl;
				Arm.move_to(((double)cur.x + x_dist)/scale, ((double)cur.y + y_dist+2)/scale, height);
				usleep(wait_t);
				prev = cur;
				totalPoints++;
			}
			cout << endl;
			cout << "Contour " << i << " length is " << contours[i].size() << endl;
			usleep(wait_t*3);
			// Arm.move_to(Arm.cmd_position[0], Arm.cmd_position[1] + 0.005, hoverH  );
			Arm.cmd_angles[1] -= pi/12; 
			Arm.move_joints(Arm.cmd_angles);
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
        drawH = 0.12;

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


	cout << "\n============ MAIN PROGRAM RUNNING =============\n";
	
	//Picasso.draw();

	// Command loop
	while(true) {
	    double a, b, c;
	    string cmd;
		
	    cout << "command > ";
    	cin >> cmd;

	    if (cmd == "draw") {
	        cout << "drawing figure" << endl;
	        Mat fliped_src;
			flip(src_gray, fliped_src, 1);
			image_contours = GetContours(fliped_src);

	        Picasso.draw(false);

	    } else if (cmd == "square" || cmd == "circle" || cmd == "sine") {
	        cout << "drawing " << cmd << endl;
	    	Picasso.basic_shape(cmd);

	    } else if (cmd == "skeleton") {
	    	cout << "drawing skeleton of input picture!" << endl;
	    	Picasso.draw(true);

	    } else if (cmd == "quit") {
	        Arm.running = false;
	        break;
	    }
	}

	// pthread_join (vx_thread, NULL);
	return 0;
}
