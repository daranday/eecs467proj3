#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <unordered_set>

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
#include "a3_blob_detector.h"
#include "a3_inverse_kinematics.h"
#include "a3_image_to_arm_coord.h"
#include "a3_ai.h"

//lcm
#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/ttt_turn_t.hpp"


using namespace cv;
using namespace std;

Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

/// Function header
void thresh_callback(int, void* );

struct pair_hash {
    inline std::size_t operator()(const Point & v) const {
        return v.x+v.y*200;
    }
};

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


void* start_inverse_kinematics(void* user) {
    kin_main(state->argc, state->argv);
    return NULL;
}
/** @function main */
int main( int argc, char** argv )
{

  pthread_t  kinematics_thread;
  pthread_create (&kinematics_thread, NULL, start_inverse_kinematics, (void*)NULL);

  /// Load source image and convert it to gray
  if (argc == 1) src = imread("sud2.jpg");
  else src = imread( argv[1] );
  //resize(src,src,Size(),600./src.cols,600./src.rows,INTER_AREA);
  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  /// Create Window
  char* source_window = "Source";
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, src );

  createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh, thresh_callback );
  thresh_callback( 0, 0 );

  waitKey(0);
  return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  double imageSize = 150;
  resize(src_gray,src_gray,Size(),imageSize/src_gray.cols,imageSize/src_gray.rows,INTER_AREA);
  /// Detect edges using canny
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  cout << contours.size() << " ";
  if (contours.size() != 0) cout << contours[0].size() << endl;
  /*
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }
  */

  unordered_set<Point, pair_hash> endPoints;
  double h = 0.103;
  int wait_t = 500000;
  int totalPoints = 0;
  Point prev;
  Point cur;

  // stand_arm();
  //usleep(wait_t*2);
  stand_arm();

  for (int i = 0; i < contours.size(); i++){
    prev.x = contours[i][0].x;
    prev.y = contours[i][0].y;
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

    // lift arm to start point of current contour
    double start_x = -1*((double)contours[i][0].x + 50)/1000.;
    double start_y = ((double)contours[i][0].y )/1000.;
    move_to(start_x, start_y, h);

    for (int j = 0; j < contours[i].size() / 2; j++){
      endPoints.insert(cur);
      cout << contours[i][j] << " ";
      cur.x = contours[i][j].x;
      cur.y = contours[i][j].y;      
      // cout << "[" << cur.x - prev.x << "," << cur.y - prev.y << "]" << " ";
      line(drawing,cur,prev,color,1,8,0);

      prev = cur;
      totalPoints++;
  
      cout << -1*((double)cur.x + 50)/1000. << " " << ((double)cur.y )/1000. << endl;
      move_to(-1*((double)cur.x + 50)/1000., ((double)cur.y)/1000., h);
      usleep(wait_t);
    }
    cout << endl;
    //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    //drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    cout << contours[i].size() << endl;
    stand_arm();
    //relax_arm();
    //usleep(wait_t);
  }
  cout << contours.size() << " " << totalPoints << endl;
  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}