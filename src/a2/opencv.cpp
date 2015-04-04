#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

/// Function header
void thresh_callback(int, void* );

/** @function main */
int main( int argc, char** argv )
{
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
  int totalPoints = 0;
  Point prev;
  Point cur; 
  for (int i = 0; i < contours.size(); i++){
    prev.x = contours[i][0].x;
    prev.y = contours[i][0].y;
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    for (int j = 0; j < contours[i].size(); j++){
      cout << contours[i][j] << " ";
      cur.x = contours[i][j].x;
      cur.y = contours[i][j].y;      
      //cout << "[" << cur.x - prev.x << "," << cur.y - prev.y << "]" << " ";
      line(drawing,cur,prev,color,1,8,0);
      prev = cur;
      totalPoints++;;
    } 
    cout << endl;
    //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    //drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    cout << contours[i].size() << endl;
  }
   cout << contours.size() << " " << totalPoints << endl;
  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}