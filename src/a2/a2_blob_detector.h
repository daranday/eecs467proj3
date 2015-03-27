#ifndef __A2_BLOB_DETECTOR__
#define __A2_BLOB_DETECTOR__

#include "imagesource/image_u32.h"
#include "imagesource/image_u8x3.h"
#include <stack>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;

//this blob detector will only look for the blue calibration boxes

//is HSV for HSV and RGB for RGB, ABC is a placeholder
struct HSV{
  double H;
  double S;
  double V;
};

struct RGB{
  uint8_t R;
  uint8_t G;
  uint8_t B;
};

struct thresh{
  double Hmin;
  double Hmax;
  double Smin;
  double Smax;
  double Vmin;
  double Vmax;
};

void HSV_to_RGB( double H, double S, double V );
void RGB_to_HSV( uint8_t R, uint8_t G, uint8_t B, HSV &out );

struct r_data{
  int area;
  double x, y;
  int label;
  double H;
  double S;
  double V;
  //center of mass will be intersection of these 4 points

  r_data(){
    area = 0;
    x = y = 0;
  }
};

class blob_detect{
public:
  image_u8x3 *image_83;
  image_u8x3 *region_83;
  int region;
  string file;
  //index in region is the region's area
  vector<r_data> region_data;
  vector<thresh> colors;

  int x_mask_min, x_mask_max, y_mask_min, y_mask_max;

  blob_detect();
  ~blob_detect();
  void get_mask( vector<int> &input);
  void run(image_u32_t *image_32);
  void get_colors( vector<vector<double>> &input);
  int which_color( HSV input );
  void get_u8x3(image_u32_t *image_32);
  bool good_pixel(int pixel, int c_index);
  void connect_pixels(int pixel, int c_index);
  void run_detector();

};

#endif
