#include "imagesource/image_u32.h"
#include "imagesource/image_u8x3.h"
#include <stack>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include "a2_blob_detector.h"
using namespace std;

blob_detect::blob_detect(){
  region = 1;
}

blob_detect::~blob_detect(){
  image_u8x3_destroy( image_83 );
  image_u8x3_destroy( region_83 );
}

void blob_detect::run(image_u32_t *image_32){
  get_u8x3(image_32);
  //image_u32_destroy(image_32);
  region = 1;
  region_data.clear();
  region_83 = image_u8x3_create( image_83->width, image_83->height );
  x_mask_min =  y_mask_min =  0;
  x_mask_max = image_83->width;
  y_mask_max = image_83->height;
  run_detector();
}

void blob_detect::get_mask( vector<int> &input ){
  x_mask_min = input[0];
  x_mask_max = input[2];
  y_mask_min = input[1];
  y_mask_max = input[3];

}

void blob_detect::get_colors( vector<vector<double>> &input){
  thresh color;
  for(size_t i = 0; i < input.size(); i++){
    color.Hmin = input[i][0];
    color.Hmax = input[i][1];
    color.Smin = input[i][2];
    color.Smax = input[i][3];
    color.Vmin = input[i][4];
    color.Vmax = input[i][5];
    colors.push_back(color);
  }

  
  /*
  ifstream input
  input.open( file );
  while( !input.eof() ){
    input >> color.Hmin >> color.Hmax >> color.Smin >> color.Smax >> color.Vmin >> color.Vmax;
    colors.push_back(color);
  }
  cout << "colors " << colors.size() << endl;
  cout << colors[1].Hmin << " " << colors[1].Hmax << " " << colors[1].Smin << " " << colors[1].Smax << " " << colors[1].Vmin << " " << colors[1].Vmax << endl;
  */
}

int blob_detect::which_color( HSV input ){
  for(int i = 0; i < int(colors.size()); i++){
    if( input.H >= colors[i].Hmin && input.H <= colors[i].Hmax &&
    	input.S >= colors[i].Smin && input.S <= colors[i].Smax &&
    	input.V >= colors[i].Vmin && input.V <= colors[i].Vmax){
        return i;
    }
  }
  return -1;

}

//read in some image and convert it to u8x3 format
void blob_detect::get_u8x3(image_u32_t *image_32){
  uint8_t R,G,B;
  cout << image_32->width << ",  " << image_32->height << endl;
  image_83 = image_u8x3_create( image_32->width, image_32->height );

  //convert u32 image to a u8x3
  //u8x3 is format R, G, B
  //u32 is a, g, r, b
  char *bufc = (char*)image_32->buf;

  for(int y = 0; y < image_32->height; y++){
    for(int x = 0; x < image_32->width; x++){
      B = bufc[4*(image_32->stride*y+x)+ 0];
      G = bufc[4*(image_32->stride*y+x)+ 1];
      R = bufc[4*(image_32->stride*y+x)+ 2];
      
      image_83->buf[3*x+y*image_83->stride + 2] = R;
      image_83->buf[3*x+y*image_83->stride + 1] = G;
      image_83->buf[3*x+y*image_83->stride + 0] = B;
    }
  }

  //output for debugging
  //image_u8x3_write_pnm( image_83, "pic_out.ppm" );

  //destroy image
  // image_u32_destroy( image_32 );
}
  

//returns true if the pixel is inbounds and the color we're looking for
bool blob_detect::good_pixel(int pixel, int c_index){
  HSV temp;
  if (pixel >= 0 && pixel < (image_83->width*3 * image_83->height*image_83->stride) && region_83->buf[pixel] == 0 ){
    RGB_to_HSV( image_83->buf[pixel], image_83->buf[pixel+1], image_83->buf[pixel+2], temp);
      
    if( colors[c_index].Hmin <= temp.H && colors[c_index].Hmax >= temp.H &&
    	colors[c_index].Smin <= temp.S && colors[c_index].Smax >= temp.S &&
    	colors[c_index].Vmin <= temp.V && colors[c_index].Vmax >= temp.V ){
        //cout << "here" << endl;
        return true;
    }
  }
  return false;

}

void blob_detect::connect_pixels(int pixel, int c_index){
  int temp = 0;
  int sum = pixel, sum_x = (pixel % image_83->stride), sum_y = (pixel / image_83->stride), area = 0;
  r_data t;
  stack<int> pixels;
  pixels.push( pixel );
  region_83->buf[pixel] = region;
  area++;
  
  

  while( !pixels.empty() ) {
    temp = pixels.top();
    pixels.pop();

    //Find the neighboring pixels
    //NW
    if( good_pixel( temp - image_83->stride - 3, c_index ) ){
      pixels.push( temp - image_83->stride - 3);
      region_83->buf[ pixels.top() ] = region;
      area++;

      sum += pixels.top();
      sum_x += (pixels.top() % image_83->stride);
      sum_y += (pixels.top() / image_83->stride);
      
    }
    //N
    if( good_pixel( temp - image_83->stride, c_index ) ){
      pixels.push( temp - image_83->stride);
      region_83->buf[ pixels.top() ] = region;
      area++;
      
      sum += pixels.top();
      sum_x += (pixels.top() % image_83->stride);
      sum_y += (pixels.top() / image_83->stride);
      
    }
    //NE
    if( good_pixel( temp - image_83->stride + 3, c_index ) ){
      pixels.push( temp - image_83->stride + 3 );
      region_83->buf[ pixels.top() ] = region;
      area++;

      sum += pixels.top();
      sum_x += (pixels.top() % image_83->stride);
      sum_y += (pixels.top() / image_83->stride);
      
    }
    //E
    if( good_pixel( temp + 3, c_index ) ){
      pixels.push( temp + 3 );
      region_83->buf[ pixels.top() ] = region;
      area++;

      sum += pixels.top();
      sum_x += (pixels.top() % image_83->stride);
      sum_y += (pixels.top() / image_83->stride);
      
    }
    //SE
    if( good_pixel( temp + image_83->stride + 3, c_index ) ){
      pixels.push( temp + image_83->stride + 3 );
      region_83->buf[ pixels.top() ] = region;
      area++;

      sum += pixels.top();
      sum_x += (pixels.top() % image_83->stride);
      sum_y += (pixels.top() / image_83->stride);
     
    }
    //S
    if( good_pixel( temp + image_83->stride, c_index ) ){
      pixels.push( temp + image_83->stride );
      region_83->buf[ pixels.top() ] = region;
      area++;

      sum += pixels.top();
      sum_x += (pixels.top() % image_83->stride);
      sum_y += (pixels.top() / image_83->stride);
      
    }
    //SW
    if( good_pixel( temp + image_83->stride - 3, c_index ) ){
      pixels.push( temp + image_83->stride - 3 );
      region_83->buf[ pixels.top() ] = region;
      area++;

      sum += pixels.top();
      sum_x += (pixels.top() % image_83->stride);
      sum_y += (pixels.top() / image_83->stride);
      
    }
    //W
    if( good_pixel( temp - 3, c_index ) ){
      pixels.push( temp - 3 );
      region_83->buf[ pixels.top() ] = region;
      area++;

      sum += pixels.top();
      sum_x += (pixels.top() % image_83->stride);
      sum_y += (pixels.top() / image_83->stride);
 
    }
      
	
  }


  double center = double(sum/3)/double(area);
  double x_center = double(sum_x)/double(area);
  double y_center = double(sum_y)/double(area);

  

  t.area = area;
  t.x = x_center / 3;
  t.y = y_center;
  // t.y = center / double(image_83->stride/3);
  area = 0;
    
  region_data.push_back( t );
}

void blob_detect::run_detector(){
    int index = 0;
    int color_index = -1;
    HSV hsv_value;
    //iterate through all elements of the buffer to find all pixels
    for(int y = y_mask_min; y < y_mask_max; y++){
        for(int x = x_mask_min; x < x_mask_max; x++){
            //looking for just green now
            index = 3*x + y*image_83->stride;
            if( region_83->buf[ index] == 0 ){
                RGB_to_HSV( image_83->buf[index], image_83->buf[index+1], image_83->buf[index+2], hsv_value);

                color_index = which_color(hsv_value);

                  
                if( color_index != -1){
                    connect_pixels( index, color_index);
                    region_data[region_data.size()-1].label = color_index;
                    region_data[region_data.size()-1].H = (colors[color_index].Hmin + colors[color_index].Hmax)/2.0;
                    region_data[region_data.size()-1].S = (colors[color_index].Smin + colors[color_index].Smax)/2.0;
                    region_data[region_data.size()-1].V = (colors[color_index].Vmin + colors[color_index].Vmax)/2.0;
                    region+=40;
                }
            }
        }
    }
	
  //output for debugging
  image_u8x3_write_pnm( region_83, "pic_out.ppm" );

  for(size_t i = 0; i < region_data.size(); i++){
    if(region_data[i].area > 200 )
      cout << "region: " << i << " area: " << region_data[i].area << " x: " << region_data[i].x << " y: " << region_data[i].y << endl;
    
  }
}

// int _main(){
//   // vector<vector<double>> color(1);
//   // color[0].resize(6);
//   // color[0][0] = 160;
//   // color[0][1] = 210;
//   // color[0][2] = 15;
//   // color[0][3] = 60;
//   // color[0][4] = 60; 
//   // color[0][5] = 100;

//   //  blob_detect B;
//   //  B.run();
//    return 0;
// }


//convert HSV values to usable RGB
void RGB_to_HSV( uint8_t R, uint8_t G, uint8_t B, HSV &out ){
    double b = double(B)/255.0;
    double g = double(G)/255.0;
    double r = double(R)/255.0;
    double rgb_max = std::max(r, std::max(g, b));
    double rgb_min = std::min(r, std::min(g, b));
    double delta = rgb_max - rgb_min;
    out.S = delta / (rgb_max + 1e-20f);
    out.V = rgb_max;

    double hue;
    if (r == rgb_max)
        hue = (g - b) / (delta + 1e-20f);
    else if (g == rgb_max)
        hue = 2 + (b - r) / (delta + 1e-20f);
    else
        hue = 4 + (r - g) / (delta + 1e-20f);
    if (hue < 0)
        hue += 6.f;
    out.H = hue * (1.f / 6.f);






  // double Cmax, Cmin, d;
  // double R1, G1, B1;

  // //cout << "R: " << int(R) << " G: " << int(G) << " B: " << int(B) << endl;

  // //get into a %
  // R1 = 100*double(R)/255.0;
  // G1 = 100*double(G)/255.0;
  // B1 = 100*double(B)/255.0;


  // //find Cmax
  // Cmax = (G1 < R1) ? R1 : G1;
  // Cmax = (Cmax > B1) ? Cmax : B1; 

  // //find Cmin
  // Cmin = ( R1 < G1 ) ? R1 : G1;
  // Cmin = ( Cmin < B1 ) ? Cmin : B1; 

  // d = Cmax- Cmin;

  // //calculate saturation
  // if( Cmax )
  //   out.S = 100*(d/Cmax);
  // else
  //   out.S = 0;

  // out.V = Cmax;

  // //calculate hue
  // if( Cmax > 0.0 ) { 
  //   out.S = 100*(d / Cmax);                  
  // } 
  // else {
  //   out.S = 0.0;
  //   out.H = NAN;                           
  //   return;
  // }
  // if( R1 >= Cmax )                           
  //   out.H = ( G1 - B1 ) / d;        
  // else if( G1 >= Cmax )
  //   out.H = 2.0 + ( B1 - R1 ) / d; 
  // else
  //   out.H = 4.0 + ( R1 - G1 ) / d;

  // out.H *=60.0;

  // if( out.H < 0.0 )
  //   out.H += 360.0;
}