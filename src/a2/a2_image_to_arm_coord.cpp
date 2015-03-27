#include "a2_image_to_arm_coord.h"
#include <gsl/gsl_linalg.h>
#include <stack>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
using namespace std;

/*things to think about
1- the dimensions of the blue calibration squares should be inputted for each board 
2- program will take the coordinates from the arm's frame and translate them to and from the board's frame

For now - measure out the board frame, and hard code it. Create the program that converts board Arm XY to board XY
*/

coord_convert::coord_convert(){
}

void coord_convert::c2a_get_factors(double a[], double c[]){
  
  int n = 3;
  matd_t *A = matd_create_data(n, n, a);
  matd_t *C = matd_create_data(n, n, c);
  matd_t *invC = matd_inverse(C);
  c2a_Conv = matd_multiply(A, invC);
  matd_t *invA = matd_inverse(A);
  a2c_Conv = matd_multiply(C, invA);
}

void coord_convert::camera_to_arm(double a[], double c[]){
  int n = 3;

  matd_t *C = matd_create_data(n, 1, c);
  matd_t *A = matd_multiply(c2a_Conv, C);
  a[0] = matd_get(A, 0, 0);
  a[1] = matd_get(A, 0, 1);
  a[2] = matd_get(A, 0, 2);
}

void coord_convert::arm_to_camera(double c[], double a[]){
  int n = 3;

  matd_t *A = matd_create_data(n, 1, a);
  matd_t *C = matd_multiply(a2c_Conv, A);
  c[0] = matd_get(C, 0, 0);
  c[1] = matd_get(C, 0, 1);
  c[2] = matd_get(C, 0, 2);
}

/*int main(){
  coord_convert C;
  double A[9] = { -125.0, -125.0, 125.0,
		  0.0, 250.0, 0.0,
		  1.0, 1.0, 1.0};

  double B[9] = { 0.0, 0.0, 250.0,
		  0.0, 250.0, 0.0,
		  1.0, 1.0, 1.0};

  C.b2a_get_factors( A, B);

  double b1[3] = { 125.0, 125.0, 1.0 };
  double a1[3] = { 0.0, 125, 1.0};
  double a[3] = {-1, -1, -1};
  
  C.board_to_arm( a, b1 );

  cout << "Following two lines should be identical" << endl;
  cout << a[0] << " " << a[1] << " " << a[2] << endl;
  cout << a1[0] << " " << a1[1] << " " << a1[2] << endl;

  

  return 0;
}*/
