#ifndef __A2_IMAGE_TO_ARM_COORD__
#define __A2_IMAGE_TO_ARM_COORD__

#include <math/gsl_util_matrix.h>
#include <math/gsl_util_linalg.h>
#include <math/gsl_util_blas.h>
#include "math/matd.h"
#include <stack>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
using namespace std;

class coord_convert{
public:
	matd_t * c2a_Conv;
	matd_t * a2c_Conv;

	coord_convert();
	void c2a_get_factors(double a[], double c[]);
	void camera_to_arm(double a[], double c[]);
	void arm_to_camera(double c[], double a[]);
};

#endif