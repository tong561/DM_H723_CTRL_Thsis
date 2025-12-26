#ifndef __3RRS_H__
#define __3RRS_H__

#include "main.h"
unsigned char  ik_3rrs_no_yaw(double H, double X_roll, double Y_pitch,
                    const double* prev_theta /*©ин╙©у*/,
                    double theta_out[3]);

										
#endif
										
										