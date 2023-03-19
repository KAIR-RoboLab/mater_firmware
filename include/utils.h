#ifndef UTILS__H
#define UTILS__H

#include <math.h>

#include "robot_constants.h"

#define US_TO_MS(x) (x / 1000)
#define US_TO_S_D(x) (double(x) / 1000000.0)

void euler_to_quat(float x, float y, float z, double *q);

inline double wrap_angle(float rad)
{
  return atan2(sin(rad), cos(rad));
}

inline double tic_to_rad(long tics)
{
  return double(tics) / double(gear_reduction * tics_per_rotation) * M_PI * 2.0;
}

#endif // UTILS__H