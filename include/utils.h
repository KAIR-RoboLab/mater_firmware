#ifndef UTILS__H
#define UTILS__H

#include <math.h>

#include "robot_constants.h"

#define US_TO_MS(x) (x / 1000)
#define US_TO_S_D(x) (double(x) / 1000000.0)
#define NS_TO_S_D(x) (double(x) / 1000000000.0)

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }

#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

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