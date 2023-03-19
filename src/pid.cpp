#include "pid.h"

#define max(a, b)           \
  ({                        \
    __typeof__(a) _a = (a); \
    __typeof__(b) _b = (b); \
    _a >= _b ? _a : _b;      \
  })

#define min(a, b)           \
  ({                        \
    __typeof__(a) _a = (a); \
    __typeof__(b) _b = (b); \
    _a <= _b ? _a : _b;      \
  })

double pid_compute(PID_stat_t *pid_stat, double error)
{
  // Proportional input
  double p_input = error * pid_stat->kp;

  // Trapezoidal integration
  pid_stat->_private_sum += 0.5 * (error + pid_stat->_private_last_error) * pid_stat->dt;
  // Integral clamping
  pid_stat->_private_sum = max(min(pid_stat->_private_sum, pid_stat->out_max /  pid_stat->ki), pid_stat->out_min /  pid_stat->ki);
  double i_input = pid_stat->_private_sum * pid_stat->ki;

  // Derivative input
  double d_input = (error - pid_stat->_private_last_error) / pid_stat->dt * pid_stat->kd;
  pid_stat->_private_last_error = error;

  double out = p_input + i_input + d_input;
  return max(min(out, pid_stat->out_max), pid_stat->out_min);
}