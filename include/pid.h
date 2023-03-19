#ifndef PID__H
#define PID__H

typedef struct
{
  double kp;
  double ki;
  double kd;
  double out_min;
  double out_max;
  double dt;
  double _private_sum;
  double _private_last_error;
} PID_stat_t;

double pid_compute(PID_stat_t *pid_stat, double error);

#endif // PID__H