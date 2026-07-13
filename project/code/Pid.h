#ifndef PROJECT_PID_H
#define PROJECT_PID_H

#include "zf_common_typedef.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integrator;
    float integrator_min;
    float integrator_max;
    float output_min;
    float output_max;
} pid_controller_t;

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float integrator_min,
              float integrator_max,
              float output_min,
              float output_max);
void pid_reset(pid_controller_t *pid);

// derivative_error is supplied by the caller. For an angle controller whose
// reference changes slowly, pass -measured_angular_rate to avoid differentiating
// a noisy angle estimate.
float pid_update(pid_controller_t *pid,
                 float error,
                 float derivative_error,
                 float dt_s);

#endif
