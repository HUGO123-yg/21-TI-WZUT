#include "Pid.h"

static float pid_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float integrator_min,
              float integrator_max,
              float output_min,
              float output_max)
{
    if (0 == pid)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->integrator_min = integrator_min;
    pid->integrator_max = integrator_max;
    pid->output_min = output_min;
    pid->output_max = output_max;
}

void pid_reset(pid_controller_t *pid)
{
    if (0 != pid)
    {
        pid->integrator = 0.0f;
    }
}

float pid_update(pid_controller_t *pid,
                 float error,
                 float derivative_error,
                 float dt_s)
{
    float proportional;
    float derivative;
    float candidate_integrator;
    float candidate_output;
    uint8 output_saturated_high;
    uint8 output_saturated_low;

    if ((0 == pid) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    proportional = pid->kp * error;
    derivative = pid->kd * derivative_error;
    candidate_integrator = pid_clamp(pid->integrator
                                     + pid->ki * error * dt_s,
                                     pid->integrator_min,
                                     pid->integrator_max);
    candidate_output = proportional + candidate_integrator + derivative;
    output_saturated_high = (uint8)(candidate_output > pid->output_max);
    output_saturated_low = (uint8)(candidate_output < pid->output_min);

    // Conditional integration prevents the integral term from driving farther
    // into saturation while still allowing it to unwind in the opposite direction.
    if ((!output_saturated_high && !output_saturated_low)
        || (output_saturated_high && (error < 0.0f))
        || (output_saturated_low && (error > 0.0f)))
    {
        pid->integrator = candidate_integrator;
    }

    return pid_clamp(proportional + pid->integrator + derivative,
                     pid->output_min,
                     pid->output_max);
}
