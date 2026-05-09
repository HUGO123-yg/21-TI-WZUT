// QA 2: PID integral clamp test — verify integral stays within [integral_min, integral_max]
#include <stdio.h>

typedef unsigned char uint8;
#define func_limit(x, y)        ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))
#define func_limit_ab(x, a, b)  ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
#define TRUE  (1)
#define FALSE (0)

typedef enum { Position_pid, Incremental_pid } pid_type_enum;

typedef struct {
    float kp, ki, kd, dt;
    float target, observation;
    float integral, integral_min, integral_max;
    float output_limit;
    float out, last_error;
    uint8 use_d;
    pid_type_enum type;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float dt,
              float integral_min, float integral_max, uint8 use_d,
              float output_limit, pid_type_enum type)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->target = 0;
    pid->observation = 0;
    pid->integral = 0;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->output_limit = output_limit;
    pid->out = 0;
    pid->last_error = 0;
    pid->use_d = use_d;
    pid->type = type;
}

void pid_set_target(pid_t *pid, float target) { pid->target = target; }
void pid_get_observation(pid_t *pid, float observation) { pid->observation = observation; }

void pid_run(pid_t *pid)
{
    float error = pid->target - pid->observation;
    pid->integral += error * pid->dt;
    pid->integral = func_limit_ab(pid->integral, pid->integral_min, pid->integral_max);
    float derivative = pid->use_d ? ((error - pid->last_error) / pid->dt) : 0;
    pid->out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->out = func_limit(pid->out, pid->output_limit);
    pid->last_error = error;
}

//-----------------------------------------------------------------------------
// Test: integral_max=50, set target far away, feed observation=0 (max error)
// Integral should accumulate but never exceed 50.
//-----------------------------------------------------------------------------
int main(void)
{
    pid_t pid;
    int clamp_violations = 0;
    float max_integral = 0;
    float min_integral = 0;

    // integral_max=50, integral_min=-50
    pid_init(&pid, 0.5f, 1.0f, 0.0f, 0.01f, -50.0f, 50.0f, FALSE, 200.0f, Position_pid);
    pid_set_target(&pid, 200.0f);  // target far above observation=0
    pid_get_observation(&pid, 0.0f);

    for (int i = 0; i < 10000; i++)
    {
        pid_run(&pid);
        if (pid.integral > max_integral) max_integral = pid.integral;
        if (pid.integral < min_integral) min_integral = pid.integral;
        if (pid.integral > pid.integral_max + 0.001f)
        {
            clamp_violations++;
            printf("  CLAMP VIOLATION at iter %d: integral=%.6f (max=%.1f)\n",
                   i, pid.integral, pid.integral_max);
        }
    }

    printf("QA 2 - PID integral clamp test:\n");
    printf("  Bounds: integral_min=-50, integral_max=50\n");
    printf("  Target=200, Observation=0 (constant max error)\n");
    printf("  Max integral observed: %.6f\n", max_integral);
    printf("  Min integral observed: %.6f\n", min_integral);
    printf("  Clamp violations: %d\n", clamp_violations);

    if (clamp_violations == 0 && max_integral <= 50.0001f && min_integral >= -50.0001f)
    {
        printf("  VERDICT: PASS — integral stays within bounds\n");
        return 0;
    }
    else
    {
        printf("  VERDICT: FAIL — integral exceeded bounds\n");
        return 1;
    }
}
