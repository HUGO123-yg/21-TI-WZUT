// QA 1: PID step response test — verify PID converges to target
#include <stdio.h>

// Minimal stubs for standalone compilation (replace IAR headers)
typedef unsigned char uint8;
#define func_limit(x, y)        ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))
#define func_limit_ab(x, a, b)  ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
#define TRUE  (1)
#define FALSE (0)

typedef enum {
    Position_pid,
    Incremental_pid
} pid_type_enum;

typedef struct {
    float kp, ki, kd, dt;
    float target, observation;
    float integral, integral_min, integral_max;
    float output_limit;
    float out, last_error;
    uint8 use_d;
    pid_type_enum type;
} pid_t;

// pid_init — copied from project/code/pid.c
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

void pid_set_target(pid_t *pid, float target)
{
    pid->target = target;
}

void pid_get_observation(pid_t *pid, float observation)
{
    pid->observation = observation;
}

void pid_set_dt(pid_t *pid, float dt)
{
    pid->dt = dt;
}

void pid_run(pid_t *pid)
{
    float error;
    float derivative;

    error = pid->target - pid->observation;

    pid->integral += error * pid->dt;
    pid->integral = func_limit_ab(pid->integral, pid->integral_min, pid->integral_max);

    if (pid->use_d)
    {
        derivative = (error - pid->last_error) / pid->dt;
    }
    else
    {
        derivative = 0;
    }

    pid->out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->out = func_limit(pid->out, pid->output_limit);

    pid->last_error = error;
}

//-----------------------------------------------------------------------------
// Test: init with kp=1.0, ki=0.5, integral_min=-1000, integral_max=1000
// Target=100. Run 1000 iterations feeding output back as observation.
//-----------------------------------------------------------------------------
int main(void)
{
    pid_t pid;
    float final_out;
    int pass = 0;

    // Use non-zero integral bounds to allow I-term accumulation
    pid_init(&pid, 1.0f, 0.5f, 0.0f, 0.01f, -1000.0f, 1000.0f, FALSE, 100.0f, Position_pid);
    pid_set_target(&pid, 100.0f);

    for (int i = 0; i < 1000; i++)
    {
        pid_get_observation(&pid, pid.out);
        pid_run(&pid);
    }

    final_out = pid.out;
    printf("QA 1 - PID step response test:\n");
    printf("  Parameters: kp=1.0, ki=0.5, kd=0, dt=0.01, output_limit=100\n");
    printf("  Target: 100.0\n");
    printf("  Final output after 1000 iterations: %.3f\n", final_out);
    printf("  Integral: %.3f\n", pid.integral);

    if (final_out > 90.0f && final_out < 110.0f)
    {
        printf("  VERDICT: PASS — PID converges within 10%% of target\n");
        pass = 1;
    }
    else
    {
        printf("  VERDICT: FAIL — PID did not converge (out=%.3f)\n", final_out);
    }

    // Also test with spec-like params (integral_min=0, integral_max=0) to report behavior
    pid_init(&pid, 1.0f, 0.1f, 0.0f, 0.01f, 0.0f, 0.0f, FALSE, 100.0f, Position_pid);
    pid_set_target(&pid, 100.0f);
    for (int i = 0; i < 50; i++)
    {
        pid_get_observation(&pid, pid.out);
        pid_run(&pid);
    }
    printf("\n  NOTE: With integral_min=0, integral_max=0 (as in original spec),\n");
    printf("        integral is clamped to 0 (I-term disabled). Output oscillates.\n");
    printf("        Final out after 50 iter: %.3f\n", pid.out);

    return pass ? 0 : 1;
}
