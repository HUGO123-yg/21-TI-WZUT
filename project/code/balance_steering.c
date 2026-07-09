#include "balance_steering.h"
#include "Common_peripherals.h"

pid_struct_t turn_pid;
float yaw_target = 0.0f;
float yaw_current = 0.0f;
float left_duty = 0.0f;
float right_duty = 0.0f;

void balance_steering_init(void)
{
    pid_init(&turn_pid, 0.088f, 0.0f, 0.0580f, 5000.0f, 10000.0f);
}

void balance_steering_calc(float base_duty, float gyro_z, float yaw_error)
{
    pid_calc(&turn_pid, 0.0f, yaw_error * car_speed);

    left_duty = base_duty + gyro_z / 2.0f - (float)turn_pid.output;
    right_duty = base_duty - gyro_z / 2.0f + (float)turn_pid.output;
}

float yaw_angle_diff(float target, float current)
{
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

void balance_steering_set_yaw_target(float target)
{
    yaw_target = target;
}
