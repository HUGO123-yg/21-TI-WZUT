#ifndef __BALANCE_STEERING_H__
#define __BALANCE_STEERING_H__

#include "zf_common_headfile.h"
#include "pid.h"

extern pid_struct_t turn_pid;
extern float yaw_target;
extern float yaw_current;
extern float left_duty;
extern float right_duty;

void balance_steering_init(void);
void balance_steering_calc(float base_duty, float gyro_z, float yaw_error);
float yaw_angle_diff(float target, float current);
void balance_steering_set_yaw_target(float target);

#endif
