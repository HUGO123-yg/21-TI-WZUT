/*
 * pid.c - PID 控制器实现
 * Copyright (c) 2024
 */

#include "pid.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       pid_init
// 功能说明       PID 控制器参数初始化
// 输入参数       pid               PID 控制结构体指针
//                kp                比例系数
//                ki                积分系数
//                kd                微分系数
//                dt                控制周期
//                integral_min      积分下限
//                integral_max      积分上限
//                use_d             是否使用微分项 (TRUE/FALSE)
//                output_limit      输出限幅值
//                type              PID 类型 (Position_pid / Incremental_pid)
// 返回参数       void
// 使用示例       pid_init(&gyro, 0.742, 9.185, 0, 0.002, 0, 0, FALSE, 10000, Position_pid);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       pid_set_target
// 功能说明       设置 PID 目标值
// 输入参数       pid                PID 控制结构体指针
//                target             目标值
// 返回参数       void
// 使用示例       pid_set_target(&gyro, Angle_Out);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void pid_set_target(pid_t *pid, float target)
{
    pid->target = target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       pid_get_observation
// 功能说明       设置 PID 观测值
// 输入参数       pid                PID 控制结构体指针
//                observation        观测值
// 返回参数       void
// 使用示例       pid_get_observation(&gyro, imu660ra_gyro_y);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void pid_get_observation(pid_t *pid, float observation)
{
    pid->observation = observation;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       pid_set_dt
// 功能说明       设置 PID 控制周期
// 输入参数       pid                PID 控制结构体指针
//                dt                 控制周期
// 返回参数       void
// 使用示例       pid_set_dt(&gyro, dt_pid_gyro);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void pid_set_dt(pid_t *pid, float dt)
{
    pid->dt = dt;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       pid_run
// 功能说明       执行 PID 计算（位置式）
// 输入参数       pid                PID 控制结构体指针
// 返回参数       void
// 使用示例       pid_run(&gyro);
// 备注信息       位置式 PID 算法：
//                error = target - observation
//                integral += error * dt，积分限幅至 [integral_min, integral_max]
//                derivative = use_d ? (error - last_error) / dt : 0
//                out = kp * error + ki * integral + kd * derivative，输出限幅至 [-output_limit, +output_limit]
//-------------------------------------------------------------------------------------------------------------------
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
