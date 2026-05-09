#ifndef _pid_h_
#define _pid_h_

#include "zf_common_headfile.h"

// PID 类型枚举
typedef enum
{
    Position_pid,           // 位置式PID
    Incremental_pid         // 增量式PID
} pid_type_enum;

// PID 控制结构体
typedef struct
{
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float dt;               // 控制周期

    float target;           // 目标值
    float observation;      // 观测值
    float integral;         // 积分累加值
    float integral_min;     // 积分下限
    float integral_max;     // 积分上限
    float output_limit;     // 输出限幅

    float out;              // PID 输出
    float last_error;       // 上一次误差
    uint8 use_d;            // 是否使用微分项
    pid_type_enum type;     // PID 类型
} pid_t;

extern void pid_init(pid_t *pid, float kp, float ki, float kd, float dt,
                     float integral_min, float integral_max, uint8 use_d,
                     float output_limit, pid_type_enum type);
extern void pid_set_target(pid_t *pid, float target);
extern void pid_get_observation(pid_t *pid, float observation);
extern void pid_set_dt(pid_t *pid, float dt);
extern void pid_run(pid_t *pid);

#endif
