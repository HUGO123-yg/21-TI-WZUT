#ifndef _control_h_
#define _control_h_

// 控制模块头文件

#include "zf_common_headfile.h"
#include "servo.h"

//===== 通用工具宏 =====

#define DEG_TO_RAD (57.29577951308232f)    // 180/PI，弧度转角度的除数
#define ABS(x)     (((x) >= 0) ? (x) : -(x))
#define clip2(x, max) (((x) > (max)) ? (max) : ((x) < -(max)) ? -(max) : (x))

//===== 跳跃控制阶段配置 =====

// 跳跃控制阶段配置结构体
typedef struct
{
    int min;                // 阶段最小时间索引
    int max;                // 阶段最大时间索引
    void (*handler)(int);   // 阶段处理回调函数
    const char *name;       // 阶段名称
} jump_control_struct;

// [TODO: extern declarations and function prototypes will be added after companion modules are created]

//===== 配套模块头文件引用 =====
#include "pid.h"
#include "euler.h"
#include "foc.h"
#include "timer_control.h"
#include "servo_kinematics.h"

//===== LQR 控制器常量 =====
extern const float LQR_K[8];
extern const jump_control_struct jump_control_config[];
extern const uint8 jump_step_num;
extern const float Lmoto_K;
extern const float Rmoto_K;

//===== PID 控制器实例 =====
extern pid_t leg_hight;
extern pid_t turn_angle;
extern pid_t turn_gyro;
extern pid_t gyro;
extern pid_t angle;
extern pid_t speed;
extern pid_t turn;

//===== 标定参数 =====
extern float angle_kd;
extern float pitch_mid;
extern float roll_mid;

//===== PID 控制时间间隔 =====
extern float dt_pid_gyro;
extern float dt_pid_angle;
extern float dt_pid_speed;
extern float dt_pid_turn;
extern float dt_leg;
extern float dt_pid_turn_angle;
extern float dt_pid_turn_gyro;

//===== 状态变量 =====
extern float leg_long;
extern float set_speed;
extern uint8 jump_flag;
extern uint8 speed_flag;
extern float turn_out;
extern float KP;
extern float KPP;
extern float KD;
extern float KDD;

//===== 未来模块引用（待实现） =====
extern float image_error;   // [TODO: 视觉模块]
extern float v_hat;         // [TODO: 状态估计器]
extern float x_hat;         // [TODO: 状态估计器]

//===== 函数原型 =====
void pid_ctrl_Init(void);
void LQR_control(float V_target, float th);
float turn_control(float image_error);
void pid_ctrl_Run(void);
void leg_control(void);
void jump_set_step(int step_num);
void jump_control(void);
void dead_compensate(int16 *input_L, int16 *input_R);
void left_leg_control(float p, float angle);
void right_leg_control(float p, float angle);

#endif
