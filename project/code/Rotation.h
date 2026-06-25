// ============================================================
// Rotation.h — 原地旋转控制模块
// 双腿对称差速 + 舵机对向偏转，实现绕中心轴原地转圈
// ============================================================

#ifndef CODE_ROTATION_H_
#define CODE_ROTATION_H_

#include "zf_common_headfile.h"

// 旋转方向枚举
typedef enum
{
    ROT_CW  = 0,    // 顺时针 (clockwise)
    ROT_CCW = 1,    // 逆时针 (counter-clockwise)
} rotation_dir_enum;

// 旋转状态枚举
typedef enum
{
    ROT_IDLE    = 0,    // 空闲
    ROT_RUNNING = 1,    // 旋转中
    ROT_BRAKING = 2,    // 反向刹车
    ROT_DONE    = 3,    // 完成
} rotation_state_enum;

// 旋转模式枚举
typedef enum
{
    ROT_MODE_TIME  = 0,    // 按时间旋转
    ROT_MODE_TURNS = 1,    // 按 yaw 累计角度旋转
} rotation_mode_enum;

// 旋转控制结构体
typedef struct
{
    rotation_state_enum state;          // 当前状态
    rotation_mode_enum  mode;           // 控制模式
    rotation_dir_enum   dir;            // 旋转方向
    int16               turn_duty;      // 当前差速值（基于 elapsed 计算衰减）
    int16               max_turn_duty;  // 最大差速值 (0-3000)
    int16               brake_duty;     // 反向刹车差速
    uint32              duration_ms;    // 旋转时长/角度模式超时 (ms)
    uint32              elapsed;        // 已过时间（PIT tick 计数）
    uint32              brake_elapsed;  // 刹车计时
    float               target_angle;   // 目标累计角度 (deg)
    float               accumulated_angle; // 已累计角度 (deg)
    float               last_yaw;       // 上一次 yaw (deg)
} rotation_control_struct;

extern rotation_control_struct rotation;

// 启动旋转
void rotation_start(rotation_dir_enum dir, int16 max_turn_duty, uint32 duration_ms);

// 启动定圈数旋转
void rotation_start_turns(rotation_dir_enum dir, int16 max_turn_duty, float turns);

// 每 1ms 调用一次（由 pit_call_back 驱动）
void rotation_run(void);

// 强制停止
void rotation_stop(void);

// 检查是否完成
uint8 rotation_is_done(void);

// 检查是否正在占用旋转控制
uint8 rotation_is_active(void);

// 状态名
const char *rotation_state_name(rotation_state_enum state);

#endif /* CODE_ROTATION_H_ */
