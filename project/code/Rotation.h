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
    ROT_DONE    = 2,    // 完成
} rotation_state_enum;

// 旋转控制结构体
typedef struct
{
    rotation_state_enum state;          // 当前状态
    rotation_dir_enum   dir;            // 旋转方向
    int16               turn_duty;      // 当前差速值（基于 elapsed 计算衰减）
    int16               max_turn_duty;  // 最大差速值 (0-3000)
    uint32              duration_ms;    // 旋转时长 (ms)
    uint32              elapsed;        // 已过时间（PIT tick 计数）
} rotation_control_struct;

extern rotation_control_struct rotation;

// 启动旋转
void rotation_start(rotation_dir_enum dir, int16 max_turn_duty, uint32 duration_ms);

// 每 1ms 调用一次（由 pit_call_back 驱动）
void rotation_run(void);

// 强制停止
void rotation_stop(void);

// 检查是否完成
uint8 rotation_is_done(void);

#endif /* CODE_ROTATION_H_ */
