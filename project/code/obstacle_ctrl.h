#ifndef _obstacle_ctrl_h_
#define _obstacle_ctrl_h_

#include "zf_common_headfile.h"

//****************************************************************************
// 文件名称    obstacle_ctrl.h
// 功能描述    方格/颠簸路障通过控制模块对外接口
//             1. 独立状态机管理路障等待、通过、卡滞反拖/冲出和恢复；
//             2. 对外提供 PID 防积分饱和、导航差速衰减和电机补偿/覆盖输出；
//             3. 所有可调参数集中在 config.h。
//****************************************************************************

typedef enum
{
    OBSTACLE_IDLE = 0,      // 空闲
    OBSTACLE_WAIT,          // 已武装，等待里程到达配置起点
    OBSTACLE_ENTER,         // 进入路障，清 PID 并建立小前倾/推力
    OBSTACLE_CROSSING,      // 低速通过路障
    OBSTACLE_BACKOFF,       // 卡住后短暂反拖
    OBSTACLE_BOOST,         // 反拖后前冲脱困
    OBSTACLE_RECOVER        // 离开路障后延迟恢复正常 PID
} obstacle_state_e;

void obstacle_init(void);
void obstacle_arm(float mileage);
void obstacle_abort(void);
void obstacle_run_1ms(float mileage, int16 speed, int16 balance_motor, float pitch_deg, float roll_deg);

uint8 obstacle_is_active(void);
obstacle_state_e obstacle_get_state(void);
float obstacle_get_distance_cm(void);
uint16 obstacle_get_stuck_count(void);
uint8 obstacle_get_retry_count(void);

float obstacle_get_angle_offset(void);
float obstacle_get_nav_scale(void);
int16 obstacle_get_motor_boost(void);
uint8 obstacle_get_motor_override(int16 *left_motor, int16 *right_motor);

uint8 obstacle_speed_pid_should_update(void);
uint8 obstacle_track_pid_should_update(void);
uint8 obstacle_should_reset_pid(void);
void obstacle_clear_reset_request(void);

#endif
