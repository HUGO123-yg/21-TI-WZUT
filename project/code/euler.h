#ifndef _euler_h_
#define _euler_h_

//===== 欧拉角数据结构 =====

#include "zf_common_typedef.h"

//-------------------------------------------------------------------------------------------------------------------
// 结构体定义      euler_angle_struct
// 功能说明       欧拉角数据结构，存储飞行器/机器人姿态角度（单位：度）
// 成员说明       pitch   俯仰角   (绕X轴旋转，抬头上仰为正)
//               roll    横滚角   (绕Y轴旋转，右倾为正)
//               yaw     偏航角   (绕Z轴旋转，右转为正)
//-------------------------------------------------------------------------------------------------------------------
typedef struct
{
    float pitch;    // 俯仰角
    float roll;     // 横滚角
    float yaw;      // 偏航角
} euler_angle_struct;

// [TODO: IMU姿态融合算法（互补滤波 / Mahony 等）将在单独模块中实现，不在此头文件中]
//
// 使用示例:
//   euler_angle.pitch  = 15.0f;  // 设置俯仰角 15 度
//   euler_angle.roll   = -5.0f;  // 设置横滚角 -5 度
//   euler_angle.yaw    = 90.0f;  // 设置偏航角 90 度
//
// 注：euler_angle 的值由 IMU 融合算法计算填充，本模块仅提供数据结构声明

extern euler_angle_struct euler_angle;

#endif
