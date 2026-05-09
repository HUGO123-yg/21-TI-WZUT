//===== 欧拉角姿态解算模块 =====
// 纯陀螺仪积分 + 启动零偏校准
// [TODO: 添加加速度计修正 (Mahony/Madgwick) 消除长期漂移]

#include "euler.h"
#include "zf_device_imu660ra.h"
#include "zf_driver_delay.h"

//===== 陀螺仪零偏校准值（静态变量，仅本文件可访问） =====

static float gyro_offset_x = 0.0f;      // X 轴角速度零偏（俯仰，单位：度/秒）
static float gyro_offset_y = 0.0f;      // Y 轴角速度零偏（横滚，单位：度/秒）
static float gyro_offset_z = 0.0f;      // Z 轴角速度零偏（偏航，单位：度/秒）

//===== 全局欧拉角实例 =====

//-------------------------------------------------------------------------------------------------------------------
// 全局变量      euler_angle
// 功能说明     全局欧拉角实例，存储当前姿态角度（单位：度）
//              pitch = 俯仰角, roll = 横滚角, yaw = 偏航角
//              由 euler_update() 实时更新
// 引用示例     control.c: euler_angle.pitch, euler_angle.roll
//-------------------------------------------------------------------------------------------------------------------
euler_angle_struct euler_angle = {0.0f, 0.0f, 0.0f};

//===== 函数实现 =====

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       euler_init
// 功能说明       IMU 姿态初始化：初始化 IMU660RA 芯片，进行 100 次陀螺仪零偏采样校准
//               校准流程：
//                 1. 调用 imu660ra_init() 初始化 IMU 芯片及 SPI 通信
//                 2. 循环 100 次：读取陀螺仪数据 → 累加 → 延时 1ms
//                 3. 计算 100 次采样平均值，保存为陀螺仪零偏补偿值
// 输入参数       void
// 返回参数       void
// 注意事项       调用此函数前请确保机器人/车辆处于静止状态，否则零偏校准失准
//               [TODO: 添加加速度计修正 (Mahony/Madgwick) 消除长期漂移]
//-------------------------------------------------------------------------------------------------------------------
void euler_init(void)
{
    uint8 i           = 0;
    float sum_gx      = 0.0f;
    float sum_gy      = 0.0f;
    float sum_gz      = 0.0f;

    // 初始化 IMU660RA 芯片（SPI 通信 + 寄存器配置）
    imu660ra_init();

    // 100 次采样零偏校准（每次间隔 1ms，总计 100ms）
    for(i = 0; i < 100; i++)
    {
        imu660ra_get_gyro();                                              // 读取陀螺仪原始数据到全局变量
        sum_gx += imu660ra_gyro_transition(imu660ra_gyro_x);              // 累加 X 轴角速度（度/秒）
        sum_gy += imu660ra_gyro_transition(imu660ra_gyro_y);              // 累加 Y 轴角速度（度/秒）
        sum_gz += imu660ra_gyro_transition(imu660ra_gyro_z);              // 累加 Z 轴角速度（度/秒）
        system_delay_us(1000);                                            // 延时 1ms
    }

    // 计算平均值作为零偏补偿
    gyro_offset_x = sum_gx / 100.0f;
    gyro_offset_y = sum_gy / 100.0f;
    gyro_offset_z = sum_gz / 100.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       euler_update
// 功能说明       纯陀螺仪积分更新欧拉角（无加速度计修正）
//               读取当前角速度 → 减去零偏 → 乘以 dt 积分到欧拉角
// 输入参数       dt          积分时间间隔（单位：秒），典型值 0.001f（对应 1kHz 调用周期）
// 返回参数       void
// 调用示例       euler_update(0.001f);    // 在 1kHz 定时器 ISR 中调用
// 注意事项       纯积分方案会产生角度漂移，需加速度计融合消除
//               [TODO: 添加加速度计修正 (Mahony/Madgwick) 消除长期漂移]
//-------------------------------------------------------------------------------------------------------------------
void euler_update(float dt)
{
    float gx, gy, gz;

    imu660ra_get_gyro();                                                  // 读取当前陀螺仪数据

    // 转换为度/秒并减去零偏补偿
    gx = imu660ra_gyro_transition(imu660ra_gyro_x) - gyro_offset_x;
    gy = imu660ra_gyro_transition(imu660ra_gyro_y) - gyro_offset_y;
    gz = imu660ra_gyro_transition(imu660ra_gyro_z) - gyro_offset_z;

    // 角速度积分 → 欧拉角（单位：度）
    euler_angle.pitch += gx * dt;
    euler_angle.roll  += gy * dt;
    euler_angle.yaw   += gz * dt;
}
