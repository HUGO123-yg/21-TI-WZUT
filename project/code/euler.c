//===== 欧拉角姿态解算模块 =====
// 纯陀螺仪积分 + 启动零偏校准
// [TODO: 添加加速度计修正 (Mahony/Madgwick) 消除长期漂移]

#include "euler.h"
#include "zf_device_imu660ra.h"
#include "zf_driver_delay.h"
#include "control.h"

//===== 陀螺仪零偏校准值（静态变量，仅本文件可访问） =====

static float gyro_offset_x = 0.0f;      // X 轴角速度零偏（俯仰，单位：度/秒）
static float gyro_offset_y = 0.0f;      // Y 轴角速度零偏（横滚，单位：度/秒）
static float gyro_offset_z = 0.0f;      // Z 轴角速度零偏（偏航，单位：度/秒）
static float accel_offset_x = 0.0f;     // X 轴加速度计零偏（单位：g）
static float accel_offset_y = 0.0f;     // Y 轴加速度计零偏（单位：g）
static float accel_offset_z = 0.0f;     // Z 轴加速度计零偏（单位：g）

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
    float sum_ax      = 0.0f;
    float sum_ay      = 0.0f;
    float sum_az      = 0.0f;

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

    // 加速度计零偏校准（100次静止采样，每次间隔1ms）
    for(i = 0; i < 100; i++)
    {
        imu660ra_get_acc();
        sum_ax += imu660ra_acc_transition(imu660ra_acc_x);
        sum_ay += imu660ra_acc_transition(imu660ra_acc_y);
        sum_az += imu660ra_acc_transition(imu660ra_acc_z);
        system_delay_us(1000);
    }
    accel_offset_x = sum_ax / 100.0f;
    accel_offset_y = sum_ay / 100.0f;
    accel_offset_z = sum_az / 100.0f;
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

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       inv_sqrt
// 功能说明       快速反平方根 (Quake III algorithm, ISR-safe, 无除法/开方)
//               用于加速度计向量归一化，避免使用 sqrtf + 除法组合
// 输入参数       x           输入值
// 返回参数       float       1/sqrt(x) 的近似值
//-------------------------------------------------------------------------------------------------------------------
static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    int32 i = *(int32*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - halfx * y * y);
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       euler_update_fused
// 功能说明       Mahony互补滤波姿态融合（陀螺仪+加速度计）
//               算法步骤：
//                 1. 读取陀螺仪角速度 → 去零偏 → 转为 rad/s
//                 2. 读取加速度计 → 去零偏 → 归一化得到重力方向测量值
//                 3. 根据当前姿态估计重力方向（小角度近似，无三角函数）
//                 4. 叉积计算姿态误差 → Mahony 比例修正角速度
//                 5. 修正后角速度积分得到欧拉角（转回度）
// 输入参数       dt          积分时间间隔（单位：秒），典型值 0.001f（1kHz 调用时）
// 返回参数       void
// 调用示例       euler_update_fused(0.001f);    // 在 1kHz ISR 中调用
// 注意事项       ISR安全：无 sinf/cosf/acosf/sqrtf 调用，无除法（inv_sqrt 替代）
//-------------------------------------------------------------------------------------------------------------------
void euler_update_fused(float dt)
{
    float gx, gy, gz;
    float ax, ay, az;
    float norm;
    float vx, vy, vz;      // 估计重力向量 (estimated gravity vector)
    float ex, ey, ez;      // 姿态误差 (attitude error via cross product)

    // Step 1: 读取陀螺仪并去零偏 → rad/s
    imu660ra_get_gyro();
    gx = (imu660ra_gyro_transition(imu660ra_gyro_x) - gyro_offset_x) / RAD_TO_DEG;
    gy = (imu660ra_gyro_transition(imu660ra_gyro_y) - gyro_offset_y) / RAD_TO_DEG;
    gz = (imu660ra_gyro_transition(imu660ra_gyro_z) - gyro_offset_z) / RAD_TO_DEG;

    // Step 2: 读取加速度计并去零偏 → g
    imu660ra_get_acc();
    ax = imu660ra_acc_transition(imu660ra_acc_x) - accel_offset_x;
    ay = imu660ra_acc_transition(imu660ra_acc_y) - accel_offset_y;
    az = imu660ra_acc_transition(imu660ra_acc_z) - accel_offset_z;

    // Step 3: 加速度计向量归一化（使用快速反平方根避免 sqrtf）
    norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Step 4: 根据当前姿态估计重力方向向量（小角度近似，无三角函数）
    // 旋转矩阵简化：R * [0, 0, -1]^T ≈ [sin(pitch), -sin(roll), -cos(pitch)*cos(roll)]
    // 小角度近似：sin(θ) ≈ θ, cos(θ) ≈ 1 - θ²/2
    float pitch_rad = euler_angle.pitch / RAD_TO_DEG;
    float roll_rad  = euler_angle.roll  / RAD_TO_DEG;

    vx = pitch_rad;                                                     // ≈ sin(pitch)
    vy = -roll_rad;                                                     // ≈ -sin(roll)
    vz = -1.0f + 0.5f * (pitch_rad * pitch_rad + roll_rad * roll_rad);  // ≈ -cos(pitch)*cos(roll)

    // Step 5: 叉积 — 加速度计测量值与估计重力向量的误差 → 角速度修正量
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // Step 6: Mahony 比例修正角速度（Ki=0 时仅比例项）
    gx += MAHONY_KP * ex;
    gy += MAHONY_KP * ey;
    gz += MAHONY_KP * ez;

    // Step 7: 积分修正后的角速度 → 欧拉角（rad/s → deg）
    euler_angle.pitch += gx * dt * RAD_TO_DEG;
    euler_angle.roll  += gy * dt * RAD_TO_DEG;
    euler_angle.yaw   += gz * dt * RAD_TO_DEG;
}
