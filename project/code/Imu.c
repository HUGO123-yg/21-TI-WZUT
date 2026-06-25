#include "zf_common_headfile.h"
#include <math.h>

#if IMU_MODE

#define RAD2DEG 57.29577951f
#define DEG2RAD  0.01745329f

cascade_value_struct roll_balance_cascade;          // 横滚平衡串级控制结构体
cascade_value_struct roll_balance_cascade_resave;  // 横滚平衡串级控制结构体备份
cascade_value_struct pitch_balance_cascade;          // 俯仰平衡串级控制结构体
cascade_value_struct pitch_balance_cascade_resave;  // 俯仰平衡串级控制结构体备份
cascade_value_struct track_cascade;                 // 轨迹串级控制结构体


// 函数功能：计算反正切值，范围 -90°~90°
// 输入参数：float - 正切值，范围负无穷到正无穷
// 返回值：  float - 角度值，单位°
// 示例：    float angle = arctan1(1.0f); // 计算 tan(45°) 的反正切，返回 45.0 °
// 注意：    该函数使用分段近似计算，提高计算效率
static float arctan2(float x, float y)
{
    return atan2f(y, x) * RAD2DEG;
}

static float arcsin(float i)
{
    return asinf(i) * RAD2DEG;
}

// 函数功能：加速度计低通滤波
// 输入参数：void
// 示例：    acc_lowpass_filter(&ax, &ay, &az, &fax, &fay, &faz, 0.8f);
// 注意：    滤波方式：filtered = alpha * filtered + (1 - alpha) * raw, alpha 越大滤波越强
static void acc_lowpass_filter(float *raw_x, float *raw_y, float *raw_z, float *filtered_x, float *filtered_y, float *filtered_z, float alpha)
{
    *filtered_x = alpha * *filtered_x + (1 - alpha) * *raw_x;   // X 轴低通滤波
    *filtered_y = alpha * *filtered_y + (1 - alpha) * *raw_y; // Y 轴低通滤波
    *filtered_z = alpha * *filtered_z + (1 - alpha) * *raw_z; // Z 轴低通滤波
}

// 函数功能：加速度计数据归一化
// 输入参数：void
// 示例：    acc_normalize(&ax, &ay, &az);
// 注意：    将加速度向量归一化到单位向量，如果模长小于0.1，直接返回 (0,0,1)
static void acc_normalize(float *ax, float *ay, float *az)
{
    float norm = sqrt(*ax * *ax + *ay * *ay + *az * *az);   // 计算向量模长
    if (norm < 0.1f)                                         // 模长过小时使用默认值
    {
        *ax = 0.0f;
        *ay = 0.0f;
        *az = 1.0f;
    }
    else                                                   // 正常归一化
    {
        *ax /= norm;
        *ay /= norm;
        *az /= norm;
    }
}
// 函数功能：判断是否静止状态
// 输入参数：bool - 静止时返回 true，运动返回 false
// 示例：    bool static_flag = is_static_state(ax_g, ay_g, az_g);
// 注意：    静止时加速度模长应该接近 1g，在 0.9~1.1g 范围内判断为静止
static bool is_static_state(float ax_g, float ay_g, float az_g)
{
    float norm = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);  // 计算加速度模长，单位 g
    return (norm >= 0.9f && norm <= 1.1f) ? true : false;        // 判断是否在静止范围内
}
// 函数功能：四元数姿态解算
// 输入参数：void
// 示例：    quaternion_module_calculate(&quaternion, 0.001f); // 每 1 ms 调用一次
// 注意：    该函数需要周期性地调用，调用周期由 cascade_value 中的参数指定

void quaternion_module_calculate(cascade_value_struct *cascade_value)
{
    static float first_count_time = 0; // 首次初始化计时
    float length;                       // 四元数模长
    float x, y, z;                       // 角速度数据/弧度
    // 将陀螺仪数据转换为弧度/秒 -> 弧度/调用周期（除以 10 ，因为数据有 10 倍放大）
    x = (float)GYRO_DATA_X / GYRO_TRANSITION_FACTOR * DEG2RAD;  // 0.01745329 为弧度转换系数 pi/180
    y = (float)GYRO_DATA_Y / GYRO_TRANSITION_FACTOR * DEG2RAD;
    z = (float)GYRO_DATA_Z / GYRO_TRANSITION_FACTOR * DEG2RAD;
    // 将加速度计数据转换为 g 单位（1g = 9.8 m/s²）
    float ax_g = (float)ACC_DATA_X / ACC_TRANSITION_FACTOR;
    float ay_g = (float)ACC_DATA_Y / ACC_TRANSITION_FACTOR;
    float az_g = (float)ACC_DATA_Z / ACC_TRANSITION_FACTOR;

    bool static_state = is_static_state(ax_g, ay_g, az_g);  // 判断静止状态
    float acc_alpha = static_state ? 0.8f : 0.5f;            // 静止时滤波更强，运动时滤波更弱
 // 对加速度数据进行低通滤波
    acc_lowpass_filter(&ax_g, &ay_g, &az_g,
                       &cascade_value->quaternion.pro.acc_filtered[0],
                       &cascade_value->quaternion.pro.acc_filtered[1],
                       &cascade_value->quaternion.pro.acc_filtered[2], acc_alpha);

    // 获取滤波后的加速度数据
    float ax = cascade_value->quaternion.pro.acc_filtered[0];
    float ay = cascade_value->quaternion.pro.acc_filtered[1];
    float az = cascade_value->quaternion.pro.acc_filtered[2];

    acc_normalize(&ax, &ay, &az);    // 加速度数据归一化

    // 获取当前四元数 (w, x, y, z)
    float q0 = cascade_value->quaternion.pro.qua[0];
    float q1 = cascade_value->quaternion.pro.qua[1];
    float q2 = cascade_value->quaternion.pro.qua[2];
    float q3 = cascade_value->quaternion.pro.qua[3];
    // 根据四元数计算重力向量在机体坐标系下的分量

    float gx = 2 * (q1 * q3 - q0 * q2);
    float gy = 2 * (q0 * q1 + q2 * q3);
    float gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    // 计算加速度计测量值与重力向量的叉积，作为误差
    float ex = ay * gz - az * gy;
    float ey = az * gx - ax * gz;
    float ez = ax * gy - ay * gx;
    // 根据静止状态调整补偿系数，运动时KP减小，KI保持不变
    float kp = static_state ? cascade_value->posture_value.correct_kp : cascade_value->posture_value.correct_kp * 0.8f;
    float ki = cascade_value->posture_value.correct_ki;

    
//---------------------------------------------------------------------------------------首次快速收敛
     // 首次初始化： 0.5 秒内使用较大的KP加速收敛
    if(first_count_time < 0.5f)
    {
        first_count_time += cascade_value->posture_value.call_cycle;  //// 累加时间
        kp = 100.0f; // 增大KP加速度收敛
    }
    
   // 首次初始化： 0.1 秒内使用较大的KP加速收敛
//    if(first_count_time < 0.1f)
//    {
//        first_count_time += cascade_value->posture_value.call_cycle;  // // 累加时间
//        kp = 10.0f;  // // 增大KP加速度收敛
//    }
//---------------------------------------------------------------------------------------首次快速收敛

    // 运动时积分增益减小为 1/10
    float integral_gain = static_state ? 1.0f : 0.1f;

    cascade_value->quaternion.parameter.acc_err[0] += (ex * cascade_value->posture_value.call_cycle) * integral_gain;
    cascade_value->quaternion.parameter.acc_err[1] += (ey * cascade_value->posture_value.call_cycle) * integral_gain;
    cascade_value->quaternion.parameter.acc_err[2] += (ez * cascade_value->posture_value.call_cycle) * integral_gain;


    // 限制积分误差范围
    float acc_err_limit = 1.0f;
    cascade_value->quaternion.parameter.acc_err[0] = (cascade_value->quaternion.parameter.acc_err[0] > acc_err_limit) ? acc_err_limit :
                                                      (cascade_value->quaternion.parameter.acc_err[0] < -acc_err_limit) ? -acc_err_limit :
                                                      cascade_value->quaternion.parameter.acc_err[0];
    cascade_value->quaternion.parameter.acc_err[1] = (cascade_value->quaternion.parameter.acc_err[1] > acc_err_limit) ? acc_err_limit :
                                                      (cascade_value->quaternion.parameter.acc_err[1] < -acc_err_limit) ? -acc_err_limit :
                                                      cascade_value->quaternion.parameter.acc_err[1];
    cascade_value->quaternion.parameter.acc_err[2] = (cascade_value->quaternion.parameter.acc_err[2] > acc_err_limit) ? acc_err_limit :
                                                      (cascade_value->quaternion.parameter.acc_err[2] < -acc_err_limit) ? -acc_err_limit :
                                                      cascade_value->quaternion.parameter.acc_err[2];
    // 将误差补偿到角速度数据上（比例+积分）
    x += kp * ex + ki * cascade_value->quaternion.parameter.acc_err[0];
    y += kp * ey + ki * cascade_value->quaternion.parameter.acc_err[1];
    z += kp * ez + ki * cascade_value->quaternion.parameter.acc_err[2];
    // 用一阶龙格库塔法更新四元数
    cascade_value->quaternion.pro.qua[0] += ((-q1 * x - q2 * y - q3 * z) * cascade_value->posture_value.call_cycle / 2.0f);
    cascade_value->quaternion.pro.qua[1] += (( q0 * x + q2 * z - q3 * y) * cascade_value->posture_value.call_cycle / 2.0f);
    cascade_value->quaternion.pro.qua[2] += (( q0 * y - q1 * z + q3 * x) * cascade_value->posture_value.call_cycle / 2.0f);
    cascade_value->quaternion.pro.qua[3] += (( q0 * z + q1 * y - q2 * x) * cascade_value->posture_value.call_cycle / 2.0f);

    // 获取更新后的四元数 (w, x, y, z)
    q0 = cascade_value->quaternion.pro.qua[0];
    q1 = cascade_value->quaternion.pro.qua[1];
    q2 = cascade_value->quaternion.pro.qua[2];
    q3 = cascade_value->quaternion.pro.qua[3];

    // 计算四元数各分量的平方
    float q0_2 = q0 * q0;
    float q1_2 = q1 * q1;
    float q2_2 = q2 * q2;
    float q3_2 = q3 * q3;
    static uint8 norm_cnt = 0;
    if (++norm_cnt >= 20)
    {
        norm_cnt = 0;
        length = sqrtf(q0_2 + q1_2 + q2_2 + q3_2);
        if (length > 0.001f)
        {
            cascade_value->quaternion.pro.qua[0] /= length;
            cascade_value->quaternion.pro.qua[1] /= length;
            cascade_value->quaternion.pro.qua[2] /= length;
            cascade_value->quaternion.pro.qua[3] /= length;
            q0 = cascade_value->quaternion.pro.qua[0];
            q1 = cascade_value->quaternion.pro.qua[1];
            q2 = cascade_value->quaternion.pro.qua[2];
            q3 = cascade_value->quaternion.pro.qua[3];
            q0_2 = q0 * q0;
            q1_2 = q1 * q1;
            q2_2 = q2 * q2;
            q3_2 = q3 * q3;
        }
    }


    cascade_value->quaternion.data.rot_mat[0][0] = q0_2 + q1_2 - q2_2 - q3_2;
    cascade_value->quaternion.data.rot_mat[0][1] = 2 * (q1 * q2 + q0 * q3);
    cascade_value->quaternion.data.rot_mat[0][2] = 2 * (q1 * q3 - q0 * q2);
    cascade_value->quaternion.data.rot_mat[1][0] = 2 * (q1 * q2 - q0 * q3);
    cascade_value->quaternion.data.rot_mat[1][1] = q0_2 - q1_2 + q2_2 - q3_2;
    cascade_value->quaternion.data.rot_mat[1][2] = 2 * (q2 * q3 + q0 * q1);
    cascade_value->quaternion.data.rot_mat[2][0] = 2 * (q1 * q3 + q0 * q2);
    cascade_value->quaternion.data.rot_mat[2][1] = 2 * (q2 * q3 - q0 * q1);
    cascade_value->quaternion.data.rot_mat[2][2] = q0_2 - q1_2 - q2_2 + q3_2;

    // 从旋转矩阵计算欧拉角（横滚、俯仰、偏航）
    cascade_value->posture_value.rol = arctan2(cascade_value->quaternion.data.rot_mat[2][2], cascade_value->quaternion.data.rot_mat[1][2]);  //
    cascade_value->posture_value.pit = -arcsin(cascade_value->quaternion.data.rot_mat[0][2]);                                            // 
    cascade_value->posture_value.yaw = arctan2(cascade_value->quaternion.data.rot_mat[0][0], cascade_value->quaternion.data.rot_mat[0][1]);   // 
}
// 函数功能：PID 控制计算
// 输入参数：pid_cycle        PID控制周期结构体
// 输入参数：target           目标值
// 输入参数：real             实际值
// 返回值：  void
// 示例：    pid_control(&roll_balance_cascade.speed_cycle, 0, (left_motor.encoder_data + right_motor.encoder_data) / 2);
// 注意：  
void pid_control (pid_cycle_struct *pid_cycle, float target, float real)
{
    float    proportion_value    = 0;          // 比例项
    float    differential_value  = 0;          // 微分项

    proportion_value = target - real;          // 比例项 = 目标值 - 实际值


    pid_cycle->i_value += (proportion_value * pid_cycle->i_value_pro);  // 积分项 = 积分项 + 比例项 * 积分系数

    pid_cycle->i_value = func_limit_ab(pid_cycle->i_value, -pid_cycle->i_value_max, pid_cycle->i_value_max);  // 限制积分项范围

    differential_value = proportion_value - pid_cycle->p_value_last; // 微分项 = 比例项 - 上一次比例项

    pid_cycle->out = (pid_cycle->p * proportion_value + pid_cycle->i * pid_cycle->i_value + pid_cycle->d * differential_value);  // PID输出

    pid_cycle->out = func_limit_ab(pid_cycle->out, -pid_cycle->out_max, pid_cycle->out_max);  // 限制PID输出范围

    pid_cycle->p_value_last = proportion_value;        // 保存本次比例项
}

// 函数功能：PID 控制计算（增量式）
// 输入参数：pid_cycle        PID控制周期结构体
// 输入参数：target           目标值
// 输入参数：real             实际值
// 返回值：  void
// 示例：    pid_control_incremental(&roll_balance_cascade.speed_cycle, 0, (left_motor.encoder_data + right_motor.encoder_data) / 2);
// 注意：   
void pid_control_incremental (pid_cycle_struct *pid_cycle, float target, float real)
{
    float    proportion_value    = 0,         // 比例项
             differential_value  = 0;       // 微分项

    pid_cycle->i_value = target - real;          // 比例项 = 目标值 - 实际值 （增量PID P ---> I）

    differential_value = proportion_value - 2 * pid_cycle->incremental_data[0] - pid_cycle->incremental_data[1];   // 微分项（增量PID I ---> D）

    proportion_value  = proportion_value - pid_cycle->incremental_data[0];  // 比例项（增量PID D ---> P）

    pid_cycle->incremental_data[1] = pid_cycle->incremental_data[0];           // 增量PID历史数据移位

    pid_cycle->incremental_data[0] = proportion_value;

    pid_cycle->out += (pid_cycle->p * proportion_value + pid_cycle->i * pid_cycle->i_value + pid_cycle->d * differential_value);  // PID输出

    pid_cycle->out = func_limit_ab(pid_cycle->out, -pid_cycle->out_max, pid_cycle->out_max);          // 限制PID输出范围
}

// 函数功能：四元数模块初始化
// 输入参数：void
// 示例：    quaternion_module_init(&quaternion);
// 注意：    初始化四元数为单位四元数，各欧拉角为 0，加速度积分误差清零
void quaternion_module_init(cascade_value_struct *cascade_value)
{
      // 初始化四元数为单位四元数 (w=1, x=y=z=0)
    cascade_value->quaternion.pro.qua[0] = 1.0f;
    cascade_value->quaternion.pro.qua[1] = 0.0f;
    cascade_value->quaternion.pro.qua[2] = 0.0f;
    cascade_value->quaternion.pro.qua[3] = 0.0f;

 
    // 初始化各欧拉角为 0 °
    cascade_value->posture_value.yaw = 0.0f;
    cascade_value->posture_value.rol = 0.0f;
    cascade_value->posture_value.pit = 0.0f;
   // 初始化滤波后的加速度数据为当前加速度计数据，单位 g
    cascade_value->quaternion.pro.acc_filtered[0] = (float)ACC_DATA_X / ACC_TRANSITION_FACTOR;
    cascade_value->quaternion.pro.acc_filtered[1] = (float)ACC_DATA_Y / ACC_TRANSITION_FACTOR;
    cascade_value->quaternion.pro.acc_filtered[2] = (float)ACC_DATA_Z / ACC_TRANSITION_FACTOR;

  // 初始化加速度积分误差为 0
    cascade_value->quaternion.parameter.acc_err[0] = 0.0f;
    cascade_value->quaternion.parameter.acc_err[1] = 0.0f;
    cascade_value->quaternion.parameter.acc_err[2] = 0.0f;
}

// 函数功能：平衡串级控制初始化
// 输入参数：void
// 示例：    balance_cascade_init();
// 注意：    初始化横滚和俯仰方向的串级PID控制环，配置P/I/D参数
//            同时备份一份初始参数用于后期恢复
void balance_cascade_init (void)
{
     // 初始化横滚方向姿态解算参数
    roll_balance_cascade.posture_value.call_cycle        = 0.001;     // 调用周期 0.001s (1ms)
    roll_balance_cascade.posture_value.mechanical_zero  = -6.0f;      // 机械零点 -6°
    roll_balance_cascade.posture_value.correct_kp        = 0.4f;        // 姿态修正 KP 0.4
    roll_balance_cascade.posture_value.correct_ki        = 0.015f;      // 姿态修正 KI 0.015

    // 初始化横滚方向角速度环 PID 参数
    roll_balance_cascade.angular_speed_cycle.i_value_max     = 1000;      // 积分项最大值
    roll_balance_cascade.angular_speed_cycle.i_value_pro    = 0.1f;     // 积分项系数
    roll_balance_cascade.angular_speed_cycle.out_max        = 10000;      // 输出最大值

    // 初始化横滚方向角度环 PID 参数
    roll_balance_cascade.angle_cycle.i_value_max        = 1000;     // 积分项最大值
    roll_balance_cascade.angle_cycle.i_value_pro         = 2.0f;        // 积分项系数
    roll_balance_cascade.angle_cycle.out_max            = 10000;      // 输出最大值
   // 初始化横滚方向速度环 PID 参数
    roll_balance_cascade.speed_cycle.i_value_max        = 500;    // 积分项最大值     
    roll_balance_cascade.speed_cycle.i_value_pro         = 0.005f;  // 积分项系数
    roll_balance_cascade.speed_cycle.out_max            = 2000;      // 输出最大值
    // 设置横滚方向各 PID 环 P/I/D 参数
    roll_balance_cascade.angular_speed_cycle.p    = 1.1f;      // 角速度环 P
    roll_balance_cascade.angular_speed_cycle.i    = 0.0f;       // 角速度环 I
    roll_balance_cascade.angular_speed_cycle.d    = 0.0f;      // 角速度环 D

    roll_balance_cascade.angle_cycle.p    = 700.0f;       // 角度环 P
    roll_balance_cascade.angle_cycle.i    = 1.0f;        // 角度环 I
    roll_balance_cascade.angle_cycle.d    = 50.0f;      // 角度环 D

    roll_balance_cascade.speed_cycle.p    = 5.0f;    // 速度环 P//5
    roll_balance_cascade.speed_cycle.i    = 0.0f;      // 速度环 I
    roll_balance_cascade.speed_cycle.d    = 0.0f;      // 速度环 D 
    
    track_cascade.track_cycle.i_value_max = 100.0f;
    track_cascade.track_cycle.i_value_pro = 0.02f;
    track_cascade.track_cycle.out_max     = 500.0f;
    track_cascade.track_cycle.p = 10.0f;
    track_cascade.track_cycle.i = 0.0f;
    track_cascade.track_cycle.d = 0.0f;

 // 备份横滚方向的初始参数
    memcpy(&roll_balance_cascade_resave, &roll_balance_cascade, sizeof(roll_balance_cascade_resave));
   // 初始化四元数模块
    quaternion_module_init(&roll_balance_cascade);

    // 初始化俯仰方向姿态解算参数

    pitch_balance_cascade.posture_value.call_cycle        = 0.001;     // 调用周期 0.001s (1ms)
    pitch_balance_cascade.posture_value.mechanical_zero  = 0.0f;       // 机械零点 0°
    pitch_balance_cascade.posture_value.correct_kp        = 0.4f;       // 姿态修正 KP 0.4
    pitch_balance_cascade.posture_value.correct_ki        = 0.015f;       // 姿态修正 KI 0.015
    // 初始化俯仰方向角速度环 PID 参数
    pitch_balance_cascade.angular_speed_cycle.i_value_max     = 1000;      // 积分项最大值
    pitch_balance_cascade.angular_speed_cycle.i_value_pro    = 0.3f;       // 积分项系数
    pitch_balance_cascade.angular_speed_cycle.out_max        = 10000;        // 输出最大值

    // 初始化俯仰方向角度环 PID 参数
    pitch_balance_cascade.angle_cycle.i_value_max        = 300;     // 积分项最大值
    pitch_balance_cascade.angle_cycle.i_value_pro         = 0.8f;        // 积分项系数
    pitch_balance_cascade.angle_cycle.out_max            = 300;      // 输出最大值

    // 初始化俯仰方向速度环 PID 参数
    pitch_balance_cascade.speed_cycle.i_value_max        = 4000;    // 积分项最大值
    pitch_balance_cascade.speed_cycle.i_value_pro         = 0.05f;    // 积分项系数
    pitch_balance_cascade.speed_cycle.out_max            = 1500;       // 输出最大值
    

   // 设置俯仰方向各 PID 环 P/I/D 参数
    pitch_balance_cascade.angular_speed_cycle.p    = 0.0f;         // 角速度环 P
    pitch_balance_cascade.angular_speed_cycle.i    = 0.0f;     // 角速度环 I
    pitch_balance_cascade.angular_speed_cycle.d    = 0.0f;        // 角速度环 D

    pitch_balance_cascade.angle_cycle.p    = 0.0f;       // 角度环 P
    pitch_balance_cascade.angle_cycle.i    = 1.0f;            // 角度环 I
    pitch_balance_cascade.angle_cycle.d    = 0.0f;       // 角度环 D

    pitch_balance_cascade.speed_cycle.p    = 0.0f;       // 速度环 P
    pitch_balance_cascade.speed_cycle.i    = 0.0f;        // 速度环 I
    pitch_balance_cascade.speed_cycle.d    = 0.0f;      // 速度环 D


    // 备份俯仰方向的初始参数
    memcpy(&pitch_balance_cascade_resave, &pitch_balance_cascade, sizeof(pitch_balance_cascade_resave));
    
}

#endif