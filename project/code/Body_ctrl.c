#include "zf_common_headfile.h"

#define WHEEL_CIRCUMFERENCE  (6.4f)
#define ACC_CONV_FACTOR      (4098.0f)   // imu660rb LSB/g

//================================================================================
// 全局变量
//================================================================================
int16   balance_duty_max  = 3000;
int16   turn_duty_max     = 3000;
float   target_speed      = 0;
int     run_state         = 1;
uint32  sys_times         = 0;
int     STOP_FLAG         = 1;
int32   car_distance      = 0;
int16   left_motor_duty   = 0;
int16   right_motor_duty  = 0;
int     jump_time         = 0;          // 保留兼容，由 jump_cfg.elapsed 替代

//================================================================================
// 跳跃配置 — 默认值
//================================================================================
jump_config_struct jump_cfg = {
    // 阶段时长（PIT ticks）
    .prepare_ticks          = 50,
    .charge_ticks           = 120,
    .launch_ticks           = 30,
    .airborne_timeout       = 300,
    .landing_ticks          = 80,
    .recover_ticks          = 200,

    // 舵机占空比偏移
    .charge_duty            = 2500,
    .preland_duty           = 1400,
    .land_damping_duty      = 14,

    // 前进动量
    .forward_tilt_target    = 5.0f,
    .forward_motor_boost    = 500.0f,
    .speed_recovery_rate    = 0.3f,

    // PID 抑制
    .airborne_pid_scale     = 0.15f,
    .landing_pid_scale      = 0.3f,
    .recover_pid_ramp_rate  = 0.01f,

    // IMU 检测阈值
    .airborne_acc_threshold = 0.3f,
    .landing_acc_threshold  = 2.5f,
    .max_tilt_abort         = 45.0f,

    // 视觉接口
    .vision_jump_trigger    = NULL,
    .vision_jump_enable     = 0,
    .vision_obstacle_dist   = 0.0f,
    .vision_min_dist        = 100.0f,
    .vision_max_dist        = 800.0f,

    // 运行时
    .state                  = JUMP_IDLE,
    .elapsed                = 0,
    .jump_count             = 0,
    .peak_acc_magnitude     = 0.0f,
    .stored_p_angle         = 0.0f,
    .stored_p_speed         = 0.0f,
    .stored_speed_target    = 0.0f,
};

//================================================================================
// 计算加速度幅值（单位：g）
//================================================================================
static float compute_acc_magnitude(void)
{
    float ax = (float)ACC_DATA_X / ACC_CONV_FACTOR;
    float ay = (float)ACC_DATA_Y / ACC_CONV_FACTOR;
    float az = (float)ACC_DATA_Z / ACC_CONV_FACTOR;
    return sqrt(ax * ax + ay * ay + az * az);
}

//================================================================================
// 计算当前总倾斜角（roll + pitch 合成）
//================================================================================
static float compute_tilt_angle(void)
{
    float rol = func_abs(roll_balance_cascade.posture_value.rol);
    float pit = func_abs(roll_balance_cascade.posture_value.pit);
    return sqrt(rol * rol + pit * pit);
}

//================================================================================
// 跳跃触发
//================================================================================
void jump_trigger(void)
{
    if (jump_cfg.state != JUMP_IDLE)   return;
    if (!run_state)                     return;
    if (compute_tilt_angle() > jump_cfg.max_tilt_abort) return;

    jump_cfg.state               = JUMP_PREPARE;
    jump_cfg.elapsed             = 0;
    jump_cfg.peak_acc_magnitude  = 0.0f;

    // 保存进入跳跃前的 PID 参数，用于恢复
    jump_cfg.stored_p_angle      = roll_balance_cascade_resave.angle_cycle.p;
    jump_cfg.stored_p_speed      = roll_balance_cascade_resave.speed_cycle.p;
    jump_cfg.stored_speed_target = target_speed;
}

//================================================================================
// 紧急终止跳跃
//================================================================================
void jump_abort(void)
{
    jump_cfg.state   = JUMP_IDLE;
    jump_cfg.elapsed = 0;

    // 立即恢复 PID
    roll_balance_cascade.angle_cycle.p = jump_cfg.stored_p_angle;
    roll_balance_cascade.speed_cycle.p = jump_cfg.stored_p_speed;
    roll_balance_cascade.angle_cycle.i_value = 0;
    pitch_balance_cascade.angle_cycle.i_value = 0;
}

//================================================================================
// 视觉模块更新障碍物距离
//================================================================================
void jump_vision_update(float distance_mm)
{
    jump_cfg.vision_obstacle_dist = distance_mm;

    if (!jump_cfg.vision_jump_enable)                          return;
    if (jump_cfg.state != JUMP_IDLE)                           return;
    if (distance_mm > jump_cfg.vision_max_dist)                return;
    if (distance_mm < jump_cfg.vision_min_dist)                return;
    if (jump_cfg.vision_jump_trigger == NULL)                   return;

    jump_cfg.vision_jump_trigger(distance_mm);
}

//================================================================================
// 重置跳跃参数为默认值
//================================================================================
void jump_config_default(void)
{
    jump_cfg.prepare_ticks         = 50;
    jump_cfg.charge_ticks          = 120;
    jump_cfg.launch_ticks          = 30;
    jump_cfg.airborne_timeout      = 300;
    jump_cfg.landing_ticks         = 80;
    jump_cfg.recover_ticks         = 200;
    jump_cfg.charge_duty           = 2500;
    jump_cfg.preland_duty          = 1400;
    jump_cfg.land_damping_duty     = 14;
    jump_cfg.forward_tilt_target   = 5.0f;
    jump_cfg.forward_motor_boost   = 500.0f;
    jump_cfg.speed_recovery_rate   = 0.3f;
    jump_cfg.airborne_pid_scale    = 0.15f;
    jump_cfg.landing_pid_scale     = 0.3f;
    jump_cfg.recover_pid_ramp_rate = 0.01f;
    jump_cfg.airborne_acc_threshold = 0.3f;
    jump_cfg.landing_acc_threshold  = 2.5f;
    jump_cfg.max_tilt_abort        = 45.0f;
    jump_cfg.vision_jump_enable    = 0;
    jump_cfg.vision_min_dist       = 100.0f;
    jump_cfg.vision_max_dist       = 800.0f;
    jump_cfg.state                 = JUMP_IDLE;
    jump_cfg.elapsed               = 0;
}

//================================================================================
// 检查是否可以触发跳跃
//================================================================================
uint8 jump_can_trigger(void)
{
    if (jump_cfg.state != JUMP_IDLE)                           return 0;
    if (!run_state)                                            return 0;
    if (compute_tilt_angle() > jump_cfg.max_tilt_abort)        return 0;
    return 1;
}

//================================================================================
// 车辆状态计算 — 含跳跃 PID 管理
//================================================================================
void car_state_calculate(void)
{
    //---------- 倾斜保护 ----------
    if (compute_tilt_angle() > jump_cfg.max_tilt_abort)
    {
        if (jump_cfg.state != JUMP_IDLE)
        {
            jump_abort();
        }
        run_state = 0;
        roll_balance_cascade.angular_speed_cycle.i_value  = 0;
        pitch_balance_cascade.angle_cycle.i_value          = 0;
    }
    else
    {
        if (run_state == 0)
        {
            sys_times = 0;
        }
        run_state = 1;
    }

    //---------- PID 软启动（非跳跃时）----------
    if (jump_cfg.state == JUMP_IDLE)
    {
        if (sys_times < 500)
        {
            float ramp = 0.2f + (float)sys_times / 500.0f * 0.8f;
            roll_balance_cascade.angle_cycle.p  = roll_balance_cascade_resave.angle_cycle.p  * ramp;
            roll_balance_cascade.speed_cycle.p  = roll_balance_cascade_resave.speed_cycle.p  * ramp;
            roll_balance_cascade.angle_cycle.i_value = 0;
        }
        else
        {
            roll_balance_cascade.angle_cycle.p  = roll_balance_cascade_resave.angle_cycle.p;
            roll_balance_cascade.speed_cycle.p  = roll_balance_cascade_resave.speed_cycle.p;
        }
        return;
    }

    //---------- 跳跃中的 PID 管理 ----------
    float pid_scale;
    switch (jump_cfg.state)
    {
        case JUMP_PREPARE:
            // 预备阶段：适度降低 PID，允许前倾
            pid_scale = 0.6f;
            break;
        case JUMP_CHARGE:
            // 蓄力阶段：进一步降低 PID
            pid_scale = 0.4f;
            break;
        case JUMP_LAUNCH:
            // 起跳阶段：大幅降低 PID，让跳跃动作不受干扰
            pid_scale = 0.2f;
            break;
        case JUMP_AIRBORNE:
            pid_scale = jump_cfg.airborne_pid_scale;
            break;
        case JUMP_LANDING:
            pid_scale = jump_cfg.landing_pid_scale;
            break;
        case JUMP_RECOVER:
            // 恢复阶段：线性爬升
            {
                float progress = (float)jump_cfg.elapsed / (float)jump_cfg.recover_ticks;
                if (progress > 1.0f) progress = 1.0f;
                pid_scale = jump_cfg.landing_pid_scale
                          + (1.0f - jump_cfg.landing_pid_scale) * progress * jump_cfg.recover_pid_ramp_rate;
                if (pid_scale > 1.0f) pid_scale = 1.0f;

                // 速度目标恢复
                target_speed = jump_cfg.stored_speed_target * progress;
            }
            break;
        default:
            pid_scale = 0.5f;
            break;
    }

    roll_balance_cascade.angle_cycle.p  = jump_cfg.stored_p_angle * pid_scale;
    roll_balance_cascade.speed_cycle.p  = jump_cfg.stored_p_speed * pid_scale;
    roll_balance_cascade.angle_cycle.i_value  = 0;
    pitch_balance_cascade.angle_cycle.i_value = 0;
}

//================================================================================
// 舵机控制 — 含 7 阶段跳跃状态机
//================================================================================
void car_steer_control(void)
{
    int16 steer_location_offset[4] = {0};
    int16 steer_target_offset[4]   = {0};
    int16 steer_output_duty        = 0;
    float steer_balance_angle      = 0;
    float steer_balance_angle_count_local = 0;
    float steer_output_duty_filter = 0;
    static float s_filter          = 0;
    static float s_angle_count     = 0;
    int16 speed_steer;

    //---------- 俯仰对转向的影响 ----------
    float pitch_offset = (30.0f - func_limit_ab(func_abs(
        roll_balance_cascade.posture_value.rol + roll_balance_cascade.posture_value.mechanical_zero
    ), 0.0f, 30.0f)) / 30.0f;

    speed_steer = func_limit_ab((int16)(roll_balance_cascade.speed_cycle.out / 7.0f), -250, 250) * 6;
    speed_steer = (int16)((float)speed_steer * pitch_offset);

    s_filter = (s_filter * 8 + (float)speed_steer) / 10.0f;
    steer_output_duty_filter = s_filter;

    //---------- 转向平衡角（跳跃时冻结）----------
    if (jump_cfg.state == JUMP_IDLE)
    {
        if (sys_times < 2000)
        {
            steer_balance_angle = 0;
            pitch_balance_cascade.angle_cycle.i_value = 0;
        }
        else
        {
            steer_balance_angle = func_limit_ab(pitch_balance_cascade.angle_cycle.out, -300, 300) * 6;
        }
        s_angle_count = steer_balance_angle;
    }
    steer_balance_angle_count_local = s_angle_count;

    //---------- 计算舵机位置偏差 ----------
    steer_location_offset[0] = (steer_1.now_location - steer_1.center_num) * steer_1.steer_dir;
    steer_location_offset[1] = (steer_2.now_location - steer_2.center_num) * steer_2.steer_dir;
    steer_location_offset[2] = (steer_3.now_location - steer_3.center_num) * steer_3.steer_dir;
    steer_location_offset[3] = (steer_4.now_location - steer_4.center_num) * steer_4.steer_dir;

    //---------- 正常模式目标偏移 ----------
    steer_target_offset[0] = (int16)( steer_output_duty_filter - (steer_balance_angle_count_local > 0 ? 0 : steer_balance_angle_count_local));
    steer_target_offset[1] = (int16)( steer_output_duty_filter + (steer_balance_angle_count_local < 0 ? 0 : steer_balance_angle_count_local));
    steer_target_offset[2] = (int16)(-steer_output_duty_filter - (steer_balance_angle_count_local > 0 ? 0 : steer_balance_angle_count_local));
    steer_target_offset[3] = (int16)(-steer_output_duty_filter + (steer_balance_angle_count_local < 0 ? 0 : steer_balance_angle_count_local));

    if (run_state == 0)
    {
        // 停机 — 缓慢回中
        steer_control(&steer_1, func_limit_ab(steer_1.center_num - steer_1.now_location, -1, 1) * steer_1.steer_dir);
        steer_control(&steer_2, func_limit_ab(steer_2.center_num - steer_2.now_location, -1, 1) * steer_2.steer_dir);
        steer_control(&steer_3, func_limit_ab(steer_3.center_num - steer_3.now_location, -1, 1) * steer_3.steer_dir);
        steer_control(&steer_4, func_limit_ab(steer_4.center_num - steer_4.now_location, -1, 1) * steer_4.steer_dir);
        return;
    }

    //================================================================
    // 跳跃状态机
    //================================================================
    if (jump_cfg.state == JUMP_IDLE)
    {
        // 正常平衡转向
        steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -10, 10));
        steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -10, 10));
        steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -10, 10));
        steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -10, 10));
        return;
    }

    jump_cfg.elapsed++;

    //---------- IMU 检测 ----------
    float acc_mag = compute_acc_magnitude();
    if (acc_mag > jump_cfg.peak_acc_magnitude)
    {
        jump_cfg.peak_acc_magnitude = acc_mag;
    }

    switch (jump_cfg.state)
    {
        //----------------------------------------------------------------
        // PREPARE — 降低重心，前倾蓄势
        //----------------------------------------------------------------
        case JUMP_PREPARE:
            steer_duty_set(&steer_1, steer_1.center_num + 500  * steer_1.steer_dir);
            steer_duty_set(&steer_2, steer_2.center_num - 500  * steer_2.steer_dir);
            steer_duty_set(&steer_3, steer_3.center_num - 500  * steer_3.steer_dir);
            steer_duty_set(&steer_4, steer_4.center_num + 500  * steer_4.steer_dir);

            if (jump_cfg.elapsed >= jump_cfg.prepare_ticks)
            {
                jump_cfg.state   = JUMP_CHARGE;
                jump_cfg.elapsed = 0;
            }
            break;

        //----------------------------------------------------------------
        // CHARGE — X 型压缩储能 + 前倾维持前进动量
        //----------------------------------------------------------------
        case JUMP_CHARGE:
        {
            int16 charge = jump_cfg.charge_duty;
            steer_duty_set(&steer_1, steer_1.center_num + charge);
            steer_duty_set(&steer_2, steer_2.center_num - charge);
            steer_duty_set(&steer_3, steer_3.center_num - charge);
            steer_duty_set(&steer_4, steer_4.center_num + charge);

            // 前进动量：加速充电阶段也保持前倾
            // forward momentum: maintain forward lean during charge
            target_speed = jump_cfg.stored_speed_target * (1.0f + jump_cfg.speed_recovery_rate);

            if (jump_cfg.elapsed >= jump_cfg.charge_ticks)
            {
                jump_cfg.state   = JUMP_LAUNCH;
                jump_cfg.elapsed = 0;
            }
        }
            break;

        //----------------------------------------------------------------
        // LAUNCH — 释放能量 + 电机助推前冲
        //----------------------------------------------------------------
        case JUMP_LAUNCH:
            // 舵机：瞬间释放到中立位
            // steer: snap release to neutral
            steer_duty_set(&steer_1, steer_1.center_num);
            steer_duty_set(&steer_2, steer_2.center_num);
            steer_duty_set(&steer_3, steer_3.center_num);
            steer_duty_set(&steer_4, steer_4.center_num);

            // 前进动量：额外电机推力
            // forward momentum: extra motor boost
            target_speed = jump_cfg.stored_speed_target + jump_cfg.forward_motor_boost * 0.01f;

            if (jump_cfg.elapsed >= jump_cfg.launch_ticks)
            {
                jump_cfg.state   = JUMP_AIRBORNE;
                jump_cfg.elapsed = 0;
            }
            break;

        //----------------------------------------------------------------
        // AIRBORNE — IMU 检测失重，等待着陆
        //----------------------------------------------------------------
        case JUMP_AIRBORNE:
            // 保持中立舵位，不干预飞行姿态
            // hold neutral steer, don't interfere with flight attitude
            steer_duty_set(&steer_1, steer_1.center_num);
            steer_duty_set(&steer_2, steer_2.center_num);
            steer_duty_set(&steer_3, steer_3.center_num);
            steer_duty_set(&steer_4, steer_4.center_num);

            // 大角度倾斜 → 强制进入着陆
            // excessive tilt → force landing
            if (compute_tilt_angle() > jump_cfg.max_tilt_abort)
            {
                jump_cfg.state   = JUMP_LANDING;
                jump_cfg.elapsed = 0;
                break;
            }

            // IMU 检测着陆冲击
            // IMU detects landing impact
            if (jump_cfg.elapsed > 20 && acc_mag > jump_cfg.landing_acc_threshold)
            {
                jump_cfg.state   = JUMP_LANDING;
                jump_cfg.elapsed = 0;
                break;
            }

            // 超时保护
            // timeout protection
            if (jump_cfg.elapsed >= jump_cfg.airborne_timeout)
            {
                jump_cfg.state   = JUMP_LANDING;
                jump_cfg.elapsed = 0;
            }
            break;

        //----------------------------------------------------------------
        // LANDING — 主动缓冲吸收冲击
        //----------------------------------------------------------------
        case JUMP_LANDING:
        {
            // 四轮同时内收，吸收冲击
            // all 4 wheels retract inward for impact absorption
            int16 damp = jump_cfg.land_damping_duty;
            steer_control(&steer_1, -damp * steer_1.steer_dir);
            steer_control(&steer_2, -damp * steer_2.steer_dir);
            steer_control(&steer_3, -damp * steer_3.steer_dir);
            steer_control(&steer_4, -damp * steer_4.steer_dir);

            // 前进动量恢复
            target_speed = jump_cfg.stored_speed_target * jump_cfg.speed_recovery_rate;

            if (jump_cfg.elapsed >= jump_cfg.landing_ticks)
            {
                jump_cfg.state    = JUMP_RECOVER;
                jump_cfg.elapsed  = 0;
            }
        }
            break;

        //----------------------------------------------------------------
        // RECOVER — 逐步恢复 PID 和正常平衡
        //----------------------------------------------------------------
        case JUMP_RECOVER:
        {
            // 恢复转向控制
            int16 rate = jump_cfg.land_damping_duty / 2;
            steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -rate, rate));
            steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -rate, rate));
            steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -rate, rate));
            steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -rate, rate));

            // PID 由 car_state_calculate() 管理爬升
            // PID ramp managed by car_state_calculate()
            target_speed = jump_cfg.stored_speed_target
                         * (jump_cfg.speed_recovery_rate
                            + (1.0f - jump_cfg.speed_recovery_rate)
                            * ((float)jump_cfg.elapsed / (float)jump_cfg.recover_ticks));

            if (jump_cfg.elapsed >= jump_cfg.recover_ticks)
            {
                jump_cfg.state    = JUMP_IDLE;
                jump_cfg.elapsed  = 0;
                jump_cfg.jump_count++;

                // 完全恢复 PID
                roll_balance_cascade.angle_cycle.p  = jump_cfg.stored_p_angle;
                roll_balance_cascade.speed_cycle.p  = jump_cfg.stored_p_speed;
                target_speed = jump_cfg.stored_speed_target;
            }
        }
            break;

        default:
            break;
    }
}

//================================================================================
// 电机控制 — 含跳跃助推
//================================================================================
void car_motor_control(void)
{
    car_distance += ((float)car_speed / 60.0f * WHEEL_CIRCUMFERENCE * PI * 0.001f);

    if (run_state)
    {
        left_motor_duty  = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);
        right_motor_duty = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);

        // 跳跃助推：起跳阶段施加额外前冲力
        // jump boost: extra forward thrust during launch
        if (jump_cfg.state == JUMP_LAUNCH)
        {
            int16 boost = (int16)jump_cfg.forward_motor_boost;
            left_motor_duty  = func_limit_ab(left_motor_duty  + boost, -balance_duty_max, balance_duty_max);
            right_motor_duty = func_limit_ab(right_motor_duty + boost, -balance_duty_max, balance_duty_max);
        }

        // Z 轴陀螺仪辅助转向
        left_motor_duty   = func_limit_ab(left_motor_duty  + imu660ra_gyro_z / 3, -turn_duty_max, turn_duty_max);
        right_motor_duty  = func_limit_ab(right_motor_duty - imu660ra_gyro_z / 3, -turn_duty_max, turn_duty_max);
    }
    else
    {
        left_motor_duty  = 0;
        right_motor_duty = 0;
    }

    small_driver_set_duty(left_motor_duty, -right_motor_duty);
}

//================================================================================
// PIT 回调 — 1ms 控制级联
//================================================================================
void pit_call_back(void)
{
    sys_times++;

    imu660rb_get_gyro();
    imu660rb_get_acc();
    quaternion_module_calculate(&roll_balance_cascade);

    if (sys_times > 500)
    {
        if (sys_times % 5 == 0)
        {
            CYT2_get_distance();
            Nag_System();

            // 角度环 PID — 前向平衡控制
            pid_control(&roll_balance_cascade.angle_cycle,
                0.0f - roll_balance_cascade.posture_value.mechanical_zero,
                -roll_balance_cascade.posture_value.pit);
        }

        // 角速度环 PID — 每 1ms
        pid_control(&roll_balance_cascade.angular_speed_cycle,
            roll_balance_cascade.angle_cycle.out,
            imu660rb_gyro_y);

        car_state_calculate();
        car_steer_control();

        if (sys_times % 20 == 0)
        {
            car_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;
            pid_control(&roll_balance_cascade.speed_cycle, target_speed, (float)car_speed);

            if (fuxian == 1)
            {
                pid_control(&track_cascade.track_cycle, N.Final_Out, 0);
            }
        }

        if (STOP_FLAG == 1)
        {
            CYT2_D_motor_ctrl(
                -(int16)roll_balance_cascade.angular_speed_cycle.out + N.Final_Out * 10,
                -(int16)roll_balance_cascade.angular_speed_cycle.out - N.Final_Out * 10);
        }
        else
        {
            CYT2_D_motor_ctrl(0, 0);
        }
    }
}
