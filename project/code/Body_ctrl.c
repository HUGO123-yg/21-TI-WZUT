#include "zf_common_headfile.h"
#include "config.h"                          // 车体控制可调参数配置（物理/机械、安全、PID、跳跃、转向等宏）
#include "bridge_ctrl.h"                     // 单边桥状态机与补偿
#include "balance_steering.h"

//****************************************************************************
// 文件名称    Body_ctrl.c
// 功能描述    平衡/智能车车体控制核心模块
//             1. car_state_calculate() - 车体倾角安全检测、运行状态机与 PID 启动渐变；
//             2. car_steer_control()   - 四舵机转向控制（含速度/平衡补偿），跳跃动作由 Jump.c 负责；
//             3. car_motor_control()   - 电机占空比计算（当前未被 pit_call_back 调用，保留备用）；
//             4. pit_call_back()       - PIT 定时中断主循环（约 1kHz），
//                                        完成 IMU 读取、四元数姿态解算、串级 PID、
//                                        导航/巡线、舵机与电机输出。
// 依赖模块    Imu.c/h, Common_peripherals.c/h, Flash.c/h,
//             small_driver_uart_control.c/h, zf_device_imu660rb.c/h
// 重要说明    1. 前后平衡主要依赖 roll_balance_cascade；pitch_balance_cascade.angle_cycle 已作为横滚轴角度环启用；
//             2. STOP_FALG 为历史拼写错误（应为 STOP_FLAG），保留原名以保持接口兼容；
//             3. 姿态坐标说明：roll_balance_cascade.posture_value.pit 实际用于前后平衡控制。
//****************************************************************************

float  target_speed  = 0;           // 目标速度，由菜单/按键/导航模块设置，供速度环 PID 使用
int run_state = 1;                  // 车体运行状态：1=正常；0=倾角过大/异常保护停机
static uint32 pid_ramp_counter = 0; // PID 软启动渐变计数器（独立于 sys_times，不受保护恢复影响）
static uint32 control_armed_times = 0; // 闭环使能后的运行计数，避免用上电时基误判启动等待

//--------------------------------------------------------------------------------
// 函数介绍    清零 PID 运行态，不改变已整定的 P/I/D 参数
//--------------------------------------------------------------------------------
static void pid_cycle_runtime_reset(pid_cycle_struct *pid_cycle)
{
    pid_cycle->p_value_last = 0.0f;
    pid_cycle->i_value = 0.0f;
    pid_cycle->out = 0.0f;
    pid_cycle->incremental_data[0] = 0.0f;
    pid_cycle->incremental_data[1] = 0.0f;
}

static void balance_pid_runtime_reset(void)
{
    pid_cycle_runtime_reset(&roll_balance_cascade.angular_speed_cycle);
    pid_cycle_runtime_reset(&roll_balance_cascade.angle_cycle);
    pid_cycle_runtime_reset(&roll_balance_cascade.speed_cycle);
    pid_cycle_runtime_reset(&pitch_balance_cascade.angle_cycle);
    pid_cycle_runtime_reset(&track_cascade.track_cycle);
}

static void jump_runtime_reset(void)
{
    jump_abort();
}

#ifdef USE_OBSTACLE_CONTROL
static void obstacle_runtime_reset_if_requested(void)
{
    if(obstacle_should_reset_pid())
    {
        balance_pid_runtime_reset();
        obstacle_clear_reset_request();
    }
}

static void obstacle_runtime_abort(void)
{
    obstacle_abort();
    obstacle_runtime_reset_if_requested();
}

static void obstacle_runtime_freeze_integral(void)
{
    if(obstacle_is_active() && !obstacle_speed_pid_should_update())
    {
        roll_balance_cascade.angle_cycle.i_value = 0.0f;
        roll_balance_cascade.angular_speed_cycle.i_value = 0.0f;
        roll_balance_cascade.speed_cycle.i_value = 0.0f;
        pitch_balance_cascade.angle_cycle.i_value = 0.0f;
        track_cascade.track_cycle.i_value = 0.0f;
    }
}
#endif

static void terrain_runtime_abort(void)
{
#ifdef USE_TERRAIN_CONTROL
    terrain_abort();
#ifdef USE_OBSTACLE_CONTROL
    obstacle_runtime_reset_if_requested();
#endif
#else
#ifdef USE_BRIDGE_CONTROL
    bridge_init();
#endif
#ifdef USE_OBSTACLE_CONTROL
    obstacle_runtime_abort();
#endif
#endif
}

#ifdef USE_BRIDGE_CONTROL
static uint8 bridge_control_active(void)
{
    if(bridge_test_active)
    {
        return 1;
    }
#ifdef USE_TERRAIN_CONTROL
    if(terrain_bridge_is_active())
    {
        return 1;
    }
#endif
    return 0;
}

static int16 bridge_speed_extra_to_duty(float speed_extra_rpm)
{
    float duty = speed_extra_rpm * BRIDGE_SPEED_EXTRA_DUTY_GAIN;
    if (duty > 0.0f && duty < (float)BRIDGE_SPEED_EXTRA_DUTY_MIN)
    {
        duty = (float)BRIDGE_SPEED_EXTRA_DUTY_MIN;
    }
    else if (duty < 0.0f && duty > -(float)BRIDGE_SPEED_EXTRA_DUTY_MIN)
    {
        duty = -(float)BRIDGE_SPEED_EXTRA_DUTY_MIN;
    }
    return (int16)func_limit_ab(duty, -(float)BRIDGE_SPEED_EXTRA_DUTY_MAX, (float)BRIDGE_SPEED_EXTRA_DUTY_MAX);
}

static float bridge_get_comp_speed_ref(void)
{
    float speed_ref = func_abs(target_speed);
    if (speed_ref >= BRIDGE_MIN_COMP_SPEED_RPM)
    {
        return speed_ref;
    }

    speed_ref = func_abs((float)car_speed);
    if (speed_ref >= BRIDGE_MIN_COMP_SPEED_RPM)
    {
        return speed_ref;
    }

    return BRIDGE_MIN_COMP_SPEED_RPM;
}
#endif

#ifdef USE_TERRAIN_CONTROL
static void terrain_runtime_update(int16 balance_motor)
{
    car_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;
    terrain_run_1ms(Car.mileage,
                    car_speed,
                    balance_motor,
                    roll_balance_cascade.posture_value.pit,
                    roll_balance_cascade.posture_value.rol,
                    control_armed_times);
}
#endif

//--------------------------------------------------------------------------------
// 函数介绍    计算并更新小车状态标志
// 返回参数    void
// 使用示例    car_state_calculate();
// 备注信息    1. 当横滚角或俯仰角绝对值超过 BODY_TILT_LIMIT_DEG 时判定翻车，立即停机并清零相关 PID 积分；
//             2. 系统运行前 STARTUP_RAMP_CYCLES 个周期（约 0.5s）将角度环与速度环 P 参数从 RAMP_START_RATIO 渐变到 RAMP_END_RATIO，
//                防止启动瞬间 P 过大导致震荡；
//             3. 跳跃过程中降低 P 参数并清零积分，提高落地稳定性。
//--------------------------------------------------------------------------------
void car_state_calculate(void)
{
    if(func_abs(roll_balance_cascade.posture_value.rol) > BODY_TILT_LIMIT_DEG || func_abs(roll_balance_cascade.posture_value.pit) > BODY_TILT_LIMIT_DEG)
    {
        jump_runtime_reset();
        run_state = 0;
        pid_ramp_counter = 0;
        control_armed_times = 0;
        balance_pid_runtime_reset();
        return;
    }

    run_state = 1;
#ifdef USE_TEST3_BALANCE_CORE
    // TEST3 2 的直立核心没有在启动后重新做 P 渐变；保持满参数，避免刚使能时输出过软扶不住。
    roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p;
    roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p;
#else
    pid_ramp_counter++;
    if(pid_ramp_counter < STARTUP_RAMP_CYCLES)
    {
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p * (RAMP_START_RATIO + (float)pid_ramp_counter / STARTUP_RAMP_CYCLES * (RAMP_END_RATIO - RAMP_START_RATIO));
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p * (RAMP_START_RATIO + (float)pid_ramp_counter / STARTUP_RAMP_CYCLES * (RAMP_END_RATIO - RAMP_START_RATIO));
        roll_balance_cascade.angle_cycle.i_value = 0;
    }
    else
    {
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p;
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p;
    }
#endif

    if(jump_flag)
    {
        roll_balance_cascade.angular_speed_cycle.i_value = 0;
        roll_balance_cascade.angle_cycle.i_value = 0;
        roll_balance_cascade.speed_cycle.i_value = 0;
        pitch_balance_cascade.angle_cycle.i_value = 0;
    }
}

//--------------------------------------------------------------------------------
// 函数介绍    转向控制
// 返回参数    void
// 使用示例    car_steer_control();
// 备注信息    1. 根据速度环输出计算基础转向量，并加入当前机械平衡角补偿；
//             2. 通过低通滤波减缓转向量突变，提升舵机跟踪平滑度；
//             3. 前 STEER_BALANCE_WAIT_CYCLES 个周期（约 2s）不引入平衡角补偿，避免刚启动时姿态未收敛造成抖动；
//             4. 当 jump_flag != 0 时调用 jump_control() 执行跳跃动作；
//             5. run_state == 0 时所有舵机以 ±STEER_EMERGENCY_RATE_LIMIT 的步长缓慢回到中心位置。
//--------------------------------------------------------------------------------
void car_steer_control(void)
{
    int16 steer_location_offset[4] = {0};   // 四个舵机当前位置相对于中心位置的偏移（带方向修正）
    
    int16 steer_target_offset[4] = {0};   // 四个舵机目标位置偏移（由转向指令与平衡补偿合成）
    
    static float steer_balance_angle_count = 0;  // 平衡角补偿量缓存（跳跃期间保持最后一次有效值）
    
    static float steer_output_duty_filter = 0;   // 转向输出低通滤波器历史值（一阶 IIR）
    
    int16 steer_output_duty = 0;            // 由速度环输出计算得到的基础转向 duty
    
    float steer_balance_angle = 0;          // 由左右平衡角度环输出计算得到的平衡补偿角
    
    // 俯仰角（rol 实际对应车体前后倾角）越大速度环输出越小，低速时转向灵敏度越低
    // pitch_offset ∈ [0, 1]，直立时（rol + mechanical_zero = 0）为 1，最大倾斜 STEER_PITCH_MAX_DEG 时降为 0
    float pitch_offset = (STEER_PITCH_MAX_DEG - func_limit_ab(func_abs(roll_balance_cascade.posture_value.rol + roll_balance_cascade.posture_value.mechanical_zero), 0.0f, STEER_PITCH_MAX_DEG)) / STEER_PITCH_MAX_DEG;

    
    // 跳跃时冻结转向输出滤波器，避免跳跃期间速度环波动污染滤波器，防止落地后转向突变产生顿挫
    if(jump_flag == 0)
    {
        // 由速度环输出计算基础转向 duty：先除以 STEER_SPEED_SCALE_DIV 进行缩放，限幅到 ±STEER_SPEED_LIMIT 后再乘以 STEER_SPEED_MULT
        // 等价于将 speed_cycle.out 映射到 [-STEER_SPEED_LIMIT*STEER_SPEED_MULT, STEER_SPEED_LIMIT*STEER_SPEED_MULT] 区间，作为舵机目标速度/位置增量
        steer_output_duty = func_limit_ab((int16)(roll_balance_cascade.speed_cycle.out / STEER_SPEED_SCALE_DIV), -STEER_SPEED_LIMIT, STEER_SPEED_LIMIT) * STEER_SPEED_MULT;//6
        
        // 根据当前俯仰角大小对转向 duty 进行衰减，车体倾斜越大转向越慢，避免失衡
        steer_output_duty = (int16)((float)steer_output_duty * pitch_offset);
        
//      steer_output_duty_filter = (steer_output_duty_filter * 19 + (float)steer_output_duty) / 20.0f;//低通滤波，相当于滤波系数 0.05
        steer_output_duty_filter = (steer_output_duty_filter * STEER_FILTER_OLD_WEIGHT + (float)steer_output_duty) / (float)STEER_FILTER_NEW_WEIGHT;   // 低通滤波，等效滤波系数 = (NEW_WEIGHT - OLD_WEIGHT) / NEW_WEIGHT
    }

    
    // 由机械平衡角计算补偿量（当前左右平衡未启用内环，仅使用 angle_cycle.out）


    // 正常行驶（非跳跃）时根据系统时序计算平衡补偿
    if(jump_flag == 0)
    {
        if(control_armed_times < STEER_BALANCE_WAIT_CYCLES)
        {
            // 使能后 STEER_BALANCE_WAIT_CYCLES 周期内不引入平衡角补偿，避免姿态未收敛导致舵机抖动
            steer_balance_angle = 0;
            pitch_balance_cascade.angle_cycle.i_value = 0;    // 同时清零左右平衡角度环积分
        }
        else
        {
            // 左右平衡角度环输出限幅到 ±STEER_BALANCE_LIMIT 后乘以 STEER_BALANCE_MULT，转换为舵机补偿量
            steer_balance_angle = func_limit_ab(pitch_balance_cascade.angle_cycle.out, -STEER_BALANCE_LIMIT, STEER_BALANCE_LIMIT) * STEER_BALANCE_MULT;//6
        }
        steer_balance_angle_count = steer_balance_angle;        // 保存当前补偿值，跳跃期间保持不变
    }

    // 计算四个舵机当前相对中心位置的偏移（带方向修正）
    steer_location_offset[0] = (steer_1.now_location - steer_1.center_num) * steer_1.steer_dir;
    steer_location_offset[1] = (steer_2.now_location - steer_2.center_num) * steer_2.steer_dir;
    steer_location_offset[2] = (steer_3.now_location - steer_3.center_num) * steer_3.steer_dir;
    steer_location_offset[3] = (steer_4.now_location - steer_4.center_num) * steer_4.steer_dir;

    // 合成四个舵机的目标偏移：
    //   steer_output_duty_filter 提供左右转向速度/位置差；
    //   steer_balance_angle_count 提供前后平衡补偿（仅单侧舵机生效，形成对角支撑）。
    // 左上舵机：右转为正，后仰（balance_angle > 0）时不补偿，前倾（balance_angle < 0）时补偿
    steer_target_offset[0] = (int16)( steer_output_duty_filter - (steer_balance_angle_count > 0 ? 0 : steer_balance_angle_count));
    // 右上舵机：左转为正，前倾（balance_angle < 0）时不补偿，后仰时补偿
    steer_target_offset[1] = (int16)( steer_output_duty_filter + (steer_balance_angle_count < 0 ? 0 : steer_balance_angle_count));
    // 左下舵机：左转为正，后仰时不补偿，前倾时补偿
    steer_target_offset[2] = (int16)(-steer_output_duty_filter - (steer_balance_angle_count > 0 ? 0 : steer_balance_angle_count));
    // 右下舵机：右转为正，前倾时不补偿，后仰时补偿
    steer_target_offset[3] = (int16)(-steer_output_duty_filter + (steer_balance_angle_count < 0 ? 0 : steer_balance_angle_count));

#ifndef USE_TEST3_BALANCE_CORE
    int16 pitch_compensation = (int16)pitch_balance_cascade.angle_cycle.out;
    steer_target_offset[0] += pitch_compensation;
    steer_target_offset[1] += pitch_compensation;
    steer_target_offset[2] -= pitch_compensation;
    steer_target_offset[3] -= pitch_compensation;
#endif

#ifdef USE_ROTATION_CONTROL
    // 原地旋转时叠加舵机偏转补偿（仅非跳跃状态）
    if (rotation_is_active() && jump_flag == 0)
    {
        int16 rotation_steer_offset = (int16)func_limit_ab((float)rotation.turn_duty * ROTATION_STEER_SCALE,
                                                           -ROTATION_STEER_MAX, ROTATION_STEER_MAX);
        steer_target_offset[0] += rotation_steer_offset;
        steer_target_offset[1] -= rotation_steer_offset;
        steer_target_offset[2] -= rotation_steer_offset;
        steer_target_offset[3] += rotation_steer_offset;
    }
#endif

#ifdef USE_BRIDGE_CONTROL
    // 单边桥腿高补偿（桥模式下调整舵机目标偏移，将 cm 差值映射为舵机 duty）
    if (jump_flag == 0)
    {
        float left_leg_delta = 0.0f, right_leg_delta = 0.0f;
        bridge_get_leg_delta(&left_leg_delta, &right_leg_delta);
        int16 left_leg_duty = (int16)(left_leg_delta * BRIDGE_LEG_TO_DUTY_RATIO);
        int16 right_leg_duty = (int16)(right_leg_delta * BRIDGE_LEG_TO_DUTY_RATIO);

        steer_target_offset[0] += left_leg_duty;   // steer_1 左上
        steer_target_offset[2] += left_leg_duty;   // steer_3 左下
        steer_target_offset[1] += right_leg_duty;  // steer_2 右上
        steer_target_offset[3] += right_leg_duty;  // steer_4 右下
    }
#endif

    // 车体处于正常运行状态时执行舵机控制
    if(run_state == 1)
    {
        // 非跳跃状态：正常行驶舵机闭环控制，每个周期最多移动 ±STEER_NORMAL_RATE_LIMIT，限制舵机速度防止抖动
        if(jump_flag == 0)
        {
            steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -STEER_NORMAL_RATE_LIMIT, STEER_NORMAL_RATE_LIMIT));//正常行驶时控制舵机
            steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -STEER_NORMAL_RATE_LIMIT, STEER_NORMAL_RATE_LIMIT));
            steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -STEER_NORMAL_RATE_LIMIT, STEER_NORMAL_RATE_LIMIT));
            steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -STEER_NORMAL_RATE_LIMIT, STEER_NORMAL_RATE_LIMIT));
          
//            steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -5, 5));//正常行驶时控制舵机
//            steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -5, 5));
//            steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -5, 5));
//            steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -5, 5));
        }
        else
        {
            // 跳跃状态：调用独立跳跃控制模块，按固定时序驱动四个舵机完成跳跃动作
            jump_control();
        }
    }
    else
        {
            // 异常停机时：所有舵机以 ±STEER_EMERGENCY_RATE_LIMIT 步长缓慢回到中心位置，防止跌落或撞击
            steer_control(&steer_1, func_limit_ab(steer_1.center_num - steer_1.now_location, -STEER_EMERGENCY_RATE_LIMIT, STEER_EMERGENCY_RATE_LIMIT) * steer_1.steer_dir);
            steer_control(&steer_2, func_limit_ab(steer_2.center_num - steer_2.now_location, -STEER_EMERGENCY_RATE_LIMIT, STEER_EMERGENCY_RATE_LIMIT) * steer_2.steer_dir);
            steer_control(&steer_3, func_limit_ab(steer_3.center_num - steer_3.now_location, -STEER_EMERGENCY_RATE_LIMIT, STEER_EMERGENCY_RATE_LIMIT) * steer_3.steer_dir);
            steer_control(&steer_4, func_limit_ab(steer_4.center_num - steer_4.now_location, -STEER_EMERGENCY_RATE_LIMIT, STEER_EMERGENCY_RATE_LIMIT) * steer_4.steer_dir);
        }
    

}

int32 car_distance = 0;             // 车体累计里程（cm），由 car_motor_control 每次调用累加（当前未被主循环调用）
int16 left_motor_duty = 0;          // 左电机输出占空比（car_motor_control 使用）
int16 right_motor_duty = 0;         // 右电机输出占空比（car_motor_control 使用）
//--------------------------------------------------------------------------------
// 函数介绍    电机占空比控制
// 返回参数    void
// 使用示例    car_motor_control();
// 备注信息    1. 根据运行标志计算左右电机占空比，限幅后通过 small_driver_set_duty 设置电机驱动；
//             2. 左右电机基础值来自角速度环输出；
//             3. 叠加 IMU660RB Z 轴陀螺仪数据实现差速转向；
//             4. 本函数当前未被 pit_call_back() 调用，属于备用/历史实现，主循环直接调用 CYT2_D_motor_ctrl()。
//--------------------------------------------------------------------------------
void car_motor_control(void)
{
    // 根据当前 car_speed（RPM）累加车体里程：
    // 转/分钟 -> 转/秒(/60) -> 线速度(× 轮径 × π) -> 按 1ms 调用周期缩放(× 0.001)
    car_distance += ((float)car_speed / 60.0f * WHEEL_CIRCUMFERENCE * PI * 0.001f);

    if(run_state)                                         // 当运行状态为 1 时计算电机占空比
    {
        // 左电机占空比：取角速度环输出并限幅到 ±BALANCE_DUTY_MAX
        left_motor_duty = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -BALANCE_DUTY_MAX, BALANCE_DUTY_MAX);
        // 右电机占空比：取角速度环输出并限幅到 ±BALANCE_DUTY_MAX
        right_motor_duty = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -BALANCE_DUTY_MAX, BALANCE_DUTY_MAX);

        // 叠加 Z 轴陀螺仪数据（IMU660RB），实现转向差速控制：
        // 左轮 += imu660rb_gyro_z / TURN_GYRO_SCALE_DIV，右轮 -= imu660rb_gyro_z / TURN_GYRO_SCALE_DIV，产生转向力矩
        left_motor_duty = func_limit_ab(left_motor_duty + imu660rb_gyro_z / TURN_GYRO_SCALE_DIV,    -TURN_DUTY_MAX, TURN_DUTY_MAX);
        right_motor_duty = func_limit_ab(right_motor_duty - imu660rb_gyro_z / TURN_GYRO_SCALE_DIV,   -TURN_DUTY_MAX, TURN_DUTY_MAX);
    }
    else                                                  // 当运行状态为 0 时电机关闭
    {  
        left_motor_duty = 0;                              // 左电机占空比置为 0
        right_motor_duty = 0;                             // 右电机占空比置为 0
    }

    small_driver_set_duty(left_motor_duty, -right_motor_duty); // 设置左右电机占空比（右电机取反，匹配安装方向）
    
//        small_driver_set_duty((int16)roll_balance_cascade.angular_speed_cycle.out,-(int16)roll_balance_cascade.angular_speed_cycle.out); // 设置左右电机占空比（右电机取反）

//    CYT2_D_motor_ctrl(-left_motor_duty,-right_motor_duty);
}

uint32 sys_times = 0;                 // PIT 中断累计计数，系统时基（每周期约 1ms）
int STOP_FALG = 1;                    // 总电机输出使能标志：1=允许输出；0=强制停车（历史拼写，保持原名）
uint8 system_armed = 0;               // 菜单启动使能标志：上电默认 0（静止），进入运行菜单后由 Menu.c 置 1
uint8 bridge_test_active = 0;         // 单边桥测试使能：仅菜单测试页置 1

//--------------------------------------------------------------------------------
// 函数介绍    PIT 定时中断回调函数（主控制循环）
// 返回参数    void
// 使用示例    由 PIT 定时中断自动调用，无需手动调用
// 备注信息    1. 本函数为整个车体控制的主实时循环，调用周期约为 1ms；
//             2. 系统启动 CONTROL_STARTUP_CYCLES 周期后（约 0.5s）进入闭环控制，执行流程：
//                a) 读取 IMU660RB 陀螺仪与加速度计数据；
//                b) 调用 quaternion_module_calculate() 更新四元数与姿态角；
//                c) 每 LOOP_DIV_ANGLE_CYCLE 个周期：里程更新、导航处理、角度环 PID；
//                d) 每个周期：角速度环 PID、舵机控制；
//                e) 每 LOOP_DIV_SPEED_CYCLE 个周期：速度计算、速度环 PID、转向环 PID（复现模式）；
//                f) 根据 STOP_FALG 输出电机占空比。
//             3. 角度环 -> 角速度环 -> 电机输出构成串级控制；速度环输出叠加到角度环/电机；
//             4. 导航复现模式（fuxian == 1）下，转向环根据 N.Final_Out 偏差控制方向。
//--------------------------------------------------------------------------------
void pit_call_back(void)
{

// static uint32 system_time_state[20] = {0};  

    sys_times ++;                                    // 系统计时累加，每进入一次中断加 1
    
//    for(int i = 0; i < 20; i ++)
//    {
//        system_time_state[i] = (sys_times % (i + 1)) == 0 ? 1 : system_time_state[i];
//    }
    
//    imu660ra_get_gyro();                             // 读取 IMU660RA 陀螺仪数据（已弃用，保留注释）
//    imu660ra_get_acc();                              // 读取 IMU660RA 加速度计数据（已弃用，保留注释）
//    quaternion_module_calculate(&roll_balance_cascade); // 计算四元数并更新姿态（已弃用，保留注释）
    
    
//    imu660ra_get_gyro();                             // 读取 IMU660RA 陀螺仪数据（已弃用，保留注释）
//    imu660ra_get_acc();                              // 读取 IMU660RA 加速度计数据（已弃用，保留注释）
//    quaternion_module_calculate(&roll_balance_cascade); // 计算四元数并更新姿态（已弃用，保留注释）
    
    
    // 读取 IMU660RB 陀螺仪数据（写入全局变量 imu660rb_gyro_x/y/z）
    imu660rb_get_gyro();
    // 读取 IMU660RB 加速度计数据（写入全局变量 imu660rb_acc_x/y/z）
    imu660rb_get_acc();
    // 基于 IMU 数据融合更新四元数并计算姿态角（rol/pit/yaw），结果存入 roll_balance_cascade.posture_value
    quaternion_module_calculate(&roll_balance_cascade);

#ifdef USE_TEST3_BALANCE_CORE
    // TEST3 2 已验证直立核心：菜单只负责改目标/模式，不参与直立闭环使能和 PID 重置。
    if(sys_times > CONTROL_STARTUP_CYCLES)
    {
        run_state = 1;
        if(!system_armed)
        {
            CYT2_D_motor_ctrl(0, 0);
            terrain_runtime_abort();
            terrain_debug_update(sys_times,
                                 0,
                                 0,
                                 roll_balance_cascade.angle_cycle.out,
                                 roll_balance_cascade.speed_cycle.out,
                                 track_cascade.track_cycle.out,
                                 N.Final_Out);
            jump_runtime_reset();
            balance_pid_runtime_reset();
            control_armed_times = 0;
            car_steer_control();
            return;
        }

        control_armed_times++;

        // 基础直立不引入横滚/腿高补偿，避免站立环被侧倾辅助污染。
        pitch_balance_cascade.angle_cycle.out = 0.0f;
        pitch_balance_cascade.angle_cycle.i_value = 0.0f;

        stair_service_1ms();

        if(sys_times % LOOP_DIV_ANGLE_CYCLE == 0)
        {
            float angle_target = 0.0f - roll_balance_cascade.posture_value.mechanical_zero;

            CYT2_get_distance();
            Nag_System();

#ifdef USE_OBSTACLE_CONTROL
            angle_target += obstacle_get_angle_offset();
#endif

            if(stair_px_angle_control_active())
            {
                angle_target += stair_get_px_angle_offset();
            }

            pid_control(&roll_balance_cascade.angle_cycle,
                        angle_target,
                        -roll_balance_cascade.posture_value.pit);
        }

#ifdef USE_TERRAIN_CONTROL
        if(STOP_FALG == 1 && system_armed && run_state == 1 && jump_flag == 0)
        {
            terrain_runtime_update((int16)(-roll_balance_cascade.angular_speed_cycle.out));
        }
        else
        {
            terrain_runtime_abort();
        }
#endif

#ifdef USE_BRIDGE_CONTROL
        if(STOP_FALG == 1 && bridge_control_active() && jump_flag == 0)
        {
            bridge_run(roll_balance_cascade.posture_value.rol, Car.mileage, bridge_get_comp_speed_ref());
        }
        else if(bridge_get_state() != BRIDGE_IDLE)
        {
            bridge_init();
        }
#endif

        pid_control(&roll_balance_cascade.angular_speed_cycle,
                    roll_balance_cascade.angle_cycle.out,
                    imu660rb_gyro_y);

#ifdef USE_OBSTACLE_CONTROL
        if(STOP_FALG == 1 && system_armed && run_state == 1 && jump_flag == 0)
        {
            car_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;
            obstacle_run_1ms(Car.mileage,
                             car_speed,
                             (int16)(-roll_balance_cascade.angular_speed_cycle.out),
                             roll_balance_cascade.posture_value.pit,
                             roll_balance_cascade.posture_value.rol);
            obstacle_runtime_reset_if_requested();
            obstacle_runtime_freeze_integral();
        }
        else if(obstacle_is_active())
        {
            obstacle_runtime_abort();
        }
#endif

        car_steer_control();

        if(sys_times % LOOP_DIV_SPEED_CYCLE == 0)
        {
            car_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;
            if(body_jump_speed_loop_should_update()
#ifdef USE_OBSTACLE_CONTROL
               && obstacle_speed_pid_should_update()
#endif
              )
            {
                pid_control(&roll_balance_cascade.speed_cycle, target_speed, (float)car_speed);
            }
            else
            {
                roll_balance_cascade.speed_cycle.out = 0.0f;
#ifdef USE_OBSTACLE_CONTROL
                roll_balance_cascade.speed_cycle.i_value = 0.0f;
                roll_balance_cascade.speed_cycle.p_value_last = 0.0f;
#endif
            }

            if (fuxian == 1
#ifdef USE_OBSTACLE_CONTROL
                && obstacle_track_pid_should_update()
#endif
               )
            {
                pid_control(&track_cascade.track_cycle, N.Final_Out, 0);
            }
#ifdef USE_OBSTACLE_CONTROL
            else if(fuxian == 1)
            {
                track_cascade.track_cycle.out = 0.0f;
                track_cascade.track_cycle.i_value = 0.0f;
                track_cascade.track_cycle.p_value_last = 0.0f;
            }
#endif
        }

        if(STOP_FALG == 1 && system_armed)
        {
            int16 nav_diff = (int16)(N.Final_Out * NAV_TURN_DIFF_MULT
#ifdef USE_OBSTACLE_CONTROL
                                     * obstacle_get_nav_scale()
#endif
                                    ) + stair_get_heading_motor_adj();
            int16 jump_boost = body_jump_motor_boost_duty;
            int16 left_extra_duty = 0;
            int16 right_extra_duty = 0;
            int16 left_motor = 0;
            int16 right_motor = 0;

#ifdef USE_BRIDGE_CONTROL
            if (bridge_control_active() && bridge_get_state() != BRIDGE_IDLE && jump_flag == 0)
            {
                float left_extra_speed = 0.0f, right_extra_speed = 0.0f;
                bridge_get_speed_extra(&left_extra_speed, &right_extra_speed);
                left_extra_duty = bridge_speed_extra_to_duty(left_extra_speed);
                right_extra_duty = bridge_speed_extra_to_duty(right_extra_speed);
            }
#endif

            left_motor = -(int16)roll_balance_cascade.angular_speed_cycle.out + nav_diff + jump_boost + left_extra_duty;
            right_motor = -(int16)roll_balance_cascade.angular_speed_cycle.out - nav_diff + jump_boost + right_extra_duty;
#ifdef USE_OBSTACLE_CONTROL
            left_motor += obstacle_get_motor_boost();
            right_motor += obstacle_get_motor_boost();
            if(obstacle_get_motor_override(&left_motor, &right_motor))
            {
                obstacle_runtime_freeze_integral();
            }
#endif
            terrain_debug_update(sys_times,
                                 left_motor,
                                 right_motor,
                                 roll_balance_cascade.angle_cycle.out,
                                 roll_balance_cascade.speed_cycle.out,
                                 track_cascade.track_cycle.out,
                                 N.Final_Out);
            CYT2_D_motor_ctrl(left_motor, right_motor);
        }
        else
        {
            terrain_runtime_abort();
            terrain_debug_update(sys_times,
                                 0,
                                 0,
                                 roll_balance_cascade.angle_cycle.out,
                                 roll_balance_cascade.speed_cycle.out,
                                 track_cascade.track_cycle.out,
                                 N.Final_Out);
            CYT2_D_motor_ctrl(0, 0);
        }
    }
    else
    {
        terrain_runtime_abort();
        terrain_debug_update(sys_times,
                             0,
                             0,
                             roll_balance_cascade.angle_cycle.out,
                             roll_balance_cascade.speed_cycle.out,
                             track_cascade.track_cycle.out,
                             N.Final_Out);
        CYT2_D_motor_ctrl(0, 0);
    }
    return;
#else

#ifdef USE_FIRST_ORDER_FILTER
    first_order_filter_update();
#endif

    // 系统启动 CONTROL_STARTUP_CYCLES 周期后（约 0.5s，等待姿态收敛）才进入闭环控制
    if(sys_times > CONTROL_STARTUP_CYCLES)
    {
        static uint8 last_system_armed = 0;
        if(!system_armed)
        {
            CYT2_D_motor_ctrl(0, 0);
            terrain_runtime_abort();
            terrain_debug_update(sys_times,
                                 0,
                                 0,
                                 roll_balance_cascade.angle_cycle.out,
                                 roll_balance_cascade.speed_cycle.out,
                                 track_cascade.track_cycle.out,
                                 N.Final_Out);
            jump_runtime_reset();
            balance_pid_runtime_reset();
            pid_ramp_counter = 0;
            control_armed_times = 0;
            last_system_armed = 0;
            return;
        }

        // system_armed 由 0->1 时重新捕获当前 yaw 作为直行目标，避免菜单等待期间搬动车体导致上电后旋转
        if (!last_system_armed)
        {
            yaw_target = roll_balance_cascade.posture_value.yaw;
            balance_steering_set_yaw_target(roll_balance_cascade.posture_value.yaw);
            balance_pid_runtime_reset();
            pid_ramp_counter = 0;
            control_armed_times = 0;
            last_system_armed = 1;
        }

        control_armed_times++;
        car_state_calculate();

        if(STOP_FALG == 0)
        {
            terrain_runtime_abort();
            jump_runtime_reset();
        }

          // 每 LOOP_DIV_ANGLE_CYCLE 个周期（约 LOOP_DIV_ANGLE_CYCLE ms）执行一次：里程更新、导航、角度环 PID
          if(sys_times % LOOP_DIV_ANGLE_CYCLE == 0)
          {
             
             CYT2_get_distance();                          // 刷新车体累计里程（Car.mileage 等）
             
             Nag_System();                                 // 惯性导航/巡线状态机：录制、读取 Flash、复现路径

             // 前后平衡角度环 PID：
             // 目标值 = 0 - mechanical_zero，默认直立为 0，机械偏置通过 config.h 微调
             // 实际值 = -posture_value.pit（姿态解算得到的俯仰角取反，与坐标轴定义一致）
             // 输出作为下一级角速度环的目标值
              // 角度环 PID：现有 pid_control() 为 legacy 接口；定义 USE_NEW_PID 宏可切换到 pid_calc()
#ifdef USE_NEW_PID
              roll_balance_cascade.angle_cycle.out = pid_calc(&roll_angle_pid,
                                                              0.0f - roll_balance_cascade.posture_value.mechanical_zero
#ifdef USE_OBSTACLE_CONTROL
                                                              + obstacle_get_angle_offset()
#endif
                                                              ,
                                                              -roll_balance_cascade.posture_value.pit);
#else
              pid_control(&roll_balance_cascade.angle_cycle,
                          0.0f - roll_balance_cascade.posture_value.mechanical_zero
#ifdef USE_OBSTACLE_CONTROL
                          + obstacle_get_angle_offset()
#endif
                          ,
                          -roll_balance_cascade.posture_value.pit);
#endif

#ifdef USE_ROLL_BALANCE_CONTROL
             // 横滚轴角度环 PID：目标 = 0（车身竖直），实际 = posture_value.rol
             // 积分已启用（pitch_balance_cascade.angle_cycle.i 与 i_value_pro 均非 0），输出驱动舵机伸缩腿高
             pid_control(&pitch_balance_cascade.angle_cycle, 0.0f, roll_balance_cascade.posture_value.rol);
#else
             pitch_balance_cascade.angle_cycle.out = 0.0f;
             pitch_balance_cascade.angle_cycle.i_value = 0.0f;
#endif
          }

#ifdef USE_TERRAIN_CONTROL
           if(STOP_FALG == 1 && system_armed && run_state == 1 && jump_flag == 0)
           {
#ifdef USE_TEST3_BALANCE_CORE
               terrain_runtime_update((int16)(-roll_balance_cascade.angular_speed_cycle.out));
#else
               terrain_runtime_update((int16)(BALANCE_MOTOR_OUTPUT_SIGN * roll_balance_cascade.angular_speed_cycle.out));
#endif
           }
           else
           {
               terrain_runtime_abort();
           }
#endif

#ifdef USE_BRIDGE_CONTROL
           // 单边桥状态机必须先用当前横滚角更新，再由 car_steer_control() 读取腿高补偿
           if (STOP_FALG == 1 && system_armed && bridge_control_active() && jump_flag == 0 && run_state == 1) {
               bridge_run(roll_balance_cascade.posture_value.rol, Car.mileage, bridge_get_comp_speed_ref());
           }
           else if (bridge_get_state() != BRIDGE_IDLE) {
               bridge_init();
           }
#endif
          
          // 前后平衡角速度环 PID：
          // 目标值 = 角度环输出（角度环期望的角速度）
          // TEST3 2 使用 imu660rb_gyro_y 原始方向；非 TEST3 模式使用 GYRO_DATA_Y 的姿态方向约定。
          // 输出直接驱动电机，响应最快
           // 角速度环 PID：定义 USE_NEW_PID 宏可切换到 pid_calc()
#ifdef USE_TEST3_BALANCE_CORE
#ifdef USE_NEW_PID
           roll_balance_cascade.angular_speed_cycle.out = pid_calc(&roll_angspeed_pid, roll_balance_cascade.angle_cycle.out, imu660rb_gyro_y);
#else
           pid_control(&roll_balance_cascade.angular_speed_cycle, roll_balance_cascade.angle_cycle.out, imu660rb_gyro_y);
#endif
#else
#ifdef USE_NEW_PID
           roll_balance_cascade.angular_speed_cycle.out = pid_calc(&roll_angspeed_pid, roll_balance_cascade.angle_cycle.out, GYRO_DATA_Y);
#else
           pid_control(&roll_balance_cascade.angular_speed_cycle, roll_balance_cascade.angle_cycle.out, GYRO_DATA_Y);
#endif
#endif

#ifdef USE_OBSTACLE_CONTROL
           if(STOP_FALG == 1 && system_armed && run_state == 1 && jump_flag == 0)
           {
               car_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;
               obstacle_run_1ms(Car.mileage,
                                car_speed,
#ifdef USE_TEST3_BALANCE_CORE
                                (int16)(-roll_balance_cascade.angular_speed_cycle.out),
#else
                                (int16)(BALANCE_MOTOR_OUTPUT_SIGN * roll_balance_cascade.angular_speed_cycle.out),
#endif
                                roll_balance_cascade.posture_value.pit,
                                roll_balance_cascade.posture_value.rol);
               obstacle_runtime_reset_if_requested();
               obstacle_runtime_freeze_integral();
           }
           else if(obstacle_is_active())
           {
               obstacle_runtime_abort();
           }
#endif
          
           // 舵机转向控制（包含速度/横滚平衡/单边桥补偿与跳跃动作），每个周期都执行以保证舵机跟踪
           car_steer_control();

#ifdef USE_ROTATION_CONTROL
           if ((STOP_FALG == 0 || run_state == 0 || jump_flag != 0) && rotation_is_active()) rotation_stop();

           rotation_run();
#endif

           // 每 LOOP_DIV_SPEED_CYCLE 个周期（约 LOOP_DIV_SPEED_CYCLE ms）执行一次：速度环、转向环（导航复现模式）
          if(sys_times % LOOP_DIV_SPEED_CYCLE == 0)
          {
              // 计算车体速度：左右电机速度取平均，注意右电机方向与左电机相反，故使用减法
              car_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;
              
              // 起跳后锁定速度环输出；加速/预压阶段保留速度环，避免 JUMP_FLAG_ACCEL 变成原地电机锁死
               if(!JUMP_MOTOR_LOCK_ACTIVE(jump_flag))
               {
                   // 速度环 PID：目标值为 target_speed，实际值为 car_speed（RPM），输出影响平衡目标角度
                    // 速度环 PID：定义 USE_NEW_PID 宏可切换到 pid_calc()
                   if(obstacle_speed_pid_should_update())
                   {
#ifdef USE_NEW_PID
                    roll_balance_cascade.speed_cycle.out = pid_calc(&roll_speed_pid, target_speed, (float)car_speed);
#else
                    pid_control(&roll_balance_cascade.speed_cycle, target_speed, (float)car_speed);
#endif
                   }
                   else
                   {
                       roll_balance_cascade.speed_cycle.out = 0.0f;
                       roll_balance_cascade.speed_cycle.i_value = 0.0f;
                       roll_balance_cascade.speed_cycle.p_value_last = 0.0f;
                   }
               }
               else
               {
                   roll_balance_cascade.speed_cycle.out = 0.0f;  // 起跳后清零速度环输出，锁定转向
               }

#ifdef USE_ROTATION_CONTROL
               if (rotation_is_active())
               {
                   roll_balance_cascade.speed_cycle.out = 0.0f;
                   N.Final_Out = 0.0f;
               }
#endif
              
              // 复现模式（fuxian == 1）：启用转向环 PID，根据导航偏差 N.Final_Out 纠偏
              // 目标值 = N.Final_Out（当前偏航与目标路径的偏差），期望值 = 0（无偏差）
              if (fuxian == 1 && obstacle_track_pid_should_update())
              {
                  pid_control(&track_cascade.track_cycle, N.Final_Out, 0);
              }
              else if(fuxian == 1)
              {
                  track_cascade.track_cycle.out = 0.0f;
                  track_cascade.track_cycle.i_value = 0.0f;
                  track_cascade.track_cycle.p_value_last = 0.0f;
              }
          }
          
            // 根据总使能标志 STOP_FALG 与菜单启动标志 system_armed 输出电机占空比
            if(STOP_FALG == 1 && system_armed && run_state == 1)
            {
               // 起跳后锁定电机输出，防止跳跃着地时车轮空转/抱死导致失稳；加速阶段仍按正常平衡输出
		               if(JUMP_MOTOR_LOCK_ACTIVE(jump_flag))
	               {
	                   terrain_debug_update(sys_times,
	                                        JUMP_MOTOR_LOCK_DUTY,
	                                        JUMP_MOTOR_LOCK_DUTY,
	                                        roll_balance_cascade.angle_cycle.out,
	                                        roll_balance_cascade.speed_cycle.out,
	                                        track_cascade.track_cycle.out,
	                                        N.Final_Out);
	                   CYT2_D_motor_ctrl(JUMP_MOTOR_LOCK_DUTY, JUMP_MOTOR_LOCK_DUTY);
	               }
		               else
		               {
#ifdef USE_TEST3_BALANCE_CORE
                              int16 rot = 0;
                              int16 left_extra_duty = 0, right_extra_duty = 0;

#ifdef USE_ROTATION_CONTROL
                              rot = rotation_is_active() ? rotation.turn_duty : 0;
#endif
#ifdef USE_BRIDGE_CONTROL
                              if (bridge_control_active() && bridge_get_state() != BRIDGE_IDLE && jump_flag == 0) {
                                  float left_extra_speed = 0.0f, right_extra_speed = 0.0f;
                                  bridge_get_speed_extra(&left_extra_speed, &right_extra_speed);
                                  left_extra_duty = bridge_speed_extra_to_duty(left_extra_speed);
                                  right_extra_duty = bridge_speed_extra_to_duty(right_extra_speed);
                              }
#endif
                              int16 nav_diff = 0;
#ifdef USE_ROTATION_CONTROL
                              nav_diff = rotation_is_active() ? 0 : (int16)(N.Final_Out * NAV_TURN_DIFF_MULT);
#else
                              nav_diff = (int16)(N.Final_Out * NAV_TURN_DIFF_MULT);
#endif
#ifdef USE_OBSTACLE_CONTROL
                              nav_diff = (int16)((float)nav_diff * obstacle_get_nav_scale());
#endif
                              int16 left_motor  = -(int16)roll_balance_cascade.angular_speed_cycle.out + nav_diff + rot;
                              int16 right_motor = -(int16)roll_balance_cascade.angular_speed_cycle.out - nav_diff - rot;
                              left_motor += left_extra_duty;
                              right_motor += right_extra_duty;
#ifdef USE_OBSTACLE_CONTROL
                              left_motor += obstacle_get_motor_boost();
                              right_motor += obstacle_get_motor_boost();
                              if(obstacle_get_motor_override(&left_motor, &right_motor))
                              {
                                  obstacle_runtime_freeze_integral();
                              }
#endif
                              terrain_debug_update(sys_times,
                                                   left_motor,
                                                   right_motor,
                                                   roll_balance_cascade.angle_cycle.out,
                                                   roll_balance_cascade.speed_cycle.out,
                                                   track_cascade.track_cycle.out,
                                                   N.Final_Out);
                              CYT2_D_motor_ctrl(left_motor, right_motor);
#else
		                      int16 base     = (int16)(BALANCE_MOTOR_OUTPUT_SIGN * roll_balance_cascade.angular_speed_cycle.out);

		                      float left_extra_speed = 0.0f, right_extra_speed = 0.0f;
		                      int16 left_extra_duty = 0, right_extra_duty = 0;
#ifdef USE_BRIDGE_CONTROL
		                      if (bridge_control_active() && bridge_get_state() != BRIDGE_IDLE && jump_flag == 0) {
		                          bridge_get_speed_extra(&left_extra_speed, &right_extra_speed);
		                          left_extra_duty = bridge_speed_extra_to_duty(left_extra_speed);
		                          right_extra_duty = bridge_speed_extra_to_duty(right_extra_speed);
		                      }
#endif

#ifdef USE_DIFFERENTIAL_STEERING
                      float yaw_error = yaw_angle_diff(yaw_target, roll_balance_cascade.posture_value.yaw);
                      balance_steering_calc((float)base, (float)imu660rb_gyro_z, yaw_error);
                      int16 left_motor  = (int16)left_duty;
                      int16 right_motor = (int16)right_duty;
#else
	                      int16 nav_diff = 0;
	                      int16 rot      = 0;
#ifdef USE_ROTATION_CONTROL
	                      nav_diff = rotation_is_active() ? 0 : (int16)(N.Final_Out * NAV_TURN_DIFF_MULT);
	                      rot      = rotation_is_active() ? rotation.turn_duty : 0;
#else
	                      nav_diff = (int16)(N.Final_Out * NAV_TURN_DIFF_MULT);
#endif
#ifdef USE_OBSTACLE_CONTROL
	                      nav_diff = (int16)((float)nav_diff * obstacle_get_nav_scale());
#endif
	                      int16 left_motor  = base + nav_diff + rot;
                      int16 right_motor = base - nav_diff - rot;
#endif

		                      left_motor += left_extra_duty;
		                      right_motor += right_extra_duty;
#ifdef USE_OBSTACLE_CONTROL
		                      left_motor += obstacle_get_motor_boost();
		                      right_motor += obstacle_get_motor_boost();
		                      if(obstacle_get_motor_override(&left_motor, &right_motor))
		                      {
		                          obstacle_runtime_freeze_integral();
		                      }
#endif
		                      terrain_debug_update(sys_times,
		                                           left_motor,
		                                           right_motor,
		                                           roll_balance_cascade.angle_cycle.out,
		                                           roll_balance_cascade.speed_cycle.out,
		                                           track_cascade.track_cycle.out,
		                                           N.Final_Out);
		                      CYT2_D_motor_ctrl(left_motor, right_motor);
#endif
		               }
		           }
            else
            {
                // 未使能、STOP_FALG == 0 或 run_state == 0 时强制停车，并清零 PID 运行态
                terrain_runtime_abort();
                terrain_debug_update(sys_times,
                                     0,
                                     0,
                                     roll_balance_cascade.angle_cycle.out,
                                     roll_balance_cascade.speed_cycle.out,
                                     track_cascade.track_cycle.out,
                                     N.Final_Out);
                CYT2_D_motor_ctrl(0, 0);
                balance_pid_runtime_reset();
            }
          
          
    }
#endif
}
