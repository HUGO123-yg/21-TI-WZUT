#include "zf_common_headfile.h"
#include "config.h"
#include "Rotation.h"

float target_speed = BODY_TARGET_SPEED_DEFAULT;
int jump_flag=0;
int jump_time=0;
int run_state = BODY_RUN_STATE_DEFAULT;
static volatile uint8 control_background_pending = 0;

typedef struct
{
    float target_speed;
    float navigation_output;
    uint32 sequence;
    uint8 motor_enable;
    uint8 track_enable;
} body_control_command_struct;

static volatile body_control_command_struct control_command_mailbox =
{
    .target_speed = BODY_TARGET_SPEED_DEFAULT,
    .navigation_output = 0.0f,
    .sequence = 0U,
    .motor_enable = BODY_STOP_FLAG_DEFAULT,
    .track_enable = 0U,
};

static body_control_command_struct active_control_command =
{
    .target_speed = BODY_TARGET_SPEED_DEFAULT,
    .navigation_output = 0.0f,
    .sequence = 0U,
    .motor_enable = BODY_STOP_FLAG_DEFAULT,
    .track_enable = 0U,
};

volatile uint32 control_uptime_ticks = 0U;

void control_publish_main_command(void)
{
    uint32 interrupt_state = Cy_SysLib_EnterCriticalSection();

    if(control_command_mailbox.target_speed != target_speed
        || control_command_mailbox.navigation_output != N.Final_Out
        || control_command_mailbox.motor_enable != ((STOP_FALG != 0) ? 1U : 0U)
        || control_command_mailbox.track_enable != ((fuxian != 0U) ? 1U : 0U))
    {
        control_command_mailbox.target_speed = target_speed;
        control_command_mailbox.navigation_output = N.Final_Out;
        control_command_mailbox.motor_enable = (STOP_FALG != 0) ? 1U : 0U;
        control_command_mailbox.track_enable = (fuxian != 0U) ? 1U : 0U;
        control_command_mailbox.sequence++;
    }
    Cy_SysLib_ExitCriticalSection(interrupt_state);
}

static void body_receive_main_command(void)
{
    uint32 sequence = control_command_mailbox.sequence;

    if(sequence != active_control_command.sequence)
    {
        active_control_command.target_speed = control_command_mailbox.target_speed;
        active_control_command.navigation_output = control_command_mailbox.navigation_output;
        active_control_command.motor_enable = control_command_mailbox.motor_enable;
        active_control_command.track_enable = control_command_mailbox.track_enable;
        active_control_command.sequence = sequence;
    }
}

static uint8 body_motor_write_nonblocking(int16 left_speed, int16 right_speed)
{
    uint8 frame[7];
    int16 left_duty;
    int16 right_duty;
    volatile stc_SCB_t *uart_module = get_scb_module(SMALL_DRIVER_UART);

    left_speed = func_limit_ab(left_speed, M_MIN, M_MAX);
    right_speed = func_limit_ab(right_speed, M_MIN, M_MAX);
    left_duty = (int16)-left_speed;
    right_duty = right_speed;

    // Never wait in the 1 ms control ISR. If a complete frame cannot fit,
    // the next control cycle retries with the newest command.
    if((Cy_SCB_GetFifoSize(uart_module) - Cy_SCB_GetNumInTxFifo(uart_module)) < 7U)
    {
        return 0U;
    }

    frame[0] = 0xA5U;
    frame[1] = 0x01U;
    frame[2] = (uint8)((left_duty & 0xFF00) >> 8);
    frame[3] = (uint8)(left_duty & 0x00FF);
    frame[4] = (uint8)((right_duty & 0xFF00) >> 8);
    frame[5] = (uint8)(right_duty & 0x00FF);
    frame[6] = 0U;

    for(uint8 index = 0U; index < 6U; index++)
    {
        frame[6] = (uint8)(frame[6] + frame[index]);
    }

    for(uint8 index = 0U; index < 7U; index++)
    {
        Cy_SCB_WriteTxFifo(uart_module, frame[index]);
    }

    return 1U;
}

static void body_roll_pid_reset(void)
{
    pitch_balance_cascade.angle_cycle.i_value = 0;
    pitch_balance_cascade.angle_cycle.p_value_last = 0;
    pitch_balance_cascade.angle_cycle.out = 0;
    pitch_balance_cascade.angular_speed_cycle.i_value = 0;
    pitch_balance_cascade.angular_speed_cycle.p_value_last = 0;
    pitch_balance_cascade.angular_speed_cycle.out = 0;
}

static void body_speed_pid_reset(void)
{
    roll_balance_cascade.speed_cycle.i_value = 0;
    roll_balance_cascade.speed_cycle.p_value_last = 0;
    roll_balance_cascade.speed_cycle.out = 0;
    car_speed = 0;
}

//--------------------------------------------------------------------------------
// 函数简介    计算并更新车辆状态标志
// 返回参数    void
// 使用示例    car_state_calculate();
// 备注信息    根据横滚角和俯仰角处理倾倒保护、恢复延时以及 PID 参数渐变
//--------------------------------------------------------------------------------
void car_state_calculate(void)
{
    float roll_angle = BODY_ROLL_ANGLE_FEEDBACK_SIGN * roll_balance_cascade.posture_value.rol;
    float pitch_angle = BODY_PITCH_ANGLE_FEEDBACK_SIGN * roll_balance_cascade.posture_value.pit;

    if(func_abs(roll_angle) > BODY_TILT_LIMIT_DEG || func_abs(pitch_angle) > BODY_TILT_LIMIT_DEG)//横滚角和俯仰角超过保护阈值时，小车关机
    {
        jump_flag = 0;
        jump_time = 0;
        run_state = 0;                          // 停止运行
        sys_times = 0;
        
        roll_balance_cascade.angular_speed_cycle.i_value = 0; // 重置角速度环 PID 积分值
        roll_balance_cascade.angular_speed_cycle.out = 0;
        roll_balance_cascade.angle_cycle.i_value = 0;
        roll_balance_cascade.angle_cycle.out = 0;
        body_roll_pid_reset();
    }
    else if(run_state == 0
            && func_abs(roll_angle) < BODY_TILT_RECOVER_DEG
            && func_abs(pitch_angle) < BODY_TILT_RECOVER_DEG)
    {
        sys_times = 0;
        run_state = 1;
    }

    if(run_state == 0)
    {
        return;
    }

    if(sys_times < BODY_PID_RAMP_CYCLES)        // 启动阶段逐步恢复 PID 参数
    {
        // 角度环 P 参数按 config.h 配置渐变
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p
            * (BODY_PID_RAMP_INITIAL_SCALE
               + (float)sys_times / (float)BODY_PID_RAMP_CYCLES * BODY_PID_RAMP_SCALE_RANGE);
        // 速度环 P 参数按 config.h 配置渐变
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p
            * (BODY_PID_RAMP_INITIAL_SCALE
               + (float)sys_times / (float)BODY_PID_RAMP_CYCLES * BODY_PID_RAMP_SCALE_RANGE);

        roll_balance_cascade.angle_cycle.i_value = 0; // 重置角度环积分值
        body_roll_pid_reset();
    }
    else                                        // 渐变结束后使用原始 PID 参数
    {
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p;   // 恢复角度环 P 参数
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p;   // 恢复速度环 P 参数
    }

    if(jump_flag)//跳跃时按配置比例降低平衡参数
    {
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p * BODY_JUMP_PID_SCALE;
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p * BODY_JUMP_PID_SCALE;

        roll_balance_cascade.angle_cycle.i_value = 0;     // 重置角速度环 PID 积分值
        body_roll_pid_reset();
    }
}

//--------------------------------------------------------------------------------
// 函数简介    车辆舵机控制
// 返回参数    void
// 使用示例    car_steer_control();
// 备注信息    控制跳跃、左右倾斜、腿部前后倾斜、自动复位等操作
//--------------------------------------------------------------------------------
void car_steer_control(void)
{
    int16 steer_location_offset[4] = {0};
    
    int16 steer_target_offset[4] = {0};
    
    static float steer_balance_angle_count = 0;
    
    static float steer_output_duty_filter = 0;
    
    int16 steer_output_duty = 0;
    
    float steer_balance_angle = 0;
    
    // 前后倾斜越大，越降低速度环到舵机的辅助量
    float pitch_offset = (STEER_PITCH_ATTENUATION_LIMIT_DEG
        - func_limit_ab(
            func_abs(
                (BODY_PITCH_TARGET_DEG - roll_balance_cascade.posture_value.mechanical_zero)
                - BODY_PITCH_ANGLE_FEEDBACK_SIGN * roll_balance_cascade.posture_value.pit),
            0.0f,
            STEER_PITCH_ATTENUATION_LIMIT_DEG))
        / STEER_PITCH_ATTENUATION_LIMIT_DEG;

    
    //将速度环串给舵机
    steer_output_duty = func_limit_ab(
        (int16)(roll_balance_cascade.speed_cycle.out / STEER_SPEED_OUTPUT_DIVISOR),
        -STEER_SPEED_OUTPUT_LIMIT,
        STEER_SPEED_OUTPUT_LIMIT) * STEER_SPEED_OUTPUT_GAIN;
    
    steer_output_duty = (int16)((float)steer_output_duty * pitch_offset);
    
    steer_output_duty_filter =
        (steer_output_duty_filter * STEER_FILTER_HISTORY_WEIGHT
         + (float)steer_output_duty * STEER_FILTER_INPUT_WEIGHT)
        / STEER_FILTER_DIVISOR;

    
    //将速度环串给舵机


    if(jump_flag == 0)
    {
        if(sys_times < STEER_ROLL_ENABLE_DELAY_CYCLES)
        {
            steer_balance_angle = 0;
            body_roll_pid_reset();
        }
        else
        {
            steer_balance_angle = func_limit_ab(
                pitch_balance_cascade.angular_speed_cycle.out,
                -STEER_ROLL_OUTPUT_LIMIT,
                STEER_ROLL_OUTPUT_LIMIT)
                * STEER_ROLL_OUTPUT_GAIN
                * STEER_ROLL_OUTPUT_SIGN;
        }
        steer_balance_angle_count = steer_balance_angle;
    }

    steer_location_offset[0] = (steer_1.now_location - steer_1.center_num) * steer_1.steer_dir;
    steer_location_offset[1] = (steer_2.now_location - steer_2.center_num) * steer_2.steer_dir;
    steer_location_offset[2] = (steer_3.now_location - steer_3.center_num) * steer_3.steer_dir;
    steer_location_offset[3] = (steer_4.now_location - steer_4.center_num) * steer_4.steer_dir;

    // 左腿为 steer_1 + steer_3，右腿为 steer_2 + steer_4。
    // 公共量控制前后摆动，横滚量以等大反向方式调节左右腿，避免改变平均腿高。
    steer_target_offset[0] = (int16)( steer_output_duty_filter - steer_balance_angle_count);
    steer_target_offset[1] = (int16)( steer_output_duty_filter + steer_balance_angle_count);
    steer_target_offset[2] = (int16)(-steer_output_duty_filter - steer_balance_angle_count);
    steer_target_offset[3] = (int16)(-steer_output_duty_filter + steer_balance_angle_count);

    if(run_state == 1)
    {
        if(jump_flag == 0)
        {
            steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -STEER_NORMAL_STEP_LIMIT, STEER_NORMAL_STEP_LIMIT));//步进控制
            steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -STEER_NORMAL_STEP_LIMIT, STEER_NORMAL_STEP_LIMIT));
            steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -STEER_NORMAL_STEP_LIMIT, STEER_NORMAL_STEP_LIMIT));
            steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -STEER_NORMAL_STEP_LIMIT, STEER_NORMAL_STEP_LIMIT));
        }
        else
        {
            jump_time ++;

            if(jump_time < JUMP_EXTEND_CYCLES)                                 // 起跳
            {
                jump_flag = 1;
                steer_duty_set(&steer_1, steer_1.center_num + JUMP_EXTEND_DUTY_OFFSET);
                steer_duty_set(&steer_2, steer_2.center_num - JUMP_EXTEND_DUTY_OFFSET);
                steer_duty_set(&steer_3, steer_3.center_num - JUMP_EXTEND_DUTY_OFFSET);
                steer_duty_set(&steer_4, steer_4.center_num + JUMP_EXTEND_DUTY_OFFSET);
            }
            else if(jump_time < (JUMP_EXTEND_CYCLES + JUMP_HOLD_CYCLES))       // 收腿
            {
                jump_flag = 2;
                steer_duty_set(&steer_1, steer_1.center_num);
                steer_duty_set(&steer_2, steer_2.center_num);
                steer_duty_set(&steer_3, steer_3.center_num);
                steer_duty_set(&steer_4, steer_4.center_num);
            }
            else if(jump_time < (JUMP_EXTEND_CYCLES + JUMP_HOLD_CYCLES + JUMP_PRELOAD_CYCLES)) // 预备缓冲
            {
                jump_flag = 3;
                steer_duty_set(&steer_1, steer_1.center_num + JUMP_PRELOAD_DUTY_OFFSET);
                steer_duty_set(&steer_2, steer_2.center_num - JUMP_PRELOAD_DUTY_OFFSET);
                steer_duty_set(&steer_3, steer_3.center_num - JUMP_PRELOAD_DUTY_OFFSET);
                steer_duty_set(&steer_4, steer_4.center_num + JUMP_PRELOAD_DUTY_OFFSET);
            }
            else if(jump_time < (JUMP_EXTEND_CYCLES + JUMP_HOLD_CYCLES + JUMP_PRELOAD_CYCLES + JUMP_EXECUTE_CYCLES)) // 执行缓冲
            {
                jump_flag = 4;
                steer_control(&steer_1, JUMP_EXECUTE_STEER_STEP);
                steer_control(&steer_2, JUMP_EXECUTE_STEER_STEP);
                steer_control(&steer_3, JUMP_EXECUTE_STEER_STEP);
                steer_control(&steer_4, JUMP_EXECUTE_STEER_STEP);
            }
            else
            {
                jump_flag = 0;
                jump_time = 0;
            }
        }
    }
    else
        {
            steer_control(&steer_1, func_limit_ab(steer_1.center_num - steer_1.now_location, -STEER_STOP_RETURN_STEP_LIMIT, STEER_STOP_RETURN_STEP_LIMIT) * steer_1.steer_dir);
            steer_control(&steer_2, func_limit_ab(steer_2.center_num - steer_2.now_location, -STEER_STOP_RETURN_STEP_LIMIT, STEER_STOP_RETURN_STEP_LIMIT) * steer_2.steer_dir);
            steer_control(&steer_3, func_limit_ab(steer_3.center_num - steer_3.now_location, -STEER_STOP_RETURN_STEP_LIMIT, STEER_STOP_RETURN_STEP_LIMIT) * steer_3.steer_dir);
            steer_control(&steer_4, func_limit_ab(steer_4.center_num - steer_4.now_location, -STEER_STOP_RETURN_STEP_LIMIT, STEER_STOP_RETURN_STEP_LIMIT) * steer_4.steer_dir);
        }
    

}

int32 car_distance=0;
int16 left_motor_duty,right_motor_duty=0;
//--------------------------------------------------------------------------------
// 函数简介    车辆电机占空比控制
// 返回参数    void
// 使用示例    car_motor_control();
// 备注信息    根据运行标志控制左右电机的占空比，通过限幅函数限制输出范围，最终设置电机驱动
//--------------------------------------------------------------------------------
void car_motor_control(void)
{
    car_distance += ((float)car_speed / 60.0f * BODY_WHEEL_DIAMETER_CM * PI * 0.001f);

    if(run_state)                                         // 当运行状态为 1 时，计算电机占空比
    {
        // 左电机占空比：取角速度环输出，限幅
        left_motor_duty = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -BODY_BALANCE_DUTY_MAX, BODY_BALANCE_DUTY_MAX);
        // 右电机占空比：取角速度环输出，限幅
        right_motor_duty = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -BODY_BALANCE_DUTY_MAX, BODY_BALANCE_DUTY_MAX);

        // 叠加Z轴陀螺仪数据，实现转向差速控制
        left_motor_duty = func_limit_ab(left_motor_duty + imu660ra_gyro_z / BODY_YAW_GYRO_DIVISOR, -BODY_TURN_DUTY_MAX, BODY_TURN_DUTY_MAX);
        right_motor_duty = func_limit_ab(right_motor_duty - imu660ra_gyro_z / BODY_YAW_GYRO_DIVISOR, -BODY_TURN_DUTY_MAX, BODY_TURN_DUTY_MAX);
    }
    else                                                  // 当运行状态为 0 时，电机关闭
    {  
        left_motor_duty = 0;                              // 左电机占空比设为 0
        right_motor_duty = 0;                             // 右电机占空比设为 0
    }

    body_motor_write_nonblocking(-left_motor_duty, -right_motor_duty); // 设置驱动的电机占空比（）
    
//        small_driver_set_duty((int16)roll_balance_cascade.angular_speed_cycle.out,-(int16)roll_balance_cascade.angular_speed_cycle.out); // 设置驱动的电机占空比（）

//    CYT2_D_motor_ctrl(-left_motor_duty,-right_motor_duty);
}

uint32 sys_times=0;
int STOP_FALG = BODY_STOP_FLAG_DEFAULT;

void control_background_task(void)
{
    uint8 should_run = 0U;
    uint32 interrupt_state = Cy_SysLib_EnterCriticalSection();

    if(control_background_pending != 0U)
    {
        control_background_pending = 0U;
        should_run = 1U;
    }
    Cy_SysLib_ExitCriticalSection(interrupt_state);

    if(should_run != 0U)
    {
        // Any tick arriving during this potentially long operation remains
        // pending for the next main-loop pass.
        Nag_System();
        control_publish_main_command();
    }
}

void pit_call_back(void)
{
    motor_speed_snapshot_struct speed_snapshot;
    uint8 motor_feedback_valid;

// static uint32 system_time_state[20] = {0};  

    sys_times ++;                                    // 系统计时自增
    control_uptime_ticks++;
    body_receive_main_command();
    motor_feedback_valid = small_driver_get_speed_snapshot(
        &speed_snapshot,
        control_uptime_ticks,
        BODY_MOTOR_FEEDBACK_TIMEOUT_CYCLES);
    if(motor_feedback_valid == 0U)
    {
        body_speed_pid_reset();
    }
    
//    for(int i = 0; i < 20; i ++)
//    {
//        system_time_state[i] = (sys_times % (i + 1)) == 0 ? 1 : system_time_state[i];
//    }
    
//    imu660ra_get_gyro();                             // 获取 IMU660RA 陀螺仪数据
//    imu660ra_get_acc();                              // 获取 IMU660RA 加速度计数据
//    quaternion_module_calculate(&roll_balance_cascade); // 计算四元数，更新姿态数据
    
    
//    imu660ra_get_gyro();                             // 获取 IMU660RA 陀螺仪数据
//    imu660ra_get_acc();                              // 获取 IMU660RA 加速度计数据
//    quaternion_module_calculate(&roll_balance_cascade); // 计算四元数，更新姿态数据
    
    
    imu660rb_get_gyro();                             // 获取 IMU660RA 陀螺仪数据
    imu660rb_get_acc();                              // 获取 IMU660RA 加速度计数据
    quaternion_module_calculate(&roll_balance_cascade); // 计算四元数，更新姿态数据

    car_state_calculate();

    if(run_state == 0)
    {
        rotation_stop();
    }

    if(run_state == 0 || sys_times <= BODY_CONTROL_STARTUP_DELAY_CYCLES)
    {
        car_steer_control();
        body_motor_write_nonblocking(0, 0);
        return;
    }

    if(sys_times > BODY_CONTROL_STARTUP_DELAY_CYCLES)
    {
          int16 turn_output;

          if(motor_feedback_valid != 0U)
          {
              rotation_run();
          }
          else
          {
              rotation_stop();
          }

          if(sys_times % BODY_ANGLE_LOOP_DIVIDER == 0)     // 角度环降频执行
          {
            
             CYT2_update_distance_from_speed(
                 speed_snapshot.left_speed,
                 speed_snapshot.right_speed);
            
             // Nag_System may erase/read/write Flash. Defer it to the main
             // loop so this 1 ms interrupt always has bounded execution time.
             control_background_pending = 1U;

            
            
              // 前后姿态角度环：输出目标俯仰角速度
              pid_control(
                  &roll_balance_cascade.angle_cycle,
                  BODY_PITCH_TARGET_DEG - roll_balance_cascade.posture_value.mechanical_zero,
                  BODY_PITCH_ANGLE_FEEDBACK_SIGN * roll_balance_cascade.posture_value.pit);
              
              // 横滚角度环：输出目标横滚角速度
              if(BODY_ROLL_CONTROL_ENABLE)
              {
                  pid_control(
                      &pitch_balance_cascade.angle_cycle,
                      BODY_ROLL_TARGET_DEG - pitch_balance_cascade.posture_value.mechanical_zero,
                      BODY_ROLL_ANGLE_FEEDBACK_SIGN * roll_balance_cascade.posture_value.rol);
              }
              else
              {
                  body_roll_pid_reset();
              }
             }

              // 前后角速度环
              pid_control(
                  &roll_balance_cascade.angular_speed_cycle,
                  roll_balance_cascade.angle_cycle.out,
                  BODY_PITCH_RATE_FEEDBACK_SIGN * imu660rb_gyro_y);

              // 横滚角速度环：输出左右腿差分 PWM
              if(BODY_ROLL_CONTROL_ENABLE)
              {
                  pid_control(
                      &pitch_balance_cascade.angular_speed_cycle,
                      pitch_balance_cascade.angle_cycle.out,
                      BODY_ROLL_RATE_FEEDBACK_SIGN * imu660rb_gyro_x);
              }
          
//              CYT2_D_motor_ctrl(-(int16)roll_balance_cascade.angular_speed_cycle.out,-(int16)roll_balance_cascade.angular_speed_cycle.out);
      //        
      //        car_state_calculate(); // 检测车辆状态
              car_steer_control();   // 车辆舵机控制
//              car_motor_control();   // 车辆电机控制
          
              
              
                  //速度环
          if(sys_times % BODY_SPEED_LOOP_DIVIDER == 0)     // 速度环降频执行
          {
              if(motor_feedback_valid != 0U)
              {
                  car_speed = (speed_snapshot.left_speed - speed_snapshot.right_speed) / 2;

                  pid_control(
                      &roll_balance_cascade.speed_cycle,
                      active_control_command.target_speed,
                      (float)car_speed);

                  if(active_control_command.track_enable != 0U)
                  {
                      pid_control(
                          &track_cascade.track_cycle,
                          active_control_command.navigation_output,
                          0.0f);
                  }
              }

              
          
          }
//          
          if(active_control_command.motor_enable != 0U
              && run_state == 1
              && motor_feedback_valid != 0U)
          {
             if(rotation_owns_output())
             {
                 // 原地旋转独占差速量；前后平衡公共输出仍然保留。
                 turn_output = rotation.turn_duty;
             }
             else
             {
                 turn_output = (int16)(active_control_command.navigation_output * BODY_TRACK_OUTPUT_GAIN);
             }
//             CYT2_D_motor_ctrl(-(int16)roll_balance_cascade.angular_speed_cycle.out+track_cascade.track_cycle.out,-(int16)roll_balance_cascade.angular_speed_cycle.out-track_cascade.track_cycle.out);
             body_motor_write_nonblocking(
                 -(int16)roll_balance_cascade.angular_speed_cycle.out + turn_output,
                 -(int16)roll_balance_cascade.angular_speed_cycle.out - turn_output);

          }
          else
          {
             rotation_stop();
             body_motor_write_nonblocking(0,0);

          }
          
          
    }

    
    


}
