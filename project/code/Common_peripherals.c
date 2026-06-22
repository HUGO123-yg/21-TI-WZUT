#include "zf_common_headfile.h"

//----------------------------------------------------蜂鸣器
void BUZZER_init(void)
{
  gpio_init(BUZZER_PIN,GPO,0,GPO_PUSH_PULL);
}

void BUZZER_check(int TIME)
{
  gpio_set_level(BUZZER_PIN,1);
  system_delay_ms(TIME);
  gpio_set_level(BUZZER_PIN,0);
}


//----------------------------------------------------按键

uint8 key1_state = 1;                                                               // 按键动作状态
uint8 key2_state = 1;                                                               // 按键动作状态
uint8 key3_state = 1;                                                               // 按键动作状态
uint8 key4_state = 1;                                                               // 按键动作状态

uint8 key1_state_last = 0;                                                          // 上一次按键动作状态
uint8 key2_state_last = 0;                                                          // 上一次按键动作状态
uint8 key3_state_last = 0;                                                          // 上一次按键动作状态
uint8 key4_state_last = 0;                                                          // 上一次按键动作状态

volatile uint8 key1_flag;
volatile uint8 key2_flag;
volatile uint8 key3_flag;
volatile uint8 key4_flag;

int Key_close_flag=0;//外部按键扫描隔绝标志位

void Key_init(void)//按键与LED初始化
{


       gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
       gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
       gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
       gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入

       gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_FLOATING_IN);    // 初始化 SWITCH1 输入 默认高电平 浮空输入
       gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_FLOATING_IN);    // 初始化 SWITCH2 输入 默认高电平 浮空输入

}

void key_scan(void)//按键扫描
    {
        //使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费

        //保存按键状态
        key1_state_last = key1_state;
        key2_state_last = key2_state;
        key3_state_last = key3_state;
        key4_state_last = key4_state;

        //读取当前按键状态
        key1_state = gpio_get_level(KEY1);
        key2_state = gpio_get_level(KEY2);
        key3_state = gpio_get_level(KEY3);
        key4_state = gpio_get_level(KEY4);


        //检测到按键按下之后  并放开置位标志位
        if(key1_state && !key1_state_last)   {key1_flag = 1;}
        if(key2_state && !key2_state_last)   {key2_flag = 1;}
        if(key3_state && !key3_state_last)   {key3_flag = 1;}
        if(key4_state && !key4_state_last)   {key4_flag = 1;}

        //标志位置位之后，可以使用标志位执行自己想要做的事件

//        system_delay_ms(10);//延时，按键程序应该保证调用时间不小于10ms

    }


void key1_clear(void)
{
  key1_flag=0;
  BUZZER_check(50);

}

void key2_clear(void)
{
  key2_flag=0;
  BUZZER_check(50);

}

void key3_clear(void)
{
  key3_flag=0;
  BUZZER_check(50);

}

void key4_clear(void)
{
  key4_flag=0;
  BUZZER_check(50);

}

//----------------------------------------------------无刷驱动

int16 car_speed;        //左右平均速度
int16_t Difspeed;       //差速        




void CYT2_D_motor_ctrl(int16 L_SPEED,int16_t R_SPEED)
{
  L_SPEED=L_SPEED>M_MAX?M_MAX:(L_SPEED<M_MIN)?M_MIN:L_SPEED;
  R_SPEED=R_SPEED>M_MAX?M_MAX:(R_SPEED<M_MIN)?M_MIN:R_SPEED;
  small_driver_set_duty( -L_SPEED, R_SPEED);
}


//----------------------------------------------------编码器

Car_param_t Car = {0, 0, 0, 0, 0, 0}; // 速度结构体

void CYT2_get_speed(void)
{
    Car.speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2; // 车体速度

    Car.speed_L = motor_value.receive_left_speed_data; // 左轮速度
    // printf("Car.speed_L=%f\r\n", Car.speed_L);
    Car.speed_R = -motor_value.receive_right_speed_data; // 右轮速度
    // printf("Car.speed_R=%f\r\n", Car.speed_R);
    // printf("Car.speed=%f\r\n", Car.speed);
}


// 编码器测距
void CYT2_get_distance(void)
{
    CYT2_get_speed();
    Car.mileage += ((float)Car.speed / 60.0f * wheel_diameter * PI * 0.005f);    // 车体里程  单位厘米
    Car.mileage_L = ((float)Car.speed_L / 60.0f * wheel_diameter * PI * 0.005f); // 左轮里程  单位厘米
    Car.mileage_R = ((float)Car.speed_R / 60.0f * wheel_diameter * PI * 0.005f); // 右轮里程  单位厘米
    // printf("Car.mileage=%f\r\n", Car.mileage);
}


//----------------------------------------------------轮腿舵机

// 定义 4 个舵机控制结构体实例，分别对应 4 路舵机
steer_control_struct steer_1;
steer_control_struct steer_2;
steer_control_struct steer_3;
steer_control_struct steer_4;

//================================================================================
// 函数简介    舵机控制初始化函数
// 返回参数    void
// 使用示例    steer_control_init();
// 备注信息    初始化 4 路舵机的 PWM 引脚、控制频率、转向方向、中心值等参数，设置初始位置为中心值并初始化 PWM，默认使能所有舵机
//================================================================================
void steer_control_init(void)
{
    steer_1.pwm_pin          = STEER_1_PWM;        // 舵机 1 PWM 引脚配置
    steer_1.control_frequency = STEER_1_FRE;       // 舵机 1 控制频率配置
    steer_1.steer_dir        = STEER_1_DIR;        // 舵机 1 转动方向配置
    steer_1.center_num       = STEER_1_CENTER;     // 舵机 1 中心位置（初始位置）配置

    steer_2.pwm_pin          = STEER_2_PWM;        // 舵机 2 PWM 引脚配置
    steer_2.control_frequency = STEER_2_FRE;       // 舵机 2 控制频率配置
    steer_2.steer_dir        = STEER_2_DIR;        // 舵机 2 转动方向配置
    steer_2.center_num       = STEER_2_CENTER;     // 舵机 2 中心位置（初始位置）配置

    steer_3.pwm_pin          = STEER_3_PWM;        // 舵机 3 PWM 引脚配置
    steer_3.control_frequency = STEER_3_FRE;       // 舵机 3 控制频率配置
    steer_3.steer_dir        = STEER_3_DIR;        // 舵机 3 转动方向配置
    steer_3.center_num       = STEER_3_CENTER;     // 舵机 3 中心位置（初始位置）配置

    steer_4.pwm_pin          = STEER_4_PWM;        // 舵机 4 PWM 引脚配置
    steer_4.control_frequency = STEER_4_FRE;       // 舵机 4 控制频率配置
    steer_4.steer_dir        = STEER_4_DIR;        // 舵机 4 转动方向配置
    steer_4.center_num       = STEER_4_CENTER;     // 舵机 4 中心位置（初始位置）配置

    steer_1.now_location     = steer_1.center_num; // 舵机 1 当前位置初始化为中心位置
    steer_2.now_location     = steer_2.center_num; // 舵机 2 当前位置初始化为中心位置
    steer_3.now_location     = steer_3.center_num; // 舵机 3 当前位置初始化为中心位置
    steer_4.now_location     = steer_4.center_num; // 舵机 4 当前位置初始化为中心位置

    // 初始化 4 路舵机的 PWM 输出，频率为配置值，初始占空比为中心位置值
    pwm_init(steer_1.pwm_pin, steer_1.control_frequency, steer_1.now_location);
    pwm_init(steer_2.pwm_pin, steer_2.control_frequency, steer_2.now_location);
    pwm_init(steer_3.pwm_pin, steer_3.control_frequency, steer_3.now_location);
    pwm_init(steer_4.pwm_pin, steer_4.control_frequency, steer_4.now_location);

    steer_1.steer_state     = 1;                  // 舵机 1 使能（1 为使能，0 为禁用）
    steer_2.steer_state     = 1;                  // 舵机 2 使能（1 为使能，0 为禁用）
    steer_3.steer_state     = 1;                  // 舵机 3 使能（1 为使能，0 为禁用）
    steer_4.steer_state     = 1;                  // 舵机 4 使能（1 为使能，0 为禁用）
}

//--------------------------------------------------------------------------------
// 函数简介    舵机占空比直接设置函数
// 返回参数    void
// 使用示例    steer_duty_set(&steer_1, 4500);
// 备注信息    直接设置舵机的 PWM 占空比，占空比范围被限制在 -10000 ~ 10000 之间，仅在舵机使能状态下生效
//--------------------------------------------------------------------------------
void steer_duty_set(steer_control_struct *control_data, int16 duty)
{
    if(control_data->steer_state)                  // 判断舵机是否处于使能状态
    {
        // PWM duty is unsigned in the low-level driver; clamp before sending.
        control_data->now_location = func_limit_ab(duty, 0, 10000);

        pwm_set_duty(control_data->pwm_pin, control_data->now_location);
    }
}

//--------------------------------------------------------------------------------
// 函数简介    舵机相对位置控制函数
// 返回参数    void
// 使用示例    steer_control(&steer_1, 100);
// 备注信息    根据偏移量控制舵机相对当前位置转动，偏移量正负由转向方向参数 steer_dir 决定，最终位置限制在 -10000 ~ 10000
//--------------------------------------------------------------------------------
void steer_control(steer_control_struct *control_data, int16 move_num)
{
    if(control_data->steer_state)                          // 判断舵机是否处于使能状态
    {
        // 根据转向方向计算新位置：steer_dir 为 1 时正向偏移，为 -1 时反向偏移
        control_data->now_location = control_data->now_location + (control_data->steer_dir == 1 ? move_num : -move_num);

        // PWM duty is unsigned in the low-level driver; clamp before sending.
        control_data->now_location = func_limit_ab(control_data->now_location, 0, 10000);

        pwm_set_duty(control_data->pwm_pin, control_data->now_location);
    }
}

//--------------------------------------------------------------------------------
// 函数简介    舵机禁用函数
// 返回参数    void
// 使用示例    steer_disable(&steer_1);
// 备注信息    禁用指定舵机，设置舵机状态为禁用，并将 PWM 占空比设置为 0
//--------------------------------------------------------------------------------
void steer_disable(steer_control_struct *control_data)
{
    control_data->steer_state = 0;                  // 将舵机状态设置为禁用（0 为禁用）
    pwm_set_duty(control_data->pwm_pin, 0);         // PWM 占空比设置为 0，舵机停止工作
}

//--------------------------------------------------------------------------------
// 函数简介    舵机使能函数
// 返回参数    void
// 使用示例    steer_enable(&steer_1);
// 备注信息    使能指定舵机，设置舵机状态为使能，并恢复到禁用前的最后位置
//--------------------------------------------------------------------------------
void steer_enable(steer_control_struct *control_data)
{
    control_data->steer_state = 1;                 // 将舵机状态设置为使能（1 为使能）
    pwm_set_duty(control_data->pwm_pin, control_data->now_location); // 恢复 PWM 占空比到最后位置
}

