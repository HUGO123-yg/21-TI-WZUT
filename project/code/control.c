#include "zf_common_headfile.h"
#include <math.h>

// 跨上下文共享变量 (volatile: IPC callback 写入, ISR 读取)
volatile float image_error = 0.0f;
volatile float v_hat       = 0.0f;
volatile float x_hat       = 0.0f;

//LQR���������K, �޸Ĳ���ʹ��matlab
const float LQR_K[8]={
        -0.0281 ,  -0.8067 ,  -1.4867 ,  -0.1745,
        -0.0281 ,  -0.8067 ,  -1.4867 ,  -0.1745

//        -0.0380 ,  -0.8670 ,  -1.6225  , -0.1791,
//        -0.0380 ,  -0.8670  , -1.6225  , -0.1791
};

//������Ծ����
const jump_control_struct jump_control_config[] =
{
        {0, 5, jump_set_step, "����"},
        {5, 10, jump_set_step, "����"},
        {10, 15, jump_set_step, "����"},
        {15, 25, jump_set_step, "����"},
};
const uint8 jump_step_num = sizeof(jump_control_config) / sizeof(jump_control_struct);


//��ˢ���������, ��ȷ�������޸�
const float Lmoto_K = 4980;
const float Rmoto_K = 4980;

//PID��ʼ��
pid_t  leg_hight, turn_angle, turn_gyro, gyro, angle, speed, turn;

float angle_kd = 85.5;      //�ǶȻ�kd
float pitch_mid = 2.275;    //pitch��е��ֵ
float roll_mid = -1.84;     //roll��е��ֵ

//��������PID����������
float dt_pid_gyro = 0.002f;
float dt_pid_angle = 0.01f;
float dt_pid_speed = 0.02f;
float dt_pid_turn = 0.01f;
float dt_leg = 0.025f;
float dt_pid_turn_angle = 0.003f;
float dt_pid_turn_gyro = 0.001f;

//��ʼ�ȸ�
float leg_long = 5.5f;
//float leg_high_integral = 0;

//��Ծ��־λ
float set_speed = 0;
uint8 jump_flag = 0;
uint8 speed_flag = 0;

//ת�򻷲���
float turn_out = 0;
float KP = 35.24f;//25.24f;
float KPP = 0.61f;//0.4;
float KD = 0.75f;
float KDD = 0.49f;

/*-------------------------------------------------------------------------------------------------------------------
// �������     PID���Ƴ�ʼ��
// ����˵��     null
// ���ز���     null
// ʹ��ʾ��     pid_ctrl_Init();
// ��ע��Ϣ     �ܳ�ʼ������
-------------------------------------------------------------------------------------------------------------------*/
void pid_ctrl_Init(void)
{
    //pid_init(&turn, 1.0087, 15, 0, 0.01, 0, 0, FALSE, 5000, Position_pid);
    pid_init(&leg_hight, 0.02, 0, 0, 0.025, 0, 0, FALSE, 10, Position_pid);
    pid_init(&turn_angle, 2.045, 0, 0.15, 0.003, 0, 0, FALSE, 10000, Position_pid);
    pid_init(&turn_gyro, 2.087, 15, 0, 0.001, 0, 0, FALSE, 10000, Position_pid);
    //pid_init(&turn, 1.87, 19, 0, 0.01, 0, 0, FALSE, 5000, Position_pid);
    //pid_init(&gyro, 0.7423888, 9.1845288, 0, 0.002, 0, 0, FALSE, 10000, Position_pid);
    //pid_init(&angle, 308.195, 0, 0, 0.01, 0, 0, FALSE, 10000, Position_pid);
    //pid_init(&speed, 0.01, 0.0000667, 0, 0.02, 0, 0, FALSE, 10000, Position_pid);


    pid_set_target(&leg_hight, roll_mid);
    //pid_set_target(&speed, 500);
}



/*-------------------------------------------------------------------------------------------------------------------
// 函数名称     turn_control
// 功能说明     转向环控制 — 读取全局 volatile float image_error (IPC callback 写入)
// 输入参数     void (使用全局变量 image_error)
// 返回参数     float turn_out     转向环输出值
// 使用示例     float turn_out = turn_control();
// 备注信息     isr 中断调用, 无有效视觉数据 (|image_error| < 0.001) 时直行 (return 0.0f)
-------------------------------------------------------------------------------------------------------------------*/
float turn_control(void)
{
    // 零负载回退: 无有效视觉数据时直行
    if (fabsf(image_error) < 0.001f)
    {
        return 0.0f;
    }

    return turn_gyro.out;
}
}


/*-------------------------------------------------------------------------------------------------------------------
// �������     ת�򻷿���
// ����˵��     image_error    ͼ�����
// ���ز���     null
// ʹ��ʾ��     turn_control(93 - mid_point);
// ��ע��Ϣ     isr�жϵ���
-------------------------------------------------------------------------------------------------------------------*/
float turn_control(float image_error)
{



    return turn_gyro.out;
}



/*-------------------------------------------------------------------------------------------------------------------
// �������     PID����ƽ�����ʻ
// ����˵��     null
// ���ز���     null
// ʹ��ʾ��     pid_ctrl_Run();
// ��ע��Ϣ     isr�жϵ���
-------------------------------------------------------------------------------------------------------------------*/
void pid_ctrl_Run(void)
{
    static uint16 pid_time_gyro = 0;
    static uint16 pid_time_angle = 0;
    static uint16 pid_time_speed = 0;
    static uint16 pid_time_turn = 0;
    static uint8 timer_flag = 0;
    static float Angle_Out = 0;
    imu660ra_get_gyro();

    if(0 == timer_flag)     //�ٶȻ�
    {
        pid_get_observation(&speed, -motor_value.receive_left_speed_data + motor_value.receive_right_speed_data);
        get_timer(TOM0_CH1, &pid_time_speed, &dt_pid_speed);
        pid_set_dt(&speed, dt_pid_speed);
        pid_run(&speed);
    }

    if(0 == timer_flag % 5)     //�ǶȻ�
    {
        pid_set_target(&angle, pitch_mid - speed.out);
        pid_get_observation(&angle, euler_angle.pitch);
        get_timer(TOM0_CH2, &pid_time_angle, &dt_pid_angle);
        pid_set_dt(&angle, dt_pid_angle);
        pid_run(&angle);
        Angle_Out = angle.out + angle_kd * imu660ra_gyro_y * dt_pid_angle;
    }

    //���ٶȻ�
    pid_set_target(&gyro, Angle_Out);
    pid_get_observation(&gyro, imu660ra_gyro_y);
    get_timer(TOM0_CH3, &pid_time_gyro, &dt_pid_gyro);
    pid_set_dt(&gyro, dt_pid_gyro);
    pid_run(&gyro);

    //ת��
    pid_set_target(&turn, mid_point);
    pid_get_observation(&turn, imu660ra_gyro_transition(imu660ra_gyro_z));
    get_timer(TOM0_CH4, &pid_time_turn, &dt_pid_turn);
    pid_set_dt(&turn, dt_pid_turn);
    pid_run(&turn);

    foc_set_duty((int16)-(gyro.out + turn.out), (int16)(gyro.out - turn.out));
    timer_flag = (timer_flag + 1) % 20;
}



/*-------------------------------------------------------------------------------------------------------------------
// �������     �����ȸ�
// ����˵��     null
// ���ز���     null
// ʹ��ʾ��     leg_control();
// ��ע��Ϣ     isr�жϵ���, ��Ե�����ʹ��
-------------------------------------------------------------------------------------------------------------------*/
void leg_control(void)
{
   // static float leg_long = 5.5f;//4.4f
    static uint16 leg_time = 0;
   // static uint16 flag = 40 * 2;    //2s
    static float leg_high_integral = 0;

    get_timer(TOM0_CH0, &leg_time, &dt_leg);

    pid_get_observation(&leg_hight, euler_angle.roll);
    pid_set_dt(&leg_hight, dt_leg);
    pid_run(&leg_hight);

//    if(flag && ABS(pitch_mid - euler_angle.pitch) < 5.0f && ABS(roll_mid - euler_angle.roll) < 1.5f)
//    {
//        flag--;
//        leg_high_integral = 0;
//    }
//    else
//    {
//        if(!flag)leg_high_integral += leg_hight.out;
//    }

    leg_high_integral += leg_hight.out;
    left_leg_control(leg_long - leg_high_integral, 0);
    right_leg_control(leg_long + leg_high_integral, 0);

    //������Ծ
//    if(1 == 1)
//    {
//        jump_flag = 1;
//    }
}



/*-------------------------------------------------------------------------------------------------------------------
// �������     ִ����Ծ����
// ����˵��     step_num    ִ��Ŀ��
// ���ز���     null
// ʹ��ʾ��     jump_set_step(step_num);
// ��ע��Ϣ     jump_control�����е���, ����ϰ�ʹ��
-------------------------------------------------------------------------------------------------------------------*/
void jump_set_step(int step_num)
{
    switch(step_num)
    {
        case 0:
        {
            leg_long = 12.5;
        }break;

        case 1:
        {
            leg_long = 5.5;
        }break;

        case 2:
        {
            leg_long = 7.5;
        }break;

        case 3:
        {
            leg_long = 5.5;
        }break;

        default:break;
    }
}



/*-------------------------------------------------------------------------------------------------------------------
// �������     ������Ծ
// ����˵��     null
// ���ز���     null
// ʹ��ʾ��     jump_control();
// ��ע��Ϣ     isr�жϵ���, ����ϰ�ʹ��
-------------------------------------------------------------------------------------------------------------------*/
void jump_control(void)
{
    static int jump_time = 0;
    if(jump_flag == 1)
    {
        jump_time++;

        if(jump_time < jump_control_config[jump_step_num - 1].max)
        {
            for(int i = 0; i < jump_step_num; i++)
            {
                if(jump_time >= jump_control_config[i].min && jump_time <= jump_control_config[i].max)
                {
                    jump_control_config[i].handler(i);
                    //ips200_show_int(0,0,i,3);
                    break;
                }
            }
        }
        else
        {
            jump_flag = 0;
            jump_time = 0;
        }
    }
}



/*-------------------------------------------------------------------------------------------------------------------
// �������     ��������
// ����˵��     *input_L    ����
              *input_R    �ҵ��
// ���ز���     null
// ʹ��ʾ��     dead_compensate(&input_L, &input_R);
// ��ע��Ϣ     LQR�����е���
-------------------------------------------------------------------------------------------------------------------*/
void dead_compensate(int16 *input_L, int16 *input_R)
{
    if(*input_L > 0)
    {
        *input_L = clip2(*input_L + L_dead_zone_correct, 10000);
    }
    else if(*input_L < 0)
    {
        *input_L = clip2(*input_L + L_dead_zone_negative, 10000);
    }
    else
    {
        *input_L = 0;
    }
    if(*input_R > 0)
    {
        *input_R = clip2(*input_R + R_dead_zone_correct, 10000);
    }
    else if(*input_R < 0)
    {
        *input_R = clip2(*input_R + R_dead_zone_negative, 10000);
    }
    else
    {
        *input_R = 0;
    }
}



/*-------------------------------------------------------------------------------------------------------------------
// �������     ��������
// ����˵��     p       �ȸ�
              angle   �Ƕ�
// ���ز���     null
// ʹ��ʾ��     left_leg_control(5.5, 0);
// ��ע��Ϣ     �����ȸߺ�������
-------------------------------------------------------------------------------------------------------------------*/
void left_leg_control(float p, float angle)
{
    int16 pwm_ph1, pwm_ph4;

    servo_control_table(p, -angle, &pwm_ph1, &pwm_ph4);

    if(10000 == pwm_ph4 || 10000 == pwm_ph1)
    {
        ASSERT(10000 == pwm_ph4 || 10000 == pwm_ph1);
        return;
    }

    pwm_set_duty(SERVO_1, SERVO1_MID + pwm_ph4);
    pwm_set_duty(SERVO_2, SERVO2_MID - pwm_ph1);
}



/*-------------------------------------------------------------------------------------------------------------------
// �������     ��������
// ����˵��     p       �ȸ�
              angle   �Ƕ�
// ���ز���     null
// ʹ��ʾ��     right_leg_control(5.5, 0);
// ��ע��Ϣ     �����ȸߺ�������
-------------------------------------------------------------------------------------------------------------------*/
void right_leg_control(float p, float angle)
{
    int16 pwm_ph1, pwm_ph4;

    servo_control_table(p, -angle, &pwm_ph1, &pwm_ph4);

    if(10000 == pwm_ph4 || 10000 == pwm_ph1)
    {
        ASSERT(10000 == pwm_ph4 || 10000 == pwm_ph1);
        return;
    }

    pwm_set_duty(SERVO_3, SERVO3_MID - pwm_ph4);
    pwm_set_duty(SERVO_4, SERVO4_MID + pwm_ph1);
}
