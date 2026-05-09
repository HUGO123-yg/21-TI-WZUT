#ifndef _servo_h_
#define _servo_h_

//四路舵机驱动

#include "zf_common_typedef.h"
#include "zf_common_headfile.h"



#define STEER_1_PWM     (TCPWM_CH10_P05_0)   //控制引脚   ATOM0_CH3_P21_5
#define STEER_1_FRE     (50            )   //频率 
#define STEER_1_DIR     (1              )   //旋转方向
#define STEER_1_CENTER  (1000   )   //中心值

#define STEER_2_PWM     (TCPWM_CH10_P05_1)   //控制引脚   ATOM0_CH3_P21_5
#define STEER_2_FRE     (30            )   //频率 
#define STEER_2_DIR     (1              )   //旋转方向
#define STEER_2_CENTER  (1000   )   //中心值




#define STEER_3_PWM     (TCPWM_CH10_P05_2)   //控制引脚   ATOM0_CH3_P21_5
#define STEER_3_FRE     (50           )   //频率 
#define STEER_3_DIR     (1              )   //旋转方向
#define STEER_3_CENTER  (1000   )   //中心值




#define STEER_4_PWM     (TCPWM_CH10_P05_3)   //控制引脚   ATOM0_CH3_P21_5
#define STEER_4_FRE     (50            )   //频率 
#define STEER_4_DIR     (1              )   //旋转方向
#define STEER_4_CENTER  (1000   )   //中心值

typedef struct
{
  pwm_channel_enum  pwm_pin;            //控制引脚
  int16             control_frequency;  //频率
  int16             steer_dir;          //舵机旋转方向
  int16             center_num;         //舵机中心值
  
  int16             now_location;       //舵机当前输出位置
}steer_control_struct;


extern steer_control_struct steer_1;
extern steer_control_struct steer_2;
extern steer_control_struct steer_3;
extern steer_control_struct steer_4;

void steering_Init(void);
#endif
