#ifndef _servo_h_
#define _servo_h_

#include "zf_common_typedef.h"
#include "zf_common_headfile.h"

typedef int pwm_channel_enum;

#define TCPWM_CH10_P05_0  (0)
#define TCPWM_CH10_P05_1  (1)
#define TCPWM_CH10_P05_2  (2)
#define TCPWM_CH10_P05_3  (3)

#define STEER_1_PWM     (TCPWM_CH10_P05_0)
#define STEER_1_FRE     (50)
#define STEER_1_DIR     (1)
#define STEER_1_CENTER  (1000)
#define STEER_2_PWM     (TCPWM_CH10_P05_1)
#define STEER_2_FRE     (30)
#define STEER_2_DIR     (1)
#define STEER_2_CENTER  (1000)
#define STEER_3_PWM     (TCPWM_CH10_P05_2)
#define STEER_3_FRE     (50)
#define STEER_3_DIR     (1)
#define STEER_3_CENTER  (1000)
#define STEER_4_PWM     (TCPWM_CH10_P05_3)
#define STEER_4_FRE     (50)
#define STEER_4_DIR     (1)
#define STEER_4_CENTER  (1000)

typedef struct
{
    pwm_channel_enum  pwm_pin;
    int16             control_frequency;
    int16             steer_dir;
    int16             center_num;
    int16             now_location;
} steer_control_struct;

extern steer_control_struct steer_1;
extern steer_control_struct steer_2;
extern steer_control_struct steer_3;
extern steer_control_struct steer_4;

void steering_Init(void);

#endif
