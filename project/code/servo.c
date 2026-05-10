#include "servo.h"
#include "zf_common_headfile.h"


//��·�������

steer_control_struct steer_1;
steer_control_struct steer_2;
steer_control_struct steer_3;
steer_control_struct steer_4;
void steering_Init(void)
{
    // STEER_1 — 左腿上舵机
    steer_1.pwm_pin           = STEER_1_PWM;
    steer_1.control_frequency = STEER_1_FRE;
    steer_1.steer_dir         = STEER_1_DIR;
    steer_1.center_num        = STEER_1_CENTER;
    steer_1.now_location      = steer_1.center_num;
    pwm_init(steer_1.pwm_pin, steer_1.control_frequency, steer_1.now_location);

    // STEER_2 — 左腿下舵机
    steer_2.pwm_pin           = STEER_2_PWM;
    steer_2.control_frequency = STEER_2_FRE;
    steer_2.steer_dir         = STEER_2_DIR;
    steer_2.center_num        = STEER_2_CENTER;
    steer_2.now_location      = steer_2.center_num;
    pwm_init(steer_2.pwm_pin, steer_2.control_frequency, steer_2.now_location);

    // STEER_3 — 右腿上舵机
    steer_3.pwm_pin           = STEER_3_PWM;
    steer_3.control_frequency = STEER_3_FRE;
    steer_3.steer_dir         = STEER_3_DIR;
    steer_3.center_num        = STEER_3_CENTER;
    steer_3.now_location      = steer_3.center_num;
    pwm_init(steer_3.pwm_pin, steer_3.control_frequency, steer_3.now_location);

    // STEER_4 — 右腿下舵机
    steer_4.pwm_pin           = STEER_4_PWM;
    steer_4.control_frequency = STEER_4_FRE;
    steer_4.steer_dir         = STEER_4_DIR;
    steer_4.center_num        = STEER_4_CENTER;
    steer_4.now_location      = steer_4.center_num;
    pwm_init(steer_4.pwm_pin, steer_4.control_frequency, steer_4.now_location);
}


