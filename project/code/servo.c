#include "servo.h"
#include "zf_common_headfile.h"


//四路舵机驱动

steer_control_struct steer_2;
void steering_Init(void)

{
//  测试
  
  steer_2.pwm_pin           = STEER_2_PWM;      //引脚
  steer_2.control_frequency = STEER_2_FRE;      //频率
  steer_2.steer_dir         = STEER_2_DIR;      //方向
  steer_2.center_num        = STEER_2_CENTER;   //中心值pwm
  
  steer_2.now_location = steer_2.center_num;
   pwm_init(steer_2.pwm_pin, steer_2.control_frequency, steer_2.now_location);
}


