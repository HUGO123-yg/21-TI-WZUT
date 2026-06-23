#define BUZZER_PIN    P19_4

#define KEY1                    (P20_3)
#define KEY2                    (P20_2)
#define KEY3                    (P20_1)
#define KEY4                    (P20_0)

#define SWITCH2                 (P21_5)
#define SWITCH1                 (P21_6)

////IPS114宏定义
//#define ips_show_string                  ips114_show_string
//#define ips_show_int                     ips114_show_int
//#define ips_show_uint                    ips114_show_uint
//#define ips_show_float                   ips114_show_float
//#define ips_show_chinese                 ips114_show_chinese
//#define ips_clear                        ips114_clear
//#define ips_init                         ips114_init
//#define ips_show_rgb565_image            ips114_show_rgb565_image

//IPS200宏定义
#define ips_show_string                  ips200_show_string
#define ips_show_int                     ips200_show_int
#define ips_show_uint                    ips200_show_uint
#define ips_show_float                   ips200_show_float
#define ips_show_chinese                 ips200_show_chinese
#define ips_clear                        ips200_clear
#define ips_init                         ips200_init
#define ips_show_rgb565_image            ips200_show_rgb565_image

#define M_MAX  3000
#define M_MIN  -3000


#define STEER_1_PWM      (TCPWM_CH10_P05_1)   // 舵机控制引脚  注：左上舵机  
#define STEER_1_FRE      (300)                // 舵机控制频率
#define STEER_1_DIR      (1)                  // 舵机旋转方向(车体升高方向)
#define STEER_1_CENTER   (4400)               // 舵机中心值(初始保持位置，大腿水平)

#define STEER_2_PWM      (TCPWM_CH12_P05_3)   // 舵机控制引脚  注：右上舵机
#define STEER_2_FRE      (300)                // 舵机控制频率
#define STEER_2_DIR      (-1)                 // 舵机旋转方向(车体升高方向)
#define STEER_2_CENTER   (4400)               // 舵机中心值(初始保持位置，大腿水平)//4700

#define STEER_3_PWM      (TCPWM_CH09_P05_0)   // 舵机控制引脚  注：左下舵机
#define STEER_3_FRE      (300)                // 舵机控制频率
#define STEER_3_DIR      (-1)                 // 舵机旋转方向(车体升高方向)
#define STEER_3_CENTER   (4700)               // 舵机中心值(初始保持位置，大腿水平)

#define STEER_4_PWM      (TCPWM_CH11_P05_2)   // 舵机控制引脚  注：右下舵机
#define STEER_4_FRE      (300)                // 舵机控制频率
#define STEER_4_DIR      (1)                  // 舵机旋转方向(车体升高方向)
#define STEER_4_CENTER   (4200)               // 舵机中心值(初始保持位置，大腿水平)

#define STEER_1_DEFAULT_OFFSET  (900)
#define STEER_2_DEFAULT_OFFSET  (900)
#define STEER_3_DEFAULT_OFFSET  (900)
#define STEER_4_DEFAULT_OFFSET  (900)


typedef struct
{
    pwm_channel_enum        pwm_pin;                // PWM 通道引脚
    int16                   control_frequency;      // 控制频率（Hz）
    int16                   steer_dir;              // 转动方向（1 正方向 / -1 反方向）
    int16                   center_num;              // 中心位置值（初始位置）

    int16                   steer_state;             // 舵机当前状态（0 禁用 / 1 使能）
    int16                   now_location;            // 舵机当前位置（PWM 占空比值）
} steer_control_struct;


// ******************车体速度与里程计算******************
#define wheel_diameter (6.4f) // 轮子直径 单位CM

typedef struct
{
    float speed_L;   // 左轮速度
    float speed_R;   // 右轮速度
    float speed;     // 车体速度
    float mileage_L; // 左轮里程
    float mileage_R; // 右轮里程
    float mileage;   // 车体里程
} Car_param_t;

extern Car_param_t Car;

extern volatile uint8 key1_flag;
extern volatile uint8 key2_flag;
extern volatile uint8 key3_flag;
extern volatile uint8 key4_flag;


extern int16 car_speed;
extern int16_t Difspeed; 



extern steer_control_struct  steer_1;
extern steer_control_struct  steer_2;
extern steer_control_struct  steer_3;
extern steer_control_struct  steer_4;


void BUZZER_init(void);
void BUZZER_check(int TIME);

void Key_init(void);
void key_scan(void);
void key1_clear(void);
void key2_clear(void);
void key3_clear(void);
void key4_clear(void);

void CYT2_D_motor_ctrl(int16 L_SPEED,int16_t R_SPEED);

void CYT2_get_speed(void);
void CYT2_get_distance(void);

void steer_control_init(void);
void steer_control(steer_control_struct *control_data, int16 move_num);
void steer_duty_set(steer_control_struct *control_data, int16 duty);