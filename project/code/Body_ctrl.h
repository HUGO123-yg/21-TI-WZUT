
//****************************************************************************
// 文件名称    Body_ctrl.h
// 功能描述    车体控制模块对外接口头文件
//             声明 PIT 主控制循环、全局速度/状态/电机占空比变量，供 Menu.c、
//             Flash.c 等模块引用。
// 注意事项    1. 所有变量定义在 Body_ctrl.c 中，此处仅作 extern 声明；
//             2. STOP_FALG 为历史拼写错误（应为 STOP_FLAG），保持原名以兼容既有调用；
//             3. pit_call_back() 由 PIT 定时中断自动调用，约 1kHz。
//****************************************************************************

extern float  target_speed;              // 目标速度，由菜单/按键/导航模块设置，供速度环 PID 使用（单位与 car_speed 一致，当前为 RPM）
extern uint32 sys_times;                 // PIT 中断累计周期计数，系统时基（每周期约 1ms）
extern int16 left_motor_duty;            // 左电机输出占空比（当前由 car_motor_control 使用，pit_call_back 中未直接使用）
extern int16 right_motor_duty;           // 右电机输出占空比（当前由 car_motor_control 使用，pit_call_back 中未直接使用）
extern int STOP_FALG;                    // 总电机输出使能标志：1=允许电机输出；0=强制停车（历史拼写，保持原名）
extern uint8 system_armed;               // 菜单启动使能标志：0=上电默认静止；1=进入运行功能后允许电机输出
extern uint8 bridge_test_active;         // 单边桥测试使能：仅菜单测试页置 1，避免桥控影响其他运行功能
extern int run_state;                    // 车体运行状态：1=正常；0=倾角过大/异常保护停机

void pit_call_back(void);                // PIT 定时中断回调函数，主控制循环，约 1kHz，包含 IMU、PID、导航、舵机与电机控制
