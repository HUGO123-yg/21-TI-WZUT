
extern float  target_speed;
extern uint32 sys_times;
extern volatile uint32 control_uptime_ticks;
extern int16 left_motor_duty,right_motor_duty;
extern int STOP_FALG;

void pit_call_back(void);
void control_background_task(void);
void control_publish_main_command(void);
