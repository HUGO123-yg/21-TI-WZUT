
extern float  target_speed;
extern uint32 sys_times;
extern int16 left_motor_duty,right_motor_duty;
extern int STOP_FALG;

void pit_call_back(void);
void control_background_task(void);
