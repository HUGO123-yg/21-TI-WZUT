#ifndef _jump_h_
#define _jump_h_

//*******************************************************************************************************************
// 文件名称    Jump.h
// 功能描述    跳跃控制模块对外接口头文件
//             移植 eg/Body_ctrl_jump.c 的状态机思路，保留旧 jump_flag/jump_time 兼容接口。
//*******************************************************************************************************************

typedef enum
{
    JUMP_IDLE = 0,
    JUMP_PREPARE,
    JUMP_CHARGE,
    JUMP_LAUNCH,
    JUMP_AIRBORNE,
    JUMP_LANDING,
    JUMP_RECOVER
} jump_state_enum;

typedef enum
{
    JUMP_TRIGGER_OK = 0,
    JUMP_TRIGGER_BUSY,
    JUMP_TRIGGER_NOT_RUNNING,
    JUMP_TRIGGER_TILT
} jump_trigger_result_enum;

typedef struct
{
    uint16 prepare_ticks;
    uint16 charge_ticks;
    uint16 launch_ticks;
    uint16 airborne_timeout;
    uint16 landing_ticks;
    uint16 recover_ticks;

    int16 charge_duty;
    int16 launch_duty;
    int16 preland_duty;
    int16 land_damping_duty;

    float forward_tilt_target;
    float forward_motor_boost;
    float speed_recovery_rate;

    float airborne_pid_scale;
    float landing_pid_scale;
    float recover_pid_ramp_rate;

    float airborne_acc_threshold;
    float landing_acc_threshold;
    float max_tilt_abort;

    void (*vision_jump_trigger)(float distance_mm);
    uint8 vision_jump_enable;
    float vision_obstacle_dist;
    float vision_min_dist;
    float vision_max_dist;

    uint8 state;
    uint16 elapsed;
    uint16 jump_count;
    float peak_acc_magnitude;
    float stored_p_angle;
    float stored_p_speed;
    float stored_speed_target;
    uint8 last_trigger_result;
} jump_config_struct;

// 旧状态标志兼容：保留给 Body_ctrl.c 与菜单显示使用。
#define JUMP_FLAG_ACCEL     1
#define JUMP_FLAG_TAKEOFF   2
#define JUMP_FLAG_RETRACT   4
#define JUMP_FLAG_EXTEND    8
#define JUMP_FLAG_BUFFER    16
#define JUMP_FLAG_RECOVER   32

#define JUMP_MOTOR_LOCK_ACTIVE(flag)     (jump_motor_lock_active())

extern jump_config_struct jump_cfg;
extern int jump_flag;
extern int jump_time;
extern int16 body_jump_motor_boost_duty;

uint8 jump_trigger(void);
void jump_start(void);
void jump_abort(void);
void jump_config_default(void);
void jump_control(void);
uint8 jump_can_trigger(void);
uint8 jump_is_active(void);
uint8 jump_motor_lock_active(void);
uint8 body_jump_speed_loop_should_update(void);
void body_jump_restore_saved_control(uint8 stop_speed);
void body_jump_set_all_leg_offset(int16 offset);
void body_jump_set_neutral_leg_offset(void);
const char *jump_state_name(uint8 state);
const char *jump_trigger_result_name(uint8 result);

#endif // _jump_h_
