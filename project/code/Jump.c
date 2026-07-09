#include "zf_common_headfile.h"
#include "config.h"

//*******************************************************************************************************************
// 文件名称    Jump.c
// 功能描述    跳跃状态机与舵机动作控制
//             从 eg/Body_ctrl_jump.c 移植核心结构：jump_cfg、jump_trigger、jump_abort、状态名/触发结果。
//             当前版本只在跳跃激活期间接管舵机与跳跃 PID 保护，不改变 TEST3 基础直立环结构。
//*******************************************************************************************************************

int jump_flag = 0;
int jump_time = 0;
int16 body_jump_motor_boost_duty = 0;

static pid_cycle_struct jump_saved_angular_speed_cycle;
static pid_cycle_struct jump_saved_angle_cycle;
static pid_cycle_struct jump_saved_speed_cycle;
static uint8 jump_saved_valid = 0;

jump_config_struct jump_cfg = {
    JUMP_PREPARE_TICKS,
    JUMP_CHARGE_TICKS,
    JUMP_LAUNCH_TICKS,
    JUMP_AIRBORNE_TIMEOUT,
    JUMP_LANDING_TICKS,
    JUMP_RECOVER_TICKS,

    JUMP_CHARGE_DUTY,
    JUMP_LAUNCH_DUTY,
    JUMP_PRELAND_DUTY,
    JUMP_LAND_DAMPING_DUTY,

    JUMP_FORWARD_TILT_TARGET,
    JUMP_FORWARD_MOTOR_BOOST,
    JUMP_SPEED_RECOVERY_RATE,

    JUMP_AIRBORNE_PID_SCALE,
    JUMP_LANDING_PID_SCALE,
    JUMP_RECOVER_PID_RAMP_RATE,

    JUMP_AIRBORNE_ACC_THRESHOLD,
    JUMP_LANDING_ACC_THRESHOLD,
    JUMP_MAX_TILT_ABORT,

    0,
    0,
    0.0f,
    JUMP_VISION_MIN_DIST,
    JUMP_VISION_MAX_DIST,

    JUMP_IDLE,
    0,
    0,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    JUMP_TRIGGER_OK,
};

static void jump_sync_legacy_flag(void)
{
    switch(jump_cfg.state)
    {
    case JUMP_IDLE:
        jump_flag = 0;
        break;
    case JUMP_PREPARE:
    case JUMP_CHARGE:
        jump_flag = JUMP_FLAG_ACCEL;
        break;
    case JUMP_LAUNCH:
        jump_flag = JUMP_FLAG_TAKEOFF;
        break;
    case JUMP_AIRBORNE:
        jump_flag = JUMP_FLAG_RETRACT;
        break;
    case JUMP_LANDING:
        jump_flag = JUMP_FLAG_EXTEND;
        break;
    case JUMP_RECOVER:
        jump_flag = JUMP_FLAG_RECOVER;
        break;
    default:
        jump_flag = 0;
        break;
    }
}

static void jump_reset_pid_memory(void)
{
    roll_balance_cascade.angle_cycle.i_value = 0.0f;
    roll_balance_cascade.angle_cycle.out = 0.0f;
    roll_balance_cascade.angle_cycle.p_value_last = 0.0f;
    roll_balance_cascade.angular_speed_cycle.i_value = 0.0f;
    roll_balance_cascade.angular_speed_cycle.out = 0.0f;
    roll_balance_cascade.angular_speed_cycle.p_value_last = 0.0f;
    roll_balance_cascade.speed_cycle.i_value = 0.0f;
    roll_balance_cascade.speed_cycle.out = 0.0f;
    roll_balance_cascade.speed_cycle.p_value_last = 0.0f;
    pitch_balance_cascade.angle_cycle.i_value = 0.0f;
    pitch_balance_cascade.angle_cycle.out = 0.0f;
    pitch_balance_cascade.angle_cycle.p_value_last = 0.0f;
}

static float jump_compute_tilt_angle(void)
{
    float rol_abs = func_abs(roll_balance_cascade.posture_value.rol);
    float pit_abs = func_abs(roll_balance_cascade.posture_value.pit);

    return (rol_abs > pit_abs) ? rol_abs : pit_abs;
}

static float jump_acc_magnitude(void)
{
    float ax = (float)ACC_DATA_X / ACC_TRANSITION_FACTOR;
    float ay = (float)ACC_DATA_Y / ACC_TRANSITION_FACTOR;
    float az = (float)ACC_DATA_Z / ACC_TRANSITION_FACTOR;

    return sqrtf(ax * ax + ay * ay + az * az);
}

static int16 jump_steer_duty_from_offset(const steer_control_struct *control_data, int16 physical_offset)
{
    int16 duty = control_data->center_num + physical_offset * control_data->steer_dir;

    return func_limit_ab(duty, 0, 10000);
}

static int16 jump_lerp_offset(int16 start, int16 end, uint16 elapsed, uint16 total)
{
    if(total == 0)
    {
        return end;
    }
    if(elapsed >= total)
    {
        return end;
    }

    return (int16)(start + (int32)(end - start) * elapsed / total);
}

static void jump_apply_fixed_pid(void)
{
    roll_balance_cascade.angle_cycle = jump_saved_angle_cycle;
    roll_balance_cascade.angular_speed_cycle = jump_saved_angular_speed_cycle;
    roll_balance_cascade.speed_cycle = jump_saved_speed_cycle;

    roll_balance_cascade.angle_cycle.i = 0.0f;
    roll_balance_cascade.angle_cycle.d = 0.0f;
    roll_balance_cascade.angular_speed_cycle.i = 0.0f;
    roll_balance_cascade.angular_speed_cycle.d = 0.0f;
    roll_balance_cascade.speed_cycle.i = 0.0f;
    roll_balance_cascade.speed_cycle.d = 0.0f;

    jump_reset_pid_memory();
}

static void jump_finish(uint8 stop_speed)
{
    body_jump_motor_boost_duty = 0;
    body_jump_restore_saved_control(stop_speed);
    body_jump_set_neutral_leg_offset();
    jump_cfg.state = JUMP_IDLE;
    jump_cfg.elapsed = 0;
    jump_time = 0;
    jump_sync_legacy_flag();
}

void body_jump_set_all_leg_offset(int16 offset)
{
    steer_duty_set(&steer_1, jump_steer_duty_from_offset(&steer_1, offset));
    steer_duty_set(&steer_2, jump_steer_duty_from_offset(&steer_2, offset));
    steer_duty_set(&steer_3, jump_steer_duty_from_offset(&steer_3, offset));
    steer_duty_set(&steer_4, jump_steer_duty_from_offset(&steer_4, offset));
}

void body_jump_set_neutral_leg_offset(void)
{
    body_jump_set_all_leg_offset(0);
}

void body_jump_restore_saved_control(uint8 stop_speed)
{
    if(jump_saved_valid)
    {
        roll_balance_cascade.angle_cycle = jump_saved_angle_cycle;
        roll_balance_cascade.angular_speed_cycle = jump_saved_angular_speed_cycle;
        roll_balance_cascade.speed_cycle = jump_saved_speed_cycle;
    }

    pitch_balance_cascade.angle_cycle.i_value = 0.0f;
    pitch_balance_cascade.angle_cycle.out = 0.0f;
    pitch_balance_cascade.angle_cycle.p_value_last = 0.0f;

    target_speed = stop_speed ? 0.0f : jump_cfg.stored_speed_target;
}

uint8 body_jump_speed_loop_should_update(void)
{
    switch(jump_cfg.state)
    {
    case JUMP_IDLE:
    case JUMP_PREPARE:
    case JUMP_CHARGE:
    case JUMP_LANDING:
    case JUMP_RECOVER:
        return 1;
    default:
        return 0;
    }
}

uint8 jump_trigger(void)
{
    if(jump_cfg.state != JUMP_IDLE)
    {
        jump_cfg.last_trigger_result = JUMP_TRIGGER_BUSY;
        return JUMP_TRIGGER_BUSY;
    }

    if(!run_state)
    {
        jump_cfg.last_trigger_result = JUMP_TRIGGER_NOT_RUNNING;
        return JUMP_TRIGGER_NOT_RUNNING;
    }

    if(jump_compute_tilt_angle() > jump_cfg.max_tilt_abort)
    {
        jump_cfg.last_trigger_result = JUMP_TRIGGER_TILT;
        return JUMP_TRIGGER_TILT;
    }

    jump_saved_angle_cycle = roll_balance_cascade.angle_cycle;
    jump_saved_angular_speed_cycle = roll_balance_cascade.angular_speed_cycle;
    jump_saved_speed_cycle = roll_balance_cascade.speed_cycle;
    jump_saved_valid = 1;

    jump_cfg.stored_p_angle = roll_balance_cascade.angle_cycle.p;
    jump_cfg.stored_p_speed = roll_balance_cascade.speed_cycle.p;
    jump_cfg.stored_speed_target = target_speed;
    jump_cfg.peak_acc_magnitude = 0.0f;
    jump_cfg.last_trigger_result = JUMP_TRIGGER_OK;
    jump_cfg.state = JUMP_PREPARE;
    jump_cfg.elapsed = 0;
    jump_time = 0;
    body_jump_motor_boost_duty = 0;

    body_jump_set_neutral_leg_offset();
    jump_apply_fixed_pid();
    jump_sync_legacy_flag();

    return JUMP_TRIGGER_OK;
}

void jump_start(void)
{
    (void)jump_trigger();
}

void jump_abort(void)
{
    uint8 was_active = jump_is_active();

    body_jump_motor_boost_duty = 0;
    jump_cfg.state = JUMP_IDLE;
    jump_cfg.elapsed = 0;
    jump_time = 0;

    if(was_active)
    {
        body_jump_restore_saved_control(1);
    }

    body_jump_set_neutral_leg_offset();
    jump_sync_legacy_flag();
}

void jump_config_default(void)
{
    void (*saved_vision_cb)(float) = jump_cfg.vision_jump_trigger;
    uint8 saved_vision_enable = jump_cfg.vision_jump_enable;

    if(jump_is_active())
    {
        jump_abort();
    }

    jump_cfg.prepare_ticks = JUMP_PREPARE_TICKS;
    jump_cfg.charge_ticks = JUMP_CHARGE_TICKS;
    jump_cfg.launch_ticks = JUMP_LAUNCH_TICKS;
    jump_cfg.airborne_timeout = JUMP_AIRBORNE_TIMEOUT;
    jump_cfg.landing_ticks = JUMP_LANDING_TICKS;
    jump_cfg.recover_ticks = JUMP_RECOVER_TICKS;
    jump_cfg.charge_duty = JUMP_CHARGE_DUTY;
    jump_cfg.launch_duty = JUMP_LAUNCH_DUTY;
    jump_cfg.preland_duty = JUMP_PRELAND_DUTY;
    jump_cfg.land_damping_duty = JUMP_LAND_DAMPING_DUTY;
    jump_cfg.forward_tilt_target = JUMP_FORWARD_TILT_TARGET;
    jump_cfg.forward_motor_boost = JUMP_FORWARD_MOTOR_BOOST;
    jump_cfg.speed_recovery_rate = JUMP_SPEED_RECOVERY_RATE;
    jump_cfg.airborne_pid_scale = JUMP_AIRBORNE_PID_SCALE;
    jump_cfg.landing_pid_scale = JUMP_LANDING_PID_SCALE;
    jump_cfg.recover_pid_ramp_rate = JUMP_RECOVER_PID_RAMP_RATE;
    jump_cfg.airborne_acc_threshold = JUMP_AIRBORNE_ACC_THRESHOLD;
    jump_cfg.landing_acc_threshold = JUMP_LANDING_ACC_THRESHOLD;
    jump_cfg.max_tilt_abort = JUMP_MAX_TILT_ABORT;
    jump_cfg.vision_jump_trigger = saved_vision_cb;
    jump_cfg.vision_jump_enable = saved_vision_enable;
    jump_cfg.vision_obstacle_dist = 0.0f;
    jump_cfg.vision_min_dist = JUMP_VISION_MIN_DIST;
    jump_cfg.vision_max_dist = JUMP_VISION_MAX_DIST;
    jump_cfg.state = JUMP_IDLE;
    jump_cfg.elapsed = 0;
    jump_cfg.jump_count = 0;
    jump_cfg.peak_acc_magnitude = 0.0f;
    jump_cfg.stored_p_angle = 0.0f;
    jump_cfg.stored_p_speed = 0.0f;
    jump_cfg.stored_speed_target = 0.0f;
    jump_cfg.last_trigger_result = JUMP_TRIGGER_OK;

    body_jump_motor_boost_duty = 0;
    body_jump_set_neutral_leg_offset();
    jump_sync_legacy_flag();
}

void jump_control(void)
{
    int16 offset;
    float acc_mag;

    if(jump_cfg.state == JUMP_IDLE)
    {
        jump_sync_legacy_flag();
        return;
    }

    jump_time++;
    jump_cfg.elapsed++;

    acc_mag = jump_acc_magnitude();
    if(acc_mag > jump_cfg.peak_acc_magnitude)
    {
        jump_cfg.peak_acc_magnitude = acc_mag;
    }

    switch(jump_cfg.state)
    {
    case JUMP_PREPARE:
        body_jump_motor_boost_duty = 0;
        body_jump_set_neutral_leg_offset();
        if(jump_cfg.elapsed >= jump_cfg.prepare_ticks)
        {
            jump_cfg.state = JUMP_CHARGE;
            jump_cfg.elapsed = 0;
        }
        break;

    case JUMP_CHARGE:
        offset = jump_lerp_offset(0, jump_cfg.charge_duty, jump_cfg.elapsed, jump_cfg.charge_ticks);
        body_jump_set_all_leg_offset(offset);
        if(jump_cfg.elapsed >= jump_cfg.charge_ticks)
        {
            jump_cfg.state = JUMP_LAUNCH;
            jump_cfg.elapsed = 0;
        }
        break;

    case JUMP_LAUNCH:
        body_jump_motor_boost_duty = (int16)jump_cfg.forward_motor_boost;
        offset = jump_lerp_offset(jump_cfg.charge_duty, jump_cfg.launch_duty, jump_cfg.elapsed, jump_cfg.launch_ticks);
        body_jump_set_all_leg_offset(offset);
        if(jump_cfg.elapsed >= jump_cfg.launch_ticks)
        {
            jump_cfg.state = JUMP_AIRBORNE;
            jump_cfg.elapsed = 0;
            body_jump_motor_boost_duty = 0;
        }
        break;

    case JUMP_AIRBORNE:
        body_jump_motor_boost_duty = 0;
        body_jump_set_all_leg_offset(jump_cfg.preland_duty);
        if((jump_cfg.elapsed >= jump_cfg.airborne_timeout)
            || (jump_cfg.elapsed > 20 && acc_mag >= jump_cfg.landing_acc_threshold))
        {
            jump_cfg.state = JUMP_LANDING;
            jump_cfg.elapsed = 0;
        }
        break;

    case JUMP_LANDING:
        offset = jump_lerp_offset(jump_cfg.preland_duty, 0, jump_cfg.elapsed, jump_cfg.landing_ticks);
        body_jump_set_all_leg_offset(offset);
        if(jump_cfg.elapsed >= jump_cfg.landing_ticks)
        {
            jump_cfg.state = JUMP_RECOVER;
            jump_cfg.elapsed = 0;
        }
        break;

    case JUMP_RECOVER:
        body_jump_set_neutral_leg_offset();
        if(jump_cfg.elapsed >= jump_cfg.recover_ticks)
        {
            jump_cfg.jump_count++;
            jump_finish(0);
        }
        break;

    default:
        jump_finish(1);
        break;
    }

    jump_sync_legacy_flag();
}

uint8 jump_can_trigger(void)
{
    if(jump_cfg.state != JUMP_IDLE)                         return 0;
    if(!run_state)                                          return 0;
    if(jump_compute_tilt_angle() > jump_cfg.max_tilt_abort) return 0;
    return 1;
}

uint8 jump_is_active(void)
{
    return (jump_cfg.state != JUMP_IDLE) ? 1 : 0;
}

uint8 jump_motor_lock_active(void)
{
    return (jump_cfg.state == JUMP_LAUNCH || jump_cfg.state == JUMP_AIRBORNE) ? 1 : 0;
}

const char *jump_state_name(uint8 state)
{
    switch(state)
    {
    case JUMP_IDLE:     return "IDLE";
    case JUMP_PREPARE:  return "PREP";
    case JUMP_CHARGE:   return "CHARGE";
    case JUMP_LAUNCH:   return "LAUNCH";
    case JUMP_AIRBORNE: return "AIR";
    case JUMP_LANDING:  return "LAND";
    case JUMP_RECOVER:  return "RECOVER";
    default:            return "UNKNOWN";
    }
}

const char *jump_trigger_result_name(uint8 result)
{
    switch(result)
    {
    case JUMP_TRIGGER_OK:          return "OK";
    case JUMP_TRIGGER_BUSY:        return "BUSY";
    case JUMP_TRIGGER_NOT_RUNNING: return "STOP";
    case JUMP_TRIGGER_TILT:        return "TILT";
    default:                       return "UNKNOWN";
    }
}
