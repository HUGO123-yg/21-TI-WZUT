#include "zf_common_headfile.h"
#include "config.h"

//****************************************************************************
// 文件名称    obstacle_ctrl.c
// 功能描述    方格/颠簸路障通过控制模块
//****************************************************************************

#ifdef USE_OBSTACLE_CONTROL

static obstacle_state_e s_state = OBSTACLE_IDLE;
static uint16 s_state_ticks = 0;
static uint16 s_stuck_count = 0;
static uint8 s_retry_count = 0;
static uint8 s_reset_request = 0;
static float s_arm_mileage = 0.0f;
static float s_enter_mileage = 0.0f;
static float s_distance_cm = 0.0f;
static int16 s_override_left = 0;
static int16 s_override_right = 0;
static uint8 s_override_active = 0;
static float s_saved_target_speed = 0.0f;
static uint8 s_saved_speed_valid = 0;

static void obstacle_request_pid_reset(void)
{
    s_reset_request = 1;
}

static void obstacle_restore_target_speed(void)
{
    if(s_saved_speed_valid)
    {
        target_speed = s_saved_target_speed;
        s_saved_speed_valid = 0;
    }
}

static void obstacle_set_state(obstacle_state_e next_state, float mileage)
{
    s_state = next_state;
    s_state_ticks = 0;
    s_stuck_count = 0;
    s_override_active = 0;
    s_override_left = 0;
    s_override_right = 0;

    if(next_state == OBSTACLE_ENTER || next_state == OBSTACLE_CROSSING)
    {
        if(!s_saved_speed_valid)
        {
            s_saved_target_speed = target_speed;
            s_saved_speed_valid = 1;
        }
    }

    if(next_state == OBSTACLE_ENTER)
    {
        s_enter_mileage = mileage;
    }

    if(next_state == OBSTACLE_ENTER || next_state == OBSTACLE_BACKOFF ||
       next_state == OBSTACLE_BOOST || next_state == OBSTACLE_RECOVER)
    {
        obstacle_request_pid_reset();
    }
}

static void obstacle_finish(void)
{
    obstacle_restore_target_speed();
    s_state = OBSTACLE_IDLE;
    s_state_ticks = 0;
    s_stuck_count = 0;
    s_retry_count = 0;
    s_distance_cm = 0.0f;
    s_override_active = 0;
    s_override_left = 0;
    s_override_right = 0;
    obstacle_request_pid_reset();
}

static void obstacle_apply_speed_limit(void)
{
    float speed_limit = OBSTACLE_TARGET_SPEED_RPM;

    if(speed_limit <= 0.0f)
    {
        return;
    }

    if(target_speed > speed_limit)
    {
        target_speed = speed_limit;
    }
    else if(target_speed < -speed_limit)
    {
        target_speed = -speed_limit;
    }
}

static uint8 obstacle_tilt_too_large(float pitch_deg, float roll_deg)
{
    return (func_abs(pitch_deg) > OBSTACLE_TILT_ABORT_DEG ||
            func_abs(roll_deg) > OBSTACLE_TILT_ABORT_DEG) ? 1 : 0;
}

static uint8 obstacle_stuck_detected(int16 speed, int16 balance_motor)
{
    if(func_abs(speed) <= OBSTACLE_STUCK_SPEED_RPM &&
       func_abs(balance_motor) >= OBSTACLE_STUCK_BALANCE_DUTY)
    {
        if(s_stuck_count < OBSTACLE_STUCK_DETECT_CYCLES)
        {
            s_stuck_count++;
        }
    }
    else
    {
        s_stuck_count = 0;
    }

    return (s_stuck_count >= OBSTACLE_STUCK_DETECT_CYCLES) ? 1 : 0;
}

#endif

void obstacle_init(void)
{
#ifdef USE_OBSTACLE_CONTROL
    obstacle_finish();
    s_reset_request = 0;
#endif
}

void obstacle_arm(float mileage)
{
#ifdef USE_OBSTACLE_CONTROL
    obstacle_restore_target_speed();
    s_arm_mileage = mileage;
    s_enter_mileage = mileage;
    s_distance_cm = 0.0f;
    s_retry_count = 0;
    s_stuck_count = 0;
    s_reset_request = 0;
    s_override_active = 0;
    s_saved_target_speed = target_speed;
    s_saved_speed_valid = 1;

    if(OBSTACLE_START_DISTANCE_CM <= 0.0f)
    {
        obstacle_set_state(OBSTACLE_ENTER, mileage);
    }
    else
    {
        s_state = OBSTACLE_WAIT;
        s_state_ticks = 0;
    }
#else
    (void)mileage;
#endif
}

void obstacle_abort(void)
{
#ifdef USE_OBSTACLE_CONTROL
    if(s_state != OBSTACLE_IDLE || s_saved_speed_valid)
    {
        obstacle_finish();
    }
#endif
}

void obstacle_run_1ms(float mileage, int16 speed, int16 balance_motor, float pitch_deg, float roll_deg)
{
#ifdef USE_OBSTACLE_CONTROL
    if(s_state == OBSTACLE_IDLE)
    {
        return;
    }

    s_state_ticks++;
    s_distance_cm = func_abs(mileage - s_enter_mileage);

    if(obstacle_tilt_too_large(pitch_deg, roll_deg))
    {
        obstacle_set_state(OBSTACLE_RECOVER, mileage);
        return;
    }

    switch(s_state)
    {
    case OBSTACLE_WAIT:
        if(func_abs(mileage - s_arm_mileage) >= OBSTACLE_START_DISTANCE_CM)
        {
            obstacle_set_state(OBSTACLE_ENTER, mileage);
        }
        break;

    case OBSTACLE_ENTER:
        obstacle_apply_speed_limit();
        if(s_state_ticks >= OBSTACLE_ENTER_CYCLES)
        {
            obstacle_set_state(OBSTACLE_CROSSING, mileage);
        }
        break;

    case OBSTACLE_CROSSING:
        obstacle_apply_speed_limit();
        if(s_distance_cm >= OBSTACLE_LENGTH_CM ||
           s_state_ticks >= OBSTACLE_CROSSING_TIMEOUT_CYCLES)
        {
            obstacle_set_state(OBSTACLE_RECOVER, mileage);
        }
        else if(obstacle_stuck_detected(speed, balance_motor))
        {
            if(s_retry_count < OBSTACLE_MAX_RETRY)
            {
                s_retry_count++;
                obstacle_set_state(OBSTACLE_BACKOFF, mileage);
            }
            else
            {
                obstacle_set_state(OBSTACLE_RECOVER, mileage);
            }
        }
        break;

    case OBSTACLE_BACKOFF:
        s_override_left = OBSTACLE_BACKOFF_DUTY;
        s_override_right = OBSTACLE_BACKOFF_DUTY;
        s_override_active = 1;
        if(s_state_ticks >= OBSTACLE_BACKOFF_CYCLES)
        {
            obstacle_set_state(OBSTACLE_BOOST, mileage);
        }
        break;

    case OBSTACLE_BOOST:
        s_override_left = OBSTACLE_BOOST_DUTY;
        s_override_right = OBSTACLE_BOOST_DUTY;
        s_override_active = 1;
        if(s_state_ticks >= OBSTACLE_BOOST_CYCLES)
        {
            obstacle_set_state(OBSTACLE_CROSSING, mileage);
        }
        break;

    case OBSTACLE_RECOVER:
        if(s_state_ticks >= OBSTACLE_RECOVER_CYCLES)
        {
            obstacle_finish();
        }
        break;

    default:
        obstacle_finish();
        break;
    }
#else
    (void)mileage;
    (void)speed;
    (void)balance_motor;
    (void)pitch_deg;
    (void)roll_deg;
#endif
}

uint8 obstacle_is_active(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return (s_state != OBSTACLE_IDLE) ? 1 : 0;
#else
    return 0;
#endif
}

obstacle_state_e obstacle_get_state(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return s_state;
#else
    return OBSTACLE_IDLE;
#endif
}

float obstacle_get_distance_cm(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return s_distance_cm;
#else
    return 0.0f;
#endif
}

uint16 obstacle_get_stuck_count(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return s_stuck_count;
#else
    return 0;
#endif
}

uint8 obstacle_get_retry_count(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return s_retry_count;
#else
    return 0;
#endif
}

float obstacle_get_angle_offset(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return (s_state == OBSTACLE_ENTER || s_state == OBSTACLE_CROSSING ||
            s_state == OBSTACLE_BOOST) ? OBSTACLE_FORWARD_TILT_DEG : 0.0f;
#else
    return 0.0f;
#endif
}

float obstacle_get_nav_scale(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return (s_state == OBSTACLE_ENTER || s_state == OBSTACLE_CROSSING ||
            s_state == OBSTACLE_BACKOFF || s_state == OBSTACLE_BOOST ||
            s_state == OBSTACLE_RECOVER) ? OBSTACLE_NAV_SCALE : 1.0f;
#else
    return 1.0f;
#endif
}

int16 obstacle_get_motor_boost(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return (s_state == OBSTACLE_ENTER || s_state == OBSTACLE_CROSSING) ?
           OBSTACLE_MOTOR_BOOST_DUTY : 0;
#else
    return 0;
#endif
}

uint8 obstacle_get_motor_override(int16 *left_motor, int16 *right_motor)
{
#ifdef USE_OBSTACLE_CONTROL
    if(s_override_active)
    {
        if(left_motor != 0)
        {
            *left_motor = s_override_left;
        }
        if(right_motor != 0)
        {
            *right_motor = s_override_right;
        }
        return 1;
    }
#else
    (void)left_motor;
    (void)right_motor;
#endif

    return 0;
}

uint8 obstacle_speed_pid_should_update(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return (s_state == OBSTACLE_IDLE || s_state == OBSTACLE_WAIT) ? 1 : 0;
#else
    return 1;
#endif
}

uint8 obstacle_track_pid_should_update(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return (s_state == OBSTACLE_IDLE || s_state == OBSTACLE_WAIT) ? 1 : 0;
#else
    return 1;
#endif
}

uint8 obstacle_should_reset_pid(void)
{
#ifdef USE_OBSTACLE_CONTROL
    return s_reset_request;
#else
    return 0;
#endif
}

void obstacle_clear_reset_request(void)
{
#ifdef USE_OBSTACLE_CONTROL
    s_reset_request = 0;
#endif
}
