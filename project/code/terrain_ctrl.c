#include "zf_common_headfile.h"
#include "config.h"

//****************************************************************************
// 文件名称    terrain_ctrl.c
// 功能描述    路况仲裁层：自动触发单边桥与颠簸路段，草地按普通路处理。
//****************************************************************************

static terrain_state_e s_state = TERRAIN_NORMAL;
static uint16 s_bridge_count = 0;
static uint16 s_obstacle_score = 0;
static uint16 s_obstacle_cooldown = 0;
static uint16 s_state_ticks = 0;
static float s_state_mileage = 0.0f;
static float s_course_zero_mileage = 0.0f;
static uint8 s_bridge_mileage_used = 0;
static uint8 s_obstacle_mileage_used = 0;

#ifdef USE_TERRAIN_DEBUG
static terrain_debug_sample_t s_debug_sample;
static volatile uint8 s_debug_pending = 0;
static uint32 s_debug_last_send_tick = 0;
#endif

static float terrain_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float terrain_course_mileage_from(float mileage)
{
    return mileage - s_course_zero_mileage;
}

static uint8 terrain_mileage_in_window(float course_mileage, float start_cm, float end_cm)
{
    if(end_cm <= start_cm)
    {
        return 0;
    }

    return (course_mileage >= (start_cm - TERRAIN_MILEAGE_PRE_ARM_CM) &&
            course_mileage <= (end_cm + TERRAIN_MILEAGE_POST_HOLD_CM)) ? 1 : 0;
}

uint8 terrain_bridge_mileage_window_active(void)
{
#if defined(USE_TERRAIN_MILEAGE_TRIGGER) && (TERRAIN_BRIDGE_MILEAGE_ENABLE != 0)
    return terrain_mileage_in_window(terrain_get_course_mileage(),
                                     TERRAIN_BRIDGE_START_CM,
                                     TERRAIN_BRIDGE_END_CM);
#else
    return 0;
#endif
}

uint8 terrain_obstacle_mileage_window_active(void)
{
#if defined(USE_TERRAIN_MILEAGE_TRIGGER) && (TERRAIN_OBSTACLE_MILEAGE_ENABLE != 0)
    return terrain_mileage_in_window(terrain_get_course_mileage(),
                                     TERRAIN_OBSTACLE_START_CM,
                                     TERRAIN_OBSTACLE_END_CM);
#else
    return 0;
#endif
}

static uint8 terrain_bridge_auto_allowed(void)
{
#if defined(USE_TERRAIN_MILEAGE_TRIGGER) && (TERRAIN_MILEAGE_GATE_AUTO != 0) && (TERRAIN_BRIDGE_MILEAGE_ENABLE != 0)
    return terrain_bridge_mileage_window_active();
#else
    return 1;
#endif
}

static uint8 terrain_obstacle_auto_allowed(void)
{
#if defined(USE_TERRAIN_MILEAGE_TRIGGER) && (TERRAIN_MILEAGE_GATE_AUTO != 0) && (TERRAIN_OBSTACLE_MILEAGE_ENABLE != 0)
    return terrain_obstacle_mileage_window_active();
#else
    return 1;
#endif
}

static uint8 terrain_bridge_mileage_should_force(void)
{
#if defined(USE_TERRAIN_MILEAGE_TRIGGER) && (TERRAIN_MILEAGE_FORCE_TRIGGER != 0) && (TERRAIN_BRIDGE_MILEAGE_ENABLE != 0)
    return (terrain_bridge_mileage_window_active() && !s_bridge_mileage_used) ? 1 : 0;
#else
    return 0;
#endif
}

static uint8 terrain_obstacle_mileage_should_force(void)
{
#if defined(USE_TERRAIN_MILEAGE_TRIGGER) && (TERRAIN_MILEAGE_FORCE_TRIGGER != 0) && (TERRAIN_OBSTACLE_MILEAGE_ENABLE != 0)
    return (terrain_obstacle_mileage_window_active() && !s_obstacle_mileage_used) ? 1 : 0;
#else
    return 0;
#endif
}

static void terrain_enter_bridge(float mileage, float roll_deg, uint8 force_enter)
{
    s_state = TERRAIN_BRIDGE;
    s_state_mileage = mileage;
    s_state_ticks = 0;
    s_obstacle_score = 0;
    s_bridge_mileage_used = 1;
#ifdef USE_OBSTACLE_CONTROL
    obstacle_abort();
#endif
#ifdef USE_BRIDGE_CONTROL
    if(force_enter)
    {
        bridge_force_enter(roll_deg, mileage);
    }
    else
    {
        bridge_run(roll_deg, mileage, TERRAIN_BRIDGE_AUTO_SPEED_RPM);
    }
#endif
}

static void terrain_enter_obstacle(float mileage)
{
    s_state = TERRAIN_OBSTACLE;
    s_state_mileage = mileage;
    s_state_ticks = 0;
    s_bridge_count = 0;
    s_obstacle_mileage_used = 1;
#ifdef USE_BRIDGE_CONTROL
    bridge_init();
#endif
#ifdef USE_OBSTACLE_CONTROL
    obstacle_arm(mileage);
#endif
}

static uint8 terrain_bridge_candidate(float roll_deg)
{
#if defined(USE_BRIDGE_CONTROL) && defined(USE_TERRAIN_AUTO_BRIDGE)
    if(terrain_absf(roll_deg) > TERRAIN_BRIDGE_ROLL_THRESHOLD)
    {
        if(s_bridge_count < TERRAIN_BRIDGE_DETECT_CYCLES)
        {
            s_bridge_count++;
        }
    }
    else
    {
        s_bridge_count = 0;
    }

    return (s_bridge_count >= TERRAIN_BRIDGE_DETECT_CYCLES) ? 1 : 0;
#else
    (void)roll_deg;
    s_bridge_count = 0;
    return 0;
#endif
}

static uint8 terrain_obstacle_candidate(int16 speed, int16 balance_motor, float pitch_deg, float roll_deg)
{
#if defined(USE_OBSTACLE_CONTROL) && defined(USE_TERRAIN_AUTO_OBSTACLE)
    float pitch_rate_dps = (float)GYRO_DATA_Y / GYRO_TRANSITION_FACTOR;
    float roll_rate_dps = (float)GYRO_DATA_X / GYRO_TRANSITION_FACTOR;
    uint8 shock_hit = 0;
    uint8 load_hit = 0;

    if(terrain_absf(pitch_deg) < TERRAIN_OBSTACLE_MAX_PITCH_DEG &&
       terrain_absf(roll_deg) < TERRAIN_OBSTACLE_MAX_ROLL_DEG)
    {
        if(terrain_absf(pitch_rate_dps) > TERRAIN_OBSTACLE_GYRO_THRESH_DPS ||
           terrain_absf(roll_rate_dps) > TERRAIN_OBSTACLE_GYRO_THRESH_DPS)
        {
            shock_hit = 1;
        }

        if(terrain_absf((float)balance_motor) > TERRAIN_OBSTACLE_MOTOR_DUTY_THRESH &&
           terrain_absf((float)speed) < TERRAIN_OBSTACLE_SPEED_DROP_RPM)
        {
            load_hit = 1;
        }
    }

    if(shock_hit)
    {
        if(s_obstacle_score < TERRAIN_OBSTACLE_SCORE_ON)
        {
            s_obstacle_score += TERRAIN_OBSTACLE_SCORE_SHOCK_STEP;
            if(s_obstacle_score > TERRAIN_OBSTACLE_SCORE_ON)
            {
                s_obstacle_score = TERRAIN_OBSTACLE_SCORE_ON;
            }
        }
    }
    else if(load_hit && s_obstacle_score > 0)
    {
        if(s_obstacle_score < TERRAIN_OBSTACLE_SCORE_ON)
        {
            s_obstacle_score += TERRAIN_OBSTACLE_SCORE_LOAD_STEP;
            if(s_obstacle_score > TERRAIN_OBSTACLE_SCORE_ON)
            {
                s_obstacle_score = TERRAIN_OBSTACLE_SCORE_ON;
            }
        }
    }
    else if(s_obstacle_score > TERRAIN_OBSTACLE_SCORE_DECAY)
    {
        s_obstacle_score -= TERRAIN_OBSTACLE_SCORE_DECAY;
    }
    else
    {
        s_obstacle_score = 0;
    }

    return (s_obstacle_score >= TERRAIN_OBSTACLE_SCORE_ON) ? 1 : 0;
#else
    (void)speed;
    (void)balance_motor;
    (void)pitch_deg;
    (void)roll_deg;
    s_obstacle_score = 0;
    return 0;
#endif
}

void terrain_init(void)
{
    s_state = TERRAIN_NORMAL;
    s_bridge_count = 0;
    s_obstacle_score = 0;
    s_obstacle_cooldown = 0;
    s_state_ticks = 0;
    s_state_mileage = 0.0f;
    s_bridge_mileage_used = 0;
    s_obstacle_mileage_used = 0;

#ifdef USE_BRIDGE_CONTROL
    bridge_init();
#endif
#ifdef USE_OBSTACLE_CONTROL
    obstacle_abort();
#endif
}

void terrain_course_reset(float mileage)
{
    terrain_init();
    s_course_zero_mileage = mileage;
    s_bridge_mileage_used = 0;
    s_obstacle_mileage_used = 0;
}

void terrain_abort(void)
{
    s_state = TERRAIN_NORMAL;
    s_bridge_count = 0;
    s_obstacle_score = 0;
    s_obstacle_cooldown = 0;
    s_state_ticks = 0;
    s_state_mileage = 0.0f;
#ifdef USE_BRIDGE_CONTROL
    bridge_init();
#endif
#ifdef USE_OBSTACLE_CONTROL
    obstacle_abort();
#endif
}

void terrain_run_1ms(float mileage, int16 speed, int16 balance_motor, float pitch_deg, float roll_deg, uint32 armed_ticks)
{
    if(s_obstacle_cooldown > 0)
    {
        s_obstacle_cooldown--;
    }
    s_state_ticks++;

    if(armed_ticks < TERRAIN_STARTUP_IGNORE_CYCLES && s_state == TERRAIN_NORMAL)
    {
        s_bridge_count = 0;
        s_obstacle_score = 0;
        return;
    }

    switch(s_state)
    {
    case TERRAIN_NORMAL:
        if(terrain_bridge_mileage_should_force())
        {
            terrain_enter_bridge(mileage, roll_deg, 1);
            break;
        }

        if(terrain_obstacle_mileage_should_force())
        {
            terrain_enter_obstacle(mileage);
            break;
        }

        if(terrain_bridge_auto_allowed() && terrain_bridge_candidate(roll_deg))
        {
            terrain_enter_bridge(mileage, roll_deg, 0);
            break;
        }

        if(s_obstacle_cooldown == 0 &&
           terrain_obstacle_auto_allowed() &&
           terrain_obstacle_candidate(speed, balance_motor, pitch_deg, roll_deg))
        {
            terrain_enter_obstacle(mileage);
        }
        break;

    case TERRAIN_BRIDGE:
#ifdef USE_BRIDGE_CONTROL
        if(bridge_get_state() == BRIDGE_IDLE &&
           (terrain_absf(mileage - s_state_mileage) > TERRAIN_BRIDGE_MIN_HOLD_CM ||
            s_state_ticks > TERRAIN_BRIDGE_IDLE_TIMEOUT_CYCLES))
        {
            s_state = TERRAIN_NORMAL;
            s_state_ticks = 0;
            s_bridge_count = 0;
            s_obstacle_score = 0;
            s_obstacle_cooldown = TERRAIN_OBSTACLE_COOLDOWN_CYCLES;
        }
#else
        s_state = TERRAIN_NORMAL;
#endif
        break;

    case TERRAIN_OBSTACLE:
        if((terrain_bridge_mileage_should_force()) ||
           (terrain_bridge_auto_allowed() && terrain_bridge_candidate(roll_deg)))
        {
            terrain_enter_bridge(mileage, roll_deg, terrain_bridge_mileage_should_force());
            break;
        }

#ifdef USE_OBSTACLE_CONTROL
        if(!obstacle_is_active())
        {
            s_state = TERRAIN_NORMAL;
            s_state_ticks = 0;
            s_obstacle_score = 0;
            s_bridge_count = 0;
            s_obstacle_cooldown = TERRAIN_OBSTACLE_COOLDOWN_CYCLES;
        }
#else
        s_state = TERRAIN_NORMAL;
#endif
        break;

    default:
        terrain_init();
        break;
    }
}

terrain_state_e terrain_get_state(void)
{
    return s_state;
}

uint8 terrain_bridge_is_active(void)
{
    return (s_state == TERRAIN_BRIDGE) ? 1 : 0;
}

uint8 terrain_obstacle_is_active(void)
{
    return (s_state == TERRAIN_OBSTACLE) ? 1 : 0;
}

uint16 terrain_get_bridge_count(void)
{
    return s_bridge_count;
}

uint16 terrain_get_obstacle_score(void)
{
    return s_obstacle_score;
}

float terrain_get_course_mileage(void)
{
    return terrain_course_mileage_from(Car.mileage);
}

#ifdef USE_TERRAIN_DEBUG
void terrain_debug_update(uint32 tick,
                          int16 left_motor,
                          int16 right_motor,
                          float angle_out,
                          float speed_out,
                          float track_out,
                          float nav_out)
{
    s_debug_sample.tick = tick;
    s_debug_sample.terrain_state = s_state;
#ifdef USE_BRIDGE_CONTROL
    s_debug_sample.bridge_state = bridge_get_state();
#else
    s_debug_sample.bridge_state = BRIDGE_IDLE;
#endif
#ifdef USE_OBSTACLE_CONTROL
    s_debug_sample.obstacle_state = obstacle_get_state();
#else
    s_debug_sample.obstacle_state = OBSTACLE_IDLE;
#endif
    s_debug_sample.mileage = Car.mileage;
    s_debug_sample.course_mileage = terrain_get_course_mileage();
    s_debug_sample.speed = car_speed;
    s_debug_sample.balance_motor = (int16)(BALANCE_MOTOR_OUTPUT_SIGN * roll_balance_cascade.angular_speed_cycle.out);
    s_debug_sample.left_motor = left_motor;
    s_debug_sample.right_motor = right_motor;
    s_debug_sample.pitch_deg = roll_balance_cascade.posture_value.pit;
    s_debug_sample.roll_deg = roll_balance_cascade.posture_value.rol;
    s_debug_sample.gyro_pitch_dps = (float)GYRO_DATA_Y / GYRO_TRANSITION_FACTOR;
    s_debug_sample.gyro_roll_dps = (float)GYRO_DATA_X / GYRO_TRANSITION_FACTOR;
    s_debug_sample.angle_out = angle_out;
    s_debug_sample.speed_out = speed_out;
    s_debug_sample.track_out = track_out;
    s_debug_sample.nav_out = nav_out;
    s_debug_sample.bridge_count = s_bridge_count;
    s_debug_sample.obstacle_score = s_obstacle_score;
#ifdef USE_OBSTACLE_CONTROL
    s_debug_sample.obstacle_stuck_count = obstacle_get_stuck_count();
    s_debug_sample.obstacle_retry_count = obstacle_get_retry_count();
#else
    s_debug_sample.obstacle_stuck_count = 0;
    s_debug_sample.obstacle_retry_count = 0;
#endif
    s_debug_sample.bridge_window = terrain_bridge_mileage_window_active();
    s_debug_sample.obstacle_window = terrain_obstacle_mileage_window_active();
    s_debug_pending = 1;
}

void terrain_debug_service(void)
{
#if TERRAIN_DEBUG_SEND_ENABLE != 0
    if(!s_debug_pending)
    {
        return;
    }

    if((uint32)(s_debug_sample.tick - s_debug_last_send_tick) < TERRAIN_DEBUG_SEND_PERIOD_MS)
    {
        return;
    }

    s_debug_last_send_tick = s_debug_sample.tick;
    s_debug_pending = 0;

    seekfree_assistant_oscilloscope_data.channel_num = 8;
    seekfree_assistant_oscilloscope_data.data[0] = s_debug_sample.course_mileage;
    seekfree_assistant_oscilloscope_data.data[1] = (float)s_debug_sample.terrain_state;
    seekfree_assistant_oscilloscope_data.data[2] = (float)s_debug_sample.bridge_state;
    seekfree_assistant_oscilloscope_data.data[3] = (float)s_debug_sample.obstacle_state;
    seekfree_assistant_oscilloscope_data.data[4] = s_debug_sample.roll_deg;
    seekfree_assistant_oscilloscope_data.data[5] = s_debug_sample.pitch_deg;
    seekfree_assistant_oscilloscope_data.data[6] = (float)s_debug_sample.speed;
    seekfree_assistant_oscilloscope_data.data[7] = (float)s_debug_sample.balance_motor;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
#endif
}

const terrain_debug_sample_t *terrain_debug_get_sample(void)
{
    return &s_debug_sample;
}
#endif
