/*
 * One_bridge.c — 单边桥自动通过控制模块
 *
 * 反应式方案：roll 连续偏大时进入桥模式，桥上限制速度、用腿差动拉平车身，
 * 并给高侧轮叠加小差速，退出时平滑释放补偿。
 */

#include "zf_common_headfile.h"

one_bridge_control_struct one_bridge = {
    .auto_enable        = ONE_BRIDGE_AUTO_ENABLE_DEFAULT,
    .state              = ONE_BRIDGE_IDLE,
    .side               = ONE_BRIDGE_SIDE_NONE,
    .entry_stable_count = 0,
    .exit_stable_count  = 0,
    .elapsed            = 0,
    .cooldown           = 0,
    .roll_error         = 0.0f,
    .roll_filtered      = 0.0f,
    .leg_integral       = 0.0f,
    .leg_offset         = 0,
    .motor_adj          = 0,
    .start_mileage      = 0.0f,
    .distance           = 0.0f,
    .saved_speed_target = 0.0f,
};

static float one_bridge_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int16 one_bridge_sign_to_int16(float value)
{
    return (value >= 0.0f) ? 1 : -1;
}

static uint8 one_bridge_is_motion_allowed(void)
{
    float speed_abs = one_bridge_absf((float)car_speed);
    float target_abs = one_bridge_absf(target_speed);

    if (!STOP_FLAG || !run_state)                         return 0;
    if (jump_is_active())                                 return 0;
    if (stair_is_active())                                return 0;
    if (rotation_is_active())                             return 0;
    if (straight_test.state != STRAIGHT_IDLE)             return 0;
    if (speed_abs < ONE_BRIDGE_MIN_SPEED
        && target_abs < ONE_BRIDGE_MIN_SPEED)             return 0;

    return 1;
}

static void one_bridge_limit_target_speed(void)
{
    if (target_speed > ONE_BRIDGE_MAX_SPEED)
    {
        target_speed = ONE_BRIDGE_MAX_SPEED;
    }
    else if (target_speed < -ONE_BRIDGE_MAX_SPEED)
    {
        target_speed = -ONE_BRIDGE_MAX_SPEED;
    }
}

static void one_bridge_enter_running(uint8 side)
{
    one_bridge.state              = ONE_BRIDGE_RUNNING;
    one_bridge.side               = side;
    one_bridge.entry_stable_count = 0;
    one_bridge.exit_stable_count  = 0;
    one_bridge.elapsed            = 0;
    one_bridge.start_mileage      = Car.mileage;
    one_bridge.distance           = 0.0f;
    one_bridge.saved_speed_target = target_speed;
    one_bridge.leg_integral       = 0.0f;
    one_bridge.leg_offset         = 0;
    one_bridge.motor_adj          = 0;

    one_bridge_limit_target_speed();
}

static void one_bridge_enter_exiting(void)
{
    one_bridge.state             = ONE_BRIDGE_EXITING;
    one_bridge.elapsed           = 0;
    one_bridge.exit_stable_count = 0;
    one_bridge.motor_adj         = 0;

    target_speed = one_bridge.saved_speed_target;
}

static void one_bridge_finish_idle(void)
{
    one_bridge.state              = ONE_BRIDGE_IDLE;
    one_bridge.side               = ONE_BRIDGE_SIDE_NONE;
    one_bridge.entry_stable_count = 0;
    one_bridge.exit_stable_count  = 0;
    one_bridge.elapsed            = 0;
    one_bridge.cooldown           = ONE_BRIDGE_COOLDOWN_MS;
    one_bridge.leg_integral       = 0.0f;
    one_bridge.leg_offset         = 0;
    one_bridge.motor_adj          = 0;
}

static void one_bridge_update_leg_and_motor(void)
{
    float leg_cmd;
    float speed_abs;
    float move_sign;
    float side_sign;
    int16 comp;

    one_bridge.leg_integral += one_bridge.roll_filtered
                             * ONE_BRIDGE_LEG_I_DUTY_PER_DEG_MS
                             * ONE_BRIDGE_LEG_DIR;
    one_bridge.leg_integral = func_limit_ab(one_bridge.leg_integral,
                                            -ONE_BRIDGE_LEG_MAX_DUTY,
                                            ONE_BRIDGE_LEG_MAX_DUTY);

    leg_cmd = one_bridge.roll_filtered
            * ONE_BRIDGE_LEG_P_DUTY_PER_DEG
            * ONE_BRIDGE_LEG_DIR
            + one_bridge.leg_integral;
    leg_cmd = func_limit_ab(leg_cmd, -ONE_BRIDGE_LEG_MAX_DUTY, ONE_BRIDGE_LEG_MAX_DUTY);
    one_bridge.leg_offset = (int16)leg_cmd;

    speed_abs = one_bridge_absf(target_speed);
    if (speed_abs < one_bridge_absf((float)car_speed))
    {
        speed_abs = one_bridge_absf((float)car_speed);
    }

    comp = (int16)(speed_abs * ONE_BRIDGE_SPEED_TO_DUTY_GAIN * ONE_BRIDGE_PATH_RATIO);
    comp = func_limit_ab(comp, 0, ONE_BRIDGE_MOTOR_COMP_MAX);

    move_sign = (target_speed != 0.0f)
              ? (float)one_bridge_sign_to_int16(target_speed)
              : (float)one_bridge_sign_to_int16((float)car_speed);
    side_sign = (one_bridge.side == ONE_BRIDGE_SIDE_POS_ROLL) ? 1.0f : -1.0f;
    one_bridge.motor_adj = (int16)(side_sign * move_sign
                                  * (float)comp
                                  * (float)ONE_BRIDGE_MOTOR_DIR);
}

void one_bridge_service_1ms(void)
{
    uint8 current_side;
    float raw_roll;
    float abs_roll;

    raw_roll = roll_balance_cascade.posture_value.rol - ONE_BRIDGE_ROLL_ZERO_DEG;
    one_bridge.roll_error = raw_roll;
    one_bridge.roll_filtered = one_bridge.roll_filtered * ONE_BRIDGE_ROLL_FILTER_ALPHA
                             + raw_roll * (1.0f - ONE_BRIDGE_ROLL_FILTER_ALPHA);
    abs_roll = one_bridge_absf(one_bridge.roll_filtered);

    if (one_bridge.cooldown > 0)
    {
        one_bridge.cooldown--;
    }

    if (one_bridge.state == ONE_BRIDGE_IDLE)
    {
        if (!one_bridge.auto_enable || one_bridge.cooldown > 0 || !one_bridge_is_motion_allowed())
        {
            one_bridge.entry_stable_count = 0;
            return;
        }

        if (abs_roll >= ONE_BRIDGE_ENTRY_ROLL_DEG)
        {
            current_side = (one_bridge.roll_filtered >= 0.0f)
                         ? ONE_BRIDGE_SIDE_POS_ROLL
                         : ONE_BRIDGE_SIDE_NEG_ROLL;

            if (one_bridge.side != current_side)
            {
                one_bridge.side = current_side;
                one_bridge.entry_stable_count = 0;
            }

            if (one_bridge.entry_stable_count < ONE_BRIDGE_ENTRY_STABLE_MS)
            {
                one_bridge.entry_stable_count++;
            }

            if (one_bridge.entry_stable_count >= ONE_BRIDGE_ENTRY_STABLE_MS)
            {
                one_bridge_enter_running(current_side);
            }
        }
        else
        {
            one_bridge.entry_stable_count = 0;
            one_bridge.side = ONE_BRIDGE_SIDE_NONE;
        }

        return;
    }

    if (one_bridge.state == ONE_BRIDGE_RUNNING)
    {
        if (!one_bridge_is_motion_allowed())
        {
            one_bridge_abort();
            return;
        }

        one_bridge.elapsed++;
        one_bridge.distance = one_bridge_absf(Car.mileage - one_bridge.start_mileage);
        one_bridge_limit_target_speed();
        one_bridge_update_leg_and_motor();

        if (one_bridge.elapsed >= ONE_BRIDGE_MAX_ACTIVE_MS
            || one_bridge.distance >= ONE_BRIDGE_RUN_DISTANCE_CM)
        {
            one_bridge_enter_exiting();
            return;
        }

        if (one_bridge.elapsed > ONE_BRIDGE_ENTRY_STABLE_MS
            && one_bridge.distance >= ONE_BRIDGE_MIN_EXIT_DISTANCE_CM
            && abs_roll <= ONE_BRIDGE_EXIT_ROLL_DEG)
        {
            if (one_bridge.exit_stable_count < ONE_BRIDGE_EXIT_STABLE_MS)
            {
                one_bridge.exit_stable_count++;
            }
            if (one_bridge.exit_stable_count >= ONE_BRIDGE_EXIT_STABLE_MS)
            {
                one_bridge_enter_exiting();
            }
        }
        else
        {
            one_bridge.exit_stable_count = 0;
        }

        return;
    }

    if (one_bridge.state == ONE_BRIDGE_EXITING)
    {
        one_bridge.elapsed++;
        one_bridge.motor_adj = 0;
        one_bridge.leg_integral *= 0.92f;
        one_bridge.leg_offset = (int16)one_bridge.leg_integral;

        if (one_bridge_absf(one_bridge.leg_integral) < 2.0f
            || one_bridge.elapsed > 400)
        {
            one_bridge_finish_idle();
        }
    }
}

void one_bridge_abort(void)
{
    one_bridge.state              = ONE_BRIDGE_IDLE;
    one_bridge.side               = ONE_BRIDGE_SIDE_NONE;
    one_bridge.entry_stable_count = 0;
    one_bridge.exit_stable_count  = 0;
    one_bridge.elapsed            = 0;
    one_bridge.cooldown           = ONE_BRIDGE_COOLDOWN_MS;
    one_bridge.leg_integral       = 0.0f;
    one_bridge.leg_offset         = 0;
    one_bridge.motor_adj          = 0;
}

void one_bridge_set_auto(uint8 enable)
{
    one_bridge.auto_enable = enable ? 1 : 0;
    if (!one_bridge.auto_enable)
    {
        one_bridge_abort();
    }
}

uint8 one_bridge_is_active(void)
{
    return (one_bridge.state != ONE_BRIDGE_IDLE) ? 1 : 0;
}

void one_bridge_get_steer_offsets(int16 offsets[4])
{
    offsets[0] = 0;
    offsets[1] = 0;
    offsets[2] = 0;
    offsets[3] = 0;

    if (!one_bridge_is_active())
    {
        return;
    }

    offsets[0] = one_bridge.leg_offset;
    offsets[1] = -one_bridge.leg_offset;
    offsets[2] = one_bridge.leg_offset;
    offsets[3] = -one_bridge.leg_offset;
}

int16 one_bridge_get_motor_adj(void)
{
    return one_bridge_is_active() ? one_bridge.motor_adj : 0;
}

const char *one_bridge_state_name(uint8 state)
{
    switch (state)
    {
    case ONE_BRIDGE_IDLE:    return "IDLE";
    case ONE_BRIDGE_RUNNING: return "RUN";
    case ONE_BRIDGE_EXITING: return "EXIT";
    default:                 return "UNKNOWN";
    }
}

const char *one_bridge_side_name(uint8 side)
{
    switch (side)
    {
    case ONE_BRIDGE_SIDE_POS_ROLL: return "POS";
    case ONE_BRIDGE_SIDE_NEG_ROLL: return "NEG";
    default:                       return "NONE";
    }
}
