#include "Balance_ctrl.h"

#include <math.h>
#include <string.h>

#include "Pid.h"
#include "Inverted_pendulum.h"
#include "config.h"

#define BALANCE_DEG_TO_RAD  (0.017453292519943295f)
#define BALANCE_PI          (3.14159265358979323846f)

static pid_controller_t speed_pid;
static pid_controller_t pitch_pid;
static pid_controller_t pitch_rate_pid;
static pid_controller_t yaw_rate_pid;
static balance_command_t balance_target;
static balance_state_t balance_state;

uint8 balance_ctrl_config_is_valid(void)
{
#if !BALANCE_USE_STATE_FEEDBACK
    float pitch_response_gain;
    float rate_response_gain;
#endif

    if ((fabsf(fabsf(BALANCE_WHEEL_OUTPUT_DIRECTION) - 1.0f) > 0.0001f)
        || (BALANCE_RATE_KP <= 0.0f)
        || (BALANCE_MAX_PITCH_REFERENCE_RAD <= 0.0f)
        || (BALANCE_MAX_PITCH_RATE_RAD_S <= 0.0f)
        || (BALANCE_MAX_RATE_INTEGRAL < 0.0f)
        || (fabsf(BALANCE_PITCH_ZERO_RAD) >= 1.57079633f))
    {
        return 0U;
    }

#if !BALANCE_USE_STATE_FEEDBACK
    // With the verified wheel/IMU convention, pitch above the trim must produce
    // a negative logical wheel command, while positive pitch rate must produce
    // a positive damping command. Both coefficients are therefore positive.
    pitch_response_gain = BALANCE_WHEEL_OUTPUT_DIRECTION
        * BALANCE_RATE_KP * BALANCE_PITCH_KP;
    rate_response_gain = BALANCE_WHEEL_OUTPUT_DIRECTION
        * BALANCE_RATE_KP * (-(BALANCE_PITCH_KD + 1.0f));
    if ((pitch_response_gain <= 0.0f) || (rate_response_gain <= 0.0f))
    {
        return 0U;
    }
#endif
    return 1U;
}

static float balance_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static void balance_reset_controllers(void)
{
    pid_reset(&speed_pid);
    pid_reset(&pitch_pid);
    pid_reset(&pitch_rate_pid);
    pid_reset(&yaw_rate_pid);
    balance_state.pitch_reference_rad = BALANCE_PITCH_ZERO_RAD;
    balance_state.pitch_rate_reference_rad_s = 0.0f;
    balance_state.balance_command = 0.0f;
    balance_state.yaw_command = 0.0f;
    balance_state.left_wheel_command = 0;
    balance_state.right_wheel_command = 0;
    balance_state.leg_speed_control_active = 0U;
    balance_state.position_reference_m = balance_state.measured_position_m;
}

static void balance_mix_wheels(float common_command, float yaw_command)
{
    float common;
    float differential;
    float differential_limit;
    float left;
    float right;

    common = balance_clamp(common_command,
                           -(float)WHEEL_MAX_COMMAND,
                           (float)WHEEL_MAX_COMMAND);
    differential_limit = (float)WHEEL_MAX_COMMAND - fabsf(common);
    differential = balance_clamp(yaw_command,
                                 -differential_limit,
                                 differential_limit);
    left = common - differential;
    right = common + differential;
    balance_state.left_wheel_command = (int16)left;
    balance_state.right_wheel_command = (int16)right;
}

void balance_ctrl_init(void)
{
    memset(&balance_target, 0, sizeof(balance_target));
    memset(&balance_state, 0, sizeof(balance_state));

    pid_init(&speed_pid,
             BALANCE_SPEED_KP,
             BALANCE_SPEED_KI,
             0.0f,
             -BALANCE_MAX_PITCH_REFERENCE_RAD,
             BALANCE_MAX_PITCH_REFERENCE_RAD,
             -BALANCE_MAX_PITCH_REFERENCE_RAD,
             BALANCE_MAX_PITCH_REFERENCE_RAD);
    pid_init(&pitch_pid,
             BALANCE_PITCH_KP,
             BALANCE_PITCH_KI,
             BALANCE_PITCH_KD,
             -BALANCE_MAX_PITCH_RATE_RAD_S,
             BALANCE_MAX_PITCH_RATE_RAD_S,
             -BALANCE_MAX_PITCH_RATE_RAD_S,
             BALANCE_MAX_PITCH_RATE_RAD_S);
    pid_init(&pitch_rate_pid,
             BALANCE_RATE_KP,
             BALANCE_RATE_KI,
             0.0f,
             -BALANCE_MAX_RATE_INTEGRAL,
             BALANCE_MAX_RATE_INTEGRAL,
             -(float)WHEEL_MAX_COMMAND,
             (float)WHEEL_MAX_COMMAND);
    pid_init(&yaw_rate_pid,
             BALANCE_YAW_RATE_KP,
             BALANCE_YAW_RATE_KI,
             0.0f,
             -BALANCE_MAX_YAW_COMMAND,
             BALANCE_MAX_YAW_COMMAND,
             -BALANCE_MAX_YAW_COMMAND,
             BALANCE_MAX_YAW_COMMAND);
    balance_reset_controllers();
    if (!balance_ctrl_config_is_valid())
    {
        balance_state.fault_flags |= (uint32)BALANCE_FAULT_CONFIG;
    }
}

void balance_ctrl_set_command(const balance_command_t *command)
{
    if (0 != command)
    {
        balance_target = *command;
    }
}

const balance_command_t *balance_ctrl_get_command(void)
{
    return &balance_target;
}

uint8 balance_ctrl_set_enabled(uint8 enabled)
{
    if (!enabled)
    {
        balance_state.enabled = 0U;
        balance_reset_controllers();
        return 1U;
    }
    if (BALANCE_FAULT_NONE != balance_state.fault_flags)
    {
        return 0U;
    }

    balance_reset_controllers();
    balance_state.enabled = 1U;
    return 1U;
}

void balance_ctrl_clear_faults(void)
{
    if (!balance_state.enabled)
    {
        balance_state.fault_flags = balance_ctrl_config_is_valid()
            ? (uint32)BALANCE_FAULT_NONE
            : (uint32)BALANCE_FAULT_CONFIG;
    }
}

void balance_ctrl_force_fault(balance_fault_t fault)
{
    balance_state.fault_flags |= (uint32)fault;
    balance_state.enabled = 0U;
    balance_reset_controllers();
}

void balance_ctrl_update(const imu_data_t *imu,
                         const wheel_feedback_t *wheel,
                         uint8 run_speed_loop)
{
    float pitch_rad;
    float roll_rad;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;
#if !BALANCE_USE_STATE_FEEDBACK
    float speed_error;
    float pitch_error;
    float pitch_rate_error;
#endif
    float yaw_rate_error;
    float wheel_circumference_m;
    float left_speed_m_s;
    float right_speed_m_s;
#if BALANCE_USE_STATE_FEEDBACK
    pendulum_state_t pendulum_error;
    const pendulum_feedback_gain_t pendulum_gain =
    {
        PENDULUM_K_POSITION,
        PENDULUM_K_SPEED,
        PENDULUM_K_PITCH,
        PENDULUM_K_PITCH_RATE
    };
#endif

    if ((0 == imu) || (0 == wheel))
    {
        balance_ctrl_force_fault(BALANCE_FAULT_IMU);
        return;
    }

    pitch_rad = imu->pitch_deg * BALANCE_DEG_TO_RAD;
    roll_rad = imu->roll_deg * BALANCE_DEG_TO_RAD;
    pitch_rate_rad_s = imu->gyro_dps[1] * BALANCE_DEG_TO_RAD;
    yaw_rate_rad_s = imu->gyro_dps[2] * BALANCE_DEG_TO_RAD;
    wheel_circumference_m = BALANCE_PI * WHEEL_DIAMETER_M;
    left_speed_m_s = (float)wheel->left_rpm * wheel_circumference_m / 60.0f;
    right_speed_m_s = (float)wheel->right_rpm * wheel_circumference_m / 60.0f;
    balance_state.measured_speed_m_s =
        0.5f * (left_speed_m_s + right_speed_m_s);
    balance_state.measured_position_m += balance_state.measured_speed_m_s
                                         * CONTROL_FAST_PERIOD_S;

    if (!balance_state.enabled)
    {
        balance_reset_controllers();
        return;
    }
    if (fabsf(pitch_rad - BALANCE_PITCH_ZERO_RAD)
        > CONTROL_FALL_PITCH_ERROR_RAD)
    {
        balance_ctrl_force_fault(BALANCE_FAULT_PITCH_LIMIT);
        return;
    }
    if (fabsf(roll_rad) > CONTROL_FALL_ROLL_RAD)
    {
        balance_ctrl_force_fault(BALANCE_FAULT_ROLL_LIMIT);
        return;
    }

#if BALANCE_USE_STATE_FEEDBACK
    (void)run_speed_loop;
    balance_state.leg_speed_control_active
        = balance_target.use_leg_speed_control;
    if (balance_target.use_leg_speed_control)
    {
        balance_state.position_reference_m = balance_state.measured_position_m;
        pendulum_error.position_m = 0.0f;
        pendulum_error.speed_m_s = 0.0f;
    }
    else
    {
        balance_state.position_reference_m += balance_target.target_speed_m_s
                                              * CONTROL_FAST_PERIOD_S;
        pendulum_error.position_m = balance_state.measured_position_m
                                    - balance_state.position_reference_m;
        pendulum_error.speed_m_s = balance_state.measured_speed_m_s
                                   - balance_target.target_speed_m_s;
    }
    pendulum_error.pitch_rad = pitch_rad - BALANCE_PITCH_ZERO_RAD;
    pendulum_error.pitch_rate_rad_s = pitch_rate_rad_s;
    balance_state.balance_command = BALANCE_WHEEL_OUTPUT_DIRECTION
        * inverted_pendulum_state_feedback(&pendulum_error, &pendulum_gain);
    balance_state.pitch_reference_rad = BALANCE_PITCH_ZERO_RAD;
    balance_state.pitch_rate_reference_rad_s = 0.0f;
#else

    balance_state.leg_speed_control_active
        = balance_target.use_leg_speed_control;
    if (balance_target.use_leg_speed_control)
    {
        // The terrain controller owns the speed loop and moves the common leg
        // x target. Resetting here prevents the old speed integral from being
        // injected into pitch when normal control resumes.
        pid_reset(&speed_pid);
        balance_state.pitch_reference_rad = BALANCE_PITCH_ZERO_RAD;
    }
    else if (run_speed_loop)
    {
        speed_error = balance_target.target_speed_m_s
                      - balance_state.measured_speed_m_s;
        balance_state.pitch_reference_rad = BALANCE_PITCH_ZERO_RAD
            + pid_update(&speed_pid,
                         speed_error,
                         0.0f,
                         CONTROL_FAST_PERIOD_S
                         * (float)CONTROL_SPEED_INTERVAL_STEPS);
        balance_state.pitch_reference_rad = balance_clamp(
            balance_state.pitch_reference_rad,
            BALANCE_PITCH_ZERO_RAD - BALANCE_MAX_PITCH_REFERENCE_RAD,
            BALANCE_PITCH_ZERO_RAD + BALANCE_MAX_PITCH_REFERENCE_RAD);
    }

    pitch_error = balance_state.pitch_reference_rad - pitch_rad;
    balance_state.pitch_rate_reference_rad_s = pid_update(
        &pitch_pid,
        pitch_error,
        -pitch_rate_rad_s,
        CONTROL_FAST_PERIOD_S);
    pitch_rate_error = balance_state.pitch_rate_reference_rad_s
                       - pitch_rate_rad_s;
    balance_state.balance_command = BALANCE_WHEEL_OUTPUT_DIRECTION
        * pid_update(&pitch_rate_pid,
                     pitch_rate_error,
                     0.0f,
                     CONTROL_FAST_PERIOD_S);
#endif

    yaw_rate_error = balance_target.target_yaw_rate_rad_s - yaw_rate_rad_s;
    balance_state.yaw_command = pid_update(&yaw_rate_pid,
                                           yaw_rate_error,
                                           0.0f,
                                           CONTROL_FAST_PERIOD_S);
    balance_mix_wheels(balance_state.balance_command,
                       balance_state.yaw_command);
}

const balance_state_t *balance_ctrl_get_state(void)
{
    return &balance_state;
}
