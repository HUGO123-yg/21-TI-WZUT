#include "Rotation_ctrl.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "config.h"

#define ROTATION_DEG_TO_RAD  (0.017453292519943295f)

static rotation_ctrl_state_t rotation_state;

static uint8 rotation_float_is_finite(float value)
{
    return (uint8)((value == value)
                   && (value <= FLT_MAX)
                   && (value >= -FLT_MAX));
}

static float rotation_clamp(float value, float minimum, float maximum)
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

static float rotation_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float rotation_slew(float current, float target)
{
    const float maximum_step = ROTATION_MAX_YAW_ACCEL_RAD_S2
                               * CONTROL_FAST_PERIOD_S;

    if (target > current + maximum_step)
    {
        return current + maximum_step;
    }
    if (target < current - maximum_step)
    {
        return current - maximum_step;
    }
    return target;
}

static uint8 rotation_config_is_valid(void)
{
    return (uint8)((ROTATION_MAX_TURNS > 0.0f)
                   && ((1 == ROTATION_CW_YAW_SIGN)
                       || (-1 == ROTATION_CW_YAW_SIGN))
                   && (ROTATION_MIN_YAW_RATE_RAD_S >= 0.0f)
                   && (ROTATION_MAX_YAW_RATE_RAD_S
                       > ROTATION_MIN_YAW_RATE_RAD_S)
                   && (ROTATION_ANGLE_KP_RAD_S_PER_RAD > 0.0f)
                   && (ROTATION_MAX_YAW_ACCEL_RAD_S2 > 0.0f)
                   && (ROTATION_ANGLE_TOLERANCE_DEG > 0.0f)
                   && (ROTATION_ANGLE_TOLERANCE_DEG < 180.0f)
                   && (ROTATION_SETTLE_YAW_RATE_RAD_S >= 0.0f)
                   && (ROTATION_SETTLE_STEPS > 0U)
                   && (ROTATION_TIMEOUT_BASE_STEPS > 0U)
                   && (ROTATION_TIMEOUT_PER_TURN_STEPS > 0U));
}

static void rotation_shape_zero_speed(balance_command_t *shaped)
{
    shaped->target_speed_m_s = 0.0f;
    shaped->target_yaw_rate_rad_s = rotation_state.target_yaw_rate_rad_s;
    shaped->use_leg_speed_control = 0U;
}

static void rotation_enter_holding(rotation_result_t result)
{
    rotation_state.phase = ROTATION_PHASE_HOLDING;
    rotation_state.result = result;
    rotation_state.target_yaw_rate_rad_s = 0.0f;
    rotation_state.settle_steps = 0U;
}

rotation_ctrl_status_t rotation_ctrl_init(void)
{
    memset(&rotation_state, 0, sizeof(rotation_state));
    rotation_state.status = ROTATION_CTRL_STATUS_DISABLED;
    rotation_state.phase = ROTATION_PHASE_IDLE;
    rotation_state.result = ROTATION_RESULT_IDLE;
    if (!ROTATION_CONTROL_ENABLE)
    {
        return rotation_state.status;
    }
    if (!rotation_config_is_valid())
    {
        rotation_state.status = ROTATION_CTRL_STATUS_INVALID_CONFIG;
        return rotation_state.status;
    }

    rotation_state.enabled = 1U;
    rotation_state.status = ROTATION_CTRL_STATUS_OK;
    return rotation_state.status;
}

rotation_ctrl_status_t rotation_ctrl_start(rotation_dir_t direction,
                                           float turns,
                                           const imu_data_t *imu)
{
    if (!rotation_state.enabled)
    {
        return ROTATION_CTRL_STATUS_DISABLED;
    }
    if (rotation_ctrl_is_active())
    {
        return ROTATION_CTRL_STATUS_BUSY;
    }
    if ((0 == imu)
        || ((ROTATION_DIR_CW != direction)
            && (ROTATION_DIR_CCW != direction))
        || !rotation_float_is_finite(turns)
        || !rotation_float_is_finite(imu->yaw_deg)
        || (turns <= 0.0f)
        || (turns > ROTATION_MAX_TURNS))
    {
        rotation_state.status = ROTATION_CTRL_STATUS_INVALID_ARGUMENT;
        return rotation_state.status;
    }

    rotation_state.status = ROTATION_CTRL_STATUS_OK;
    rotation_state.phase = ROTATION_PHASE_RUNNING;
    rotation_state.result = ROTATION_RESULT_RUNNING;
    rotation_state.direction = direction;
    rotation_state.yaw_sign = (ROTATION_DIR_CW == direction)
        ? (int8)ROTATION_CW_YAW_SIGN
        : (int8)-ROTATION_CW_YAW_SIGN;
    rotation_state.elapsed_steps = 0U;
    rotation_state.timeout_steps = ROTATION_TIMEOUT_BASE_STEPS
        + (uint32)(turns * (float)ROTATION_TIMEOUT_PER_TURN_STEPS
                   + 0.5f);
    rotation_state.settle_steps = 0U;
    rotation_state.target_angle_deg = turns * 360.0f;
    rotation_state.progress_angle_deg = 0.0f;
    rotation_state.remaining_angle_deg = rotation_state.target_angle_deg;
    rotation_state.last_yaw_deg = imu->yaw_deg;
    rotation_state.target_yaw_rate_rad_s = 0.0f;
    rotation_state.measured_yaw_rate_rad_s = 0.0f;
    return rotation_state.status;
}

rotation_ctrl_status_t rotation_ctrl_update(
    const imu_data_t *imu,
    const balance_command_t *requested,
    balance_command_t *shaped)
{
    float yaw_delta_deg;
    float remaining_rad;
    float requested_rate_magnitude;
    float requested_rate;

    if ((0 == requested) || (0 == shaped))
    {
        rotation_state.status = ROTATION_CTRL_STATUS_INVALID_ARGUMENT;
        if (rotation_ctrl_is_active())
        {
            rotation_enter_holding(ROTATION_RESULT_FAULT);
        }
        return rotation_state.status;
    }
    *shaped = *requested;
    if (!rotation_ctrl_is_active())
    {
        return rotation_state.status;
    }

    rotation_shape_zero_speed(shaped);
    if ((0 == imu)
        || !rotation_float_is_finite(imu->yaw_deg)
        || !rotation_float_is_finite(imu->gyro_dps[2]))
    {
        rotation_state.status = ROTATION_CTRL_STATUS_INVALID_ARGUMENT;
        rotation_enter_holding(ROTATION_RESULT_FAULT);
        rotation_shape_zero_speed(shaped);
        return rotation_state.status;
    }

    rotation_state.measured_yaw_rate_rad_s
        = imu->gyro_dps[2] * ROTATION_DEG_TO_RAD;
    if (ROTATION_PHASE_HOLDING == rotation_state.phase)
    {
        return rotation_state.status;
    }

    rotation_state.elapsed_steps++;
    yaw_delta_deg = rotation_wrap_deg(imu->yaw_deg
                                      - rotation_state.last_yaw_deg);
    rotation_state.progress_angle_deg += (float)rotation_state.yaw_sign
                                         * yaw_delta_deg;
    rotation_state.remaining_angle_deg = rotation_state.target_angle_deg
                                         - rotation_state.progress_angle_deg;
    rotation_state.last_yaw_deg = imu->yaw_deg;

    if (rotation_state.elapsed_steps >= rotation_state.timeout_steps)
    {
        rotation_enter_holding(ROTATION_RESULT_TIMEOUT);
        rotation_shape_zero_speed(shaped);
        return rotation_state.status;
    }

    if (ROTATION_PHASE_RUNNING == rotation_state.phase)
    {
        if (fabsf(rotation_state.remaining_angle_deg)
            <= ROTATION_ANGLE_TOLERANCE_DEG)
        {
            rotation_state.phase = ROTATION_PHASE_SETTLING;
            rotation_state.settle_steps = 0U;
            rotation_state.target_yaw_rate_rad_s = 0.0f;
        }
        else
        {
            remaining_rad = fabsf(rotation_state.remaining_angle_deg)
                            * ROTATION_DEG_TO_RAD;
            requested_rate_magnitude = ROTATION_ANGLE_KP_RAD_S_PER_RAD
                                       * remaining_rad;
            requested_rate_magnitude = rotation_clamp(
                requested_rate_magnitude,
                ROTATION_MIN_YAW_RATE_RAD_S,
                ROTATION_MAX_YAW_RATE_RAD_S);
            requested_rate = (float)rotation_state.yaw_sign
                             * ((rotation_state.remaining_angle_deg >= 0.0f)
                                ? requested_rate_magnitude
                                : -requested_rate_magnitude);
            rotation_state.target_yaw_rate_rad_s = rotation_slew(
                rotation_state.target_yaw_rate_rad_s,
                requested_rate);
        }
    }
    else if (ROTATION_PHASE_SETTLING == rotation_state.phase)
    {
        rotation_state.target_yaw_rate_rad_s = 0.0f;
        if (fabsf(rotation_state.remaining_angle_deg)
            > ROTATION_ANGLE_TOLERANCE_DEG)
        {
            rotation_state.phase = ROTATION_PHASE_RUNNING;
            rotation_state.settle_steps = 0U;
        }
        else if (fabsf(rotation_state.measured_yaw_rate_rad_s)
            <= ROTATION_SETTLE_YAW_RATE_RAD_S)
        {
            rotation_state.settle_steps++;
            if (rotation_state.settle_steps >= ROTATION_SETTLE_STEPS)
            {
                rotation_enter_holding(ROTATION_RESULT_COMPLETED);
            }
        }
        else
        {
            rotation_state.settle_steps = 0U;
        }
    }

    rotation_shape_zero_speed(shaped);
    return rotation_state.status;
}

rotation_ctrl_status_t rotation_ctrl_abort(void)
{
    if (!rotation_state.enabled)
    {
        return ROTATION_CTRL_STATUS_DISABLED;
    }
    if (rotation_ctrl_is_active()
        || (ROTATION_RESULT_RUNNING == rotation_state.result))
    {
        rotation_state.result = ROTATION_RESULT_ABORTED;
    }
    rotation_state.phase = ROTATION_PHASE_IDLE;
    rotation_state.target_yaw_rate_rad_s = 0.0f;
    rotation_state.settle_steps = 0U;
    rotation_state.status = ROTATION_CTRL_STATUS_OK;
    return rotation_state.status;
}

rotation_ctrl_status_t rotation_ctrl_release(void)
{
    if (!rotation_state.enabled)
    {
        return ROTATION_CTRL_STATUS_DISABLED;
    }
    if (ROTATION_PHASE_HOLDING != rotation_state.phase)
    {
        return ROTATION_CTRL_STATUS_BUSY;
    }

    rotation_state.phase = ROTATION_PHASE_IDLE;
    rotation_state.target_yaw_rate_rad_s = 0.0f;
    rotation_state.status = ROTATION_CTRL_STATUS_OK;
    return rotation_state.status;
}

uint8 rotation_ctrl_is_active(void)
{
    return (uint8)(rotation_state.enabled
                   && (ROTATION_PHASE_IDLE != rotation_state.phase));
}

const rotation_ctrl_state_t *rotation_ctrl_get_state(void)
{
    return &rotation_state;
}
