#include "Bridge_ctrl.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "config.h"

#define BRIDGE_DEG_TO_RAD  (0.017453292519943295f)

static bridge_ctrl_state_t bridge_state;

static float bridge_clamp(float value, float minimum, float maximum)
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

static float bridge_slew(float current, float target)
{
    float delta;

    delta = bridge_clamp(target - current,
                         -BRIDGE_MAX_DIFFERENTIAL_STEP_M,
                         BRIDGE_MAX_DIFFERENTIAL_STEP_M);
    return current + delta;
}

static uint8 bridge_float_is_finite(float value)
{
    return (uint8)((value == value)
                   && (value <= FLT_MAX)
                   && (value >= -FLT_MAX));
}

static uint8 bridge_config_is_valid(void)
{
    return (uint8)((BRIDGE_ROLL_DETECT_RAD > BRIDGE_ROLL_RECOVER_RAD)
                   && (BRIDGE_ROLL_RECOVER_RAD >= 0.0f)
                   && (BRIDGE_DETECT_STEPS > 0U)
                   && (BRIDGE_ENTER_STEPS > 0U)
                   && (BRIDGE_EXIT_STEPS > 0U)
                   && (BRIDGE_RECOVER_STEPS > 0U)
                   && (BRIDGE_RECOVER_TIMEOUT_STEPS
                       >= BRIDGE_RECOVER_STEPS)
                   && (BRIDGE_CROSSING_TIMEOUT_STEPS > 0U)
                   && (BRIDGE_CROSSING_DISTANCE_M > 0.0f)
                   && (BRIDGE_LATERAL_SUPPORT_SPAN_M >= 0.0f)
                   && ((1.0f == BRIDGE_ROLL_TO_LEG_DIRECTION)
                       || (-1.0f == BRIDGE_ROLL_TO_LEG_DIRECTION))
                   && (BRIDGE_ROLL_KP_M_PER_RAD >= 0.0f)
                   && (BRIDGE_ROLL_KD_M_PER_RAD_S >= 0.0f)
                   && (BRIDGE_MAX_DIFFERENTIAL_OFFSET_M > 0.0f)
                   && (BRIDGE_MAX_DIFFERENTIAL_OFFSET_M
                       <= LEG_MAX_DIFFERENTIAL_Z_OFFSET_M)
                   && (BRIDGE_MAX_DIFFERENTIAL_STEP_M > 0.0f)
                   && (BRIDGE_BODY_Z_OFFSET_M >= 0.0f)
                   && (BRIDGE_MAX_SPEED_M_S > 0.0f)
                   && (BRIDGE_YAW_RATE_SCALE >= 0.0f)
                   && (BRIDGE_YAW_RATE_SCALE <= 1.0f)
                   && (BRIDGE_MAX_YAW_RATE_RAD_S > 0.0f));
}

static void bridge_enter_phase(bridge_phase_t phase)
{
    bridge_state.phase = phase;
    bridge_state.phase_elapsed_steps = 0U;
}

static void bridge_reset_runtime(void)
{
    bridge_state.phase = BRIDGE_PHASE_IDLE;
    bridge_state.phase_elapsed_steps = 0U;
    bridge_state.detect_count = 0U;
    bridge_state.recover_count = 0U;
    bridge_state.roll_sign = 0;
    bridge_state.forced_entry = 0U;
    bridge_state.entry_distance_m = 0.0f;
    bridge_state.crossing_distance_m = 0.0f;
    bridge_state.latched_feedforward_m = 0.0f;
    bridge_state.roll_feedback_m = 0.0f;
    bridge_state.differential_leg_offset_m = 0.0f;
    bridge_state.shaped_speed_m_s = 0.0f;
    bridge_state.shaped_yaw_rate_rad_s = 0.0f;
}

static float bridge_geometry_feedforward(float roll_abs_rad, int8 roll_sign)
{
    float offset;

    offset = 0.5f * BRIDGE_LATERAL_SUPPORT_SPAN_M
             * tanf(roll_abs_rad);
    offset *= BRIDGE_ROLL_TO_LEG_DIRECTION * (float)roll_sign;
    return bridge_clamp(offset,
                        -BRIDGE_MAX_DIFFERENTIAL_OFFSET_M,
                        BRIDGE_MAX_DIFFERENTIAL_OFFSET_M);
}

static void bridge_latch_feedforward(float roll_rad)
{
    float candidate;

    if ((bridge_state.roll_sign > 0 && roll_rad <= 0.0f)
        || (bridge_state.roll_sign < 0 && roll_rad >= 0.0f))
    {
        return;
    }
    candidate = bridge_geometry_feedforward(fabsf(roll_rad),
                                            bridge_state.roll_sign);
    if (fabsf(candidate) > fabsf(bridge_state.latched_feedforward_m))
    {
        bridge_state.latched_feedforward_m = candidate;
    }
}

static uint8 bridge_compensation_phase(void)
{
    return (uint8)((BRIDGE_PHASE_ENTERING == bridge_state.phase)
                   || (BRIDGE_PHASE_CROSSING == bridge_state.phase));
}

static uint8 bridge_command_shaping_phase(void)
{
    return (uint8)((BRIDGE_PHASE_ENTERING == bridge_state.phase)
                   || (BRIDGE_PHASE_CROSSING == bridge_state.phase)
                   || (BRIDGE_PHASE_EXITING == bridge_state.phase)
                   || (BRIDGE_PHASE_RECOVERING == bridge_state.phase));
}

static void bridge_shape_command(const balance_command_t *requested,
                                 balance_command_t *shaped)
{
    *shaped = *requested;
    if (!bridge_command_shaping_phase())
    {
        bridge_state.shaped_speed_m_s = shaped->target_speed_m_s;
        bridge_state.shaped_yaw_rate_rad_s
            = shaped->target_yaw_rate_rad_s;
        return;
    }

    shaped->target_speed_m_s = bridge_clamp(
        shaped->target_speed_m_s,
        -BRIDGE_MAX_SPEED_M_S,
        BRIDGE_MAX_SPEED_M_S);
    shaped->target_yaw_rate_rad_s = bridge_clamp(
        shaped->target_yaw_rate_rad_s * BRIDGE_YAW_RATE_SCALE,
        -BRIDGE_MAX_YAW_RATE_RAD_S,
        BRIDGE_MAX_YAW_RATE_RAD_S);
    if (shaped->target_leg_z_offset_m < BRIDGE_BODY_Z_OFFSET_M)
    {
        shaped->target_leg_z_offset_m = BRIDGE_BODY_Z_OFFSET_M;
    }
    bridge_state.shaped_speed_m_s = shaped->target_speed_m_s;
    bridge_state.shaped_yaw_rate_rad_s
        = shaped->target_yaw_rate_rad_s;
}

bridge_ctrl_status_t bridge_ctrl_init(void)
{
    memset(&bridge_state, 0, sizeof(bridge_state));
    bridge_reset_runtime();
    bridge_state.status = BRIDGE_CTRL_STATUS_DISABLED;
    bridge_state.result = BRIDGE_RESULT_IDLE;
    if (!BRIDGE_CONTROL_ENABLE)
    {
        return bridge_state.status;
    }
    return bridge_ctrl_set_enabled(1U);
}

bridge_ctrl_status_t bridge_ctrl_set_enabled(uint8 enabled)
{
    if (!enabled
        && (BRIDGE_PHASE_IDLE != bridge_state.phase)
        && (BRIDGE_RESULT_FAULT != bridge_state.result))
    {
        bridge_state.result = BRIDGE_RESULT_ABORTED;
    }
    bridge_reset_runtime();
    if (!enabled)
    {
        bridge_state.enabled = 0U;
        bridge_state.status = BRIDGE_CTRL_STATUS_DISABLED;
        return bridge_state.status;
    }
    if (!bridge_config_is_valid())
    {
        bridge_state.enabled = 0U;
        bridge_state.status = BRIDGE_CTRL_STATUS_INVALID_CONFIG;
        bridge_state.result = BRIDGE_RESULT_FAULT;
        return bridge_state.status;
    }

    bridge_state.enabled = 1U;
    bridge_state.status = BRIDGE_CTRL_STATUS_OK;
    bridge_state.result = BRIDGE_RESULT_IDLE;
    return bridge_state.status;
}

bridge_ctrl_status_t bridge_ctrl_force_enter(int8 roll_sign,
                                             float traveled_distance_m)
{
    if (!bridge_state.enabled)
    {
        return BRIDGE_CTRL_STATUS_DISABLED;
    }
    if (((1 != roll_sign) && (-1 != roll_sign))
        || !bridge_float_is_finite(traveled_distance_m))
    {
        bridge_state.status = BRIDGE_CTRL_STATUS_INVALID_ARGUMENT;
        bridge_state.result = BRIDGE_RESULT_FAULT;
        return bridge_state.status;
    }

    bridge_reset_runtime();
    bridge_state.enabled = 1U;
    bridge_state.status = BRIDGE_CTRL_STATUS_OK;
    bridge_state.result = BRIDGE_RESULT_RUNNING;
    bridge_state.roll_sign = roll_sign;
    bridge_state.forced_entry = 1U;
    bridge_state.detect_count = BRIDGE_DETECT_STEPS;
    bridge_state.entry_distance_m = traveled_distance_m;
    bridge_state.latched_feedforward_m = bridge_geometry_feedforward(
        BRIDGE_FORCED_ENTRY_ROLL_RAD,
        roll_sign);
    bridge_enter_phase(BRIDGE_PHASE_ENTERING);
    return bridge_state.status;
}

bridge_ctrl_status_t bridge_ctrl_abort(void)
{
    uint8 was_enabled;

    was_enabled = bridge_state.enabled;
    if ((BRIDGE_PHASE_IDLE != bridge_state.phase)
        && (BRIDGE_RESULT_FAULT != bridge_state.result))
    {
        bridge_state.result = BRIDGE_RESULT_ABORTED;
    }
    bridge_reset_runtime();
    bridge_state.enabled = was_enabled;
    bridge_state.status = was_enabled
        ? BRIDGE_CTRL_STATUS_OK
        : BRIDGE_CTRL_STATUS_DISABLED;
    return bridge_state.status;
}

bridge_ctrl_status_t bridge_ctrl_update(const imu_data_t *imu,
                                        float traveled_distance_m,
                                        const balance_command_t *requested,
                                        balance_command_t *shaped)
{
    float roll_rad;
    float roll_rate_rad_s;
    float target_differential_m;
    int8 current_roll_sign;

    if ((0 == requested) || (0 == shaped))
    {
        bridge_state.status = BRIDGE_CTRL_STATUS_INVALID_ARGUMENT;
        bridge_state.result = BRIDGE_RESULT_FAULT;
        return bridge_state.status;
    }
    *shaped = *requested;
    if (!bridge_state.enabled)
    {
        bridge_state.status = BRIDGE_CTRL_STATUS_DISABLED;
        return bridge_state.status;
    }
    if ((0 == imu)
        || !bridge_float_is_finite(traveled_distance_m)
        || !bridge_float_is_finite(imu->roll_deg)
        || !bridge_float_is_finite(imu->attitude_rate_dps[0]))
    {
        bridge_state.status = BRIDGE_CTRL_STATUS_INVALID_ARGUMENT;
        bridge_state.result = BRIDGE_RESULT_FAULT;
        return bridge_state.status;
    }

    roll_rad = imu->roll_deg * BRIDGE_DEG_TO_RAD;
    roll_rate_rad_s = imu->attitude_rate_dps[0] * BRIDGE_DEG_TO_RAD;
    current_roll_sign = (roll_rad >= 0.0f) ? 1 : -1;

    switch (bridge_state.phase)
    {
        case BRIDGE_PHASE_IDLE:
            if (BRIDGE_AUTO_DETECT_ENABLE
                && (fabsf(roll_rad) >= BRIDGE_ROLL_DETECT_RAD))
            {
                bridge_state.result = BRIDGE_RESULT_RUNNING;
                bridge_state.roll_sign = current_roll_sign;
                bridge_state.detect_count = 1U;
                bridge_latch_feedforward(roll_rad);
                bridge_enter_phase(BRIDGE_PHASE_DETECTING);
            }
            break;

        case BRIDGE_PHASE_DETECTING:
            bridge_state.phase_elapsed_steps++;
            if (fabsf(roll_rad) < BRIDGE_ROLL_DETECT_RAD)
            {
                bridge_reset_runtime();
                bridge_state.enabled = 1U;
                bridge_state.status = BRIDGE_CTRL_STATUS_OK;
                bridge_state.result = BRIDGE_RESULT_IDLE;
            }
            else
            {
                if (current_roll_sign != bridge_state.roll_sign)
                {
                    bridge_state.roll_sign = current_roll_sign;
                    bridge_state.detect_count = 1U;
                    bridge_state.latched_feedforward_m = 0.0f;
                }
                else
                {
                    bridge_state.detect_count++;
                }
                bridge_latch_feedforward(roll_rad);
                if (bridge_state.detect_count >= BRIDGE_DETECT_STEPS)
                {
                    bridge_state.entry_distance_m = traveled_distance_m;
                    bridge_enter_phase(BRIDGE_PHASE_ENTERING);
                }
            }
            break;

        case BRIDGE_PHASE_ENTERING:
            bridge_state.phase_elapsed_steps++;
            bridge_latch_feedforward(roll_rad);
            if (bridge_state.phase_elapsed_steps >= BRIDGE_ENTER_STEPS)
            {
                bridge_enter_phase(BRIDGE_PHASE_CROSSING);
            }
            break;

        case BRIDGE_PHASE_CROSSING:
            bridge_state.phase_elapsed_steps++;
            bridge_latch_feedforward(roll_rad);
            bridge_state.crossing_distance_m = fabsf(
                traveled_distance_m - bridge_state.entry_distance_m);
            if (bridge_state.crossing_distance_m
                >= BRIDGE_CROSSING_DISTANCE_M)
            {
                bridge_enter_phase(BRIDGE_PHASE_EXITING);
            }
            else if (bridge_state.phase_elapsed_steps
                     >= BRIDGE_CROSSING_TIMEOUT_STEPS)
            {
                bridge_state.result = BRIDGE_RESULT_TIMEOUT;
                bridge_enter_phase(BRIDGE_PHASE_EXITING);
            }
            break;

        case BRIDGE_PHASE_EXITING:
            bridge_state.phase_elapsed_steps++;
            if (bridge_state.phase_elapsed_steps >= BRIDGE_EXIT_STEPS)
            {
                bridge_state.recover_count = 0U;
                bridge_enter_phase(BRIDGE_PHASE_RECOVERING);
            }
            break;

        case BRIDGE_PHASE_RECOVERING:
            bridge_state.phase_elapsed_steps++;
            if (fabsf(roll_rad) <= BRIDGE_ROLL_RECOVER_RAD)
            {
                bridge_state.recover_count++;
            }
            else
            {
                bridge_state.recover_count = 0U;
            }
            if ((bridge_state.recover_count >= BRIDGE_RECOVER_STEPS)
                && (fabsf(bridge_state.differential_leg_offset_m)
                    <= BRIDGE_MAX_DIFFERENTIAL_STEP_M))
            {
                if (BRIDGE_RESULT_RUNNING == bridge_state.result)
                {
                    bridge_state.result = BRIDGE_RESULT_COMPLETED;
                }
                bridge_reset_runtime();
                bridge_state.enabled = 1U;
                bridge_state.status = BRIDGE_CTRL_STATUS_OK;
            }
            else if (bridge_state.phase_elapsed_steps
                     >= BRIDGE_RECOVER_TIMEOUT_STEPS)
            {
                bridge_state.result = BRIDGE_RESULT_TIMEOUT;
                bridge_reset_runtime();
                bridge_state.enabled = 1U;
                bridge_state.status = BRIDGE_CTRL_STATUS_OK;
            }
            break;

        default:
            bridge_state.status = BRIDGE_CTRL_STATUS_INVALID_CONFIG;
            bridge_state.result = BRIDGE_RESULT_FAULT;
            return bridge_state.status;
    }

    target_differential_m = 0.0f;
    bridge_state.roll_feedback_m = 0.0f;
    if (bridge_compensation_phase())
    {
        bridge_state.roll_feedback_m = BRIDGE_ROLL_TO_LEG_DIRECTION
            * (BRIDGE_ROLL_KP_M_PER_RAD * roll_rad
               + BRIDGE_ROLL_KD_M_PER_RAD_S * roll_rate_rad_s);
        target_differential_m = bridge_clamp(
            bridge_state.latched_feedforward_m
            + bridge_state.roll_feedback_m,
            -BRIDGE_MAX_DIFFERENTIAL_OFFSET_M,
            BRIDGE_MAX_DIFFERENTIAL_OFFSET_M);
    }
    bridge_state.differential_leg_offset_m = bridge_slew(
        bridge_state.differential_leg_offset_m,
        target_differential_m);
    bridge_shape_command(requested, shaped);
    bridge_state.status = BRIDGE_CTRL_STATUS_OK;
    return bridge_state.status;
}

uint8 bridge_ctrl_is_active(void)
{
    return (uint8)(bridge_state.enabled
                   && (BRIDGE_PHASE_IDLE != bridge_state.phase));
}

const bridge_ctrl_state_t *bridge_ctrl_get_state(void)
{
    return &bridge_state;
}
