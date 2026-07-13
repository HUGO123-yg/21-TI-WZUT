#include "Bumpy_ctrl.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "Pid.h"
#include "config.h"

static bumpy_ctrl_state_t bumpy_state;
static pid_controller_t bumpy_speed_pid;

static float bumpy_clamp(float value, float minimum, float maximum)
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

static float bumpy_slew(float current, float target)
{
    return current + bumpy_clamp(target - current,
                                 -BUMPY_MAX_LEG_X_STEP_M,
                                 BUMPY_MAX_LEG_X_STEP_M);
}

static uint8 bumpy_float_is_finite(float value)
{
    return (uint8)((value == value)
                   && (value <= FLT_MAX)
                   && (value >= -FLT_MAX));
}

static uint8 bumpy_config_is_valid(void)
{
    return (uint8)((BUMPY_IMPACT_DETECT_DELTA_G
                    > BUMPY_IMPACT_RELEASE_DELTA_G)
                   && (BUMPY_IMPACT_RELEASE_DELTA_G
                       >= BUMPY_STABLE_DELTA_G)
                   && (BUMPY_STABLE_DELTA_G >= 0.0f)
                   && (BUMPY_IMPACT_REQUIRED_COUNT >= 2U)
                   && (BUMPY_IMPACT_REFRACTORY_STEPS > 0U)
                   && (BUMPY_DETECT_WINDOW_STEPS
                       > BUMPY_IMPACT_REFRACTORY_STEPS)
                   && (BUMPY_CROSSING_DISTANCE_M > 0.0f)
                   && (BUMPY_CROSSING_TIMEOUT_STEPS > 0U)
                   && (BUMPY_RECOVER_STABLE_STEPS > 0U)
                   && (BUMPY_RECOVER_TIMEOUT_STEPS
                       >= BUMPY_RECOVER_STABLE_STEPS)
                   && (BUMPY_SPEED_KP_M_PER_M_S >= 0.0f)
                   && (BUMPY_SPEED_KI_M_PER_M >= 0.0f)
                   && ((1.0f == BUMPY_SPEED_TO_LEG_DIRECTION)
                       || (-1.0f == BUMPY_SPEED_TO_LEG_DIRECTION))
                   && (BUMPY_MAX_SPEED_LEG_X_OFFSET_M > 0.0f)
                   && (BUMPY_MAX_TOTAL_LEG_X_OFFSET_M
                       >= BUMPY_MAX_SPEED_LEG_X_OFFSET_M)
                   && (BUMPY_MAX_LEG_X_STEP_M > 0.0f)
                   && (BUMPY_BODY_Z_OFFSET_M >= 0.0f)
                   && (BUMPY_BODY_Z_OFFSET_M <= LEG_MAX_ABSOLUTE_Z_M)
                   && (BUMPY_MAX_SPEED_M_S > 0.0f)
                   && (BUMPY_YAW_RATE_SCALE >= 0.0f)
                   && (BUMPY_YAW_RATE_SCALE <= 1.0f)
                   && (BUMPY_MAX_YAW_RATE_RAD_S > 0.0f));
}

static void bumpy_enter_phase(bumpy_phase_t phase)
{
    bumpy_state.phase = phase;
    bumpy_state.phase_elapsed_steps = 0U;
}

static void bumpy_reset_runtime(void)
{
    bumpy_state.phase = BUMPY_PHASE_IDLE;
    bumpy_state.phase_elapsed_steps = 0U;
    bumpy_state.impact_count = 0U;
    bumpy_state.impact_refractory_steps = 0U;
    bumpy_state.recover_stable_steps = 0U;
    bumpy_state.forced_entry = 0U;
    bumpy_state.impact_latched = 0U;
    bumpy_state.entry_distance_m = 0.0f;
    bumpy_state.crossing_distance_m = 0.0f;
    bumpy_state.impact_magnitude_g = 0.0f;
    bumpy_state.speed_error_m_s = 0.0f;
    bumpy_state.speed_leg_x_offset_m = 0.0f;
    bumpy_state.shaped_speed_m_s = 0.0f;
    bumpy_state.shaped_yaw_rate_rad_s = 0.0f;
    pid_reset(&bumpy_speed_pid);
}

static uint8 bumpy_new_impact(float acceleration_norm_g)
{
    uint8 new_impact;

    new_impact = 0U;
    bumpy_state.impact_magnitude_g = fabsf(acceleration_norm_g - 1.0f);
    if (bumpy_state.impact_refractory_steps > 0U)
    {
        bumpy_state.impact_refractory_steps--;
    }
    if (bumpy_state.impact_latched)
    {
        if (bumpy_state.impact_magnitude_g
            <= BUMPY_IMPACT_RELEASE_DELTA_G)
        {
            bumpy_state.impact_latched = 0U;
        }
    }
    else if ((0U == bumpy_state.impact_refractory_steps)
             && (bumpy_state.impact_magnitude_g
                 >= BUMPY_IMPACT_DETECT_DELTA_G))
    {
        bumpy_state.impact_latched = 1U;
        bumpy_state.impact_refractory_steps
            = BUMPY_IMPACT_REFRACTORY_STEPS;
        new_impact = 1U;
    }
    return new_impact;
}

static void bumpy_begin_crossing(float traveled_distance_m,
                                 uint8 forced_entry)
{
    bumpy_state.entry_distance_m = traveled_distance_m;
    bumpy_state.crossing_distance_m = 0.0f;
    bumpy_state.forced_entry = forced_entry;
    bumpy_state.recover_stable_steps = 0U;
    pid_reset(&bumpy_speed_pid);
    bumpy_enter_phase(BUMPY_PHASE_CROSSING);
}

static void bumpy_shape_command(const balance_command_t *requested,
                                balance_command_t *shaped,
                                float measured_speed_m_s,
                                uint8 run_speed_loop)
{
    float target_x_offset_m;

    *shaped = *requested;
    if (!bumpy_ctrl_is_active())
    {
        bumpy_state.shaped_speed_m_s = shaped->target_speed_m_s;
        bumpy_state.shaped_yaw_rate_rad_s
            = shaped->target_yaw_rate_rad_s;
        return;
    }

    shaped->target_speed_m_s = bumpy_clamp(shaped->target_speed_m_s,
                                            -BUMPY_MAX_SPEED_M_S,
                                            BUMPY_MAX_SPEED_M_S);
    shaped->target_yaw_rate_rad_s = bumpy_clamp(
        shaped->target_yaw_rate_rad_s * BUMPY_YAW_RATE_SCALE,
        -BUMPY_MAX_YAW_RATE_RAD_S,
        BUMPY_MAX_YAW_RATE_RAD_S);
    shaped->use_leg_speed_control = 1U;
    if (shaped->target_leg_z_offset_m < BUMPY_BODY_Z_OFFSET_M)
    {
        shaped->target_leg_z_offset_m = BUMPY_BODY_Z_OFFSET_M;
    }

    target_x_offset_m = 0.0f;
    if (BUMPY_PHASE_CROSSING == bumpy_state.phase)
    {
        bumpy_state.speed_error_m_s = shaped->target_speed_m_s
                                      - measured_speed_m_s;
        if (run_speed_loop)
        {
            target_x_offset_m = BUMPY_SPEED_TO_LEG_DIRECTION
                * pid_update(&bumpy_speed_pid,
                             bumpy_state.speed_error_m_s,
                             0.0f,
                             CONTROL_FAST_PERIOD_S
                             * (float)CONTROL_SPEED_INTERVAL_STEPS);
            bumpy_state.speed_leg_x_offset_m = bumpy_slew(
                bumpy_state.speed_leg_x_offset_m,
                target_x_offset_m);
        }
    }
    else
    {
        bumpy_state.speed_error_m_s = 0.0f;
        bumpy_state.speed_leg_x_offset_m = bumpy_slew(
            bumpy_state.speed_leg_x_offset_m,
            0.0f);
    }

    shaped->target_leg_x_offset_m = bumpy_clamp(
        requested->target_leg_x_offset_m
        + bumpy_state.speed_leg_x_offset_m,
        -BUMPY_MAX_TOTAL_LEG_X_OFFSET_M,
        BUMPY_MAX_TOTAL_LEG_X_OFFSET_M);
    bumpy_state.shaped_speed_m_s = shaped->target_speed_m_s;
    bumpy_state.shaped_yaw_rate_rad_s
        = shaped->target_yaw_rate_rad_s;
}

bumpy_ctrl_status_t bumpy_ctrl_init(void)
{
    memset(&bumpy_state, 0, sizeof(bumpy_state));
    pid_init(&bumpy_speed_pid,
             BUMPY_SPEED_KP_M_PER_M_S,
             BUMPY_SPEED_KI_M_PER_M,
             0.0f,
             -BUMPY_MAX_SPEED_LEG_X_OFFSET_M,
             BUMPY_MAX_SPEED_LEG_X_OFFSET_M,
             -BUMPY_MAX_SPEED_LEG_X_OFFSET_M,
             BUMPY_MAX_SPEED_LEG_X_OFFSET_M);
    bumpy_reset_runtime();
    bumpy_state.status = BUMPY_CTRL_STATUS_DISABLED;
    if (!BUMPY_CONTROL_ENABLE)
    {
        return bumpy_state.status;
    }
    return bumpy_ctrl_set_enabled(1U);
}

bumpy_ctrl_status_t bumpy_ctrl_set_enabled(uint8 enabled)
{
    bumpy_reset_runtime();
    if (!enabled)
    {
        bumpy_state.enabled = 0U;
        bumpy_state.status = BUMPY_CTRL_STATUS_DISABLED;
        return bumpy_state.status;
    }
    if (!bumpy_config_is_valid())
    {
        bumpy_state.enabled = 0U;
        bumpy_state.status = BUMPY_CTRL_STATUS_INVALID_CONFIG;
        return bumpy_state.status;
    }

    bumpy_state.enabled = 1U;
    bumpy_state.status = BUMPY_CTRL_STATUS_OK;
    return bumpy_state.status;
}

bumpy_ctrl_status_t bumpy_ctrl_force_enter(float traveled_distance_m)
{
    if (!bumpy_state.enabled)
    {
        return BUMPY_CTRL_STATUS_DISABLED;
    }
    if (!bumpy_float_is_finite(traveled_distance_m))
    {
        bumpy_state.status = BUMPY_CTRL_STATUS_INVALID_ARGUMENT;
        return bumpy_state.status;
    }

    bumpy_reset_runtime();
    bumpy_state.enabled = 1U;
    bumpy_state.status = BUMPY_CTRL_STATUS_OK;
    bumpy_begin_crossing(traveled_distance_m, 1U);
    return bumpy_state.status;
}

bumpy_ctrl_status_t bumpy_ctrl_abort(void)
{
    uint8 was_enabled;

    was_enabled = bumpy_state.enabled;
    bumpy_reset_runtime();
    bumpy_state.enabled = was_enabled;
    bumpy_state.status = was_enabled
        ? BUMPY_CTRL_STATUS_OK
        : BUMPY_CTRL_STATUS_DISABLED;
    return bumpy_state.status;
}

bumpy_ctrl_status_t bumpy_ctrl_update(const imu_data_t *imu,
                                      float traveled_distance_m,
                                      float measured_speed_m_s,
                                      uint8 run_speed_loop,
                                      const balance_command_t *requested,
                                      balance_command_t *shaped)
{
    uint8 new_impact;

    if ((0 == requested) || (0 == shaped))
    {
        bumpy_state.status = BUMPY_CTRL_STATUS_INVALID_ARGUMENT;
        return bumpy_state.status;
    }
    *shaped = *requested;
    if (!bumpy_state.enabled)
    {
        bumpy_state.status = BUMPY_CTRL_STATUS_DISABLED;
        return bumpy_state.status;
    }
    if ((0 == imu)
        || !bumpy_float_is_finite(traveled_distance_m)
        || !bumpy_float_is_finite(measured_speed_m_s)
        || !bumpy_float_is_finite(imu->acc_norm_g))
    {
        bumpy_state.status = BUMPY_CTRL_STATUS_INVALID_ARGUMENT;
        return bumpy_state.status;
    }

    new_impact = bumpy_new_impact(imu->acc_norm_g);
    switch (bumpy_state.phase)
    {
        case BUMPY_PHASE_IDLE:
            if (BUMPY_AUTO_DETECT_ENABLE && new_impact)
            {
                bumpy_state.impact_count = 1U;
                bumpy_enter_phase(BUMPY_PHASE_DETECTING);
            }
            break;

        case BUMPY_PHASE_DETECTING:
            bumpy_state.phase_elapsed_steps++;
            if (new_impact)
            {
                bumpy_state.impact_count++;
                if (bumpy_state.impact_count
                    >= BUMPY_IMPACT_REQUIRED_COUNT)
                {
                    bumpy_begin_crossing(traveled_distance_m, 0U);
                }
            }
            else if (bumpy_state.phase_elapsed_steps
                     >= BUMPY_DETECT_WINDOW_STEPS)
            {
                bumpy_reset_runtime();
                bumpy_state.enabled = 1U;
            }
            break;

        case BUMPY_PHASE_CROSSING:
            bumpy_state.phase_elapsed_steps++;
            bumpy_state.crossing_distance_m = fabsf(
                traveled_distance_m - bumpy_state.entry_distance_m);
            if ((bumpy_state.crossing_distance_m
                 >= BUMPY_CROSSING_DISTANCE_M)
                || (bumpy_state.phase_elapsed_steps
                    >= BUMPY_CROSSING_TIMEOUT_STEPS))
            {
                pid_reset(&bumpy_speed_pid);
                bumpy_state.recover_stable_steps = 0U;
                bumpy_enter_phase(BUMPY_PHASE_RECOVERING);
            }
            break;

        case BUMPY_PHASE_RECOVERING:
            bumpy_state.phase_elapsed_steps++;
            if (bumpy_state.impact_magnitude_g <= BUMPY_STABLE_DELTA_G)
            {
                bumpy_state.recover_stable_steps++;
            }
            else
            {
                bumpy_state.recover_stable_steps = 0U;
            }
            if (((bumpy_state.recover_stable_steps
                  >= BUMPY_RECOVER_STABLE_STEPS)
                 && (fabsf(bumpy_state.speed_leg_x_offset_m)
                     <= BUMPY_MAX_LEG_X_STEP_M))
                || (bumpy_state.phase_elapsed_steps
                    >= BUMPY_RECOVER_TIMEOUT_STEPS))
            {
                bumpy_reset_runtime();
                bumpy_state.enabled = 1U;
            }
            break;

        default:
            bumpy_state.status = BUMPY_CTRL_STATUS_INVALID_CONFIG;
            return bumpy_state.status;
    }

    bumpy_shape_command(requested,
                        shaped,
                        measured_speed_m_s,
                        run_speed_loop);
    bumpy_state.status = BUMPY_CTRL_STATUS_OK;
    return bumpy_state.status;
}

uint8 bumpy_ctrl_is_active(void)
{
    return (uint8)(bumpy_state.enabled
                   && ((BUMPY_PHASE_CROSSING == bumpy_state.phase)
                       || (BUMPY_PHASE_RECOVERING
                           == bumpy_state.phase)));
}

uint8 bumpy_ctrl_is_monitoring(void)
{
    return (uint8)(bumpy_state.enabled
                   && (BUMPY_PHASE_IDLE != bumpy_state.phase));
}

const bumpy_ctrl_state_t *bumpy_ctrl_get_state(void)
{
    return &bumpy_state;
}
