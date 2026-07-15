#include "Control_system.h"

#include <math.h>
#include <string.h>

#include "Imu.h"
#include "Navigation.h"
#include "Wheel_driver.h"
#include "config.h"

static control_system_state_t control_state;
static uint32 last_wheel_frame_count;
static uint32 fast_divider;
static uint32 fast_step_count;
static balance_command_t requested_command;
static uint8 restore_balance_after_jump;

#define CONTROL_DEG_TO_RAD (0.017453292519943295f)

static void control_system_set_zero_command(void)
{
    memset(&requested_command, 0, sizeof(requested_command));
    balance_ctrl_set_command(&requested_command);
}

static uint8 control_system_try_start_stand(void)
{
    const balance_state_t *balance;
    const imu_data_t *imu;

    if (!control_state.stand_request_pending)
    {
        return 0U;
    }
    if (control_state.scheduler_tick_ms < CONTROL_STAND_ARM_DELAY_MS)
    {
        control_state.startup_state = CONTROL_STARTUP_WAITING_DELAY;
        return 0U;
    }
    if ((CONTROL_STATUS_OK != control_state.status)
        || wheel_driver_is_stop_locked()
        || jump_ctrl_is_active()
        || rotation_ctrl_is_active())
    {
        control_state.stand_request_pending = 0U;
        control_state.startup_state = CONTROL_STARTUP_FAULT;
        return 0U;
    }
    if (!control_state.wheel_feedback_ready
        || (control_state.wheel_feedback_age_ms
            > CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS))
    {
        control_state.startup_state =
            CONTROL_STARTUP_WAITING_WHEEL_FEEDBACK;
        wheel_driver_stop();
        return 0U;
    }

    imu = imu_get_data();
    if ((0 == imu)
        || (fabsf(imu->pitch_deg * CONTROL_DEG_TO_RAD)
            > CONTROL_STAND_ARM_MAX_PITCH_RAD)
        || (fabsf(imu->roll_deg * CONTROL_DEG_TO_RAD)
            > CONTROL_STAND_ARM_MAX_ROLL_RAD))
    {
        control_state.startup_state = CONTROL_STARTUP_WAITING_UPRIGHT;
        wheel_driver_stop();
        return 0U;
    }

    balance = balance_ctrl_get_state();
    if (BALANCE_FAULT_NONE != balance->fault_flags)
    {
        control_state.stand_request_pending = 0U;
        control_state.startup_state = CONTROL_STARTUP_FAULT;
        wheel_driver_stop();
        return 0U;
    }
    if (!balance->enabled && !balance_ctrl_set_enabled(1U))
    {
        control_state.stand_request_pending = 0U;
        control_state.startup_state = CONTROL_STARTUP_FAULT;
        wheel_driver_stop();
        return 0U;
    }

    control_state.stand_request_pending = 0U;
    control_state.balance_enabled = 1U;
    control_state.startup_state = CONTROL_STARTUP_STANDING;
    return 1U;
}

static void control_system_sync_rotation_state(void)
{
    const rotation_ctrl_state_t *rotation;
    uint32 result;

    rotation = rotation_ctrl_get_state();
    result = (uint32)rotation->result;
    if (((uint32)ROTATION_RESULT_TIMEOUT == result
         || (uint32)ROTATION_RESULT_FAULT == result)
        && (control_state.last_rotation_result != result))
    {
        control_state.rotation_error_count++;
    }
    control_state.last_rotation_status = (uint32)rotation->status;
    control_state.last_rotation_result = result;
    control_state.rotation_active = rotation_ctrl_is_active();
}

static void control_system_finish_jump(void)
{
    const jump_state_t *jump;

    jump = jump_ctrl_get_state();
    control_state.jump_active = 0U;
    control_state.last_jump_result = (uint32)jump->result;
    wheel_driver_stop();

    if (JUMP_RESULT_COMPLETED == jump->result)
    {
        wheel_driver_set_stop_lock(0U);
        if (restore_balance_after_jump
            && !balance_ctrl_set_enabled(1U))
        {
            control_state.status = CONTROL_STATUS_NOT_READY;
        }
        restore_balance_after_jump = 0U;
        return;
    }

    control_state.status = CONTROL_STATUS_JUMP_ERROR;
    control_state.jump_error_count++;
}

control_status_t control_system_init(void)
{
    imu_status_t imu_status;
    leg_ctrl_status_t leg_status;
    navigation_status_t navigation_status;
    bridge_ctrl_status_t bridge_status;
    bumpy_ctrl_status_t bumpy_status;
    rotation_ctrl_status_t rotation_status;

    memset(&control_state, 0, sizeof(control_state));
    control_state.startup_state = CONTROL_STARTUP_DISABLED;
    last_wheel_frame_count = 0U;
    fast_divider = 0U;
    fast_step_count = 0U;
    restore_balance_after_jump = 0U;
    memset(&requested_command, 0, sizeof(requested_command));
    balance_ctrl_init();
    wheel_driver_init();
    leg_status = leg_ctrl_init();
    if ((LEG_CTRL_STATUS_OK != leg_status)
        && (LEG_CTRL_STATUS_DISABLED != leg_status))
    {
        control_state.leg_error_count++;
    }
    bridge_status = bridge_ctrl_init();
    bumpy_status = bumpy_ctrl_init();
    rotation_status = rotation_ctrl_init();
    if (!leg_ctrl_is_ready() || !CONTROL_COMPETITION_MODULES_ON_BOOT)
    {
        bridge_status = bridge_ctrl_set_enabled(0U);
        bumpy_status = bumpy_ctrl_set_enabled(0U);
    }
    control_state.last_bridge_status = (uint32)bridge_status;
    control_state.last_bumpy_status = (uint32)bumpy_status;
    control_system_sync_rotation_state();
    if ((ROTATION_CTRL_STATUS_OK != rotation_status)
        && (ROTATION_CTRL_STATUS_DISABLED != rotation_status))
    {
        control_state.rotation_error_count++;
    }
    jump_ctrl_init();
    control_state.last_jump_result = (uint32)JUMP_RESULT_IDLE;

    imu_status = imu_init();
    control_state.last_imu_status = (uint32)imu_status;
    if (IMU_STATUS_OK != imu_status)
    {
        control_state.status = CONTROL_STATUS_IMU_ERROR;
        control_state.startup_state = CONTROL_STARTUP_FAULT;
        control_state.imu_error_count++;
        balance_ctrl_force_fault(BALANCE_FAULT_IMU);
        wheel_driver_stop();
        return control_state.status;
    }

    navigation_status = navigation_init(imu_get_data());
    control_state.last_navigation_status = (uint32)navigation_status;
    if (NAVIGATION_STATUS_OK != navigation_status)
    {
        control_state.navigation_error_count++;
    }

    control_state.status = CONTROL_STATUS_OK;
    control_system_set_zero_command();
    if (CONTROL_DEFAULT_STAND_ON_BOOT)
    {
        control_state.stand_request_pending = 1U;
        control_state.startup_state = CONTROL_STARTUP_WAITING_DELAY;
        wheel_driver_stop();
    }
    return control_state.status;
}

void control_system_tick_1ms(void)
{
    wheel_feedback_t wheel_feedback;
    const imu_data_t *imu;
    const balance_state_t *balance;
    const balance_command_t *command;
    const navigation_state_t *navigation;
    const bridge_ctrl_state_t *bridge;
    balance_command_t active_command;
    balance_command_t rotation_command;
    balance_command_t bridge_command;
    balance_command_t bumpy_command;
    imu_status_t imu_status;
    leg_ctrl_status_t leg_status;
    navigation_status_t navigation_status;
    bridge_ctrl_status_t bridge_status;
    bumpy_ctrl_status_t bumpy_status;
    uint8 run_speed_loop;
    uint8 wheel_feedback_valid;

    control_state.scheduler_tick_ms++;
    if (jump_ctrl_is_active())
    {
        // Redundant with the driver lock by design: refresh zero duty every
        // millisecond so a stale UART command cannot survive during a jump.
        wheel_driver_stop();
        jump_ctrl_tick_1ms();
    }
    fast_divider++;
    if (fast_divider < CONTROL_FAST_INTERVAL_TICKS)
    {
        return;
    }
    fast_divider = 0U;
    fast_step_count++;

    imu_status = imu_update();
    control_state.last_imu_status = (uint32)imu_status;
    if (IMU_STATUS_OK != imu_status)
    {
        if (rotation_ctrl_is_active())
        {
            (void)rotation_ctrl_abort();
            control_system_sync_rotation_state();
        }
        control_state.status = CONTROL_STATUS_IMU_ERROR;
        control_state.stand_request_pending = 0U;
        control_state.balance_enabled = 0U;
        control_state.startup_state = CONTROL_STARTUP_FAULT;
        control_state.imu_error_count++;
        balance_ctrl_force_fault(BALANCE_FAULT_IMU);
        wheel_driver_stop();
        return;
    }

    wheel_driver_get_feedback(&wheel_feedback);
    if (wheel_feedback.valid_frame_count != last_wheel_frame_count)
    {
        last_wheel_frame_count = wheel_feedback.valid_frame_count;
        control_state.wheel_feedback_age_ms = 0U;
        control_state.wheel_feedback_ready = 1U;
    }
    else
    {
        control_state.wheel_feedback_age_ms +=
            (uint32)(CONTROL_FAST_PERIOD_S * 1000.0f + 0.5f);
    }

    if ((fast_step_count % CONTROL_WHEEL_REQUEST_INTERVAL_STEPS) == 0U)
    {
        wheel_driver_request_speed();
    }

    (void)control_system_try_start_stand();

    balance = balance_ctrl_get_state();
    control_state.balance_enabled = balance->enabled;
    if (balance->enabled
        && (!control_state.wheel_feedback_ready
            || (control_state.wheel_feedback_age_ms
                > CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS)))
    {
        balance_ctrl_force_fault(BALANCE_FAULT_WHEEL_FEEDBACK);
    }
    if (rotation_ctrl_is_active() && !balance->enabled)
    {
        (void)rotation_ctrl_abort();
        control_system_sync_rotation_state();
    }

    imu = imu_get_data();
    wheel_feedback_valid = (uint8)(control_state.wheel_feedback_ready
        && (control_state.wheel_feedback_age_ms
            <= CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS));
    navigation_status = navigation_update(imu,
                                          &wheel_feedback,
                                          wheel_feedback_valid,
                                          CONTROL_FAST_PERIOD_S);
    control_state.last_navigation_status = (uint32)navigation_status;
    if (NAVIGATION_STATUS_OK != navigation_status)
    {
        control_state.navigation_error_count++;
    }

    if (wheel_driver_is_stop_locked())
    {
        wheel_driver_stop();
        if (control_state.jump_active && !jump_ctrl_is_active())
        {
            control_system_finish_jump();
        }
        return;
    }

    active_command = requested_command;
    navigation = navigation_get_state();
    if (NAVIGATION_ROUTE_CONTROL_ENABLE
        && (NAVIGATION_MODE_REPLAYING == navigation->mode))
    {
        active_command.target_yaw_rate_rad_s
            = navigation->target_yaw_rate_rad_s;
    }
    else if (NAVIGATION_ROUTE_CONTROL_ENABLE
             && (NAVIGATION_MODE_REPLAY_COMPLETE == navigation->mode))
    {
        active_command.target_speed_m_s = 0.0f;
        active_command.target_yaw_rate_rad_s = 0.0f;
    }
    run_speed_loop = (uint8)((fast_step_count
                              % CONTROL_SPEED_INTERVAL_STEPS) == 0U);
    rotation_command = active_command;
    bridge_command = active_command;
    bumpy_command = active_command;
    if (rotation_ctrl_is_active())
    {
        if (bridge_ctrl_is_active())
        {
            (void)bridge_ctrl_abort();
        }
        if (bumpy_ctrl_is_monitoring())
        {
            (void)bumpy_ctrl_abort();
        }
        (void)leg_ctrl_set_differential_z_offset(0.0f);
        (void)rotation_ctrl_update(imu,
                                   &active_command,
                                   &rotation_command);
        bridge_status = bridge_ctrl_get_state()->status;
        bumpy_status = bumpy_ctrl_get_state()->status;
    }
    else if (balance->enabled && leg_ctrl_is_ready())
    {
        if (bumpy_ctrl_is_active())
        {
            if (bridge_ctrl_is_active())
            {
                (void)bridge_ctrl_abort();
            }
            bridge_status = bridge_ctrl_get_state()->status;
        }
        else
        {
            bridge_status = bridge_ctrl_update(
                imu,
                navigation->traveled_distance_m,
                &active_command,
                &bridge_command);
        }

        if (bridge_ctrl_is_active())
        {
            if (bumpy_ctrl_is_monitoring())
            {
                (void)bumpy_ctrl_abort();
            }
            bumpy_status = bumpy_ctrl_get_state()->status;
        }
        else if ((BRIDGE_CTRL_STATUS_OK == bridge_status)
                 || (BRIDGE_CTRL_STATUS_DISABLED == bridge_status))
        {
            bumpy_status = bumpy_ctrl_update(
                imu,
                navigation->traveled_distance_m,
                navigation->forward_speed_m_s,
                run_speed_loop,
                &bridge_command,
                &bumpy_command);
        }
        else
        {
            bumpy_status = bumpy_ctrl_get_state()->status;
        }
    }
    else
    {
        if (bridge_ctrl_is_active())
        {
            (void)bridge_ctrl_abort();
        }
        if (bumpy_ctrl_is_monitoring())
        {
            (void)bumpy_ctrl_abort();
        }
        bridge_status = bridge_ctrl_get_state()->status;
        bumpy_status = bumpy_ctrl_get_state()->status;
    }
    control_state.last_bridge_status = (uint32)bridge_status;
    control_state.bridge_active = bridge_ctrl_is_active();
    control_state.last_bumpy_status = (uint32)bumpy_status;
    control_state.bumpy_active = bumpy_ctrl_is_active();
    control_system_sync_rotation_state();
    if ((BRIDGE_CTRL_STATUS_OK != bridge_status)
        && (BRIDGE_CTRL_STATUS_DISABLED != bridge_status))
    {
        control_state.bridge_error_count++;
        (void)bridge_ctrl_abort();
        control_state.bridge_active = 0U;
    }
    if ((BUMPY_CTRL_STATUS_OK != bumpy_status)
        && (BUMPY_CTRL_STATUS_DISABLED != bumpy_status))
    {
        control_state.bumpy_error_count++;
        (void)bumpy_ctrl_abort();
        control_state.bumpy_active = 0U;
    }
    if (control_state.rotation_active)
    {
        active_command = rotation_command;
    }
    else if (control_state.bridge_active)
    {
        active_command = bridge_command;
    }
    else if ((BUMPY_CTRL_STATUS_OK == bumpy_status)
             || (BUMPY_CTRL_STATUS_DISABLED == bumpy_status))
    {
        active_command = bumpy_command;
    }
    balance_ctrl_set_command(&active_command);

    balance_ctrl_update(imu, &wheel_feedback, run_speed_loop);
    balance = balance_ctrl_get_state();
    control_state.balance_enabled = balance->enabled;
    if (BALANCE_FAULT_NONE != balance->fault_flags)
    {
        control_state.stand_request_pending = 0U;
        control_state.startup_state = CONTROL_STARTUP_FAULT;
    }
    else if (balance->enabled)
    {
        control_state.startup_state = CONTROL_STARTUP_STANDING;
    }
    if (rotation_ctrl_is_active() && !balance->enabled)
    {
        (void)rotation_ctrl_abort();
        control_system_sync_rotation_state();
    }
    wheel_driver_set_command(balance->left_wheel_command,
                             balance->right_wheel_command);

    if ((fast_step_count % CONTROL_LEG_INTERVAL_STEPS) == 0U)
    {
        command = balance_ctrl_get_command();
        bridge = bridge_ctrl_get_state();
        leg_status = leg_ctrl_set_target_offset(
            command->target_leg_x_offset_m,
            command->target_leg_z_offset_m);
        if (LEG_CTRL_STATUS_OK == leg_status)
        {
            leg_status = leg_ctrl_set_differential_z_offset(
                bridge->differential_leg_offset_m);
        }
        if (balance->enabled && leg_ctrl_is_ready())
        {
            if (LEG_CTRL_STATUS_OK == leg_status)
            {
                leg_status = leg_ctrl_update(
                    imu->roll_deg * 0.017453292519943295f,
                    imu->gyro_dps[0] * 0.017453292519943295f);
            }
            if (LEG_CTRL_STATUS_OK != leg_status)
            {
                control_state.leg_error_count++;
                if (bridge_ctrl_is_active())
                {
                    (void)bridge_ctrl_abort();
                    control_state.bridge_active = 0U;
                    control_state.bridge_error_count++;
                }
                if (bumpy_ctrl_is_active())
                {
                    (void)bumpy_ctrl_abort();
                    control_state.bumpy_active = 0U;
                    control_state.bumpy_error_count++;
                }
            }
        }
    }
}

void control_system_set_command(const balance_command_t *command)
{
    if (0 != command)
    {
        requested_command = *command;
        // Terrain modules own this arbitration flag. Callers request speed and
        // leg pose, but cannot accidentally disable wheel speed stabilization.
        requested_command.use_leg_speed_control = 0U;
    }
}

uint8 control_system_request_stand(void)
{
    if ((CONTROL_STATUS_OK != control_state.status)
        || wheel_driver_is_stop_locked()
        || jump_ctrl_is_active())
    {
        return 0U;
    }

    (void)rotation_ctrl_abort();
    control_system_sync_rotation_state();
    control_state.last_bridge_status =
        (uint32)bridge_ctrl_set_enabled(0U);
    control_state.last_bumpy_status =
        (uint32)bumpy_ctrl_set_enabled(0U);
    (void)leg_ctrl_set_differential_z_offset(0.0f);
    control_state.bridge_active = 0U;
    control_state.bumpy_active = 0U;
    control_system_set_zero_command();
    control_state.stand_request_pending = 1U;
    if (control_state.scheduler_tick_ms < CONTROL_STAND_ARM_DELAY_MS)
    {
        control_state.startup_state = CONTROL_STARTUP_WAITING_DELAY;
    }
    else
    {
        control_state.startup_state =
            CONTROL_STARTUP_WAITING_WHEEL_FEEDBACK;
    }
    (void)control_system_try_start_stand();
    return 1U;
}

uint8 control_system_set_enabled(uint8 enabled)
{
    if (!enabled)
    {
        (void)rotation_ctrl_abort();
        control_system_sync_rotation_state();
        control_state.last_bridge_status =
            (uint32)bridge_ctrl_set_enabled(0U);
        control_state.last_bumpy_status =
            (uint32)bumpy_ctrl_set_enabled(0U);
        (void)leg_ctrl_set_differential_z_offset(0.0f);
        control_state.bridge_active = 0U;
        control_state.bumpy_active = 0U;
        control_state.stand_request_pending = 0U;
        control_state.balance_enabled = 0U;
        control_state.startup_state = CONTROL_STARTUP_DISABLED;
        (void)balance_ctrl_set_enabled(0U);
        wheel_driver_stop();
        return 1U;
    }
    return control_system_request_stand();
}

uint8 control_system_start_jump(void)
{
    const balance_state_t *balance;

    if ((CONTROL_STATUS_OK != control_state.status)
        || jump_ctrl_is_active()
        || rotation_ctrl_is_active()
        || bridge_ctrl_is_active()
        || bumpy_ctrl_is_active()
        || wheel_driver_is_stop_locked())
    {
        return 0U;
    }

    balance = balance_ctrl_get_state();
    (void)bridge_ctrl_abort();
    (void)bumpy_ctrl_abort();
    (void)leg_ctrl_set_differential_z_offset(0.0f);
    restore_balance_after_jump = balance->enabled;
    wheel_driver_set_stop_lock(1U);
    (void)balance_ctrl_set_enabled(0U);
    if (!jump_ctrl_start())
    {
        control_state.last_jump_result =
            (uint32)jump_ctrl_get_state()->result;
        wheel_driver_stop();
        wheel_driver_set_stop_lock(0U);
        if (restore_balance_after_jump)
        {
            (void)balance_ctrl_set_enabled(1U);
        }
        restore_balance_after_jump = 0U;
        return 0U;
    }

    control_state.jump_active = 1U;
    control_state.last_jump_result = (uint32)JUMP_RESULT_RUNNING;
    return 1U;
}

uint8 control_system_abort_jump(void)
{
    uint8 recovered;

    wheel_driver_set_stop_lock(1U);
    (void)balance_ctrl_set_enabled(0U);
    recovered = jump_ctrl_abort();
    control_state.jump_active = 0U;
    control_state.last_jump_result =
        (uint32)jump_ctrl_get_state()->result;
    wheel_driver_stop();
    if (!recovered)
    {
        control_state.status = CONTROL_STATUS_JUMP_ERROR;
        control_state.jump_error_count++;
        return 0U;
    }

    wheel_driver_set_stop_lock(0U);
    if (CONTROL_STATUS_JUMP_ERROR == control_state.status
        && imu_is_initialized())
    {
        control_state.status = CONTROL_STATUS_OK;
    }
    if (restore_balance_after_jump
        && !balance_ctrl_set_enabled(1U))
    {
        control_state.status = CONTROL_STATUS_NOT_READY;
        restore_balance_after_jump = 0U;
        return 0U;
    }
    restore_balance_after_jump = 0U;
    return 1U;
}

uint8 control_system_start_rotation(rotation_dir_t direction, float turns)
{
    const balance_state_t *balance;
    rotation_ctrl_status_t status;

    if ((CONTROL_STATUS_OK != control_state.status)
        || jump_ctrl_is_active()
        || rotation_ctrl_is_active()
        || bridge_ctrl_is_active()
        || bumpy_ctrl_is_active()
        || wheel_driver_is_stop_locked()
        || !control_state.wheel_feedback_ready
        || (control_state.wheel_feedback_age_ms
            > CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS))
    {
        return 0U;
    }
    balance = balance_ctrl_get_state();
    if (!balance->enabled)
    {
        return 0U;
    }

    status = rotation_ctrl_start(direction, turns, imu_get_data());
    control_system_sync_rotation_state();
    if (ROTATION_CTRL_STATUS_OK != status)
    {
        return 0U;
    }
    (void)bridge_ctrl_abort();
    (void)bumpy_ctrl_abort();
    (void)leg_ctrl_set_differential_z_offset(0.0f);
    return 1U;
}

uint8 control_system_abort_rotation(void)
{
    rotation_ctrl_status_t status;

    status = rotation_ctrl_abort();
    control_system_sync_rotation_state();
    return (uint8)((ROTATION_CTRL_STATUS_OK == status)
                   || (ROTATION_CTRL_STATUS_DISABLED == status));
}

uint8 control_system_release_rotation(void)
{
    rotation_ctrl_status_t status;

    status = rotation_ctrl_release();
    control_system_sync_rotation_state();
    return (uint8)(ROTATION_CTRL_STATUS_OK == status);
}

const rotation_ctrl_state_t *control_system_get_rotation_state(void)
{
    return rotation_ctrl_get_state();
}

uint8 control_system_set_bridge_enabled(uint8 enabled)
{
    bridge_ctrl_status_t status;

    if (enabled
        && (!leg_ctrl_is_ready()
            || jump_ctrl_is_active()
            || rotation_ctrl_is_active()
            || bumpy_ctrl_is_active()
            || wheel_driver_is_stop_locked()))
    {
        return 0U;
    }
    status = bridge_ctrl_set_enabled(enabled);
    control_state.last_bridge_status = (uint32)status;
    control_state.bridge_active = 0U;
    (void)leg_ctrl_set_differential_z_offset(0.0f);
    return (uint8)((BRIDGE_CTRL_STATUS_OK == status)
                   || (!enabled
                       && (BRIDGE_CTRL_STATUS_DISABLED == status)));
}

uint8 control_system_start_bridge(int8 roll_sign)
{
    const navigation_state_t *navigation;
    const balance_state_t *balance;
    bridge_ctrl_status_t status;

    if ((CONTROL_STATUS_OK != control_state.status)
        || !leg_ctrl_is_ready()
        || jump_ctrl_is_active()
        || rotation_ctrl_is_active()
        || bumpy_ctrl_is_active()
        || wheel_driver_is_stop_locked())
    {
        return 0U;
    }
    balance = balance_ctrl_get_state();
    if (!balance->enabled)
    {
        return 0U;
    }
    navigation = navigation_get_state();
    (void)bumpy_ctrl_abort();
    status = bridge_ctrl_force_enter(roll_sign,
                                     navigation->traveled_distance_m);
    control_state.last_bridge_status = (uint32)status;
    control_state.bridge_active = bridge_ctrl_is_active();
    return (uint8)(BRIDGE_CTRL_STATUS_OK == status);
}

uint8 control_system_abort_bridge(void)
{
    bridge_ctrl_status_t status;

    status = bridge_ctrl_abort();
    control_state.last_bridge_status = (uint32)status;
    control_state.bridge_active = 0U;
    (void)leg_ctrl_set_differential_z_offset(0.0f);
    return (uint8)((BRIDGE_CTRL_STATUS_OK == status)
                   || (BRIDGE_CTRL_STATUS_DISABLED == status));
}

uint8 control_system_set_bumpy_enabled(uint8 enabled)
{
    bumpy_ctrl_status_t status;

    if (enabled
        && (!leg_ctrl_is_ready()
            || jump_ctrl_is_active()
            || rotation_ctrl_is_active()
            || bridge_ctrl_is_active()
            || wheel_driver_is_stop_locked()))
    {
        return 0U;
    }
    status = bumpy_ctrl_set_enabled(enabled);
    control_state.last_bumpy_status = (uint32)status;
    control_state.bumpy_active = 0U;
    return (uint8)((BUMPY_CTRL_STATUS_OK == status)
                   || (!enabled
                       && (BUMPY_CTRL_STATUS_DISABLED == status)));
}

uint8 control_system_start_bumpy(void)
{
    const navigation_state_t *navigation;
    const balance_state_t *balance;
    bumpy_ctrl_status_t status;

    if ((CONTROL_STATUS_OK != control_state.status)
        || !leg_ctrl_is_ready()
        || jump_ctrl_is_active()
        || rotation_ctrl_is_active()
        || bridge_ctrl_is_active()
        || wheel_driver_is_stop_locked())
    {
        return 0U;
    }
    balance = balance_ctrl_get_state();
    if (!balance->enabled)
    {
        return 0U;
    }
    navigation = navigation_get_state();
    (void)bridge_ctrl_abort();
    status = bumpy_ctrl_force_enter(navigation->traveled_distance_m);
    control_state.last_bumpy_status = (uint32)status;
    control_state.bumpy_active = bumpy_ctrl_is_active();
    return (uint8)(BUMPY_CTRL_STATUS_OK == status);
}

uint8 control_system_abort_bumpy(void)
{
    bumpy_ctrl_status_t status;

    status = bumpy_ctrl_abort();
    control_state.last_bumpy_status = (uint32)status;
    control_state.bumpy_active = 0U;
    return (uint8)((BUMPY_CTRL_STATUS_OK == status)
                   || (BUMPY_CTRL_STATUS_DISABLED == status));
}

void control_system_clear_faults(void)
{
    (void)control_system_abort_rotation();
    (void)control_system_abort_bridge();
    (void)control_system_abort_bumpy();
    balance_ctrl_clear_faults();
    if (imu_is_initialized())
    {
        control_state.status = CONTROL_STATUS_OK;
    }
}

const control_system_state_t *control_system_get_state(void)
{
    return &control_state;
}
