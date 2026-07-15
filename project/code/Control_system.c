#include "Control_system.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "syslib/cy_syslib.h"
#include "Imu.h"
#include "Navigation.h"
#include "Wheel_driver.h"
#include "config.h"

static control_system_state_t control_state;
static uint32 last_wheel_frame_count;
static uint32 fast_divider;
static uint32 fast_step_count;
static volatile control_drive_command_t requested_drive_command;
static volatile uint32 requested_drive_command_sequence;
static uint8 restore_balance_after_jump;

#define CONTROL_DEG_TO_RAD (0.017453292519943295f)

static uint8 control_system_float_is_finite(float value)
{
    return (uint8)((value == value)
                   && (value <= FLT_MAX)
                   && (value >= -FLT_MAX));
}

static uint8 control_system_drive_command_is_valid(
    const control_drive_command_t *command)
{
    if (0 == command)
    {
        return 0U;
    }

    return (uint8)(control_system_float_is_finite(
                       command->target_speed_m_s)
                   && control_system_float_is_finite(
                       command->target_yaw_rate_rad_s)
                   && control_system_float_is_finite(
                       command->target_leg_x_offset_m)
                   && control_system_float_is_finite(
                       command->target_leg_z_offset_m));
}

static void control_system_store_drive_command(
    const control_drive_command_t *command)
{
    uint32 interrupt_state;
    uint32 next_sequence;

    interrupt_state = Cy_SysLib_EnterCriticalSection();
    requested_drive_command = *command;
    next_sequence = requested_drive_command_sequence + 1U;
    if (0U == next_sequence)
    {
        next_sequence = 1U;
    }
    requested_drive_command_sequence = next_sequence;
    control_state.drive_command_sequence = next_sequence;
    Cy_SysLib_ExitCriticalSection(interrupt_state);
}

static void control_system_snapshot_drive_command(
    control_drive_command_t *command,
    uint32 *sequence)
{
    uint32 interrupt_state;

    interrupt_state = Cy_SysLib_EnterCriticalSection();
    *command = requested_drive_command;
    *sequence = requested_drive_command_sequence;
    Cy_SysLib_ExitCriticalSection(interrupt_state);
}

static void control_system_make_balance_command(
    const control_drive_command_t *drive,
    balance_command_t *balance)
{
    balance->target_speed_m_s = drive->target_speed_m_s;
    balance->target_yaw_rate_rad_s = drive->target_yaw_rate_rad_s;
    balance->target_leg_x_offset_m = drive->target_leg_x_offset_m;
    balance->target_leg_z_offset_m = drive->target_leg_z_offset_m;
    balance->use_leg_speed_control = 0U;
}

static void control_system_set_zero_command(void)
{
    control_drive_command_t drive_command;
    balance_command_t balance_command;

    memset(&drive_command, 0, sizeof(drive_command));
    memset(&balance_command, 0, sizeof(balance_command));
    control_system_store_drive_command(&drive_command);
    balance_ctrl_set_command(&balance_command);
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
        || (CONTROL_FAULT_NONE != control_state.fault_flags)
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
        || (fabsf(imu->pitch_deg * CONTROL_DEG_TO_RAD
                  - BALANCE_PITCH_ZERO_RAD)
            > CONTROL_STAND_ARM_MAX_PITCH_ERROR_RAD)
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

static void control_system_cancel_active_actions(void)
{
    jump_ctrl_emergency_stop();
    control_state.jump_active = jump_ctrl_is_active();
    control_state.last_jump_result =
        (uint32)jump_ctrl_get_state()->result;
    (void)rotation_ctrl_abort();
    control_system_sync_rotation_state();
    control_state.last_bridge_status =
        (uint32)bridge_ctrl_set_enabled(0U);
    control_state.last_bumpy_status =
        (uint32)bumpy_ctrl_set_enabled(0U);
    control_state.bridge_active = 0U;
    control_state.bumpy_active = 0U;
    (void)leg_ctrl_set_differential_z_offset(0.0f);
}

static void control_system_stop_navigation_action(void)
{
    const navigation_state_t *navigation;

    navigation = navigation_get_state();
    if (NAVIGATION_MODE_RECORDING == navigation->mode)
    {
        (void)navigation_stop_recording();
    }
    else if ((NAVIGATION_MODE_REPLAYING == navigation->mode)
             || (NAVIGATION_MODE_REPLAY_COMPLETE == navigation->mode))
    {
        navigation_stop_replay();
    }
}

static void control_system_latch_fault(control_status_t status,
                                       uint32 fault_flags)
{
    // Lock the driver before touching any higher-level state so an interrupt or
    // main-loop race cannot emit a non-zero command during fault handling.
    wheel_driver_set_stop_lock(1U);
    control_state.fault_flags |= fault_flags;
    control_state.stand_request_pending = 0U;
    control_state.balance_enabled = 0U;
    restore_balance_after_jump = 0U;
    (void)balance_ctrl_set_enabled(0U);
    control_system_cancel_active_actions();
    control_system_set_zero_command();
    wheel_driver_stop();

    if (0U != (control_state.fault_flags
               & (uint32)CONTROL_FAULT_EMERGENCY_STOP))
    {
        control_state.status = CONTROL_STATUS_EMERGENCY_STOP;
        control_state.startup_state = CONTROL_STARTUP_EMERGENCY_STOP;
    }
    else
    {
        control_state.status = status;
        control_state.startup_state = CONTROL_STARTUP_FAULT;
    }
}

static void control_system_finish_jump(void)
{
    const jump_state_t *jump;
    uint8 request_stand;
    uint32 fault_flags;

    jump = jump_ctrl_get_state();
    control_state.jump_active = 0U;
    control_state.last_jump_result = (uint32)jump->result;
    control_state.last_leg_status = (uint32)jump->last_leg_status;
    wheel_driver_stop();

    if (JUMP_RESULT_COMPLETED == jump->result)
    {
        request_stand = restore_balance_after_jump;
        restore_balance_after_jump = 0U;
        wheel_driver_set_stop_lock(0U);
        control_state.startup_state = CONTROL_STARTUP_DISABLED;
        if (request_stand && !control_system_request_stand())
        {
            control_state.status = CONTROL_STATUS_NOT_READY;
        }
        return;
    }

    fault_flags = (uint32)CONTROL_FAULT_JUMP;
    if (JUMP_RESULT_LEG_ERROR == jump->result)
    {
        fault_flags |= (uint32)CONTROL_FAULT_LEG;
        if (0U == (control_state.fault_flags
                   & (uint32)CONTROL_FAULT_LEG))
        {
            control_state.leg_error_count++;
        }
    }
    if (0U == (control_state.fault_flags & (uint32)CONTROL_FAULT_JUMP))
    {
        control_state.jump_error_count++;
    }
    control_system_latch_fault(CONTROL_STATUS_JUMP_ERROR, fault_flags);
}

control_status_t control_system_init(void)
{
    imu_status_t imu_status;
    leg_ctrl_status_t leg_status;
    navigation_status_t navigation_status;
    bridge_ctrl_status_t bridge_status;
    bumpy_ctrl_status_t bumpy_status;
    rotation_ctrl_status_t rotation_status;
    uint8 leg_init_failed;

    memset(&control_state, 0, sizeof(control_state));
    control_state.startup_state = CONTROL_STARTUP_DISABLED;
    last_wheel_frame_count = 0U;
    fast_divider = 0U;
    fast_step_count = 0U;
    requested_drive_command_sequence = 0U;
    restore_balance_after_jump = 0U;
    leg_init_failed = 0U;
    requested_drive_command.target_speed_m_s = 0.0f;
    requested_drive_command.target_yaw_rate_rad_s = 0.0f;
    requested_drive_command.target_leg_x_offset_m = 0.0f;
    requested_drive_command.target_leg_z_offset_m = 0.0f;
    balance_ctrl_init();
    control_state.balance_config_valid = balance_ctrl_config_is_valid();
    wheel_driver_init();
    control_state.wheel_config_valid = wheel_driver_config_is_valid();
    leg_status = leg_ctrl_init();
    control_state.last_leg_status = (uint32)leg_status;
    if ((LEG_CTRL_STATUS_OK != leg_status)
        && (LEG_CTRL_STATUS_DISABLED != leg_status))
    {
        control_state.leg_error_count++;
        leg_init_failed = 1U;
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
        control_state.imu_error_count++;
        balance_ctrl_force_fault(BALANCE_FAULT_IMU);
        control_system_latch_fault(
            CONTROL_STATUS_IMU_ERROR,
            (uint32)CONTROL_FAULT_IMU
            | (uint32)CONTROL_FAULT_BALANCE);
        return control_state.status;
    }

    navigation_status = navigation_init(imu_get_data());
    control_state.last_navigation_status = (uint32)navigation_status;
    if (NAVIGATION_STATUS_OK != navigation_status)
    {
        control_state.navigation_error_count++;
    }

    if (leg_init_failed)
    {
        control_system_latch_fault(CONTROL_STATUS_LEG_ERROR,
                                   (uint32)CONTROL_FAULT_LEG);
        return control_state.status;
    }

    if (!control_state.wheel_config_valid)
    {
        balance_ctrl_force_fault(BALANCE_FAULT_WHEEL_FEEDBACK);
        control_system_latch_fault(
            CONTROL_STATUS_WHEEL_CONFIG_ERROR,
            (uint32)CONTROL_FAULT_WHEEL
            | (uint32)CONTROL_FAULT_BALANCE);
        return control_state.status;
    }

    if (!control_state.balance_config_valid)
    {
        control_system_latch_fault(
            CONTROL_STATUS_BALANCE_CONFIG_ERROR,
            (uint32)CONTROL_FAULT_BALANCE);
        return control_state.status;
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
    control_drive_command_t drive_command;
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
    uint32 drive_command_sequence;

    control_state.scheduler_tick_ms++;
    if (wheel_driver_is_stop_locked())
    {
        // The lock rejects non-zero commands; refreshing zero duty also handles
        // a wheel controller reset that occurred after the original stop frame.
        wheel_driver_stop();
    }
    if (jump_ctrl_is_active())
    {
        // The stop lock was refreshed once at the start of this tick.
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
        if (0U == (control_state.fault_flags
                   & (uint32)CONTROL_FAULT_IMU))
        {
            control_state.imu_error_count++;
        }
        balance_ctrl_force_fault(BALANCE_FAULT_IMU);
        control_system_latch_fault(
            CONTROL_STATUS_IMU_ERROR,
            (uint32)CONTROL_FAULT_IMU
            | (uint32)CONTROL_FAULT_BALANCE);
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
        if (control_state.jump_active && !jump_ctrl_is_active())
        {
            control_system_finish_jump();
        }
        return;
    }

    control_system_snapshot_drive_command(&drive_command,
                                          &drive_command_sequence);
    control_system_make_balance_command(&drive_command, &active_command);
    control_state.applied_drive_command_sequence = drive_command_sequence;
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
        control_system_latch_fault(CONTROL_STATUS_NOT_READY,
                                   (uint32)CONTROL_FAULT_BALANCE);
        return;
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
        control_state.last_leg_status = (uint32)leg_status;
        if (LEG_CTRL_STATUS_OK == leg_status)
        {
            leg_status = leg_ctrl_set_differential_z_offset(
                bridge->differential_leg_offset_m);
            control_state.last_leg_status = (uint32)leg_status;
        }
        if (balance->enabled && leg_ctrl_is_ready())
        {
            if (LEG_CTRL_STATUS_OK == leg_status)
            {
                leg_status = leg_ctrl_update(
                    imu->roll_deg * 0.017453292519943295f,
                    imu->gyro_dps[0] * 0.017453292519943295f);
                control_state.last_leg_status = (uint32)leg_status;
            }
            if (LEG_CTRL_STATUS_OK != leg_status)
            {
                if (0U == (control_state.fault_flags
                           & (uint32)CONTROL_FAULT_LEG))
                {
                    control_state.leg_error_count++;
                }
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
                control_system_latch_fault(
                    CONTROL_STATUS_LEG_ERROR,
                    (uint32)CONTROL_FAULT_LEG);
                return;
            }
        }
    }
}

uint8 control_system_submit_drive_command(
    const control_drive_command_t *command)
{
    if (!control_system_drive_command_is_valid(command))
    {
        uint32 interrupt_state;

        interrupt_state = Cy_SysLib_EnterCriticalSection();
        control_state.rejected_drive_command_count++;
        Cy_SysLib_ExitCriticalSection(interrupt_state);
        return 0U;
    }

    control_system_store_drive_command(command);
    return 1U;
}

uint8 control_system_request_stand(void)
{
    if ((CONTROL_STATUS_OK != control_state.status)
        || (CONTROL_FAULT_NONE != control_state.fault_flags)
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
        control_system_emergency_stop();
        return 1U;
    }
    return control_system_request_stand();
}

void control_system_emergency_stop(void)
{
    if (0U == (control_state.fault_flags
               & (uint32)CONTROL_FAULT_EMERGENCY_STOP))
    {
        control_state.emergency_stop_count++;
    }
    control_system_latch_fault(
        CONTROL_STATUS_EMERGENCY_STOP,
        (uint32)CONTROL_FAULT_EMERGENCY_STOP);

#if CONTROL_ESTOP_DISABLE_LEG_OUTPUT
    leg_ctrl_disable_output();
    control_state.last_leg_status =
        (uint32)leg_ctrl_get_state()->status;
#endif
}

uint8 control_system_recover_faults(void)
{
    const imu_data_t *imu;
    const jump_state_t *jump;
    const balance_state_t *balance;
    leg_ctrl_status_t leg_status;
    uint8 recover_jump;

    control_state.recovery_attempt_count++;
    wheel_driver_set_stop_lock(1U);
    (void)balance_ctrl_set_enabled(0U);
    control_system_cancel_active_actions();
    control_system_stop_navigation_action();
    control_system_set_zero_command();
    wheel_driver_stop();

    control_state.wheel_config_valid = wheel_driver_config_is_valid();
    if (!control_state.wheel_config_valid)
    {
        control_state.recovery_failure_count++;
        balance_ctrl_force_fault(BALANCE_FAULT_WHEEL_FEEDBACK);
        control_system_latch_fault(
            CONTROL_STATUS_WHEEL_CONFIG_ERROR,
            (uint32)CONTROL_FAULT_WHEEL
            | (uint32)CONTROL_FAULT_BALANCE);
        return 0U;
    }
    control_state.balance_config_valid = balance_ctrl_config_is_valid();
    if (!control_state.balance_config_valid)
    {
        control_state.recovery_failure_count++;
        control_system_latch_fault(
            CONTROL_STATUS_BALANCE_CONFIG_ERROR,
            (uint32)CONTROL_FAULT_BALANCE);
        return 0U;
    }

    imu = imu_get_data();
    if (!imu_is_initialized()
        || (IMU_STATUS_OK != (imu_status_t)control_state.last_imu_status)
        || (0 == imu)
        || (imu->pitch_deg != imu->pitch_deg)
        || (imu->roll_deg != imu->roll_deg))
    {
        control_state.recovery_failure_count++;
        control_system_latch_fault(
            CONTROL_STATUS_IMU_ERROR,
            (uint32)CONTROL_FAULT_IMU
            | (uint32)CONTROL_FAULT_BALANCE);
        return 0U;
    }
    if ((fabsf(imu->pitch_deg * CONTROL_DEG_TO_RAD
               - BALANCE_PITCH_ZERO_RAD)
            > CONTROL_FAULT_RECOVERY_MAX_PITCH_ERROR_RAD)
        || (fabsf(imu->roll_deg * CONTROL_DEG_TO_RAD)
            > CONTROL_FAULT_RECOVERY_MAX_ROLL_RAD))
    {
        control_state.recovery_failure_count++;
        control_system_latch_fault(CONTROL_STATUS_NOT_READY,
                                   (uint32)CONTROL_FAULT_BALANCE);
        return 0U;
    }

    jump = jump_ctrl_get_state();
    recover_jump = (uint8)((0U != (control_state.fault_flags
                                   & (uint32)CONTROL_FAULT_JUMP))
        || (JUMP_RESULT_LEG_ERROR == jump->result)
        || (JUMP_RESULT_EMERGENCY_STOP == jump->result));
    if (recover_jump)
    {
        if (!jump_ctrl_recover())
        {
            control_state.last_jump_result =
                (uint32)jump_ctrl_get_state()->result;
            control_state.last_leg_status =
                (uint32)jump_ctrl_get_state()->last_leg_status;
            control_state.recovery_failure_count++;
            control_system_latch_fault(
                CONTROL_STATUS_JUMP_ERROR,
                (uint32)CONTROL_FAULT_JUMP
                | (uint32)CONTROL_FAULT_LEG);
            return 0U;
        }
        control_state.last_jump_result =
            (uint32)jump_ctrl_get_state()->result;
        leg_status = jump_ctrl_get_state()->last_leg_status;
    }
    else
    {
        leg_status = leg_ctrl_recover();
    }
    control_state.last_leg_status = (uint32)leg_status;
    if ((LEG_CTRL_STATUS_OK != leg_status)
        && !((LEG_CTRL_STATUS_DISABLED == leg_status)
             && !LEG_CONTROL_ENABLE))
    {
        control_state.recovery_failure_count++;
        control_system_latch_fault(CONTROL_STATUS_LEG_ERROR,
                                   (uint32)CONTROL_FAULT_LEG);
        return 0U;
    }

    balance_ctrl_clear_faults();
    balance = balance_ctrl_get_state();
    if (BALANCE_FAULT_NONE != balance->fault_flags)
    {
        control_state.recovery_failure_count++;
        control_system_latch_fault(CONTROL_STATUS_NOT_READY,
                                   (uint32)CONTROL_FAULT_BALANCE);
        return 0U;
    }

    control_state.fault_flags = CONTROL_FAULT_NONE;
    control_state.status = CONTROL_STATUS_OK;
    control_state.startup_state = CONTROL_STARTUP_DISABLED;
    control_state.stand_request_pending = 0U;
    control_state.balance_enabled = 0U;
    restore_balance_after_jump = 0U;
    wheel_driver_stop();
    wheel_driver_set_stop_lock(0U);
    return 1U;
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
        control_state.last_leg_status =
            (uint32)jump_ctrl_get_state()->last_leg_status;
        wheel_driver_stop();
        if (JUMP_RESULT_LEG_ERROR == jump_ctrl_get_state()->result)
        {
            if (0U == (control_state.fault_flags
                       & (uint32)CONTROL_FAULT_JUMP))
            {
                control_state.jump_error_count++;
                control_state.leg_error_count++;
            }
            control_system_latch_fault(
                CONTROL_STATUS_JUMP_ERROR,
                (uint32)CONTROL_FAULT_JUMP
                | (uint32)CONTROL_FAULT_LEG);
            return 0U;
        }
        wheel_driver_set_stop_lock(0U);
        if (restore_balance_after_jump)
        {
            (void)control_system_request_stand();
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
    uint8 request_stand;

    wheel_driver_set_stop_lock(1U);
    (void)balance_ctrl_set_enabled(0U);
    recovered = jump_ctrl_abort();
    control_state.jump_active = 0U;
    control_state.last_jump_result =
        (uint32)jump_ctrl_get_state()->result;
    control_state.last_leg_status =
        (uint32)jump_ctrl_get_state()->last_leg_status;
    wheel_driver_stop();
    if (!recovered)
    {
        if (0U == (control_state.fault_flags
                   & (uint32)CONTROL_FAULT_JUMP))
        {
            control_state.jump_error_count++;
            control_state.leg_error_count++;
        }
        control_system_latch_fault(
            CONTROL_STATUS_JUMP_ERROR,
            (uint32)CONTROL_FAULT_JUMP
            | (uint32)CONTROL_FAULT_LEG);
        return 0U;
    }

    if (CONTROL_FAULT_NONE != control_state.fault_flags)
    {
        return 0U;
    }
    request_stand = restore_balance_after_jump;
    restore_balance_after_jump = 0U;
    wheel_driver_set_stop_lock(0U);
    control_state.startup_state = CONTROL_STARTUP_DISABLED;
    if (request_stand && !control_system_request_stand())
    {
        control_state.status = CONTROL_STATUS_NOT_READY;
        return 0U;
    }
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

const control_system_state_t *control_system_get_state(void)
{
    return &control_state;
}
