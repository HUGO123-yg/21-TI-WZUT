#include "Control_system.h"

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
    navigation_status_t navigation_status;

    memset(&control_state, 0, sizeof(control_state));
    last_wheel_frame_count = 0U;
    fast_divider = 0U;
    fast_step_count = 0U;
    restore_balance_after_jump = 0U;
    memset(&requested_command, 0, sizeof(requested_command));
    balance_ctrl_init();
    wheel_driver_init();
    (void)leg_ctrl_init();
    jump_ctrl_init();
    control_state.last_jump_result = (uint32)JUMP_RESULT_IDLE;

    imu_status = imu_init();
    control_state.last_imu_status = (uint32)imu_status;
    if (IMU_STATUS_OK != imu_status)
    {
        control_state.status = CONTROL_STATUS_IMU_ERROR;
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
    if (CONTROL_ENABLE_ON_BOOT)
    {
        (void)control_system_set_enabled(1U);
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
    balance_command_t active_command;
    imu_status_t imu_status;
    leg_ctrl_status_t leg_status;
    navigation_status_t navigation_status;
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
        control_state.status = CONTROL_STATUS_IMU_ERROR;
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

    balance = balance_ctrl_get_state();
    if (balance->enabled
        && (!control_state.wheel_feedback_ready
            || (control_state.wheel_feedback_age_ms
                > CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS)))
    {
        balance_ctrl_force_fault(BALANCE_FAULT_WHEEL_FEEDBACK);
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
    balance_ctrl_set_command(&active_command);

    run_speed_loop = (uint8)((fast_step_count
                              % CONTROL_SPEED_INTERVAL_STEPS) == 0U);
    balance_ctrl_update(imu, &wheel_feedback, run_speed_loop);
    balance = balance_ctrl_get_state();
    wheel_driver_set_command(balance->left_wheel_command,
                             balance->right_wheel_command);

    if ((fast_step_count % CONTROL_LEG_INTERVAL_STEPS) == 0U)
    {
        command = balance_ctrl_get_command();
        (void)leg_ctrl_set_target_offset(command->target_leg_x_offset_m,
                                         command->target_leg_z_offset_m);
        if (balance->enabled && leg_ctrl_is_ready())
        {
            leg_status = leg_ctrl_update(
                imu->roll_deg * 0.017453292519943295f,
                imu->gyro_dps[0] * 0.017453292519943295f);
            if (LEG_CTRL_STATUS_OK != leg_status)
            {
                control_state.leg_error_count++;
            }
        }
    }
}

void control_system_set_command(const balance_command_t *command)
{
    if (0 != command)
    {
        requested_command = *command;
    }
}

uint8 control_system_set_enabled(uint8 enabled)
{
    if (!enabled)
    {
        (void)balance_ctrl_set_enabled(0U);
        wheel_driver_stop();
        return 1U;
    }

    if (wheel_driver_is_stop_locked() || jump_ctrl_is_active())
    {
        return 0U;
    }

    if ((CONTROL_STATUS_OK != control_state.status)
        || !control_state.wheel_feedback_ready
        || (control_state.wheel_feedback_age_ms
            > CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS))
    {
        return 0U;
    }
    return balance_ctrl_set_enabled(1U);
}

uint8 control_system_start_jump(void)
{
    const balance_state_t *balance;

    if ((CONTROL_STATUS_OK != control_state.status)
        || jump_ctrl_is_active()
        || wheel_driver_is_stop_locked())
    {
        return 0U;
    }

    balance = balance_ctrl_get_state();
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

void control_system_clear_faults(void)
{
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
