#include "Control_system.h"

#include <string.h>

#include "Imu.h"
#include "Wheel_driver.h"
#include "config.h"

static control_system_state_t control_state;
static uint32 last_wheel_frame_count;
static uint32 fast_divider;
static uint32 fast_step_count;

control_status_t control_system_init(void)
{
    imu_status_t imu_status;

    memset(&control_state, 0, sizeof(control_state));
    last_wheel_frame_count = 0U;
    fast_divider = 0U;
    fast_step_count = 0U;
    balance_ctrl_init();
    wheel_driver_init();
    (void)leg_ctrl_init();

    imu_status = imu_init();
    if (IMU_STATUS_OK != imu_status)
    {
        control_state.status = CONTROL_STATUS_IMU_ERROR;
        control_state.imu_error_count++;
        balance_ctrl_force_fault(BALANCE_FAULT_IMU);
        wheel_driver_stop();
        return control_state.status;
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
    leg_ctrl_status_t leg_status;
    uint8 run_speed_loop;

    control_state.scheduler_tick_ms++;
    fast_divider++;
    if (fast_divider < CONTROL_FAST_INTERVAL_TICKS)
    {
        return;
    }
    fast_divider = 0U;
    fast_step_count++;

    if (IMU_STATUS_OK != imu_update())
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

    balance = balance_ctrl_get_state();
    if (balance->enabled
        && (!control_state.wheel_feedback_ready
            || (control_state.wheel_feedback_age_ms
                > CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS)))
    {
        balance_ctrl_force_fault(BALANCE_FAULT_WHEEL_FEEDBACK);
    }

    imu = imu_get_data();
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
    balance_ctrl_set_command(command);
}

uint8 control_system_set_enabled(uint8 enabled)
{
    if (!enabled)
    {
        (void)balance_ctrl_set_enabled(0U);
        wheel_driver_stop();
        return 1U;
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
