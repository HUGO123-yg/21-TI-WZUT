#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Bridge_ctrl.h"
#include "Bumpy_ctrl.h"
#include "Rotation_ctrl.h"
#include "Route_plan.h"

static float wrap_yaw_deg(float yaw_deg)
{
    while (yaw_deg > 180.0f)
    {
        yaw_deg -= 360.0f;
    }
    while (yaw_deg <= -180.0f)
    {
        yaw_deg += 360.0f;
    }
    return yaw_deg;
}

static void test_route_subject_semantics(void)
{
    const route_point_t route_1_valid[] = {
        { 0.0f, 0.10f, ROUTE_ACTION_NONE, 0.0f },
        { 1.0f, 0.00f, ROUTE_ACTION_ROTATE_CCW, 0.5f }
    };
    const route_point_t route_1_invalid[] = {
        { 0.0f, 0.10f, ROUTE_ACTION_MINE_ROTATE_CCW, 2.0f }
    };
    const route_point_t route_2_valid[] = {
        { 0.0f, 0.10f, ROUTE_ACTION_NONE, 0.0f },
        { 1.0f, 0.00f, ROUTE_ACTION_MINE_ROTATE_CW, 2.0f },
        { 2.0f, 0.00f, ROUTE_ACTION_MINE_ROTATE_CCW, 2.5f }
    };
    const route_point_t route_2_unguarded[] = {
        { 0.0f, 0.00f, ROUTE_ACTION_ROTATE_CW, 2.0f }
    };
    const route_point_t route_2_short_turn[] = {
        { 0.0f, 0.00f, ROUTE_ACTION_MINE_ROTATE_CW, 1.5f }
    };
    const route_point_t route_2_moving_turn[] = {
        { 0.0f, 0.10f, ROUTE_ACTION_MINE_ROTATE_CW, 2.0f }
    };
    const route_point_t route_1_invalid_none_parameter[] = {
        { 0.0f, 0.10f, ROUTE_ACTION_NONE, 1.0f }
    };
    const route_point_t route_1_moving_stop[] = {
        { 0.0f, 0.10f, ROUTE_ACTION_STOP, 0.0f }
    };
    const route_point_t route_3_valid[] = {
        { 0.0f, 0.10f, ROUTE_ACTION_BRIDGE_LEFT, 0.0f },
        { 1.0f, 0.10f, ROUTE_ACTION_BUMPY, 0.0f },
        { 2.0f, 0.00f, ROUTE_ACTION_STAIR_DESCENT_JUMP, 0.0f }
    };

    assert(route_plan_definition_is_valid(
        1U, route_1_valid,
        (uint8)(sizeof(route_1_valid) / sizeof(route_1_valid[0]))));
    assert(!route_plan_definition_is_valid(
        1U, route_1_invalid,
        (uint8)(sizeof(route_1_invalid) / sizeof(route_1_invalid[0]))));
    assert(route_plan_definition_is_valid(
        2U, route_2_valid,
        (uint8)(sizeof(route_2_valid) / sizeof(route_2_valid[0]))));
    assert(!route_plan_definition_is_valid(
        2U, route_2_unguarded,
        (uint8)(sizeof(route_2_unguarded)
                / sizeof(route_2_unguarded[0]))));
    assert(!route_plan_definition_is_valid(
        2U, route_2_short_turn,
        (uint8)(sizeof(route_2_short_turn)
                / sizeof(route_2_short_turn[0]))));
    assert(!route_plan_definition_is_valid(
        2U, route_2_moving_turn,
        (uint8)(sizeof(route_2_moving_turn)
                / sizeof(route_2_moving_turn[0]))));
    assert(!route_plan_definition_is_valid(
        1U, route_1_invalid_none_parameter,
        (uint8)(sizeof(route_1_invalid_none_parameter)
                / sizeof(route_1_invalid_none_parameter[0]))));
    assert(!route_plan_definition_is_valid(
        1U, route_1_moving_stop,
        (uint8)(sizeof(route_1_moving_stop)
                / sizeof(route_1_moving_stop[0]))));
    assert(route_plan_definition_is_valid(
        3U, route_3_valid,
        (uint8)(sizeof(route_3_valid) / sizeof(route_3_valid[0]))));
    assert(!route_plan_definition_is_valid(
        2U, route_3_valid,
        (uint8)(sizeof(route_3_valid) / sizeof(route_3_valid[0]))));
}

static void test_two_turn_rotation_unwrap(void)
{
    imu_data_t imu;
    balance_command_t requested;
    balance_command_t shaped;
    float unwrapped_yaw_deg;
    uint32 step;

    memset(&imu, 0, sizeof(imu));
    memset(&requested, 0, sizeof(requested));
    requested.target_speed_m_s = 0.20f;
    assert(ROTATION_CTRL_STATUS_OK == rotation_ctrl_init());
    assert(ROTATION_CTRL_STATUS_OK
           == rotation_ctrl_start(ROTATION_DIR_CCW, 2.0f, &imu));

    unwrapped_yaw_deg = 0.0f;
    for (step = 0U; step < 72U; step++)
    {
        unwrapped_yaw_deg += 10.0f;
        imu.yaw_deg = wrap_yaw_deg(unwrapped_yaw_deg);
        imu.attitude_rate_dps[2] = 200.0f;
        assert(ROTATION_CTRL_STATUS_OK
               == rotation_ctrl_update(&imu, &requested, &shaped));
        assert(fabsf(shaped.target_speed_m_s) < 0.0001f);
    }
    imu.attitude_rate_dps[2] = 0.0f;
    for (step = 0U; step < 20U; step++)
    {
        assert(ROTATION_CTRL_STATUS_OK
               == rotation_ctrl_update(&imu, &requested, &shaped));
    }
    assert(ROTATION_RESULT_COMPLETED
           == rotation_ctrl_get_state()->result);
    assert(ROTATION_PHASE_HOLDING == rotation_ctrl_get_state()->phase);
    assert(rotation_ctrl_is_active());
    assert(ROTATION_CTRL_STATUS_OK == rotation_ctrl_release());
    assert(!rotation_ctrl_is_active());
}

static void test_bridge_forced_completion(void)
{
    imu_data_t imu;
    balance_command_t requested;
    balance_command_t shaped;
    uint32 step;

    memset(&imu, 0, sizeof(imu));
    memset(&requested, 0, sizeof(requested));
    requested.target_speed_m_s = 0.50f;
    imu.roll_deg = 5.0f;
    assert(BRIDGE_CTRL_STATUS_OK == bridge_ctrl_init());
    assert(BRIDGE_CTRL_STATUS_OK == bridge_ctrl_force_enter(1, 0.0f));
    for (step = 0U; step < 25U; step++)
    {
        assert(BRIDGE_CTRL_STATUS_OK
               == bridge_ctrl_update(&imu, 0.0f, &requested, &shaped));
        assert(shaped.target_speed_m_s <= 0.2601f);
    }
    assert(BRIDGE_CTRL_STATUS_OK
           == bridge_ctrl_update(&imu, 0.25f, &requested, &shaped));
    for (step = 0U; step < 20U; step++)
    {
        assert(BRIDGE_CTRL_STATUS_OK
               == bridge_ctrl_update(&imu, 0.25f, &requested, &shaped));
    }
    imu.roll_deg = 0.0f;
    for (step = 0U; step < 100U && bridge_ctrl_is_active(); step++)
    {
        assert(BRIDGE_CTRL_STATUS_OK
               == bridge_ctrl_update(&imu, 0.25f, &requested, &shaped));
    }
    assert(!bridge_ctrl_is_active());
    assert(BRIDGE_RESULT_COMPLETED == bridge_ctrl_get_state()->result);
}

static void test_bumpy_forced_completion(void)
{
    imu_data_t imu;
    balance_command_t requested;
    balance_command_t shaped;
    uint32 step;

    memset(&imu, 0, sizeof(imu));
    memset(&requested, 0, sizeof(requested));
    requested.target_speed_m_s = 0.50f;
    imu.acc_norm_g = 1.0f;
    assert(BUMPY_CTRL_STATUS_OK == bumpy_ctrl_init());
    assert(BUMPY_CTRL_STATUS_OK == bumpy_ctrl_force_enter(0.0f));
    assert(BUMPY_CTRL_STATUS_OK
           == bumpy_ctrl_update(&imu,
                                1.10f,
                                0.20f,
                                1U,
                                &requested,
                                &shaped));
    assert(shaped.target_speed_m_s <= 0.3001f);
    for (step = 0U; step < 40U && bumpy_ctrl_is_monitoring(); step++)
    {
        assert(BUMPY_CTRL_STATUS_OK
               == bumpy_ctrl_update(&imu,
                                    1.10f,
                                    0.0f,
                                    1U,
                                    &requested,
                                    &shaped));
    }
    assert(!bumpy_ctrl_is_monitoring());
    assert(BUMPY_RESULT_COMPLETED == bumpy_ctrl_get_state()->result);
}

int main(void)
{
    test_route_subject_semantics();
    test_two_turn_rotation_unwrap();
    test_bridge_forced_completion();
    test_bumpy_forced_completion();
    puts("control_modules_test: PASS");
    return 0;
}
