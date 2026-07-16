#include "Route_plan.h"

#include <float.h>
#include <string.h>

#include "config.h"

#define ROUTE_PLAN_POINT_NONE (0xFFU)

static const route_point_t route_1_points[] =
{
    ROUTE_PLAN_ROUTE_1_POINTS
};

static const route_point_t route_2_points[] =
{
    ROUTE_PLAN_ROUTE_2_POINTS
};

static const route_point_t route_3_points[] =
{
    ROUTE_PLAN_ROUTE_3_POINTS
};

static route_plan_state_t route_state;
static const route_point_t *active_points;
static uint8 active_point_count;
static uint32 action_reject_attempts;

static uint8 route_plan_float_is_finite(float value)
{
    return (uint8)((value == value)
                   && (value <= FLT_MAX)
                   && (value >= -FLT_MAX));
}

static uint8 route_plan_action_is_valid(route_action_t action,
                                        float parameter)
{
    if ((action < ROUTE_ACTION_NONE) || (action > ROUTE_ACTION_STOP))
    {
        return 0U;
    }
    if (((ROUTE_ACTION_ROTATE_CW == action)
         || (ROUTE_ACTION_ROTATE_CCW == action))
        && ((parameter <= 0.0f) || (parameter > ROTATION_MAX_TURNS)))
    {
        return 0U;
    }
    if (((ROUTE_ACTION_MINE_ROTATE_CW == action)
         || (ROUTE_ACTION_MINE_ROTATE_CCW == action))
        && ((parameter < MISSION_PERCEPTION_MINE_MIN_TURNS)
            || (parameter > ROTATION_MAX_TURNS)
            || (parameter > MISSION_PERCEPTION_MINE_MAX_TURNS)))
    {
        return 0U;
    }
    if (((ROUTE_ACTION_NONE == action)
         || (ROUTE_ACTION_STAIR_DESCENT_JUMP == action)
         || (ROUTE_ACTION_BRIDGE_LEFT == action)
         || (ROUTE_ACTION_BRIDGE_RIGHT == action)
         || (ROUTE_ACTION_BUMPY == action)
         || (ROUTE_ACTION_STOP == action))
        && (0.0f != parameter))
    {
        return 0U;
    }
    return 1U;
}

static uint8 route_plan_action_requires_zero_speed(route_action_t action)
{
    return (uint8)((ROUTE_ACTION_STAIR_DESCENT_JUMP == action)
        || (ROUTE_ACTION_ROTATE_CW == action)
        || (ROUTE_ACTION_ROTATE_CCW == action)
        || (ROUTE_ACTION_MINE_ROTATE_CW == action)
        || (ROUTE_ACTION_MINE_ROTATE_CCW == action)
        || (ROUTE_ACTION_STOP == action));
}

static uint8 route_plan_action_matches_route(uint8 route_id,
                                             route_action_t action)
{
    switch (action)
    {
        case ROUTE_ACTION_MINE_ROTATE_CW:
        case ROUTE_ACTION_MINE_ROTATE_CCW:
            return (uint8)(2U == route_id);

        case ROUTE_ACTION_STAIR_DESCENT_JUMP:
        case ROUTE_ACTION_BRIDGE_LEFT:
        case ROUTE_ACTION_BRIDGE_RIGHT:
        case ROUTE_ACTION_BUMPY:
            return (uint8)(3U == route_id);

        case ROUTE_ACTION_ROTATE_CW:
        case ROUTE_ACTION_ROTATE_CCW:
            return (uint8)(2U != route_id);

        case ROUTE_ACTION_NONE:
        case ROUTE_ACTION_STOP:
        default:
            return 1U;
    }
}

uint8 route_plan_definition_is_valid(uint8 route_id,
                                     const route_point_t *points,
                                     uint8 count)
{
    float previous_distance;
    uint8 index;

    if ((route_id < 1U) || (route_id > 3U)
        || (0 == points) || (0U == count)
        || (count > ROUTE_PLAN_MAX_POINT_COUNT)
        || (ROUTE_PLAN_MAX_SPEED_M_S <= 0.0f)
        || (ROUTE_PLAN_SPEED_SLEW_M_S2 <= 0.0f)
        || (ROUTE_PLAN_ACTION_RETRY_LIMIT == 0U))
    {
        return 0U;
    }

    previous_distance = -1.0f;
    for (index = 0U; index < count; index++)
    {
        if (!route_plan_float_is_finite(points[index].distance_m)
            || !route_plan_float_is_finite(points[index].target_speed_m_s)
            || !route_plan_float_is_finite(points[index].action_parameter)
            || (points[index].distance_m < 0.0f)
            || (points[index].distance_m < previous_distance)
            || (points[index].target_speed_m_s < 0.0f)
            || (points[index].target_speed_m_s
                > ROUTE_PLAN_MAX_SPEED_M_S)
            || (route_plan_action_requires_zero_speed(points[index].action)
                && (0.0f != points[index].target_speed_m_s))
            || !route_plan_action_is_valid(points[index].action,
                                           points[index].action_parameter)
            || !route_plan_action_matches_route(route_id,
                                                points[index].action))
        {
            return 0U;
        }
        previous_distance = points[index].distance_m;
    }
    return 1U;
}

static uint8 route_plan_load(uint8 route_id)
{
    switch (route_id)
    {
        case 1U:
            active_points = route_1_points;
            active_point_count = (uint8)(sizeof(route_1_points)
                                         / sizeof(route_1_points[0]));
            break;

        case 2U:
            active_points = route_2_points;
            active_point_count = (uint8)(sizeof(route_2_points)
                                         / sizeof(route_2_points[0]));
            break;

        case 3U:
            active_points = route_3_points;
            active_point_count = (uint8)(sizeof(route_3_points)
                                         / sizeof(route_3_points[0]));
            break;

        default:
            active_points = 0;
            active_point_count = 0U;
            return 0U;
    }
    return route_plan_definition_is_valid(route_id,
                                          active_points,
                                          active_point_count);
}

static float route_plan_slew_speed(float current, float target)
{
    float maximum_step;

    maximum_step = ROUTE_PLAN_SPEED_SLEW_M_S2 * CONTROL_FAST_PERIOD_S;
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

static void route_plan_stop(route_plan_status_t status)
{
    route_state.status = status;
    route_state.active = 0U;
    route_state.action_pending = 0U;
    route_state.action_running = 0U;
    route_state.selected_speed_m_s = 0.0f;
    route_state.command_speed_m_s = 0.0f;
    active_points = 0;
    active_point_count = 0U;
    action_reject_attempts = 0U;
}

route_plan_status_t route_plan_init(void)
{
    memset(&route_state, 0, sizeof(route_state));
    active_points = 0;
    active_point_count = 0U;
    action_reject_attempts = 0U;
    route_state.current_point_index = ROUTE_PLAN_POINT_NONE;
    route_state.status = ROUTE_PLAN_ENABLE
        ? ROUTE_PLAN_STATUS_IDLE
        : ROUTE_PLAN_STATUS_DISABLED;
    return route_state.status;
}

route_plan_status_t route_plan_start(uint8 route_id)
{
    route_plan_status_t status;

    status = route_plan_init();
    if (ROUTE_PLAN_STATUS_DISABLED == status)
    {
        return status;
    }
    if (!route_plan_load(route_id))
    {
        route_state.status = (route_id < 1U || route_id > 3U)
            ? ROUTE_PLAN_STATUS_INVALID_ARGUMENT
            : ROUTE_PLAN_STATUS_INVALID_CONFIG;
        return route_state.status;
    }

    route_state.route_id = route_id;
    route_state.active = 1U;
    route_state.status = ROUTE_PLAN_STATUS_OK;
    return route_state.status;
}

void route_plan_abort(void)
{
    route_plan_stop(ROUTE_PLAN_ENABLE
        ? ROUTE_PLAN_STATUS_IDLE
        : ROUTE_PLAN_STATUS_DISABLED);
    route_state.route_id = 0U;
    route_state.current_action = ROUTE_ACTION_NONE;
    route_state.last_action = ROUTE_ACTION_NONE;
    route_state.current_point_index = ROUTE_PLAN_POINT_NONE;
    route_state.next_point_index = 0U;
}

route_plan_status_t route_plan_update(float route_distance_m)
{
    const route_point_t *point;

    if (!route_state.active)
    {
        return route_state.status;
    }
    if (!route_plan_float_is_finite(route_distance_m)
        || (route_distance_m < 0.0f)
        || (route_distance_m + ROUTE_PLAN_DISTANCE_BACKTRACK_TOLERANCE_M
            < route_state.route_distance_m))
    {
        route_plan_stop(ROUTE_PLAN_STATUS_INVALID_ARGUMENT);
        return route_state.status;
    }

    if (route_distance_m > route_state.route_distance_m)
    {
        route_state.route_distance_m = route_distance_m;
    }
    route_state.update_count++;

    while (!route_state.action_pending
           && !route_state.action_running
           && (route_state.next_point_index < active_point_count))
    {
        point = &active_points[route_state.next_point_index];
        if (point->distance_m
            > route_state.route_distance_m + ROUTE_PLAN_TRIGGER_EPSILON_M)
        {
            break;
        }

        route_state.current_point_index = route_state.next_point_index;
        route_state.next_point_index++;
        route_state.selected_speed_m_s = point->target_speed_m_s;
        route_state.action_distance_m = point->distance_m;
        route_state.action_parameter = point->action_parameter;
        if (ROUTE_ACTION_STOP == point->action)
        {
            route_state.last_action = ROUTE_ACTION_STOP;
            route_plan_stop(ROUTE_PLAN_STATUS_COMPLETE);
            return route_state.status;
        }
        if (ROUTE_ACTION_NONE != point->action)
        {
            route_state.current_action = point->action;
            route_state.action_pending = 1U;
            action_reject_attempts = 0U;
        }
    }

    route_state.command_speed_m_s = route_plan_slew_speed(
        route_state.command_speed_m_s,
        route_state.selected_speed_m_s);
    return route_state.status;
}

uint8 route_plan_get_pending_action(route_action_t *action,
                                    float *parameter)
{
    if (!route_state.active || !route_state.action_pending
        || (0 == action) || (0 == parameter))
    {
        return 0U;
    }
    *action = route_state.current_action;
    *parameter = route_state.action_parameter;
    return 1U;
}

void route_plan_action_started(void)
{
    if (!route_state.active || !route_state.action_pending)
    {
        return;
    }
    route_state.action_pending = 0U;
    route_state.action_running = 1U;
    route_state.action_start_count++;
    action_reject_attempts = 0U;
}

void route_plan_action_rejected(void)
{
    if (!route_state.active || !route_state.action_pending)
    {
        return;
    }
    route_state.action_reject_count++;
    action_reject_attempts++;
    if (action_reject_attempts >= ROUTE_PLAN_ACTION_RETRY_LIMIT)
    {
        route_plan_stop(ROUTE_PLAN_STATUS_ACTION_REJECTED);
    }
}

void route_plan_action_completed(uint8 success)
{
    if (!route_state.action_running)
    {
        return;
    }

    route_state.action_running = 0U;
    route_state.last_action = route_state.current_action;
    route_state.current_action = ROUTE_ACTION_NONE;
    route_state.action_parameter = 0.0f;
    if (success)
    {
        route_state.action_complete_count++;
    }
    else
    {
        route_plan_stop(ROUTE_PLAN_STATUS_ACTION_FAILED);
    }
}

void route_plan_finish(void)
{
    if (route_state.active
        && !route_state.action_pending
        && !route_state.action_running)
    {
        route_plan_stop(ROUTE_PLAN_STATUS_COMPLETE);
    }
}

const route_plan_state_t *route_plan_get_state(void)
{
    return &route_state;
}
