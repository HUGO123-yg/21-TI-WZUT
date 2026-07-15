#ifndef PROJECT_ROUTE_PLAN_H
#define PROJECT_ROUTE_PLAN_H

#include "zf_common_typedef.h"

typedef enum
{
    ROUTE_PLAN_STATUS_OK = 0,
    ROUTE_PLAN_STATUS_DISABLED,
    ROUTE_PLAN_STATUS_IDLE,
    ROUTE_PLAN_STATUS_INVALID_ARGUMENT,
    ROUTE_PLAN_STATUS_INVALID_CONFIG,
    ROUTE_PLAN_STATUS_ACTION_REJECTED,
    ROUTE_PLAN_STATUS_ACTION_FAILED,
    ROUTE_PLAN_STATUS_COMPLETE
} route_plan_status_t;

typedef enum
{
    ROUTE_ACTION_NONE = 0,
    ROUTE_ACTION_JUMP,
    ROUTE_ACTION_ROTATE_CW,
    ROUTE_ACTION_ROTATE_CCW,
    ROUTE_ACTION_BRIDGE_LEFT,
    ROUTE_ACTION_BRIDGE_RIGHT,
    ROUTE_ACTION_BUMPY,
    ROUTE_ACTION_STOP
} route_action_t;

// A route point becomes due when route_distance_m reaches distance_m. Its speed
// remains selected until the next point. Non-zero actions are latched and must
// be acknowledged by Control_system, so a busy module cannot lose a trigger.
typedef struct
{
    float distance_m;
    float target_speed_m_s;
    route_action_t action;
    float action_parameter;
} route_point_t;

typedef struct
{
    route_plan_status_t status;
    route_action_t current_action;
    route_action_t last_action;
    uint32 update_count;
    uint32 action_start_count;
    uint32 action_complete_count;
    uint32 action_reject_count;
    float route_distance_m;
    float selected_speed_m_s;
    float command_speed_m_s;
    float action_distance_m;
    float action_parameter;
    uint8 route_id;
    uint8 active;
    uint8 action_pending;
    uint8 action_running;
    uint8 current_point_index;
    uint8 next_point_index;
} route_plan_state_t;

route_plan_status_t route_plan_init(void);
route_plan_status_t route_plan_start(uint8 route_id);
void route_plan_abort(void);

// Runs at CONTROL_FAST_PERIOD_S with Navigation.route_distance_m.
route_plan_status_t route_plan_update(float route_distance_m);

uint8 route_plan_get_pending_action(route_action_t *action,
                                    float *parameter);
void route_plan_action_started(void);
void route_plan_action_rejected(void);
void route_plan_action_completed(uint8 success);
void route_plan_finish(void);

const route_plan_state_t *route_plan_get_state(void);

#endif
