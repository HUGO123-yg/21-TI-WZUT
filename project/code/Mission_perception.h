#ifndef PROJECT_MISSION_PERCEPTION_H
#define PROJECT_MISSION_PERCEPTION_H

#include "Cone_vision.h"
#include "Minefield_vision.h"
#include "Navigation.h"
#include "Perception_fusion.h"

typedef enum
{
    MISSION_PERCEPTION_STATUS_OK = 0,
    MISSION_PERCEPTION_STATUS_DISABLED,
    MISSION_PERCEPTION_STATUS_IDLE,
    MISSION_PERCEPTION_STATUS_WAITING_FOR_NAVIGATION,
    MISSION_PERCEPTION_STATUS_WAITING_FOR_VISION,
    MISSION_PERCEPTION_STATUS_VISION_STALE,
    MISSION_PERCEPTION_STATUS_UNCALIBRATED,
    MISSION_PERCEPTION_STATUS_INVALID_ARGUMENT,
    MISSION_PERCEPTION_STATUS_INVALID_CONFIG,
    MISSION_PERCEPTION_STATUS_NOT_INITIALIZED
} mission_perception_status_t;

typedef enum
{
    MISSION_PERCEPTION_TASK_NONE = 0,
    MISSION_PERCEPTION_TASK_CONE_SLALOM,
    MISSION_PERCEPTION_TASK_MINEFIELD,
    MISSION_PERCEPTION_TASK_TERRAIN
} mission_perception_task_t;

typedef enum
{
    MISSION_PERCEPTION_DISTANCE_ROUTE = 0,
    MISSION_PERCEPTION_DISTANCE_TRAVELED
} mission_perception_distance_source_t;

typedef enum
{
    MISSION_PERCEPTION_ROTATION_CW = 0,
    MISSION_PERCEPTION_ROTATION_CCW
} mission_perception_rotation_direction_t;

// Context is published atomically from main context. The navigation window is
// the only place where task-specific visual observations may become usable.
typedef struct
{
    mission_perception_task_t task;
    mission_perception_distance_source_t distance_source;
    mission_perception_rotation_direction_t rotation_direction;
    float navigation_window_start_m;
    float navigation_window_end_m;
    float mine_target_turns;
    uint8 route_id;
} mission_perception_context_t;

typedef struct
{
    mission_perception_status_t status;
    mission_perception_task_t task;
    mission_perception_distance_source_t distance_source;
    mission_perception_rotation_direction_t rotation_direction;
    terrain_type_t terrain_type;
    uint32 context_sequence;
    uint32 last_visual_frame_count;
    uint32 last_visual_timestamp_ms;
    uint32 visual_age_ms;
    uint32 accepted_visual_count;
    uint32 rejected_visual_count;
    uint32 rotation_discontinuity_count;
    float navigation_distance_m;
    float navigation_window_start_m;
    float navigation_window_end_m;
    float navigation_yaw_rad;
    float navigation_yaw_rate_rad_s;
    float navigation_yaw_rate_command_rad_s;
    float raw_visual_error_norm;
    float filtered_visual_error_norm;
    float visual_yaw_rate_correction_rad_s;
    float suggested_yaw_rate_rad_s;
    float mine_center_error_norm;
    float mine_boundary_proximity_norm;
    float mine_rotation_progress_rad;
    float mine_rotation_progress_turns;
    float mine_target_turns;
    uint8 route_id;
    uint8 visual_confidence;
    uint8 navigation_window_active;
    uint8 visual_fresh;
    uint8 visual_calibrated;
    uint8 visual_observation_valid;
    uint8 advisory_valid;
    uint8 mine_center_valid;
    uint8 mine_boundary_guard_valid;
    uint8 mine_boundary_warning;
    uint8 mine_rotation_progress_valid;
    uint8 mine_rotation_complete;
} mission_perception_state_t;

// This module never calls Control_system or any actuator. It only determines
// whether task-specific observations are contextually safe to consume.
mission_perception_status_t mission_perception_init(void);
mission_perception_status_t mission_perception_start(
    const mission_perception_context_t *context);
// Maps recorded routes 1/2/3 to slalom/minefield/terrain observation contexts.
// This only enables observation; actuator consumption remains separately gated.
mission_perception_status_t mission_perception_start_route(uint8 route_id);
// Re-arms the per-zone yaw accumulator for the next minefield rotation.
mission_perception_status_t mission_perception_configure_mine_rotation(
    mission_perception_rotation_direction_t direction,
    float turns);
void mission_perception_stop(void);

// Deterministic frame-in API for host replay. now_ms must use the same monotonic
// timebase as the control scheduler in runtime use.
mission_perception_status_t mission_perception_update(
    const cone_vision_result_t *cone,
    const minefield_vision_result_t *minefield,
    const perception_fusion_state_t *terrain_fusion,
    const navigation_state_t *navigation,
    uint32 now_ms);

mission_perception_status_t mission_perception_service(
    const navigation_state_t *navigation,
    uint32 now_ms);
const mission_perception_state_t *mission_perception_get_state(void);

#endif
