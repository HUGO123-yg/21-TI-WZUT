#include "Mission_perception.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "config.h"

#define MISSION_PERCEPTION_PI       (3.14159265358979323846f)
#define MISSION_PERCEPTION_TWO_PI   (6.28318530717958647692f)

typedef struct
{
    mission_perception_context_t value;
    uint32 sequence;
} mission_published_context_t;

static mission_perception_state_t mission_state;
static mission_published_context_t mission_context[2];
static volatile uint8 mission_context_index;
static uint32 mission_next_context_sequence;
static uint8 mission_initialized;
static uint8 mission_visual_source_seen;
static uint8 mission_visual_filter_initialized;
static uint8 mission_mine_yaw_initialized;
static float mission_last_mine_yaw_rad;

static uint8 mission_float_is_finite(float value)
{
    return (uint8)((value == value)
                   && (value <= FLT_MAX)
                   && (value >= -FLT_MAX));
}

static float mission_clamp(float value, float minimum, float maximum)
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

static float mission_wrap_angle(float angle_rad)
{
    while (angle_rad > MISSION_PERCEPTION_PI)
    {
        angle_rad -= MISSION_PERCEPTION_TWO_PI;
    }
    while (angle_rad <= -MISSION_PERCEPTION_PI)
    {
        angle_rad += MISSION_PERCEPTION_TWO_PI;
    }
    return angle_rad;
}

static uint8 mission_config_is_valid(void)
{
    return (uint8)((MISSION_PERCEPTION_VISUAL_TIMEOUT_MS > 0U)
                   && (MISSION_PERCEPTION_CONE_MIN_QUALITY <= 100U)
                   && (MISSION_PERCEPTION_CONE_FILTER_ALPHA > 0.0f)
                   && (MISSION_PERCEPTION_CONE_FILTER_ALPHA <= 1.0f)
                   && (MISSION_PERCEPTION_MAX_CONE_YAW_RATE_RAD_S > 0.0f)
                   && (MISSION_PERCEPTION_MAX_SUGGESTED_YAW_RATE_RAD_S > 0.0f)
                   && (MISSION_PERCEPTION_MINE_MIN_TURNS >= 2.0f)
                   && (MISSION_PERCEPTION_MINE_MAX_TURNS
                       >= MISSION_PERCEPTION_MINE_MIN_TURNS)
                   && (MISSION_PERCEPTION_MAX_YAW_STEP_RAD > 0.0f)
                   && (MISSION_PERCEPTION_ROUTE_1_WINDOW_START_M >= 0.0f)
                   && (MISSION_PERCEPTION_ROUTE_1_WINDOW_END_M
                       > MISSION_PERCEPTION_ROUTE_1_WINDOW_START_M)
                   && (MISSION_PERCEPTION_ROUTE_2_WINDOW_START_M >= 0.0f)
                   && (MISSION_PERCEPTION_ROUTE_2_WINDOW_END_M
                       > MISSION_PERCEPTION_ROUTE_2_WINDOW_START_M)
                   && (MISSION_PERCEPTION_ROUTE_3_WINDOW_START_M >= 0.0f)
                   && (MISSION_PERCEPTION_ROUTE_3_WINDOW_END_M
                       > MISSION_PERCEPTION_ROUTE_3_WINDOW_START_M)
                   && ((MISSION_PERCEPTION_APPLY_GUIDANCE_ENABLE == 0U)
                       || (MISSION_PERCEPTION_APPLY_GUIDANCE_ENABLE == 1U))
                   && ((MISSION_PERCEPTION_MINE_ACTION_ENABLE == 0U)
                       || (MISSION_PERCEPTION_MINE_ACTION_ENABLE == 1U))
                   && (MISSION_PERCEPTION_MINE_CENTER_TOLERANCE_NORM
                       > 0.0f)
                   && (MISSION_PERCEPTION_MINE_CENTER_TOLERANCE_NORM
                       <= 1.0f)
                   && ((MISSION_PERCEPTION_MINE_GUARD_REQUIRED == 0U)
                       || (MISSION_PERCEPTION_MINE_GUARD_REQUIRED == 1U))
                   && ((TERRAIN_VISION_CALIBRATED == 0U)
                       || (TERRAIN_VISION_CALIBRATED == 1U)));
}

static uint8 mission_context_is_valid(
    const mission_perception_context_t *context)
{
    if ((0 == context)
        || (context->task <= MISSION_PERCEPTION_TASK_NONE)
        || (context->task > MISSION_PERCEPTION_TASK_TERRAIN)
        || (context->distance_source
            > MISSION_PERCEPTION_DISTANCE_TRAVELED)
        || !mission_float_is_finite(context->navigation_window_start_m)
        || !mission_float_is_finite(context->navigation_window_end_m)
        || !mission_float_is_finite(context->mine_target_turns)
        || (context->route_id > 3U)
        || (context->navigation_window_start_m < 0.0f)
        || (context->navigation_window_end_m
            <= context->navigation_window_start_m))
    {
        return 0U;
    }
    if (MISSION_PERCEPTION_TASK_MINEFIELD == context->task)
    {
        if ((context->rotation_direction
             > MISSION_PERCEPTION_ROTATION_CCW)
            || (context->mine_target_turns
                < MISSION_PERCEPTION_MINE_MIN_TURNS)
            || (context->mine_target_turns
                > MISSION_PERCEPTION_MINE_MAX_TURNS))
        {
            return 0U;
        }
    }
    return 1U;
}

static void mission_publish_context(
    const mission_perception_context_t *context)
{
    uint8 next_index = mission_context_index ^ 1U;

    mission_next_context_sequence++;
    if (0U == mission_next_context_sequence)
    {
        mission_next_context_sequence = 1U;
    }
    mission_context[next_index].value = *context;
    mission_context[next_index].sequence = mission_next_context_sequence;
    mission_context_index = next_index;
}

static mission_published_context_t mission_get_context(void)
{
    uint8 published_index = mission_context_index;

    return mission_context[published_index];
}

static void mission_apply_context(
    const mission_published_context_t *published)
{
    memset(&mission_state, 0, sizeof(mission_state));
    mission_state.status = (MISSION_PERCEPTION_TASK_NONE
                            == published->value.task)
        ? MISSION_PERCEPTION_STATUS_IDLE
        : MISSION_PERCEPTION_STATUS_WAITING_FOR_NAVIGATION;
    mission_state.task = published->value.task;
    mission_state.distance_source = published->value.distance_source;
    mission_state.rotation_direction = published->value.rotation_direction;
    mission_state.context_sequence = published->sequence;
    mission_state.navigation_window_start_m
        = published->value.navigation_window_start_m;
    mission_state.navigation_window_end_m
        = published->value.navigation_window_end_m;
    mission_state.mine_target_turns = published->value.mine_target_turns;
    mission_state.route_id = published->value.route_id;
    mission_state.visual_age_ms = 0xFFFFFFFFU;
    mission_state.terrain_type = TERRAIN_TYPE_NORMAL;
    mission_visual_source_seen = 0U;
    mission_visual_filter_initialized = 0U;
    mission_mine_yaw_initialized = 0U;
    mission_last_mine_yaw_rad = 0.0f;
}

static uint8 mission_navigation_window_is_active(
    const mission_perception_context_t *context,
    const navigation_state_t *navigation)
{
    float distance;

    if (MISSION_PERCEPTION_DISTANCE_ROUTE == context->distance_source)
    {
        if ((NAVIGATION_MODE_REPLAYING != navigation->mode)
            && (NAVIGATION_MODE_REPLAY_COMPLETE != navigation->mode))
        {
            mission_state.navigation_distance_m
                = navigation->route_distance_m;
            return 0U;
        }
        distance = navigation->route_distance_m;
    }
    else
    {
        distance = navigation->traveled_distance_m;
    }
    mission_state.navigation_distance_m = distance;
    return (uint8)((distance >= context->navigation_window_start_m)
                   && (distance <= context->navigation_window_end_m));
}

static void mission_set_navigation_state(
    const navigation_state_t *navigation)
{
    mission_state.navigation_yaw_rad = navigation->yaw_rad;
    mission_state.navigation_yaw_rate_rad_s
        = navigation->fused_yaw_rate_rad_s;
    mission_state.navigation_yaw_rate_command_rad_s
        = navigation->target_yaw_rate_rad_s;
    mission_state.visual_yaw_rate_correction_rad_s = 0.0f;
    mission_state.suggested_yaw_rate_rad_s = mission_clamp(
        navigation->target_yaw_rate_rad_s,
        -MISSION_PERCEPTION_MAX_SUGGESTED_YAW_RATE_RAD_S,
        MISSION_PERCEPTION_MAX_SUGGESTED_YAW_RATE_RAD_S);
    mission_state.advisory_valid = 0U;
}

static void mission_update_visual_freshness(uint32 now_ms)
{
    if (mission_state.accepted_visual_count > 0U)
    {
        mission_state.visual_age_ms
            = now_ms - mission_state.last_visual_timestamp_ms;
        mission_state.visual_fresh = (uint8)(mission_state.visual_age_ms
            <= MISSION_PERCEPTION_VISUAL_TIMEOUT_MS);
    }
    else
    {
        mission_state.visual_age_ms = 0xFFFFFFFFU;
        mission_state.visual_fresh = 0U;
    }
    if (!mission_state.visual_fresh)
    {
        mission_state.visual_observation_valid = 0U;
        mission_state.mine_center_valid = 0U;
        mission_state.mine_boundary_guard_valid = 0U;
        mission_state.mine_boundary_warning = 0U;
        mission_visual_filter_initialized = 0U;
    }
}

static mission_perception_status_t mission_finish_visual_status(void)
{
    if (!mission_state.navigation_window_active)
    {
        mission_state.status
            = MISSION_PERCEPTION_STATUS_WAITING_FOR_NAVIGATION;
    }
    else if (0U == mission_state.accepted_visual_count)
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_WAITING_FOR_VISION;
    }
    else if (!mission_state.visual_fresh)
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_VISION_STALE;
    }
    else if (!mission_state.visual_calibrated)
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_UNCALIBRATED;
    }
    else if (!mission_state.visual_observation_valid)
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_WAITING_FOR_VISION;
    }
    else
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_OK;
    }
    return mission_state.status;
}

static mission_perception_status_t mission_update_cone(
    const cone_vision_result_t *cone,
    const navigation_state_t *navigation,
    uint32 now_ms)
{
    uint8 new_frame = (uint8)(!mission_visual_source_seen
        || (cone->frame_count != mission_state.last_visual_frame_count));

    if (new_frame)
    {
        mission_visual_source_seen = 1U;
        mission_state.last_visual_frame_count = cone->frame_count;
        if ((CONE_VISION_STATUS_OK == cone->status)
            && cone->enabled && (cone->frame_count > 0U))
        {
            mission_state.accepted_visual_count++;
            mission_state.last_visual_timestamp_ms = now_ms;
            mission_state.visual_calibrated = cone->calibrated;
            mission_state.visual_confidence = cone->gap_quality;
            mission_state.raw_visual_error_norm
                = cone->gap_center_error_norm;
            mission_state.visual_observation_valid = (uint8)(cone->gap_valid
                && (cone->gap_quality
                    >= MISSION_PERCEPTION_CONE_MIN_QUALITY));
            if (mission_state.visual_observation_valid)
            {
                if (!mission_visual_filter_initialized)
                {
                    mission_state.filtered_visual_error_norm
                        = cone->gap_center_error_norm;
                    mission_visual_filter_initialized = 1U;
                }
                else
                {
                    mission_state.filtered_visual_error_norm +=
                        MISSION_PERCEPTION_CONE_FILTER_ALPHA
                        * (cone->gap_center_error_norm
                           - mission_state.filtered_visual_error_norm);
                }
            }
        }
        else
        {
            mission_state.rejected_visual_count++;
            mission_state.visual_observation_valid = 0U;
        }
    }
    mission_update_visual_freshness(now_ms);
    (void)mission_finish_visual_status();
    if (MISSION_PERCEPTION_STATUS_OK == mission_state.status)
    {
        mission_state.visual_yaw_rate_correction_rad_s = mission_clamp(
            MISSION_PERCEPTION_CONE_YAW_KP
            * mission_state.filtered_visual_error_norm,
            -MISSION_PERCEPTION_MAX_CONE_YAW_RATE_RAD_S,
            MISSION_PERCEPTION_MAX_CONE_YAW_RATE_RAD_S);
        mission_state.suggested_yaw_rate_rad_s = mission_clamp(
            navigation->target_yaw_rate_rad_s
            + mission_state.visual_yaw_rate_correction_rad_s,
            -MISSION_PERCEPTION_MAX_SUGGESTED_YAW_RATE_RAD_S,
            MISSION_PERCEPTION_MAX_SUGGESTED_YAW_RATE_RAD_S);
        mission_state.advisory_valid = 1U;
    }
    return mission_state.status;
}

static void mission_update_mine_rotation(
    const mission_perception_context_t *context,
    const navigation_state_t *navigation)
{
    float yaw_delta;
    float signed_delta;

    mission_state.mine_rotation_progress_valid
        = mission_state.navigation_window_active;
    if (!mission_state.navigation_window_active)
    {
        mission_mine_yaw_initialized = 0U;
        return;
    }
    if (!mission_mine_yaw_initialized)
    {
        mission_last_mine_yaw_rad = navigation->yaw_rad;
        mission_mine_yaw_initialized = 1U;
        return;
    }

    yaw_delta = mission_wrap_angle(navigation->yaw_rad
                                   - mission_last_mine_yaw_rad);
    mission_last_mine_yaw_rad = navigation->yaw_rad;
    if (fabsf(yaw_delta) > MISSION_PERCEPTION_MAX_YAW_STEP_RAD)
    {
        mission_state.rotation_discontinuity_count++;
        return;
    }
    signed_delta = (MISSION_PERCEPTION_ROTATION_CCW
                    == context->rotation_direction)
        ? yaw_delta : -yaw_delta;
    mission_state.mine_rotation_progress_rad += signed_delta;
    if (mission_state.mine_rotation_progress_rad < 0.0f)
    {
        mission_state.mine_rotation_progress_rad = 0.0f;
    }
    mission_state.mine_rotation_progress_turns
        = mission_state.mine_rotation_progress_rad
          / MISSION_PERCEPTION_TWO_PI;
    mission_state.mine_rotation_complete = (uint8)(
        mission_state.mine_rotation_progress_turns
        >= context->mine_target_turns);
}

static mission_perception_status_t mission_update_minefield(
    const mission_perception_context_t *context,
    const minefield_vision_result_t *minefield,
    const navigation_state_t *navigation,
    uint32 now_ms)
{
    uint8 new_frame = (uint8)(!mission_visual_source_seen
        || (minefield->frame_count
            != mission_state.last_visual_frame_count));

    if (new_frame)
    {
        mission_visual_source_seen = 1U;
        mission_state.last_visual_frame_count = minefield->frame_count;
        if ((MINEFIELD_VISION_STATUS_OK == minefield->status)
            && minefield->enabled && (minefield->frame_count > 0U))
        {
            mission_state.accepted_visual_count++;
            mission_state.last_visual_timestamp_ms = now_ms;
            mission_state.visual_calibrated = minefield->calibrated;
            mission_state.visual_confidence = minefield->confidence;
            mission_state.mine_center_error_norm
                = minefield->center_error_norm;
            mission_state.mine_boundary_proximity_norm
                = minefield->boundary_proximity_norm;
            mission_state.mine_center_valid = minefield->center_valid;
            mission_state.visual_observation_valid = (uint8)(
                minefield->frame_candidate || minefield->boundary_visible);
            mission_state.mine_boundary_guard_valid = (uint8)(
                minefield->boundary_visible);
            mission_state.mine_boundary_warning = (uint8)(
                minefield->boundary_visible && minefield->boundary_warning);
        }
        else
        {
            mission_state.rejected_visual_count++;
            mission_state.visual_observation_valid = 0U;
            mission_state.mine_center_valid = 0U;
            mission_state.mine_boundary_guard_valid = 0U;
            mission_state.mine_boundary_warning = 0U;
        }
    }
    mission_update_visual_freshness(now_ms);
    mission_update_mine_rotation(context, navigation);
    (void)mission_finish_visual_status();
    if ((MISSION_PERCEPTION_STATUS_OK != mission_state.status)
        || !mission_state.navigation_window_active
        || !mission_state.visual_calibrated)
    {
        mission_state.mine_center_valid = 0U;
        mission_state.mine_boundary_guard_valid = 0U;
        mission_state.mine_boundary_warning = 0U;
    }
    return mission_state.status;
}

static mission_perception_status_t mission_update_terrain(
    const perception_fusion_state_t *terrain_fusion)
{
    mission_state.last_visual_frame_count
        = terrain_fusion->last_vision_frame_count;
    mission_state.last_visual_timestamp_ms
        = terrain_fusion->last_vision_timestamp_ms;
    mission_state.visual_age_ms = terrain_fusion->vision_age_ms;
    mission_state.accepted_visual_count
        = terrain_fusion->accepted_vision_count;
    mission_state.rejected_visual_count
        = terrain_fusion->rejected_vision_count;
    mission_state.visual_fresh = terrain_fusion->vision_fresh;
    mission_state.visual_calibrated = TERRAIN_VISION_CALIBRATED ? 1U : 0U;
    mission_state.visual_confidence = terrain_fusion->fused_confidence;
    mission_state.raw_visual_error_norm
        = terrain_fusion->raw_path_center_error_norm;
    mission_state.filtered_visual_error_norm
        = terrain_fusion->filtered_path_center_error_norm;
    mission_state.terrain_type = terrain_fusion->terrain_type;
    mission_state.visual_observation_valid = (uint8)(
        terrain_fusion->path_valid && terrain_fusion->guidance_valid);
    (void)mission_finish_visual_status();
    if (MISSION_PERCEPTION_STATUS_OK == mission_state.status)
    {
        mission_state.visual_yaw_rate_correction_rad_s
            = terrain_fusion->visual_yaw_rate_correction_rad_s;
        mission_state.suggested_yaw_rate_rad_s
            = terrain_fusion->suggested_yaw_rate_rad_s;
        mission_state.advisory_valid = 1U;
    }
    return mission_state.status;
}

mission_perception_status_t mission_perception_init(void)
{
    mission_perception_context_t idle_context;

    memset(&mission_state, 0, sizeof(mission_state));
    memset(mission_context, 0, sizeof(mission_context));
    mission_context_index = 0U;
    mission_next_context_sequence = 0U;
    mission_initialized = 0U;
    mission_visual_source_seen = 0U;
    mission_visual_filter_initialized = 0U;
    mission_mine_yaw_initialized = 0U;
    mission_last_mine_yaw_rad = 0.0f;
    mission_state.visual_age_ms = 0xFFFFFFFFU;

    if (!MISSION_PERCEPTION_ENABLE)
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_DISABLED;
        return mission_state.status;
    }
    if (!mission_config_is_valid())
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_INVALID_CONFIG;
        return mission_state.status;
    }

    memset(&idle_context, 0, sizeof(idle_context));
    mission_context[0].value = idle_context;
    mission_context[0].sequence = 0U;
    mission_initialized = 1U;
    mission_state.status = MISSION_PERCEPTION_STATUS_IDLE;
    return mission_state.status;
}

mission_perception_status_t mission_perception_start(
    const mission_perception_context_t *context)
{
    if (!mission_initialized)
    {
        return MISSION_PERCEPTION_STATUS_NOT_INITIALIZED;
    }
    if (!mission_context_is_valid(context))
    {
        return MISSION_PERCEPTION_STATUS_INVALID_ARGUMENT;
    }
    mission_publish_context(context);
    return MISSION_PERCEPTION_STATUS_OK;
}

mission_perception_status_t mission_perception_start_route(uint8 route_id)
{
    mission_perception_context_t context;

    if (!MISSION_PERCEPTION_ENABLE)
    {
        return MISSION_PERCEPTION_STATUS_DISABLED;
    }
    if (!mission_initialized)
    {
        return MISSION_PERCEPTION_STATUS_NOT_INITIALIZED;
    }
    memset(&context, 0, sizeof(context));
    context.route_id = route_id;
    context.distance_source = MISSION_PERCEPTION_DISTANCE_ROUTE;
    context.rotation_direction = MISSION_PERCEPTION_ROTATION_CCW;
    context.mine_target_turns = MISSION_PERCEPTION_MINE_MIN_TURNS;
    switch (route_id)
    {
        case 1U:
            context.task = MISSION_PERCEPTION_TASK_CONE_SLALOM;
            context.navigation_window_start_m
                = MISSION_PERCEPTION_ROUTE_1_WINDOW_START_M;
            context.navigation_window_end_m
                = MISSION_PERCEPTION_ROUTE_1_WINDOW_END_M;
            break;

        case 2U:
            context.task = MISSION_PERCEPTION_TASK_MINEFIELD;
            context.navigation_window_start_m
                = MISSION_PERCEPTION_ROUTE_2_WINDOW_START_M;
            context.navigation_window_end_m
                = MISSION_PERCEPTION_ROUTE_2_WINDOW_END_M;
            break;

        case 3U:
            context.task = MISSION_PERCEPTION_TASK_TERRAIN;
            context.navigation_window_start_m
                = MISSION_PERCEPTION_ROUTE_3_WINDOW_START_M;
            context.navigation_window_end_m
                = MISSION_PERCEPTION_ROUTE_3_WINDOW_END_M;
            break;

        default:
            return MISSION_PERCEPTION_STATUS_INVALID_ARGUMENT;
    }
    return mission_perception_start(&context);
}

mission_perception_status_t mission_perception_configure_mine_rotation(
    mission_perception_rotation_direction_t direction,
    float turns)
{
    mission_published_context_t published;
    mission_perception_context_t context;

    if (!mission_initialized)
    {
        return MISSION_PERCEPTION_STATUS_NOT_INITIALIZED;
    }
    published = mission_get_context();
    if ((MISSION_PERCEPTION_TASK_MINEFIELD != published.value.task)
        || ((MISSION_PERCEPTION_ROTATION_CW != direction)
            && (MISSION_PERCEPTION_ROTATION_CCW != direction))
        || !mission_float_is_finite(turns)
        || (turns < MISSION_PERCEPTION_MINE_MIN_TURNS)
        || (turns > MISSION_PERCEPTION_MINE_MAX_TURNS))
    {
        return MISSION_PERCEPTION_STATUS_INVALID_ARGUMENT;
    }

    context = published.value;
    context.rotation_direction = direction;
    context.mine_target_turns = turns;
    mission_publish_context(&context);
    return MISSION_PERCEPTION_STATUS_OK;
}

void mission_perception_stop(void)
{
    mission_perception_context_t idle_context;

    if (!mission_initialized)
    {
        return;
    }
    memset(&idle_context, 0, sizeof(idle_context));
    mission_publish_context(&idle_context);
}

mission_perception_status_t mission_perception_update(
    const cone_vision_result_t *cone,
    const minefield_vision_result_t *minefield,
    const perception_fusion_state_t *terrain_fusion,
    const navigation_state_t *navigation,
    uint32 now_ms)
{
    mission_published_context_t published;

    if (!mission_initialized)
    {
        return MISSION_PERCEPTION_STATUS_NOT_INITIALIZED;
    }
    if ((0 == cone) || (0 == minefield) || (0 == terrain_fusion)
        || (0 == navigation))
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_INVALID_ARGUMENT;
        return mission_state.status;
    }

    published = mission_get_context();
    if (published.sequence != mission_state.context_sequence)
    {
        mission_apply_context(&published);
    }
    if (MISSION_PERCEPTION_TASK_NONE == published.value.task)
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_IDLE;
        return mission_state.status;
    }

    mission_set_navigation_state(navigation);
    mission_state.navigation_window_active
        = mission_navigation_window_is_active(&published.value, navigation);
    switch (published.value.task)
    {
        case MISSION_PERCEPTION_TASK_CONE_SLALOM:
            return mission_update_cone(cone, navigation, now_ms);

        case MISSION_PERCEPTION_TASK_MINEFIELD:
            return mission_update_minefield(&published.value,
                                            minefield,
                                            navigation,
                                            now_ms);

        case MISSION_PERCEPTION_TASK_TERRAIN:
            return mission_update_terrain(terrain_fusion);

        case MISSION_PERCEPTION_TASK_NONE:
        default:
            mission_state.status = MISSION_PERCEPTION_STATUS_IDLE;
            return mission_state.status;
    }
}

mission_perception_status_t mission_perception_service(
    const navigation_state_t *navigation,
    uint32 now_ms)
{
    cone_vision_result_t cone;
    minefield_vision_result_t minefield;
    const perception_fusion_state_t *terrain_fusion;

    if (!mission_initialized)
    {
        return MISSION_PERCEPTION_STATUS_NOT_INITIALIZED;
    }
    if (!cone_vision_get_snapshot(&cone)
        || !minefield_vision_get_snapshot(&minefield))
    {
        mission_state.status = MISSION_PERCEPTION_STATUS_INVALID_ARGUMENT;
        return mission_state.status;
    }
    terrain_fusion = perception_fusion_get_state();
    return mission_perception_update(&cone,
                                     &minefield,
                                     terrain_fusion,
                                     navigation,
                                     now_ms);
}

const mission_perception_state_t *mission_perception_get_state(void)
{
    return &mission_state;
}
