#include "Perception_fusion.h"

#include <math.h>
#include <string.h>

#include "config.h"

static perception_fusion_state_t fusion_state;
static uint8 fusion_initialized;
static uint8 vision_input_seen;
static uint8 path_filter_initialized;

static float perception_clamp(float value, float minimum, float maximum)
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

static uint8 perception_config_is_valid(void)
{
    return (uint8)((PERCEPTION_FUSION_PATH_FILTER_ALPHA > 0.0f)
                   && (PERCEPTION_FUSION_PATH_FILTER_ALPHA <= 1.0f)
                   && (PERCEPTION_FUSION_VISION_TIMEOUT_MS > 0U)
                   && (PERCEPTION_FUSION_PATH_MIN_QUALITY <= 100U)
                   && (PERCEPTION_FUSION_MAX_VISION_YAW_RATE_RAD_S > 0.0f)
                   && (PERCEPTION_FUSION_MAX_SUGGESTED_YAW_RATE_RAD_S > 0.0f)
                   && (PERCEPTION_FUSION_IMU_CONFIDENCE_BONUS <= 100U)
                   && (PERCEPTION_FUSION_BUMPY_ACCEL_DELTA_G >= 0.0f)
                   && (PERCEPTION_FUSION_BRIDGE_ROLL_DEG >= 0.0f)
                   && (PERCEPTION_FUSION_STEP_PITCH_DEG >= 0.0f));
}

static uint8 perception_imu_supports_terrain(terrain_type_t terrain,
                                             const imu_data_t *imu)
{
    float acceleration_delta_g;

    acceleration_delta_g = fabsf(imu->acc_norm_g - 1.0f);
    switch (terrain)
    {
        case TERRAIN_TYPE_BUMPY:
            return (uint8)(acceleration_delta_g
                           >= PERCEPTION_FUSION_BUMPY_ACCEL_DELTA_G);

        case TERRAIN_TYPE_BRIDGE:
            return (uint8)(fabsf(imu->roll_deg)
                           >= PERCEPTION_FUSION_BRIDGE_ROLL_DEG);

        case TERRAIN_TYPE_STEP:
            return (uint8)((fabsf(imu->pitch_deg)
                            >= PERCEPTION_FUSION_STEP_PITCH_DEG)
                           || (acceleration_delta_g
                               >= PERCEPTION_FUSION_BUMPY_ACCEL_DELTA_G));

        case TERRAIN_TYPE_NORMAL:
        case TERRAIN_TYPE_OBSTACLE:
        case TERRAIN_TYPE_LOST:
        default:
            return 0U;
    }
}

static void perception_accept_path(const terrain_vision_result_t *vision,
                                   uint8 reseed_filter)
{
    fusion_state.raw_path_center_error_norm
        = vision->path_center_error_norm;
    fusion_state.raw_path_heading_error_norm
        = vision->path_heading_error_norm;
    fusion_state.path_quality = vision->path_quality;
    fusion_state.path_valid = (uint8)(vision->path_valid
        && (vision->path_quality >= PERCEPTION_FUSION_PATH_MIN_QUALITY));

    if (!fusion_state.path_valid)
    {
        return;
    }
    if (reseed_filter || !path_filter_initialized)
    {
        fusion_state.filtered_path_center_error_norm
            = vision->path_center_error_norm;
        fusion_state.filtered_path_heading_error_norm
            = vision->path_heading_error_norm;
        path_filter_initialized = 1U;
        return;
    }

    fusion_state.filtered_path_center_error_norm +=
        PERCEPTION_FUSION_PATH_FILTER_ALPHA
        * (vision->path_center_error_norm
           - fusion_state.filtered_path_center_error_norm);
    fusion_state.filtered_path_heading_error_norm +=
        PERCEPTION_FUSION_PATH_FILTER_ALPHA
        * (vision->path_heading_error_norm
           - fusion_state.filtered_path_heading_error_norm);
}

static void perception_update_guidance(const navigation_state_t *navigation)
{
    float correction;

    fusion_state.navigation_x_m = navigation->x_m;
    fusion_state.navigation_y_m = navigation->y_m;
    fusion_state.navigation_yaw_rad = navigation->yaw_rad;
    fusion_state.navigation_yaw_rate_rad_s
        = navigation->target_yaw_rate_rad_s;
    fusion_state.guidance_valid = (uint8)(fusion_state.vision_fresh
                                          && fusion_state.path_valid
                                          && path_filter_initialized);
    if (!fusion_state.guidance_valid)
    {
        fusion_state.visual_yaw_rate_correction_rad_s = 0.0f;
        fusion_state.suggested_yaw_rate_rad_s = perception_clamp(
            navigation->target_yaw_rate_rad_s,
            -PERCEPTION_FUSION_MAX_SUGGESTED_YAW_RATE_RAD_S,
            PERCEPTION_FUSION_MAX_SUGGESTED_YAW_RATE_RAD_S);
        return;
    }

    correction = PERCEPTION_FUSION_PATH_CENTER_YAW_KP
                 * fusion_state.filtered_path_center_error_norm
                 + PERCEPTION_FUSION_PATH_HEADING_YAW_KP
                   * fusion_state.filtered_path_heading_error_norm;
    fusion_state.visual_yaw_rate_correction_rad_s = perception_clamp(
        correction,
        -PERCEPTION_FUSION_MAX_VISION_YAW_RATE_RAD_S,
        PERCEPTION_FUSION_MAX_VISION_YAW_RATE_RAD_S);
    fusion_state.suggested_yaw_rate_rad_s = perception_clamp(
        navigation->target_yaw_rate_rad_s
        + fusion_state.visual_yaw_rate_correction_rad_s,
        -PERCEPTION_FUSION_MAX_SUGGESTED_YAW_RATE_RAD_S,
        PERCEPTION_FUSION_MAX_SUGGESTED_YAW_RATE_RAD_S);
}

perception_fusion_status_t perception_fusion_init(void)
{
    memset(&fusion_state, 0, sizeof(fusion_state));
    fusion_initialized = 0U;
    vision_input_seen = 0U;
    path_filter_initialized = 0U;
    fusion_state.vision_age_ms = 0xFFFFFFFFU;
    fusion_state.terrain_type = TERRAIN_TYPE_NORMAL;
    fusion_state.bridge_side = TERRAIN_BRIDGE_SIDE_UNKNOWN;

    if (!PERCEPTION_FUSION_ENABLE)
    {
        fusion_state.status = PERCEPTION_FUSION_STATUS_DISABLED;
        return fusion_state.status;
    }
    if (!perception_config_is_valid())
    {
        fusion_state.status = PERCEPTION_FUSION_STATUS_INVALID_CONFIG;
        return fusion_state.status;
    }

    fusion_initialized = 1U;
    fusion_state.status = PERCEPTION_FUSION_STATUS_WAITING_FOR_VISION;
    return fusion_state.status;
}

perception_fusion_status_t perception_fusion_update(
    const terrain_vision_result_t *vision,
    const imu_data_t *imu,
    const navigation_state_t *navigation,
    uint32 now_ms)
{
    uint8 new_input;
    uint8 reseed_filter;
    uint16 fused_confidence;

    if (!fusion_initialized)
    {
        return fusion_state.status;
    }
    if ((0 == vision) || (0 == imu) || (0 == navigation))
    {
        fusion_state.status = PERCEPTION_FUSION_STATUS_INVALID_ARGUMENT;
        return fusion_state.status;
    }

    new_input = (uint8)(!vision_input_seen
        || (vision->frame_count != fusion_state.last_vision_frame_count));
    if (new_input)
    {
        vision_input_seen = 1U;
        fusion_state.last_vision_frame_count = vision->frame_count;
        if ((TERRAIN_VISION_STATUS_OK == vision->status)
            && vision->enabled
            && (vision->frame_count > 0U))
        {
            reseed_filter = (uint8)(!fusion_state.vision_fresh);
            fusion_state.accepted_vision_count++;
            fusion_state.last_vision_timestamp_ms = now_ms;
            fusion_state.terrain_type = vision->type;
            fusion_state.bridge_side = vision->bridge_side;
            fusion_state.vision_confidence = vision->confidence;
            perception_accept_path(vision, reseed_filter);
        }
        else
        {
            fusion_state.rejected_vision_count++;
            fusion_state.path_valid = 0U;
        }
    }

    if (fusion_state.accepted_vision_count > 0U)
    {
        fusion_state.vision_age_ms = now_ms
                                     - fusion_state.last_vision_timestamp_ms;
        fusion_state.vision_fresh = (uint8)(fusion_state.vision_age_ms
            <= PERCEPTION_FUSION_VISION_TIMEOUT_MS);
    }
    else
    {
        fusion_state.vision_age_ms = 0xFFFFFFFFU;
        fusion_state.vision_fresh = 0U;
    }

    if (!fusion_state.vision_fresh)
    {
        path_filter_initialized = 0U;
        fusion_state.guidance_valid = 0U;
        fusion_state.terrain_imu_supported = 0U;
        fusion_state.fused_confidence = 0U;
        fusion_state.status = (fusion_state.accepted_vision_count > 0U)
            ? PERCEPTION_FUSION_STATUS_VISION_STALE
            : PERCEPTION_FUSION_STATUS_WAITING_FOR_VISION;
        perception_update_guidance(navigation);
        return fusion_state.status;
    }

    fusion_state.terrain_imu_supported = perception_imu_supports_terrain(
        fusion_state.terrain_type,
        imu);
    fused_confidence = fusion_state.vision_confidence;
    if (fusion_state.terrain_imu_supported)
    {
        fused_confidence += PERCEPTION_FUSION_IMU_CONFIDENCE_BONUS;
    }
    fusion_state.fused_confidence = (fused_confidence > 100U)
        ? 100U : (uint8)fused_confidence;
    perception_update_guidance(navigation);
    fusion_state.status = PERCEPTION_FUSION_STATUS_OK;
    return fusion_state.status;
}

perception_fusion_status_t perception_fusion_service(
    const imu_data_t *imu,
    const navigation_state_t *navigation,
    uint32 now_ms)
{
    terrain_vision_result_t vision;

    if (!terrain_vision_get_snapshot(&vision))
    {
        fusion_state.status = PERCEPTION_FUSION_STATUS_INVALID_ARGUMENT;
        return fusion_state.status;
    }
    return perception_fusion_update(&vision, imu, navigation, now_ms);
}

const perception_fusion_state_t *perception_fusion_get_state(void)
{
    return &fusion_state;
}
