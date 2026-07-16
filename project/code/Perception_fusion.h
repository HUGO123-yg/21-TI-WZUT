#ifndef PROJECT_PERCEPTION_FUSION_H
#define PROJECT_PERCEPTION_FUSION_H

#include "Imu.h"
#include "Navigation.h"
#include "Terrain_vision.h"
#include "zf_common_typedef.h"

typedef enum
{
    PERCEPTION_FUSION_STATUS_OK = 0,
    PERCEPTION_FUSION_STATUS_DISABLED,
    PERCEPTION_FUSION_STATUS_WAITING_FOR_VISION,
    PERCEPTION_FUSION_STATUS_VISION_STALE,
    PERCEPTION_FUSION_STATUS_INVALID_ARGUMENT,
    PERCEPTION_FUSION_STATUS_INVALID_CONFIG
} perception_fusion_status_t;

typedef struct
{
    perception_fusion_status_t status;
    terrain_type_t terrain_type;
    terrain_bridge_side_t bridge_side;
    uint32 last_vision_frame_count;
    uint32 last_vision_timestamp_ms;
    uint32 vision_age_ms;
    uint32 accepted_vision_count;
    uint32 rejected_vision_count;
    float navigation_x_m;
    float navigation_y_m;
    float navigation_yaw_rad;
    float navigation_yaw_rate_rad_s;
    float raw_path_center_error_norm;
    float raw_path_heading_error_norm;
    float filtered_path_center_error_norm;
    float filtered_path_heading_error_norm;
    float visual_yaw_rate_correction_rad_s;
    float suggested_yaw_rate_rad_s;
    uint8 vision_confidence;
    uint8 fused_confidence;
    uint8 path_quality;
    uint8 vision_fresh;
    uint8 path_valid;
    uint8 guidance_valid;
    uint8 terrain_imu_supported;
} perception_fusion_state_t;

// This module publishes observations and an advisory yaw-rate only. It never
// calls Control_system or an actuator module.
perception_fusion_status_t perception_fusion_init(void);

// Repeated calls for the same camera frame update freshness but do not filter
// the frame twice. This frame-in API is retained for host replay tests.
perception_fusion_status_t perception_fusion_update(
    const terrain_vision_result_t *vision,
    const imu_data_t *imu,
    const navigation_state_t *navigation,
    uint32 now_ms);

// Runtime wrapper used by the low-priority 200 Hz system service. It copies a
// coherent Terrain_vision snapshot before calling the frame-in API.
perception_fusion_status_t perception_fusion_service(
    const imu_data_t *imu,
    const navigation_state_t *navigation,
    uint32 now_ms);

const perception_fusion_state_t *perception_fusion_get_state(void);

#endif
