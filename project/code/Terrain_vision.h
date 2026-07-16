#ifndef PROJECT_TERRAIN_VISION_H
#define PROJECT_TERRAIN_VISION_H

#include "Vision_frame.h"

#define TERRAIN_VISION_IMAGE_WIDTH   VISION_FRAME_WIDTH
#define TERRAIN_VISION_IMAGE_HEIGHT  VISION_FRAME_HEIGHT

typedef enum
{
    TERRAIN_VISION_STATUS_OK = 0,
    TERRAIN_VISION_STATUS_DISABLED,
    TERRAIN_VISION_STATUS_INVALID_ARGUMENT,
    TERRAIN_VISION_STATUS_CAMERA_ERROR,
    TERRAIN_VISION_STATUS_INVALID_CONFIG
} terrain_vision_status_t;

typedef enum
{
    TERRAIN_TYPE_NORMAL = 0,
    TERRAIN_TYPE_STEP,
    TERRAIN_TYPE_BRIDGE,
    TERRAIN_TYPE_BUMPY,
    TERRAIN_TYPE_OBSTACLE,
    TERRAIN_TYPE_LOST
} terrain_type_t;

typedef enum
{
    TERRAIN_BRIDGE_SIDE_UNKNOWN = 0,
    TERRAIN_BRIDGE_SIDE_LEFT,
    TERRAIN_BRIDGE_SIDE_RIGHT
} terrain_bridge_side_t;

typedef struct
{
    terrain_vision_status_t status;
    terrain_type_t type;
    terrain_bridge_side_t bridge_side;
    uint32 frame_count;
    uint32 dropped_frame_count;
    uint16 average_gray;
    uint16 exposure;
    uint8 enabled;
    uint8 confidence;
    uint8 stable_count;
    uint8 threshold;
    uint8 road_valid_count;
    uint8 road_width_near;
    uint8 road_width_far;
    uint8 road_width_max;
    // Image-space path observation. Positive errors mean that the visible
    // path continues to the right of the camera axis. These values remain
    // normalized until camera mounting and ground projection are calibrated.
    float path_center_error_norm;
    float path_heading_error_norm;
    uint8 road_center_near_px;
    uint8 road_center_far_px;
    uint8 path_quality;
    uint8 path_valid;
    uint8 bumpy_strip_count;
    uint8 bumpy_span;
    uint8 step_band_count;
    uint8 transverse_row_count;
    uint8 bridge_score;
    uint8 bumpy_score;
    uint8 obstacle_score;
    uint8 step_score;
    uint8 raw_bridge;
    uint8 raw_bumpy;
    uint8 raw_obstacle;
    uint8 raw_step;
} terrain_vision_result_t;

// Existing application entry. It initializes the shared Vision_pipeline so
// all observers receive the same live frame; no display or actuator is used.
terrain_vision_status_t terrain_vision_init(void);

// Poll from the main loop. A frame is processed only after the camera callback
// publishes a complete image; actuator modules are never called here.
void terrain_vision_task(void);

// Detector-level entry points used by Vision_pipeline. Application code should
// keep using terrain_vision_init/task above.
terrain_vision_status_t terrain_vision_detector_init(void);
terrain_vision_status_t terrain_vision_process_shared_frame(
    const vision_frame_t *frame);

// Frame-in API retained for recorded-image replay and host-side algorithm tests.
terrain_vision_status_t terrain_vision_process_frame(const uint8 *image,
                                                     uint16 width,
                                                     uint16 height);

const terrain_vision_result_t *terrain_vision_get_result(void);

// Copies the latest fully published observation. The copy stays coherent even
// if a future consumer moves to an interrupt context; callers must not retain
// the module-owned pointer returned by terrain_vision_get_result().
uint8 terrain_vision_get_snapshot(terrain_vision_result_t *result);

#endif
