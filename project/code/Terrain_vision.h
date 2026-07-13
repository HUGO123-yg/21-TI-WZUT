#ifndef PROJECT_TERRAIN_VISION_H
#define PROJECT_TERRAIN_VISION_H

#include "zf_common_typedef.h"

#define TERRAIN_VISION_IMAGE_WIDTH   (80U)
#define TERRAIN_VISION_IMAGE_HEIGHT  (60U)

typedef enum
{
    TERRAIN_VISION_STATUS_OK = 0,
    TERRAIN_VISION_STATUS_DISABLED,
    TERRAIN_VISION_STATUS_INVALID_ARGUMENT,
    TERRAIN_VISION_STATUS_CAMERA_ERROR
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

// Initializes MT9V03X acquisition. No display or actuator is initialized.
terrain_vision_status_t terrain_vision_init(void);

// Poll from the main loop. A frame is processed only after the camera callback
// publishes a complete image; actuator modules are never called here.
void terrain_vision_task(void);

// Frame-in API retained for recorded-image replay and host-side algorithm tests.
terrain_vision_status_t terrain_vision_process_frame(const uint8 *image,
                                                     uint16 width,
                                                     uint16 height);

const terrain_vision_result_t *terrain_vision_get_result(void);

#endif
