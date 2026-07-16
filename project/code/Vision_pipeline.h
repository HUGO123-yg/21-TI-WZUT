#ifndef PROJECT_VISION_PIPELINE_H
#define PROJECT_VISION_PIPELINE_H

#include "zf_common_typedef.h"

typedef enum
{
    VISION_PIPELINE_STATUS_OK = 0,
    VISION_PIPELINE_STATUS_DISABLED,
    VISION_PIPELINE_STATUS_WAITING_FOR_FRAME,
    VISION_PIPELINE_STATUS_CAMERA_ERROR,
    VISION_PIPELINE_STATUS_INVALID_CONFIG,
    VISION_PIPELINE_STATUS_OBSERVER_ERROR,
    VISION_PIPELINE_STATUS_NOT_INITIALIZED
} vision_pipeline_status_t;

typedef struct
{
    vision_pipeline_status_t status;
    uint32 last_frame_count;
    uint32 processed_frame_count;
    uint32 dropped_frame_count;
    uint32 observer_error_count;
    uint32 last_terrain_status;
    uint32 last_cone_status;
    uint32 last_minefield_status;
    uint8 enabled;
    uint8 terrain_enabled;
    uint8 cone_enabled;
    uint8 minefield_enabled;
} vision_pipeline_state_t;

// Main-context scheduler: services the camera once and distributes each newly
// published immutable frame to every enabled observer exactly once.
vision_pipeline_status_t vision_pipeline_init(void);
vision_pipeline_status_t vision_pipeline_task(void);
const vision_pipeline_state_t *vision_pipeline_get_state(void);

#endif
