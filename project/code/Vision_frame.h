#ifndef PROJECT_VISION_FRAME_H
#define PROJECT_VISION_FRAME_H

#include "zf_common_typedef.h"

#define VISION_FRAME_WIDTH   (80U)
#define VISION_FRAME_HEIGHT  (60U)

typedef enum
{
    VISION_FRAME_STATUS_OK = 0,
    VISION_FRAME_STATUS_DISABLED,
    VISION_FRAME_STATUS_INVALID_ARGUMENT,
    VISION_FRAME_STATUS_INVALID_CONFIG,
    VISION_FRAME_STATUS_CAMERA_ERROR
} vision_frame_status_t;

typedef struct
{
    vision_frame_status_t status;
    uint32 frame_count;
    uint32 dropped_frame_count;
    uint16 average_gray;
    uint16 exposure;
    uint8 enabled;
    uint8 gray[VISION_FRAME_HEIGHT][VISION_FRAME_WIDTH];
} vision_frame_t;

// Owns the MT9V03X driver and exposure control. Higher-level detectors only
// consume the immutable frame returned by vision_frame_get_latest().
vision_frame_status_t vision_frame_init(void);
vision_frame_status_t vision_frame_task(void);

// Resamples an external grayscale frame into the shared detector resolution.
// This pure helper is used by offline replay and does not touch the camera.
vision_frame_status_t vision_frame_resample(const uint8 *source,
                                            uint16 source_width,
                                            uint16 source_height,
                                            uint8 *destination,
                                            uint16 *average_gray);

// The returned buffer remains immutable until the caller returns to the main
// context. Consumers use frame_count to ignore an already processed frame.
const vision_frame_t *vision_frame_get_latest(void);
uint8 vision_frame_is_initialized(void);

#endif
