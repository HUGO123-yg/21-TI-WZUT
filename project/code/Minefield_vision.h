#ifndef PROJECT_MINEFIELD_VISION_H
#define PROJECT_MINEFIELD_VISION_H

#include "Vision_frame.h"

typedef enum
{
    MINEFIELD_VISION_STATUS_OK = 0,
    MINEFIELD_VISION_STATUS_DISABLED,
    MINEFIELD_VISION_STATUS_WAITING_FOR_FRAME,
    MINEFIELD_VISION_STATUS_INVALID_ARGUMENT,
    MINEFIELD_VISION_STATUS_INVALID_CONFIG,
    MINEFIELD_VISION_STATUS_CAMERA_ERROR
} minefield_vision_status_t;

typedef struct
{
    minefield_vision_status_t status;
    uint32 frame_count;
    uint32 dropped_frame_count;
    uint8 enabled;
    uint8 calibrated;
    uint8 white_threshold;
    uint8 frame_candidate;
    uint8 center_valid;
    uint8 boundary_visible;
    uint8 boundary_warning;
    uint8 confidence;
    uint8 x_min;
    uint8 x_max;
    uint8 y_min;
    uint8 y_max;
    uint8 left_boundary_x_px;
    uint8 right_boundary_x_px;
    uint8 near_boundary_row_px;
    uint8 side_valid_rows;
    uint8 horizontal_band_count;
    float center_error_norm;
    float boundary_proximity_norm;
} minefield_vision_result_t;

// Detects white-boundary geometry only. Mission geofencing and IMU rotation
// counting remain the responsibility of a future task decision layer.
minefield_vision_status_t minefield_vision_init(void);
void minefield_vision_task(void);
minefield_vision_status_t minefield_vision_process_frame(
    const vision_frame_t *frame);
const minefield_vision_result_t *minefield_vision_get_result(void);
uint8 minefield_vision_get_snapshot(minefield_vision_result_t *result);

#endif
