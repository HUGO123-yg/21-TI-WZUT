#ifndef PROJECT_CONE_VISION_H
#define PROJECT_CONE_VISION_H

#include "Vision_frame.h"

#define CONE_VISION_MAX_CANDIDATES   (6U)

typedef enum
{
    CONE_VISION_STATUS_OK = 0,
    CONE_VISION_STATUS_DISABLED,
    CONE_VISION_STATUS_WAITING_FOR_FRAME,
    CONE_VISION_STATUS_INVALID_ARGUMENT,
    CONE_VISION_STATUS_INVALID_CONFIG,
    CONE_VISION_STATUS_CAMERA_ERROR
} cone_vision_status_t;

typedef enum
{
    CONE_VISION_POLARITY_UNKNOWN = 0,
    CONE_VISION_POLARITY_DARK,
    CONE_VISION_POLARITY_BRIGHT
} cone_vision_polarity_t;

typedef struct
{
    cone_vision_polarity_t polarity;
    uint8 x_min;
    uint8 x_max;
    uint8 y_min;
    uint8 y_max;
    uint8 center_x_px;
    uint8 bottom_y_px;
    uint8 width_px;
    uint8 height_px;
    uint8 top_width_px;
    uint8 bottom_width_px;
    uint8 confidence;
    float center_error_norm;
} cone_vision_candidate_t;

typedef struct
{
    cone_vision_status_t status;
    uint32 frame_count;
    uint32 dropped_frame_count;
    uint8 enabled;
    uint8 calibrated;
    uint8 candidate_count;
    uint8 gap_valid;
    uint8 gap_left_candidate;
    uint8 gap_right_candidate;
    uint8 gap_center_px;
    uint8 gap_width_px;
    uint8 gap_quality;
    float gap_center_error_norm;
    cone_vision_candidate_t candidate[CONE_VISION_MAX_CANDIDATES];
} cone_vision_result_t;

// Cone observations are advisory image geometry only. This detector never
// writes navigation targets or actuator commands.
cone_vision_status_t cone_vision_init(void);
void cone_vision_task(void);

// Shared-resolution frame-in API used by replay tests and future datasets.
cone_vision_status_t cone_vision_process_frame(const vision_frame_t *frame);
const cone_vision_result_t *cone_vision_get_result(void);
uint8 cone_vision_get_snapshot(cone_vision_result_t *result);

#endif
