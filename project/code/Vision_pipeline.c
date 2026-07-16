#include "Vision_pipeline.h"

#include <string.h>

#include "Cone_vision.h"
#include "Minefield_vision.h"
#include "Terrain_vision.h"
#include "Vision_frame.h"
#include "config.h"

static vision_pipeline_state_t pipeline_state;
static uint8 pipeline_initialized;

static uint8 pipeline_terrain_status_is_enabled(
    terrain_vision_status_t status)
{
    return (uint8)(TERRAIN_VISION_STATUS_OK == status);
}

static uint8 pipeline_cone_status_is_enabled(cone_vision_status_t status)
{
    return (uint8)((CONE_VISION_STATUS_OK == status)
                   || (CONE_VISION_STATUS_WAITING_FOR_FRAME == status));
}

static uint8 pipeline_minefield_status_is_enabled(
    minefield_vision_status_t status)
{
    return (uint8)((MINEFIELD_VISION_STATUS_OK == status)
                   || (MINEFIELD_VISION_STATUS_WAITING_FOR_FRAME == status));
}

vision_pipeline_status_t vision_pipeline_init(void)
{
    terrain_vision_status_t terrain_status;
    cone_vision_status_t cone_status;
    minefield_vision_status_t minefield_status;

    memset(&pipeline_state, 0, sizeof(pipeline_state));
    pipeline_initialized = 0U;
    if (!VISION_PIPELINE_ENABLE)
    {
        pipeline_initialized = 1U;
        pipeline_state.status = VISION_PIPELINE_STATUS_DISABLED;
        return pipeline_state.status;
    }

    terrain_status = terrain_vision_detector_init();
    cone_status = cone_vision_init();
    minefield_status = minefield_vision_init();
    pipeline_state.last_terrain_status = (uint32)terrain_status;
    pipeline_state.last_cone_status = (uint32)cone_status;
    pipeline_state.last_minefield_status = (uint32)minefield_status;
    pipeline_state.terrain_enabled
        = pipeline_terrain_status_is_enabled(terrain_status);
    pipeline_state.cone_enabled
        = pipeline_cone_status_is_enabled(cone_status);
    pipeline_state.minefield_enabled
        = pipeline_minefield_status_is_enabled(minefield_status);

    pipeline_initialized = 1U;
    if ((TERRAIN_VISION_STATUS_INVALID_CONFIG == terrain_status)
        || (TERRAIN_VISION_STATUS_CAMERA_ERROR == terrain_status))
    {
        pipeline_state.observer_error_count++;
    }
    if ((CONE_VISION_STATUS_INVALID_CONFIG == cone_status)
        || (CONE_VISION_STATUS_CAMERA_ERROR == cone_status))
    {
        pipeline_state.observer_error_count++;
    }
    if ((MINEFIELD_VISION_STATUS_INVALID_CONFIG == minefield_status)
        || (MINEFIELD_VISION_STATUS_CAMERA_ERROR == minefield_status))
    {
        pipeline_state.observer_error_count++;
    }
    if (!pipeline_state.terrain_enabled
        && !pipeline_state.cone_enabled
        && !pipeline_state.minefield_enabled)
    {
        if ((TERRAIN_VISION_STATUS_CAMERA_ERROR == terrain_status)
            || (CONE_VISION_STATUS_CAMERA_ERROR == cone_status)
            || (MINEFIELD_VISION_STATUS_CAMERA_ERROR == minefield_status))
        {
            pipeline_state.status = VISION_PIPELINE_STATUS_CAMERA_ERROR;
        }
        else if ((TERRAIN_VISION_STATUS_INVALID_CONFIG == terrain_status)
                 || (CONE_VISION_STATUS_INVALID_CONFIG == cone_status)
                 || (MINEFIELD_VISION_STATUS_INVALID_CONFIG
                     == minefield_status))
        {
            pipeline_state.status = VISION_PIPELINE_STATUS_INVALID_CONFIG;
        }
        else
        {
            pipeline_state.status = VISION_PIPELINE_STATUS_DISABLED;
        }
        return pipeline_state.status;
    }

    pipeline_state.enabled = 1U;
    pipeline_state.status = VISION_PIPELINE_STATUS_WAITING_FOR_FRAME;
    return pipeline_state.status;
}

vision_pipeline_status_t vision_pipeline_task(void)
{
    const vision_frame_t *frame;
    vision_frame_status_t frame_status;
    uint8 observer_error = 0U;

    if (!pipeline_initialized)
    {
        return VISION_PIPELINE_STATUS_NOT_INITIALIZED;
    }
    if (!pipeline_state.enabled)
    {
        return pipeline_state.status;
    }

    frame_status = vision_frame_task();
    if ((VISION_FRAME_STATUS_CAMERA_ERROR == frame_status)
        || (VISION_FRAME_STATUS_INVALID_CONFIG == frame_status))
    {
        pipeline_state.status = (VISION_FRAME_STATUS_INVALID_CONFIG
                                 == frame_status)
            ? VISION_PIPELINE_STATUS_INVALID_CONFIG
            : VISION_PIPELINE_STATUS_CAMERA_ERROR;
        return pipeline_state.status;
    }
    frame = vision_frame_get_latest();
    if (!frame->enabled || (0U == frame->frame_count))
    {
        pipeline_state.status = VISION_PIPELINE_STATUS_WAITING_FOR_FRAME;
        return pipeline_state.status;
    }
    if (frame->frame_count == pipeline_state.last_frame_count)
    {
        return pipeline_state.status;
    }

    if (pipeline_state.terrain_enabled)
    {
        terrain_vision_status_t status
            = terrain_vision_process_shared_frame(frame);

        pipeline_state.last_terrain_status = (uint32)status;
        if (TERRAIN_VISION_STATUS_OK != status)
        {
            observer_error = 1U;
        }
    }
    if (pipeline_state.cone_enabled)
    {
        cone_vision_status_t status = cone_vision_process_frame(frame);

        pipeline_state.last_cone_status = (uint32)status;
        if (CONE_VISION_STATUS_OK != status)
        {
            observer_error = 1U;
        }
    }
    if (pipeline_state.minefield_enabled)
    {
        minefield_vision_status_t status
            = minefield_vision_process_frame(frame);

        pipeline_state.last_minefield_status = (uint32)status;
        if (MINEFIELD_VISION_STATUS_OK != status)
        {
            observer_error = 1U;
        }
    }

    pipeline_state.last_frame_count = frame->frame_count;
    pipeline_state.dropped_frame_count = frame->dropped_frame_count;
    pipeline_state.processed_frame_count++;
    if (observer_error)
    {
        pipeline_state.observer_error_count++;
        pipeline_state.status = VISION_PIPELINE_STATUS_OBSERVER_ERROR;
    }
    else
    {
        pipeline_state.status = VISION_PIPELINE_STATUS_OK;
    }
    return pipeline_state.status;
}

const vision_pipeline_state_t *vision_pipeline_get_state(void)
{
    return &pipeline_state;
}
