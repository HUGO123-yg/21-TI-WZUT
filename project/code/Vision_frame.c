#include "Vision_frame.h"

#include <string.h>

#include "config.h"
#include "zf_device_mt9v03x.h"

static vision_frame_t published_frame[2];
static volatile uint8 published_frame_index;
static uint32 vision_frame_count;
static uint32 vision_dropped_frame_count;
static uint16 vision_exposure;
static uint8 exposure_frame_divider;
static uint8 vision_initialized;

static uint8 vision_frame_config_is_valid(void)
{
    return (uint8)((VISION_FRAME_EXPOSURE_MIN
                    <= VISION_FRAME_EXPOSURE_MAX)
                   && (VISION_FRAME_EXPOSURE_DARK_AVERAGE
                       < VISION_FRAME_EXPOSURE_BRIGHT_AVERAGE)
                   && (VISION_FRAME_EXPOSURE_UPDATE_FRAMES > 0U)
                   && (VISION_FRAME_EXPOSURE_STEP > 0U)
                   && (VISION_FRAME_EXPOSURE_STEP
                       <= VISION_FRAME_EXPOSURE_MAX
                          - VISION_FRAME_EXPOSURE_MIN));
}

static void vision_frame_set_initial_state(vision_frame_status_t status,
                                           uint8 enabled)
{
    published_frame[0].status = status;
    published_frame[0].enabled = enabled;
    published_frame[0].exposure = vision_exposure;
}

static void vision_frame_update_exposure(uint16 average_gray)
{
#if (MT9V03X_AUTO_EXP_DEF == 0)
    exposure_frame_divider++;
    if (exposure_frame_divider < VISION_FRAME_EXPOSURE_UPDATE_FRAMES)
    {
        return;
    }
    exposure_frame_divider = 0U;

    if ((average_gray > VISION_FRAME_EXPOSURE_BRIGHT_AVERAGE)
        && (vision_exposure > VISION_FRAME_EXPOSURE_MIN))
    {
        if (vision_exposure
            < VISION_FRAME_EXPOSURE_MIN + VISION_FRAME_EXPOSURE_STEP)
        {
            vision_exposure = VISION_FRAME_EXPOSURE_MIN;
        }
        else
        {
            vision_exposure -= VISION_FRAME_EXPOSURE_STEP;
        }
        (void)mt9v03x_set_exposure_time(vision_exposure);
    }
    else if ((average_gray < VISION_FRAME_EXPOSURE_DARK_AVERAGE)
             && (vision_exposure < VISION_FRAME_EXPOSURE_MAX))
    {
        if (vision_exposure
            > VISION_FRAME_EXPOSURE_MAX - VISION_FRAME_EXPOSURE_STEP)
        {
            vision_exposure = VISION_FRAME_EXPOSURE_MAX;
        }
        else
        {
            vision_exposure += VISION_FRAME_EXPOSURE_STEP;
        }
        (void)mt9v03x_set_exposure_time(vision_exposure);
    }
#else
    (void)average_gray;
#endif
}

vision_frame_status_t vision_frame_resample(const uint8 *source,
                                            uint16 source_width,
                                            uint16 source_height,
                                            uint8 *destination,
                                            uint16 *average_gray)
{
    uint32 gray_sum = 0U;
    uint16 x;
    uint16 y;

    if ((0 == source) || (0 == destination)
        || (0U == source_width) || (0U == source_height))
    {
        return VISION_FRAME_STATUS_INVALID_ARGUMENT;
    }

    for (y = 0U; y < VISION_FRAME_HEIGHT; y++)
    {
        uint16 source_y = (uint16)(((uint32)y * source_height)
                                   / VISION_FRAME_HEIGHT);

        for (x = 0U; x < VISION_FRAME_WIDTH; x++)
        {
            uint16 source_x = (uint16)(((uint32)x * source_width)
                                       / VISION_FRAME_WIDTH);
            uint8 gray = source[(uint32)source_y * source_width + source_x];

            destination[(uint32)y * VISION_FRAME_WIDTH + x] = gray;
            gray_sum += gray;
        }
    }
    if (0 != average_gray)
    {
        *average_gray = (uint16)(gray_sum
            / ((uint32)VISION_FRAME_WIDTH * VISION_FRAME_HEIGHT));
    }
    return VISION_FRAME_STATUS_OK;
}

vision_frame_status_t vision_frame_init(void)
{
    if (vision_initialized)
    {
        return VISION_FRAME_STATUS_OK;
    }

    memset(published_frame, 0, sizeof(published_frame));
    published_frame_index = 0U;
    vision_frame_count = 0U;
    vision_dropped_frame_count = 0U;
    vision_exposure = MT9V03X_EXP_TIME_DEF;
    exposure_frame_divider = 0U;
    vision_initialized = 0U;

    if (!VISION_FRAME_ENABLE)
    {
        vision_frame_set_initial_state(VISION_FRAME_STATUS_DISABLED, 0U);
        return VISION_FRAME_STATUS_DISABLED;
    }
    if (!vision_frame_config_is_valid())
    {
        vision_frame_set_initial_state(VISION_FRAME_STATUS_INVALID_CONFIG, 0U);
        return VISION_FRAME_STATUS_INVALID_CONFIG;
    }
    if (0U != mt9v03x_init())
    {
        vision_frame_set_initial_state(VISION_FRAME_STATUS_CAMERA_ERROR, 0U);
        return VISION_FRAME_STATUS_CAMERA_ERROR;
    }

#if (MT9V03X_AUTO_EXP_DEF == 0)
    (void)mt9v03x_set_exposure_time(vision_exposure);
#endif
    vision_initialized = 1U;
    vision_frame_set_initial_state(VISION_FRAME_STATUS_OK, 1U);
    return VISION_FRAME_STATUS_OK;
}

vision_frame_status_t vision_frame_task(void)
{
    vision_frame_t *next_frame;
    uint8 next_index;
    uint16 average_gray;

    if (!vision_initialized)
    {
        return published_frame[published_frame_index].status;
    }
    if (!mt9v03x_finish_flag)
    {
        return VISION_FRAME_STATUS_OK;
    }

    mt9v03x_finish_flag = 0U;
    next_index = published_frame_index ^ 1U;
    next_frame = &published_frame[next_index];
    (void)vision_frame_resample(mt9v03x_image[0],
                                MT9V03X_W,
                                MT9V03X_H,
                                next_frame->gray[0],
                                &average_gray);

    // The camera ISR may start replacing mt9v03x_image during this copy. A new
    // completion flag means the inactive buffer is mixed and must not publish.
    if (mt9v03x_finish_flag)
    {
        vision_dropped_frame_count++;
        return VISION_FRAME_STATUS_OK;
    }

    vision_frame_count++;
    vision_frame_update_exposure(average_gray);
    next_frame->status = VISION_FRAME_STATUS_OK;
    next_frame->frame_count = vision_frame_count;
    next_frame->dropped_frame_count = vision_dropped_frame_count;
    next_frame->average_gray = average_gray;
    next_frame->exposure = vision_exposure;
    next_frame->enabled = 1U;
    published_frame_index = next_index;
    return VISION_FRAME_STATUS_OK;
}

const vision_frame_t *vision_frame_get_latest(void)
{
    return &published_frame[published_frame_index];
}

uint8 vision_frame_is_initialized(void)
{
    return vision_initialized;
}
