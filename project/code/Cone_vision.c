#include "Cone_vision.h"

#include <string.h>

#include "config.h"

#define CONE_VISION_MAX_LABELS   (96U)

typedef struct
{
    uint16 pixel_count;
    uint32 gray_sum;
    uint8 polarity;
    uint8 x_min;
    uint8 x_max;
    uint8 y_min;
    uint8 y_max;
    uint8 top_x_min;
    uint8 top_x_max;
    uint8 bottom_x_min;
    uint8 bottom_x_max;
} cone_component_t;

static cone_vision_result_t cone_result;
static cone_vision_result_t cone_published_result[2];
static volatile uint8 cone_published_index;
static uint32 cone_last_source_frame_count;
static uint8 cone_label[VISION_FRAME_HEIGHT][VISION_FRAME_WIDTH];
static uint8 cone_parent[CONE_VISION_MAX_LABELS];
static uint8 cone_label_polarity[CONE_VISION_MAX_LABELS];
static cone_component_t cone_component[CONE_VISION_MAX_LABELS];

static float cone_normalize_x(uint8 x)
{
    float center = ((float)VISION_FRAME_WIDTH - 1.0f) * 0.5f;

    return ((float)x - center) / center;
}

static void cone_publish_result(void)
{
    uint8 next_index = cone_published_index ^ 1U;

    cone_published_result[next_index] = cone_result;
    cone_published_index = next_index;
}

static uint8 cone_config_is_valid(void)
{
    return (uint8)((CONE_VISION_ROI_TOP < VISION_FRAME_HEIGHT)
                   && (CONE_VISION_CONTRAST_MIN > 0U)
                   && (CONE_VISION_MIN_HEIGHT_PX > 0U)
                   && (CONE_VISION_MIN_WIDTH_PX > 0U)
                   && (CONE_VISION_MIN_WIDTH_PX
                       <= CONE_VISION_MAX_WIDTH_PX)
                   && (CONE_VISION_MIN_FILL_PERCENT <= 100U)
                   && (CONE_VISION_MIN_HEIGHT_WIDTH_PERCENT > 0U)
                   && (CONE_VISION_GAP_MIN_WIDTH_PX > 0U));
}

static void cone_reset_result(void)
{
    memset(&cone_result, 0, sizeof(cone_result));
    memset(cone_published_result, 0, sizeof(cone_published_result));
    cone_published_index = 0U;
    cone_last_source_frame_count = 0U;
    cone_result.calibrated = CONE_VISION_CALIBRATED ? 1U : 0U;
    cone_published_result[0] = cone_result;
}

static uint8 cone_find_root(uint8 label)
{
    uint8 root = label;

    while (cone_parent[root] != root)
    {
        root = cone_parent[root];
    }
    while (cone_parent[label] != label)
    {
        uint8 next = cone_parent[label];

        cone_parent[label] = root;
        label = next;
    }
    return root;
}

static void cone_union_labels(uint8 first, uint8 second)
{
    uint8 first_root = cone_find_root(first);
    uint8 second_root = cone_find_root(second);

    if (first_root == second_root)
    {
        return;
    }
    if (first_root < second_root)
    {
        cone_parent[second_root] = first_root;
    }
    else
    {
        cone_parent[first_root] = second_root;
    }
}

static uint8 cone_pixel_polarity(uint8 gray, uint16 average_gray)
{
    if ((uint16)gray + CONE_VISION_CONTRAST_MIN <= average_gray)
    {
        return (uint8)CONE_VISION_POLARITY_DARK;
    }
    if ((uint16)gray >= average_gray + CONE_VISION_CONTRAST_MIN)
    {
        return (uint8)CONE_VISION_POLARITY_BRIGHT;
    }
    return (uint8)CONE_VISION_POLARITY_UNKNOWN;
}

static uint8 cone_assign_label(uint8 polarity,
                               uint8 left_label,
                               uint8 upper_label,
                               uint8 *next_label)
{
    uint8 left_valid = (uint8)(left_label
        && (cone_label_polarity[left_label] == polarity));
    uint8 upper_valid = (uint8)(upper_label
        && (cone_label_polarity[upper_label] == polarity));

    if (left_valid && upper_valid)
    {
        cone_union_labels(left_label, upper_label);
        return left_label;
    }
    if (left_valid)
    {
        return left_label;
    }
    if (upper_valid)
    {
        return upper_label;
    }
    if (*next_label >= CONE_VISION_MAX_LABELS)
    {
        return 0U;
    }

    cone_parent[*next_label] = *next_label;
    cone_label_polarity[*next_label] = polarity;
    (*next_label)++;
    return (uint8)(*next_label - 1U);
}

static uint8 cone_label_components(const vision_frame_t *frame)
{
    uint8 next_label = 1U;
    uint8 x;
    uint8 y;

    memset(cone_label, 0, sizeof(cone_label));
    memset(cone_parent, 0, sizeof(cone_parent));
    memset(cone_label_polarity, 0, sizeof(cone_label_polarity));
    for (y = CONE_VISION_ROI_TOP; y < VISION_FRAME_HEIGHT; y++)
    {
        for (x = 1U; x < VISION_FRAME_WIDTH - 1U; x++)
        {
            uint8 polarity = cone_pixel_polarity(frame->gray[y][x],
                                                  frame->average_gray);

            if (CONE_VISION_POLARITY_UNKNOWN == polarity)
            {
                continue;
            }
            cone_label[y][x] = cone_assign_label(
                polarity,
                cone_label[y][x - 1U],
                (y > CONE_VISION_ROI_TOP) ? cone_label[y - 1U][x] : 0U,
                &next_label);
        }
    }
    return next_label;
}

static void cone_initialize_components(void)
{
    uint8 label;

    memset(cone_component, 0, sizeof(cone_component));
    for (label = 0U; label < CONE_VISION_MAX_LABELS; label++)
    {
        cone_component[label].x_min = VISION_FRAME_WIDTH;
        cone_component[label].y_min = VISION_FRAME_HEIGHT;
        cone_component[label].top_x_min = VISION_FRAME_WIDTH;
        cone_component[label].bottom_x_min = VISION_FRAME_WIDTH;
    }
}

static void cone_measure_components(const vision_frame_t *frame,
                                    uint8 next_label)
{
    uint8 x;
    uint8 y;

    cone_initialize_components();
    for (y = CONE_VISION_ROI_TOP; y < VISION_FRAME_HEIGHT; y++)
    {
        for (x = 1U; x < VISION_FRAME_WIDTH - 1U; x++)
        {
            uint8 label = cone_label[y][x];
            cone_component_t *component;

            if ((0U == label) || (label >= next_label))
            {
                continue;
            }
            label = cone_find_root(label);
            cone_label[y][x] = label;
            component = &cone_component[label];
            component->pixel_count++;
            component->gray_sum += frame->gray[y][x];
            component->polarity = cone_label_polarity[label];
            if (x < component->x_min)
            {
                component->x_min = x;
            }
            if (x > component->x_max)
            {
                component->x_max = x;
            }
            if (y < component->y_min)
            {
                component->y_min = y;
            }
            if (y > component->y_max)
            {
                component->y_max = y;
            }
        }
    }

    for (y = CONE_VISION_ROI_TOP; y < VISION_FRAME_HEIGHT; y++)
    {
        for (x = 1U; x < VISION_FRAME_WIDTH - 1U; x++)
        {
            uint8 label = cone_label[y][x];
            cone_component_t *component;
            uint8 third;

            if ((0U == label) || (label >= next_label))
            {
                continue;
            }
            component = &cone_component[label];
            third = (component->y_max - component->y_min + 1U) / 3U;
            if (0U == third)
            {
                third = 1U;
            }
            if (y < component->y_min + third)
            {
                if (x < component->top_x_min)
                {
                    component->top_x_min = x;
                }
                if (x > component->top_x_max)
                {
                    component->top_x_max = x;
                }
            }
            if (y > component->y_max - third)
            {
                if (x < component->bottom_x_min)
                {
                    component->bottom_x_min = x;
                }
                if (x > component->bottom_x_max)
                {
                    component->bottom_x_max = x;
                }
            }
        }
    }
}

static uint8 cone_component_confidence(const cone_component_t *component,
                                       uint8 width,
                                       uint8 height,
                                       uint8 top_width,
                                       uint8 bottom_width,
                                       uint16 average_gray)
{
    uint16 component_gray = (uint16)(component->gray_sum
                                     / component->pixel_count);
    uint16 contrast = (component_gray > average_gray)
        ? (component_gray - average_gray) : (average_gray - component_gray);
    uint16 fill = (uint16)((uint32)component->pixel_count * 100U
                           / ((uint16)width * height));
    uint16 confidence = 0U;

    confidence += (contrast > 30U) ? 30U : contrast;
    confidence += (height >= 13U) ? 25U : (uint16)height * 2U;
    confidence += (fill >= 60U) ? 20U : fill / 3U;
    if (bottom_width >= top_width)
    {
        confidence += 25U;
    }
    return (confidence > 100U) ? 100U : (uint8)confidence;
}

static uint8 cone_make_candidate(const cone_component_t *component,
                                 uint16 average_gray,
                                 cone_vision_candidate_t *candidate)
{
    uint8 width;
    uint8 height;
    uint8 top_width;
    uint8 bottom_width;
    uint16 fill;

    if ((0U == component->pixel_count)
        || (component->x_min > component->x_max)
        || (component->y_min > component->y_max))
    {
        return 0U;
    }
    width = component->x_max - component->x_min + 1U;
    height = component->y_max - component->y_min + 1U;
    top_width = (component->top_x_min <= component->top_x_max)
        ? (component->top_x_max - component->top_x_min + 1U) : 0U;
    bottom_width = (component->bottom_x_min <= component->bottom_x_max)
        ? (component->bottom_x_max - component->bottom_x_min + 1U) : 0U;
    fill = (uint16)((uint32)component->pixel_count * 100U
                    / ((uint16)width * height));
    if ((height < CONE_VISION_MIN_HEIGHT_PX)
        || (width < CONE_VISION_MIN_WIDTH_PX)
        || (width > CONE_VISION_MAX_WIDTH_PX)
        || ((uint16)height * 100U
            < (uint16)width * CONE_VISION_MIN_HEIGHT_WIDTH_PERCENT)
        || (fill < CONE_VISION_MIN_FILL_PERCENT)
        || (bottom_width < top_width))
    {
        return 0U;
    }

    memset(candidate, 0, sizeof(*candidate));
    candidate->polarity = (cone_vision_polarity_t)component->polarity;
    candidate->x_min = component->x_min;
    candidate->x_max = component->x_max;
    candidate->y_min = component->y_min;
    candidate->y_max = component->y_max;
    candidate->center_x_px = (component->x_min + component->x_max) / 2U;
    candidate->bottom_y_px = component->y_max;
    candidate->width_px = width;
    candidate->height_px = height;
    candidate->top_width_px = top_width;
    candidate->bottom_width_px = bottom_width;
    candidate->confidence = cone_component_confidence(component,
                                                       width,
                                                       height,
                                                       top_width,
                                                       bottom_width,
                                                       average_gray);
    candidate->center_error_norm = cone_normalize_x(candidate->center_x_px);
    return 1U;
}

static void cone_store_candidate(const cone_vision_candidate_t *candidate)
{
    uint8 index;
    uint8 weakest = 0U;

    if (cone_result.candidate_count < CONE_VISION_MAX_CANDIDATES)
    {
        cone_result.candidate[cone_result.candidate_count] = *candidate;
        cone_result.candidate_count++;
        return;
    }
    for (index = 1U; index < cone_result.candidate_count; index++)
    {
        if (cone_result.candidate[index].confidence
            < cone_result.candidate[weakest].confidence)
        {
            weakest = index;
        }
    }
    if (candidate->confidence > cone_result.candidate[weakest].confidence)
    {
        cone_result.candidate[weakest] = *candidate;
    }
}

static void cone_sort_candidates(void)
{
    uint8 i;
    uint8 j;

    for (i = 1U; i < cone_result.candidate_count; i++)
    {
        cone_vision_candidate_t candidate = cone_result.candidate[i];

        j = i;
        while ((j > 0U)
               && (cone_result.candidate[j - 1U].center_x_px
                   > candidate.center_x_px))
        {
            cone_result.candidate[j] = cone_result.candidate[j - 1U];
            j--;
        }
        cone_result.candidate[j] = candidate;
    }
}

static void cone_select_gap(void)
{
    uint8 best_quality = 0U;
    uint8 index;

    cone_result.gap_valid = 0U;
    cone_result.gap_center_error_norm = 0.0f;
    for (index = 1U; index < cone_result.candidate_count; index++)
    {
        const cone_vision_candidate_t *left
            = &cone_result.candidate[index - 1U];
        const cone_vision_candidate_t *right
            = &cone_result.candidate[index];
        uint8 gap_width;
        uint8 quality;

        if (right->x_min <= left->x_max + CONE_VISION_GAP_MIN_WIDTH_PX)
        {
            continue;
        }
        gap_width = right->x_min - left->x_max - 1U;
        quality = (left->confidence < right->confidence)
            ? left->confidence : right->confidence;
        if (gap_width >= VISION_FRAME_WIDTH / 2U)
        {
            quality /= 2U;
        }
        if (!cone_result.gap_valid || (quality > best_quality))
        {
            cone_result.gap_valid = 1U;
            cone_result.gap_left_candidate = index - 1U;
            cone_result.gap_right_candidate = index;
            cone_result.gap_center_px = (left->x_max + right->x_min) / 2U;
            cone_result.gap_width_px = gap_width;
            cone_result.gap_quality = quality;
            cone_result.gap_center_error_norm
                = cone_normalize_x(cone_result.gap_center_px);
            best_quality = quality;
        }
    }
}

cone_vision_status_t cone_vision_init(void)
{
    vision_frame_status_t frame_status;

    cone_reset_result();
    if (!CONE_VISION_ENABLE)
    {
        cone_result.status = CONE_VISION_STATUS_DISABLED;
        cone_publish_result();
        return cone_result.status;
    }
    if (!cone_config_is_valid())
    {
        cone_result.status = CONE_VISION_STATUS_INVALID_CONFIG;
        cone_publish_result();
        return cone_result.status;
    }
    frame_status = vision_frame_init();
    if (VISION_FRAME_STATUS_DISABLED == frame_status)
    {
        cone_result.status = CONE_VISION_STATUS_DISABLED;
        cone_publish_result();
        return cone_result.status;
    }
    if (VISION_FRAME_STATUS_INVALID_CONFIG == frame_status)
    {
        cone_result.status = CONE_VISION_STATUS_INVALID_CONFIG;
        cone_publish_result();
        return cone_result.status;
    }
    if (VISION_FRAME_STATUS_OK != frame_status)
    {
        cone_result.status = CONE_VISION_STATUS_CAMERA_ERROR;
        cone_publish_result();
        return cone_result.status;
    }

    cone_result.enabled = 1U;
    cone_result.status = CONE_VISION_STATUS_WAITING_FOR_FRAME;
    cone_publish_result();
    return cone_result.status;
}

cone_vision_status_t cone_vision_process_frame(const vision_frame_t *frame)
{
    cone_vision_candidate_t candidate;
    uint8 next_label;
    uint8 label;

    if ((0 == frame) || !frame->enabled
        || (VISION_FRAME_STATUS_OK != frame->status)
        || (0U == frame->frame_count))
    {
        cone_result.status = CONE_VISION_STATUS_INVALID_ARGUMENT;
        cone_publish_result();
        return cone_result.status;
    }

    memset(cone_result.candidate, 0, sizeof(cone_result.candidate));
    cone_result.candidate_count = 0U;
    cone_result.gap_valid = 0U;
    cone_result.gap_quality = 0U;
    cone_result.gap_width_px = 0U;
    cone_result.gap_center_px = 0U;
    cone_result.gap_left_candidate = 0U;
    cone_result.gap_right_candidate = 0U;
    cone_result.gap_center_error_norm = 0.0f;
    next_label = cone_label_components(frame);
    cone_measure_components(frame, next_label);
    for (label = 1U; label < next_label; label++)
    {
        if ((cone_parent[label] == label)
            && cone_make_candidate(&cone_component[label],
                                   frame->average_gray,
                                   &candidate))
        {
            cone_store_candidate(&candidate);
        }
    }
    cone_sort_candidates();
    cone_select_gap();
    cone_result.frame_count = frame->frame_count;
    cone_result.dropped_frame_count = frame->dropped_frame_count;
    cone_last_source_frame_count = frame->frame_count;
    cone_result.status = CONE_VISION_STATUS_OK;
    cone_publish_result();
    return cone_result.status;
}

void cone_vision_task(void)
{
    const vision_frame_t *frame;

    if (!cone_result.enabled)
    {
        return;
    }
    (void)vision_frame_task();
    frame = vision_frame_get_latest();
    if ((0U == frame->frame_count)
        || (frame->frame_count == cone_last_source_frame_count))
    {
        return;
    }
    (void)cone_vision_process_frame(frame);
}

const cone_vision_result_t *cone_vision_get_result(void)
{
    return &cone_published_result[cone_published_index];
}

uint8 cone_vision_get_snapshot(cone_vision_result_t *result)
{
    uint8 published_index;

    if (0 == result)
    {
        return 0U;
    }
    published_index = cone_published_index;
    *result = cone_published_result[published_index];
    return 1U;
}
