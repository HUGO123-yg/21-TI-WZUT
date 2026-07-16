#include "Minefield_vision.h"

#include <string.h>

#include "config.h"

#define MINEFIELD_VISION_MAX_LABELS   (96U)

typedef struct
{
    uint16 pixel_count;
    uint32 gray_sum;
    uint8 x_min;
    uint8 x_max;
    uint8 y_min;
    uint8 y_max;
} mine_component_t;

typedef struct
{
    uint8 valid;
    uint8 confidence;
    uint8 x_min;
    uint8 x_max;
    uint8 y_min;
    uint8 y_max;
    uint8 left_x;
    uint8 right_x;
    uint8 near_row;
    uint8 side_rows;
    uint8 horizontal_bands;
    float center_error_norm;
} mine_candidate_t;

static minefield_vision_result_t mine_result;
static minefield_vision_result_t mine_published_result[2];
static volatile uint8 mine_published_index;
static uint32 mine_last_source_frame_count;
static uint8 mine_label[VISION_FRAME_HEIGHT][VISION_FRAME_WIDTH];
static uint8 mine_parent[MINEFIELD_VISION_MAX_LABELS];
static mine_component_t mine_component[MINEFIELD_VISION_MAX_LABELS];

static float mine_normalize_x(uint8 x)
{
    float center = ((float)VISION_FRAME_WIDTH - 1.0f) * 0.5f;

    return ((float)x - center) / center;
}

static void mine_publish_result(void)
{
    uint8 next_index = mine_published_index ^ 1U;

    mine_published_result[next_index] = mine_result;
    mine_published_index = next_index;
}

static uint8 mine_config_is_valid(void)
{
    return (uint8)((MINEFIELD_VISION_ROI_TOP < VISION_FRAME_HEIGHT)
                   && (MINEFIELD_VISION_WHITE_DELTA > 0U)
                   && (MINEFIELD_VISION_WHITE_THRESHOLD_FLOOR < 255U)
                   && (MINEFIELD_VISION_MIN_WIDTH_PX > 0U)
                   && (MINEFIELD_VISION_MIN_HEIGHT_PX > 0U)
                   && (MINEFIELD_VISION_SIDE_SPAN_PERCENT <= 100U)
                   && (MINEFIELD_VISION_MIN_SIDE_ROWS_PERCENT <= 100U)
                   && (MINEFIELD_VISION_HORIZONTAL_ROW_PERCENT <= 100U)
                   && (MINEFIELD_VISION_MIN_HORIZONTAL_BANDS > 0U)
                   && (MINEFIELD_VISION_BOUNDARY_ROW_PERCENT <= 100U)
                   && (MINEFIELD_VISION_BOUNDARY_WARNING_ROW
                       < VISION_FRAME_HEIGHT));
}

static void mine_reset_result(void)
{
    memset(&mine_result, 0, sizeof(mine_result));
    memset(mine_published_result, 0, sizeof(mine_published_result));
    mine_published_index = 0U;
    mine_last_source_frame_count = 0U;
    mine_result.calibrated = MINEFIELD_VISION_CALIBRATED ? 1U : 0U;
    mine_published_result[0] = mine_result;
}

static uint8 mine_find_root(uint8 label)
{
    uint8 root = label;

    while (mine_parent[root] != root)
    {
        root = mine_parent[root];
    }
    while (mine_parent[label] != label)
    {
        uint8 next = mine_parent[label];

        mine_parent[label] = root;
        label = next;
    }
    return root;
}

static void mine_union_labels(uint8 first, uint8 second)
{
    uint8 first_root = mine_find_root(first);
    uint8 second_root = mine_find_root(second);

    if (first_root == second_root)
    {
        return;
    }
    if (first_root < second_root)
    {
        mine_parent[second_root] = first_root;
    }
    else
    {
        mine_parent[first_root] = second_root;
    }
}

static uint8 mine_assign_label(uint8 left_label,
                               uint8 upper_label,
                               uint8 *next_label)
{
    if (left_label && upper_label)
    {
        mine_union_labels(left_label, upper_label);
        return left_label;
    }
    if (left_label)
    {
        return left_label;
    }
    if (upper_label)
    {
        return upper_label;
    }
    if (*next_label >= MINEFIELD_VISION_MAX_LABELS)
    {
        return 0U;
    }

    mine_parent[*next_label] = *next_label;
    (*next_label)++;
    return (uint8)(*next_label - 1U);
}

static uint8 mine_white_threshold(const vision_frame_t *frame)
{
    uint16 threshold = frame->average_gray + MINEFIELD_VISION_WHITE_DELTA;

    if (threshold < MINEFIELD_VISION_WHITE_THRESHOLD_FLOOR)
    {
        threshold = MINEFIELD_VISION_WHITE_THRESHOLD_FLOOR;
    }
    if (threshold > 250U)
    {
        threshold = 250U;
    }
    return (uint8)threshold;
}

static uint8 mine_label_components(const vision_frame_t *frame,
                                   uint8 threshold)
{
    uint8 next_label = 1U;
    uint8 x;
    uint8 y;

    memset(mine_label, 0, sizeof(mine_label));
    memset(mine_parent, 0, sizeof(mine_parent));
    for (y = MINEFIELD_VISION_ROI_TOP; y < VISION_FRAME_HEIGHT; y++)
    {
        for (x = 1U; x < VISION_FRAME_WIDTH - 1U; x++)
        {
            if (frame->gray[y][x] < threshold)
            {
                continue;
            }
            mine_label[y][x] = mine_assign_label(
                mine_label[y][x - 1U],
                (y > MINEFIELD_VISION_ROI_TOP) ? mine_label[y - 1U][x] : 0U,
                &next_label);
        }
    }
    return next_label;
}

static void mine_measure_components(const vision_frame_t *frame,
                                    uint8 next_label)
{
    uint8 label;
    uint8 x;
    uint8 y;

    memset(mine_component, 0, sizeof(mine_component));
    for (label = 0U; label < MINEFIELD_VISION_MAX_LABELS; label++)
    {
        mine_component[label].x_min = VISION_FRAME_WIDTH;
        mine_component[label].y_min = VISION_FRAME_HEIGHT;
    }
    for (y = MINEFIELD_VISION_ROI_TOP; y < VISION_FRAME_HEIGHT; y++)
    {
        for (x = 1U; x < VISION_FRAME_WIDTH - 1U; x++)
        {
            mine_component_t *component;

            label = mine_label[y][x];
            if ((0U == label) || (label >= next_label))
            {
                continue;
            }
            label = mine_find_root(label);
            mine_label[y][x] = label;
            component = &mine_component[label];
            component->pixel_count++;
            component->gray_sum += frame->gray[y][x];
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
}

static uint8 mine_count_horizontal_bands(const uint8 *row_count,
                                         const uint8 *row_min,
                                         const uint8 *row_max,
                                         uint8 y_min,
                                         uint8 y_max,
                                         uint8 *near_row)
{
    uint8 band_count = 0U;
    uint8 in_band = 0U;
    uint8 y;

    *near_row = 0U;
    for (y = y_min; y <= y_max; y++)
    {
        uint8 span = (row_min[y] <= row_max[y])
            ? (row_max[y] - row_min[y] + 1U) : 0U;
        uint8 horizontal = (uint8)((span > 0U)
            && ((uint16)row_count[y] * 100U
                >= (uint16)span
                   * MINEFIELD_VISION_HORIZONTAL_ROW_PERCENT));

        if (horizontal)
        {
            *near_row = y;
            if (!in_band)
            {
                band_count++;
                in_band = 1U;
            }
        }
        else
        {
            in_band = 0U;
        }
    }
    return band_count;
}

static uint8 mine_evaluate_component(uint8 label,
                                     const mine_component_t *component,
                                     uint16 average_gray,
                                     mine_candidate_t *candidate)
{
    uint8 row_count[VISION_FRAME_HEIGHT];
    uint8 row_min[VISION_FRAME_HEIGHT];
    uint8 row_max[VISION_FRAME_HEIGHT];
    uint16 left_sum = 0U;
    uint16 right_sum = 0U;
    uint16 center_sum = 0U;
    uint8 side_rows = 0U;
    uint8 horizontal_bands;
    uint8 near_row;
    uint8 width;
    uint8 height;
    uint8 x;
    uint8 y;
    uint16 component_gray;
    uint16 contrast;
    uint16 side_ratio;
    uint16 confidence;

    if ((0U == component->pixel_count)
        || (component->x_min > component->x_max)
        || (component->y_min > component->y_max))
    {
        return 0U;
    }
    width = component->x_max - component->x_min + 1U;
    height = component->y_max - component->y_min + 1U;
    if ((width < MINEFIELD_VISION_MIN_WIDTH_PX)
        || (height < MINEFIELD_VISION_MIN_HEIGHT_PX))
    {
        return 0U;
    }

    memset(row_count, 0, sizeof(row_count));
    memset(row_max, 0, sizeof(row_max));
    memset(row_min, VISION_FRAME_WIDTH, sizeof(row_min));
    for (y = component->y_min; y <= component->y_max; y++)
    {
        for (x = component->x_min; x <= component->x_max; x++)
        {
            if (mine_label[y][x] != label)
            {
                continue;
            }
            row_count[y]++;
            if (x < row_min[y])
            {
                row_min[y] = x;
            }
            if (x > row_max[y])
            {
                row_max[y] = x;
            }
        }
    }
    for (y = component->y_min; y <= component->y_max; y++)
    {
        uint8 span = (row_min[y] <= row_max[y])
            ? (row_max[y] - row_min[y] + 1U) : 0U;

        if ((row_count[y] >= 2U)
            && ((uint16)span * 100U
                >= (uint16)width * MINEFIELD_VISION_SIDE_SPAN_PERCENT))
        {
            side_rows++;
            left_sum += row_min[y];
            right_sum += row_max[y];
            center_sum += (uint16)(row_min[y] + row_max[y]) / 2U;
        }
    }
    side_ratio = (uint16)side_rows * 100U / height;
    horizontal_bands = mine_count_horizontal_bands(row_count,
                                                    row_min,
                                                    row_max,
                                                    component->y_min,
                                                    component->y_max,
                                                    &near_row);
    if ((side_ratio < MINEFIELD_VISION_MIN_SIDE_ROWS_PERCENT)
        || (horizontal_bands
            < MINEFIELD_VISION_MIN_HORIZONTAL_BANDS))
    {
        return 0U;
    }

    component_gray = (uint16)(component->gray_sum / component->pixel_count);
    contrast = (component_gray > average_gray)
        ? (component_gray - average_gray) : 0U;
    confidence = (side_ratio >= 100U) ? 40U : side_ratio * 2U / 5U;
    confidence += (horizontal_bands
                   >= MINEFIELD_VISION_MIN_HORIZONTAL_BANDS) ? 30U : 0U;
    confidence += (width >= 40U) ? 15U : (uint16)width * 3U / 8U;
    confidence += (contrast >= 60U) ? 15U : contrast / 4U;

    memset(candidate, 0, sizeof(*candidate));
    candidate->valid = 1U;
    candidate->confidence = (confidence > 100U)
        ? 100U : (uint8)confidence;
    candidate->x_min = component->x_min;
    candidate->x_max = component->x_max;
    candidate->y_min = component->y_min;
    candidate->y_max = component->y_max;
    candidate->left_x = (uint8)(left_sum / side_rows);
    candidate->right_x = (uint8)(right_sum / side_rows);
    candidate->near_row = near_row;
    candidate->side_rows = side_rows;
    candidate->horizontal_bands = horizontal_bands;
    candidate->center_error_norm = mine_normalize_x(
        (uint8)(center_sum / side_rows));
    return 1U;
}

static void mine_detect_boundary_row(const vision_frame_t *frame,
                                     uint8 threshold)
{
    uint8 best_count = 0U;
    uint8 best_row = 0U;
    uint8 x;
    uint8 y;

    mine_result.boundary_visible = 0U;
    mine_result.boundary_warning = 0U;
    mine_result.boundary_proximity_norm = 0.0f;
    for (y = MINEFIELD_VISION_ROI_TOP; y < VISION_FRAME_HEIGHT; y++)
    {
        uint8 count = 0U;

        for (x = 0U; x < VISION_FRAME_WIDTH; x++)
        {
            if (frame->gray[y][x] >= threshold)
            {
                count++;
            }
        }
        if (((uint16)count * 100U
             >= (uint16)VISION_FRAME_WIDTH
                * MINEFIELD_VISION_BOUNDARY_ROW_PERCENT)
            && ((count > best_count)
                || ((count == best_count) && (y > best_row))))
        {
            best_count = count;
            best_row = y;
        }
    }
    if (best_count > 0U)
    {
        mine_result.boundary_visible = 1U;
        mine_result.near_boundary_row_px = best_row;
        mine_result.boundary_proximity_norm
            = (float)best_row / ((float)VISION_FRAME_HEIGHT - 1.0f);
        mine_result.boundary_warning = (uint8)(best_row
            >= MINEFIELD_VISION_BOUNDARY_WARNING_ROW);
    }
}

static void mine_apply_candidate(const mine_candidate_t *candidate)
{
    mine_result.frame_candidate = candidate->valid;
    mine_result.center_valid = candidate->valid;
    mine_result.confidence = candidate->confidence;
    mine_result.x_min = candidate->x_min;
    mine_result.x_max = candidate->x_max;
    mine_result.y_min = candidate->y_min;
    mine_result.y_max = candidate->y_max;
    mine_result.left_boundary_x_px = candidate->left_x;
    mine_result.right_boundary_x_px = candidate->right_x;
    mine_result.near_boundary_row_px = candidate->near_row;
    mine_result.side_valid_rows = candidate->side_rows;
    mine_result.horizontal_band_count = candidate->horizontal_bands;
    mine_result.center_error_norm = candidate->center_error_norm;
    mine_result.boundary_visible = 1U;
    mine_result.boundary_proximity_norm
        = (float)candidate->near_row
          / ((float)VISION_FRAME_HEIGHT - 1.0f);
    mine_result.boundary_warning = (uint8)(candidate->near_row
        >= MINEFIELD_VISION_BOUNDARY_WARNING_ROW);
}

minefield_vision_status_t minefield_vision_init(void)
{
    vision_frame_status_t frame_status;

    mine_reset_result();
    if (!MINEFIELD_VISION_ENABLE)
    {
        mine_result.status = MINEFIELD_VISION_STATUS_DISABLED;
        mine_publish_result();
        return mine_result.status;
    }
    if (!mine_config_is_valid())
    {
        mine_result.status = MINEFIELD_VISION_STATUS_INVALID_CONFIG;
        mine_publish_result();
        return mine_result.status;
    }
    frame_status = vision_frame_init();
    if (VISION_FRAME_STATUS_DISABLED == frame_status)
    {
        mine_result.status = MINEFIELD_VISION_STATUS_DISABLED;
        mine_publish_result();
        return mine_result.status;
    }
    if (VISION_FRAME_STATUS_INVALID_CONFIG == frame_status)
    {
        mine_result.status = MINEFIELD_VISION_STATUS_INVALID_CONFIG;
        mine_publish_result();
        return mine_result.status;
    }
    if (VISION_FRAME_STATUS_OK != frame_status)
    {
        mine_result.status = MINEFIELD_VISION_STATUS_CAMERA_ERROR;
        mine_publish_result();
        return mine_result.status;
    }

    mine_result.enabled = 1U;
    mine_result.status = MINEFIELD_VISION_STATUS_WAITING_FOR_FRAME;
    mine_publish_result();
    return mine_result.status;
}

minefield_vision_status_t minefield_vision_process_frame(
    const vision_frame_t *frame)
{
    mine_candidate_t best_candidate;
    uint8 threshold;
    uint8 next_label;
    uint8 label;

    if ((0 == frame) || !frame->enabled
        || (VISION_FRAME_STATUS_OK != frame->status)
        || (0U == frame->frame_count))
    {
        mine_result.status = MINEFIELD_VISION_STATUS_INVALID_ARGUMENT;
        mine_publish_result();
        return mine_result.status;
    }

    mine_result.frame_candidate = 0U;
    mine_result.center_valid = 0U;
    mine_result.confidence = 0U;
    mine_result.center_error_norm = 0.0f;
    mine_result.side_valid_rows = 0U;
    mine_result.horizontal_band_count = 0U;
    mine_result.x_min = 0U;
    mine_result.x_max = 0U;
    mine_result.y_min = 0U;
    mine_result.y_max = 0U;
    mine_result.left_boundary_x_px = 0U;
    mine_result.right_boundary_x_px = 0U;
    mine_result.near_boundary_row_px = 0U;
    threshold = mine_white_threshold(frame);
    mine_result.white_threshold = threshold;
    mine_detect_boundary_row(frame, threshold);
    next_label = mine_label_components(frame, threshold);
    mine_measure_components(frame, next_label);
    memset(&best_candidate, 0, sizeof(best_candidate));
    for (label = 1U; label < next_label; label++)
    {
        mine_candidate_t candidate;

        if ((mine_parent[label] == label)
            && mine_evaluate_component(label,
                                       &mine_component[label],
                                       frame->average_gray,
                                       &candidate)
            && (!best_candidate.valid
                || (candidate.confidence > best_candidate.confidence)))
        {
            best_candidate = candidate;
        }
    }
    if (best_candidate.valid)
    {
        mine_apply_candidate(&best_candidate);
    }
    mine_result.frame_count = frame->frame_count;
    mine_result.dropped_frame_count = frame->dropped_frame_count;
    mine_last_source_frame_count = frame->frame_count;
    mine_result.status = MINEFIELD_VISION_STATUS_OK;
    mine_publish_result();
    return mine_result.status;
}

void minefield_vision_task(void)
{
    const vision_frame_t *frame;

    if (!mine_result.enabled)
    {
        return;
    }
    (void)vision_frame_task();
    frame = vision_frame_get_latest();
    if ((0U == frame->frame_count)
        || (frame->frame_count == mine_last_source_frame_count))
    {
        return;
    }
    (void)minefield_vision_process_frame(frame);
}

const minefield_vision_result_t *minefield_vision_get_result(void)
{
    return &mine_published_result[mine_published_index];
}

uint8 minefield_vision_get_snapshot(minefield_vision_result_t *result)
{
    uint8 published_index;

    if (0 == result)
    {
        return 0U;
    }
    published_index = mine_published_index;
    *result = mine_published_result[published_index];
    return 1U;
}
