#include "Terrain_vision.h"

#include <string.h>

#include "config.h"
#include "Vision_frame.h"
#include "Vision_pipeline.h"

#define TERRAIN_VISION_STATE_FILTER_LENGTH  (5U)
#define TERRAIN_VISION_STRIP_MAX             (8U)
#define TERRAIN_VISION_BORDER_INTERVAL       (5U)
#define TERRAIN_VISION_DEFAULT_END_LINE      (20U)

static terrain_vision_result_t terrain_result;
static terrain_vision_result_t terrain_published_result[2];
static volatile uint8 terrain_published_index;
static uint32 terrain_last_source_frame_count;

static uint8 terrain_gray[TERRAIN_VISION_IMAGE_HEIGHT]
                         [TERRAIN_VISION_IMAGE_WIDTH];
static uint8 terrain_binary[TERRAIN_VISION_IMAGE_HEIGHT]
                           [TERRAIN_VISION_IMAGE_WIDTH];
static uint8 terrain_left_line[TERRAIN_VISION_IMAGE_HEIGHT];
static uint8 terrain_right_line[TERRAIN_VISION_IMAGE_HEIGHT];
static uint8 terrain_road_width[TERRAIN_VISION_IMAGE_HEIGHT];
static uint8 terrain_left_valid[TERRAIN_VISION_IMAGE_HEIGHT];
static uint8 terrain_right_valid[TERRAIN_VISION_IMAGE_HEIGHT];

static uint8 strip_x1[TERRAIN_VISION_STRIP_MAX];
static uint8 strip_x2[TERRAIN_VISION_STRIP_MAX];
static uint8 strip_y1[TERRAIN_VISION_STRIP_MAX];
static uint8 strip_y2[TERRAIN_VISION_STRIP_MAX];
static uint8 strip_count;

static uint8 terrain_end_line;
static uint8 terrain_lost_line;
static uint8 terrain_left_lost_line;
static uint8 terrain_right_lost_line;
static uint8 terrain_left_lost;
static uint8 terrain_right_lost;
static uint8 terrain_left_straight;
static uint8 terrain_right_straight;
static uint8 terrain_lower_left_corner;
static uint8 terrain_lower_right_corner;
static uint8 terrain_upper_left_corner;
static uint8 terrain_upper_right_corner;
static uint8 terrain_step_band_count;
static uint8 terrain_step_band_span;
static uint8 terrain_transverse_row_count;

static uint8 bridge_score;
static uint8 bumpy_score;
static uint8 obstacle_score;
static uint8 step_score;
static uint8 bridge_stable;
static uint8 bumpy_stable;
static uint8 obstacle_stable;
static uint8 step_stable;
static terrain_bridge_side_t last_bridge_side;
static uint8 terrain_state_filter[TERRAIN_VISION_STATE_FILTER_LENGTH];

static void terrain_publish_result(void)
{
    uint8 next_index = terrain_published_index ^ 1U;

    terrain_published_result[next_index] = terrain_result;
    // Publishing one byte is atomic on the target. The inactive buffer is
    // fully copied before it becomes visible to readers.
    terrain_published_index = next_index;
}

static uint8 terrain_config_is_valid(void)
{
    uint8 far_row_count;
    uint8 near_row_count;

    if ((TERRAIN_VISION_PATH_FAR_ROW_FIRST
         > TERRAIN_VISION_PATH_FAR_ROW_LAST)
        || (TERRAIN_VISION_PATH_NEAR_ROW_FIRST
            > TERRAIN_VISION_PATH_NEAR_ROW_LAST)
        || (TERRAIN_VISION_PATH_FAR_ROW_LAST
            >= TERRAIN_VISION_IMAGE_HEIGHT)
        || (TERRAIN_VISION_PATH_NEAR_ROW_LAST
            >= TERRAIN_VISION_IMAGE_HEIGHT)
        || (0U == TERRAIN_VISION_PATH_MIN_VALID_ROWS)
        || (0U == TERRAIN_VISION_SCORE_MAX)
        || (TERRAIN_VISION_BUMPY_CONFIRM_FRAMES
            > TERRAIN_VISION_SCORE_MAX)
        || (TERRAIN_VISION_STEP_CONFIRM_FRAMES
            > TERRAIN_VISION_SCORE_MAX)
        || (TERRAIN_VISION_BRIDGE_CONFIRM_FRAMES
            > TERRAIN_VISION_SCORE_MAX)
        || (TERRAIN_VISION_OBSTACLE_CONFIRM_FRAMES
            > TERRAIN_VISION_SCORE_MAX)
        || (TERRAIN_VISION_RELEASE_SCORE > TERRAIN_VISION_SCORE_MAX))
    {
        return 0U;
    }

    far_row_count = TERRAIN_VISION_PATH_FAR_ROW_LAST
                    - TERRAIN_VISION_PATH_FAR_ROW_FIRST + 1U;
    near_row_count = TERRAIN_VISION_PATH_NEAR_ROW_LAST
                     - TERRAIN_VISION_PATH_NEAR_ROW_FIRST + 1U;
    return (uint8)((TERRAIN_VISION_PATH_MIN_VALID_ROWS <= far_row_count)
                   && (TERRAIN_VISION_PATH_MIN_VALID_ROWS
                       <= near_row_count));
}

static void terrain_vision_reset_state(void)
{
    memset(&terrain_result, 0, sizeof(terrain_result));
    memset(terrain_gray, 0, sizeof(terrain_gray));
    memset(terrain_binary, 0, sizeof(terrain_binary));
    memset(terrain_left_line, 0, sizeof(terrain_left_line));
    memset(terrain_right_line, 0, sizeof(terrain_right_line));
    memset(terrain_road_width, 0, sizeof(terrain_road_width));
    memset(terrain_left_valid, 0, sizeof(terrain_left_valid));
    memset(terrain_right_valid, 0, sizeof(terrain_right_valid));
    memset(strip_x1, 0, sizeof(strip_x1));
    memset(strip_x2, 0, sizeof(strip_x2));
    memset(strip_y1, 0, sizeof(strip_y1));
    memset(strip_y2, 0, sizeof(strip_y2));
    memset(terrain_state_filter, 0, sizeof(terrain_state_filter));
    memset(terrain_published_result, 0, sizeof(terrain_published_result));
    terrain_published_index = 0U;
    terrain_last_source_frame_count = 0U;

    strip_count = 0U;
    terrain_end_line = TERRAIN_VISION_DEFAULT_END_LINE;
    terrain_lost_line = 0U;
    terrain_left_lost_line = 0U;
    terrain_right_lost_line = 0U;
    terrain_left_lost = 0U;
    terrain_right_lost = 0U;
    terrain_left_straight = 0U;
    terrain_right_straight = 0U;
    terrain_lower_left_corner = 0U;
    terrain_lower_right_corner = 0U;
    terrain_upper_left_corner = 0U;
    terrain_upper_right_corner = 0U;
    terrain_step_band_count = 0U;
    terrain_step_band_span = 0U;
    terrain_transverse_row_count = 0U;

    bridge_score = 0U;
    bumpy_score = 0U;
    obstacle_score = 0U;
    step_score = 0U;
    bridge_stable = 0U;
    bumpy_stable = 0U;
    obstacle_stable = 0U;
    step_stable = 0U;
    last_bridge_side = TERRAIN_BRIDGE_SIDE_UNKNOWN;

    terrain_result.type = TERRAIN_TYPE_NORMAL;
    terrain_result.bridge_side = TERRAIN_BRIDGE_SIDE_UNKNOWN;
    terrain_published_result[0] = terrain_result;
}

static uint8 terrain_otsu_threshold(const uint8 *image,
                                    uint16 width,
                                    uint16 height,
                                    uint8 maximum_threshold)
{
    uint16 histogram[256] = {0};
    uint32 gray_sum = 0U;
    uint32 background_sum = 0U;
    uint32 background_count = 0U;
    uint32 pixel_count = (uint32)width * (uint32)height;
    float maximum_variance = 0.0f;
    uint8 threshold = 0U;
    uint16 x;
    uint16 y;
    uint16 gray;

    for (y = 0U; y < height; y++)
    {
        for (x = 0U; x < width; x++)
        {
            gray = image[(uint32)y * width + x];
            histogram[gray]++;
            gray_sum += gray;
        }
    }

    terrain_result.average_gray = (uint16)(gray_sum / pixel_count);

    for (gray = 0U; gray < maximum_threshold; gray++)
    {
        uint32 foreground_count;
        float background_mean;
        float foreground_mean;
        float difference;
        float variance;

        background_count += histogram[gray];
        background_sum += (uint32)gray * histogram[gray];
        if (0U == background_count)
        {
            continue;
        }

        foreground_count = pixel_count - background_count;
        if (0U == foreground_count)
        {
            break;
        }

        background_mean = (float)background_sum / (float)background_count;
        foreground_mean = (float)(gray_sum - background_sum)
                          / (float)foreground_count;
        difference = background_mean - foreground_mean;
        variance = (float)background_count * (float)foreground_count
                   * difference * difference;
        if (variance > maximum_variance)
        {
            maximum_variance = variance;
            threshold = (uint8)gray;
        }
    }

    return threshold;
}

static void terrain_make_binary(void)
{
    uint8 threshold;
    uint8 threshold_floor;
    uint8 x;
    uint8 y;

    threshold = terrain_otsu_threshold(terrain_gray[0],
                                      TERRAIN_VISION_IMAGE_WIDTH,
                                      TERRAIN_VISION_IMAGE_HEIGHT,
                                      210U);
    threshold_floor = (terrain_result.average_gray
                       < TERRAIN_VISION_DARK_SCENE_AVERAGE)
        ? TERRAIN_VISION_THRESHOLD_FLOOR_DARK
        : TERRAIN_VISION_THRESHOLD_FLOOR_NORMAL;
    if (threshold < threshold_floor)
    {
        threshold = threshold_floor;
    }
    terrain_result.threshold = threshold;

    for (y = 0U; y < TERRAIN_VISION_IMAGE_HEIGHT; y++)
    {
        for (x = 0U; x < TERRAIN_VISION_IMAGE_WIDTH; x++)
        {
            int16 row_threshold;

            if (y < 12U)
            {
                row_threshold = (int16)threshold + 18;
            }
            else if (y < 38U)
            {
                row_threshold = threshold;
            }
            else
            {
                row_threshold = (int16)threshold - 12;
            }

            if ((x < 8U) || (x > TERRAIN_VISION_IMAGE_WIDTH - 9U))
            {
                row_threshold -= 8;
            }
            if (row_threshold < 65)
            {
                row_threshold = 65;
            }
            if (row_threshold > 220)
            {
                row_threshold = 220;
            }

            terrain_binary[y][x] = (terrain_gray[y][x] > row_threshold)
                ? 1U : 0U;
        }
    }
}

static void terrain_filter_binary(void)
{
    uint8 x;
    uint8 y;

    memcpy(terrain_gray, terrain_binary, sizeof(terrain_binary));
    for (y = 2U; y < TERRAIN_VISION_IMAGE_HEIGHT - 2U; y++)
    {
        for (x = 2U; x < TERRAIN_VISION_IMAGE_WIDTH - 2U; x++)
        {
            uint8 near = terrain_binary[y - 1U][x]
                         + terrain_binary[y + 1U][x]
                         + terrain_binary[y][x - 1U]
                         + terrain_binary[y][x + 1U];

            if ((0U == terrain_binary[y][x]) && (near >= 3U))
            {
                terrain_gray[y][x] = 1U;
            }
            else if ((1U == terrain_binary[y][x]) && (near <= 1U))
            {
                terrain_gray[y][x] = 0U;
            }
        }
    }
}

static uint8 terrain_find_start_line(const uint8 *image,
                                     uint8 *start_y,
                                     uint8 *left,
                                     uint8 *right)
{
    int16 y;
    int16 best_score = -32768;
    uint8 best_y = TERRAIN_VISION_IMAGE_HEIGHT - 2U;
    uint8 best_left = 1U;
    uint8 best_right = TERRAIN_VISION_IMAGE_WIDTH - 2U;

    for (y = (int16)TERRAIN_VISION_IMAGE_HEIGHT - 2;
         y > (int16)TERRAIN_VISION_IMAGE_HEIGHT - 14;
         y--)
    {
        uint8 x = 2U;

        while (x < TERRAIN_VISION_IMAGE_WIDTH - 2U)
        {
            uint8 run_start;
            uint8 run_end;
            uint8 length;
            uint8 center;
            int16 distance;
            int16 score;

            while ((x < TERRAIN_VISION_IMAGE_WIDTH - 2U)
                   && (0U == image[(uint16)y
                                   * TERRAIN_VISION_IMAGE_WIDTH + x]))
            {
                x++;
            }
            run_start = x;
            while ((x < TERRAIN_VISION_IMAGE_WIDTH - 2U)
                   && (1U == image[(uint16)y
                                   * TERRAIN_VISION_IMAGE_WIDTH + x]))
            {
                x++;
            }
            if (x <= run_start)
            {
                continue;
            }

            run_end = x - 1U;
            length = run_end - run_start + 1U;
            center = (run_start + run_end) / 2U;
            distance = (center > TERRAIN_VISION_IMAGE_WIDTH / 2U)
                ? (center - TERRAIN_VISION_IMAGE_WIDTH / 2U)
                : (TERRAIN_VISION_IMAGE_WIDTH / 2U - center);
            score = (int16)length * 3 - distance * 2
                    - (TERRAIN_VISION_IMAGE_HEIGHT - 2 - y);

            if ((length >= 14U) && (score > best_score))
            {
                best_score = score;
                best_y = (uint8)y;
                best_left = (run_start > 1U) ? (run_start - 1U) : 1U;
                best_right = (run_end < TERRAIN_VISION_IMAGE_WIDTH - 2U)
                    ? (run_end + 1U) : (TERRAIN_VISION_IMAGE_WIDTH - 2U);
            }
        }
    }

    if (-32768 == best_score)
    {
        return 0U;
    }

    *start_y = best_y;
    *left = best_left;
    *right = best_right;
    return 1U;
}

static void terrain_find_road_lines(const uint8 *image)
{
    uint8 x;
    uint8 start_y;
    uint8 start_left;
    uint8 start_right;
    uint8 bottom_left;
    uint8 bottom_right;
    int16 y;

    terrain_end_line = TERRAIN_VISION_DEFAULT_END_LINE;
    terrain_lost_line = 0U;
    terrain_left_lost_line = 0U;
    terrain_right_lost_line = 0U;

    for (y = 0; y < (int16)TERRAIN_VISION_IMAGE_HEIGHT; y++)
    {
        terrain_left_line[y] = 0U;
        terrain_right_line[y] = TERRAIN_VISION_IMAGE_WIDTH - 1U;
        terrain_road_width[y] = TERRAIN_VISION_IMAGE_WIDTH - 1U;
        terrain_left_valid[y] = 0U;
        terrain_right_valid[y] = 0U;
    }

    if (terrain_find_start_line(image, &start_y, &start_left, &start_right))
    {
        bottom_left = start_left;
        bottom_right = start_right;
    }
    else
    {
        const uint8 *row = image
            + (TERRAIN_VISION_IMAGE_HEIGHT - 2U)
              * TERRAIN_VISION_IMAGE_WIDTH;

        start_y = TERRAIN_VISION_IMAGE_HEIGHT - 2U;
        bottom_left = 0U;
        bottom_right = TERRAIN_VISION_IMAGE_WIDTH - 1U;
        for (x = TERRAIN_VISION_IMAGE_WIDTH / 2U; x > 1U; x--)
        {
            if ((0U == row[x - 1U]) && (0U == row[x]))
            {
                bottom_left = x;
                break;
            }
        }
        for (x = TERRAIN_VISION_IMAGE_WIDTH / 2U;
             x < TERRAIN_VISION_IMAGE_WIDTH - 2U;
             x++)
        {
            if ((0U == row[x + 1U]) && (0U == row[x]))
            {
                bottom_right = x;
                break;
            }
        }
    }

    for (y = start_y; y < (int16)TERRAIN_VISION_IMAGE_HEIGHT; y++)
    {
        terrain_left_line[y] = bottom_left;
        terrain_right_line[y] = bottom_right;
        terrain_road_width[y] = bottom_right - bottom_left;
        terrain_left_valid[y] = 1U;
        terrain_right_valid[y] = 1U;
    }

    for (y = (int16)start_y - 1; y > terrain_end_line; y--)
    {
        const uint8 *row = image + (uint16)y * TERRAIN_VISION_IMAGE_WIDTH;
        uint8 left_border;
        uint8 right_border;

        left_border = (terrain_left_line[y + 1] > TERRAIN_VISION_BORDER_INTERVAL)
            ? (terrain_left_line[y + 1] - TERRAIN_VISION_BORDER_INTERVAL)
            : 1U;
        right_border = terrain_left_line[y + 1]
                       + TERRAIN_VISION_BORDER_INTERVAL;
        if (right_border > TERRAIN_VISION_IMAGE_WIDTH - 2U)
        {
            right_border = TERRAIN_VISION_IMAGE_WIDTH - 2U;
        }
        for (x = left_border; x <= right_border; x++)
        {
            if ((0U == row[x]) && (1U == row[x + 1U]))
            {
                terrain_left_line[y] = x;
                terrain_left_valid[y] = 1U;
                break;
            }
        }
        if (!terrain_left_valid[y])
        {
            terrain_left_line[y] = terrain_left_line[y + 1];
            terrain_left_lost_line++;
        }

        left_border = (terrain_right_line[y + 1]
                       > TERRAIN_VISION_BORDER_INTERVAL)
            ? (terrain_right_line[y + 1] - TERRAIN_VISION_BORDER_INTERVAL)
            : 1U;
        right_border = terrain_right_line[y + 1]
                       + TERRAIN_VISION_BORDER_INTERVAL;
        if (right_border > TERRAIN_VISION_IMAGE_WIDTH - 2U)
        {
            right_border = TERRAIN_VISION_IMAGE_WIDTH - 2U;
        }
        for (x = right_border; x > left_border; x--)
        {
            if ((0U == row[x]) && (1U == row[x - 1U]))
            {
                terrain_right_line[y] = x;
                terrain_right_valid[y] = 1U;
                break;
            }
        }
        if (!terrain_right_valid[y])
        {
            terrain_right_line[y] = terrain_right_line[y + 1];
            terrain_right_lost_line++;
        }

        if (!terrain_left_valid[y] && !terrain_right_valid[y])
        {
            terrain_lost_line++;
        }
        if (terrain_right_line[y] <= terrain_left_line[y] + 6U)
        {
            terrain_end_line = (uint8)y + 1U;
            break;
        }
        terrain_road_width[y] = terrain_right_line[y] - terrain_left_line[y];
    }
}

static void terrain_update_road_features(void)
{
    uint16 near_sum = 0U;
    uint16 far_sum = 0U;
    uint8 near_count = 0U;
    uint8 far_count = 0U;
    uint8 valid_count = 0U;
    uint8 maximum_width = 0U;
    uint8 y;

    terrain_left_lost = (terrain_left_lost_line >= 10U) ? 1U : 0U;
    terrain_right_lost = (terrain_right_lost_line >= 10U) ? 1U : 0U;

    for (y = terrain_end_line + 1U;
         y < TERRAIN_VISION_IMAGE_HEIGHT - 2U;
         y++)
    {
        uint8 width;

        if (!terrain_left_valid[y] || !terrain_right_valid[y])
        {
            continue;
        }
        if (terrain_right_line[y] <= terrain_left_line[y])
        {
            continue;
        }
        width = terrain_road_width[y];
        if ((width < 10U) || (width > TERRAIN_VISION_IMAGE_WIDTH - 4U))
        {
            continue;
        }

        valid_count++;
        if (width > maximum_width)
        {
            maximum_width = width;
        }
        if (y >= TERRAIN_VISION_IMAGE_HEIGHT - 18U)
        {
            near_sum += width;
            near_count++;
        }
        else if ((y >= 22U) && (y <= 34U))
        {
            far_sum += width;
            far_count++;
        }
    }

    terrain_result.road_valid_count = valid_count;
    terrain_result.road_width_max = maximum_width;
    terrain_result.road_width_near = (near_count > 0U)
        ? (uint8)(near_sum / near_count) : 0U;
    terrain_result.road_width_far = (far_count > 0U)
        ? (uint8)(far_sum / far_count) : 0U;
}

static void terrain_average_road_center(uint8 first_row,
                                        uint8 last_row,
                                        uint16 *center_sum,
                                        uint8 *valid_count)
{
    uint8 y;

    *center_sum = 0U;
    *valid_count = 0U;
    for (y = first_row; y <= last_row; y++)
    {
        if (!terrain_left_valid[y] || !terrain_right_valid[y]
            || (terrain_right_line[y] <= terrain_left_line[y] + 9U))
        {
            continue;
        }
        *center_sum += (uint16)(terrain_left_line[y]
                                + terrain_right_line[y]) / 2U;
        (*valid_count)++;
    }
}

static float terrain_clamp_normalized(float value)
{
    if (value < -1.0f)
    {
        return -1.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}

static void terrain_update_path_observation(void)
{
    uint16 near_center_sum;
    uint16 far_center_sum;
    uint8 near_valid_count;
    uint8 far_valid_count;
    uint8 minimum_valid_count;
    uint8 expected_count;
    float image_center = ((float)TERRAIN_VISION_IMAGE_WIDTH - 1.0f) * 0.5f;

    terrain_result.path_center_error_norm = 0.0f;
    terrain_result.path_heading_error_norm = 0.0f;
    terrain_result.road_center_near_px = 0U;
    terrain_result.road_center_far_px = 0U;
    terrain_result.path_quality = 0U;
    terrain_result.path_valid = 0U;

    terrain_average_road_center(TERRAIN_VISION_PATH_FAR_ROW_FIRST,
                                TERRAIN_VISION_PATH_FAR_ROW_LAST,
                                &far_center_sum,
                                &far_valid_count);
    terrain_average_road_center(TERRAIN_VISION_PATH_NEAR_ROW_FIRST,
                                TERRAIN_VISION_PATH_NEAR_ROW_LAST,
                                &near_center_sum,
                                &near_valid_count);
    if ((near_valid_count < TERRAIN_VISION_PATH_MIN_VALID_ROWS)
        || (far_valid_count < TERRAIN_VISION_PATH_MIN_VALID_ROWS))
    {
        return;
    }

    terrain_result.road_center_near_px
        = (uint8)(near_center_sum / near_valid_count);
    terrain_result.road_center_far_px
        = (uint8)(far_center_sum / far_valid_count);
    minimum_valid_count = (near_valid_count < far_valid_count)
        ? near_valid_count : far_valid_count;
    expected_count = TERRAIN_VISION_PATH_NEAR_ROW_LAST
                     - TERRAIN_VISION_PATH_NEAR_ROW_FIRST + 1U;
    if ((TERRAIN_VISION_PATH_FAR_ROW_LAST
         - TERRAIN_VISION_PATH_FAR_ROW_FIRST + 1U) < expected_count)
    {
        expected_count = TERRAIN_VISION_PATH_FAR_ROW_LAST
                         - TERRAIN_VISION_PATH_FAR_ROW_FIRST + 1U;
    }
    terrain_result.path_quality = (uint8)((uint16)minimum_valid_count
                                          * 100U / expected_count);
    terrain_result.path_center_error_norm = terrain_clamp_normalized(
        ((float)terrain_result.road_center_near_px - image_center)
        / image_center);
    terrain_result.path_heading_error_norm = terrain_clamp_normalized(
        ((float)terrain_result.road_center_far_px
         - (float)terrain_result.road_center_near_px)
        / image_center);
    terrain_result.path_valid = 1U;
}

static uint8 terrain_get_inner_road_roi(uint8 y,
                                        uint8 inset,
                                        uint8 *x1,
                                        uint8 *x2)
{
    uint8 left;
    uint8 right;
    uint8 width;

    if (y >= TERRAIN_VISION_IMAGE_HEIGHT)
    {
        return 0U;
    }
    left = terrain_left_line[y];
    right = terrain_right_line[y];
    if (right <= left)
    {
        return 0U;
    }
    width = right - left;
    if ((width < 10U) || (width > TERRAIN_VISION_IMAGE_WIDTH - 4U))
    {
        return 0U;
    }
    if (right <= left + 2U * inset + 2U)
    {
        return 0U;
    }

    *x1 = left + inset;
    *x2 = right - inset;
    return 1U;
}

static uint8 terrain_get_detect_roi(uint8 y,
                                    uint8 inset,
                                    uint8 *x1,
                                    uint8 *x2)
{
    uint8 half_width;

    if (terrain_get_inner_road_roi(y, inset, x1, x2))
    {
        return 1U;
    }
    if (y >= TERRAIN_VISION_IMAGE_HEIGHT)
    {
        return 0U;
    }

    half_width = 8U + y / 2U;
    if (half_width > 36U)
    {
        half_width = 36U;
    }
    if (half_width <= inset + 2U)
    {
        return 0U;
    }

    *x1 = TERRAIN_VISION_IMAGE_WIDTH / 2U - half_width + inset;
    *x2 = TERRAIN_VISION_IMAGE_WIDTH / 2U + half_width - inset;
    if (*x1 < 1U)
    {
        *x1 = 1U;
    }
    if (*x2 > TERRAIN_VISION_IMAGE_WIDTH - 2U)
    {
        *x2 = TERRAIN_VISION_IMAGE_WIDTH - 2U;
    }
    return (*x1 < *x2) ? 1U : 0U;
}

static uint8 terrain_absolute_difference(uint8 a, uint8 b)
{
    return (a > b) ? (a - b) : (b - a);
}

static uint8 terrain_line_is_straight(const uint8 *line,
                                      const uint8 *valid_flag)
{
    int16 start = (terrain_end_line > 8U) ? terrain_end_line + 4 : 12;
    int16 y;
    uint8 valid = 0U;
    uint8 smooth = 0U;
    uint8 large_jump = 0U;
    uint8 last = 0U;
    uint8 have_last = 0U;

    for (y = TERRAIN_VISION_IMAGE_HEIGHT - 5; y > start; y--)
    {
        if (!valid_flag[y])
        {
            have_last = 0U;
            continue;
        }

        valid++;
        if (have_last)
        {
            uint8 difference = terrain_absolute_difference(line[y], last);

            if (difference <= 2U)
            {
                smooth++;
            }
            else if (difference >= 6U)
            {
                large_jump++;
            }
        }
        last = line[y];
        have_last = 1U;
    }

    if ((valid < 16U) || (large_jump > 3U))
    {
        return 0U;
    }
    return (smooth >= valid / 2U) ? 1U : 0U;
}

static void terrain_find_line_corners(void)
{
    int16 start = (terrain_end_line > 6U) ? terrain_end_line + 4 : 10;
    int16 y;

    terrain_lower_left_corner = 0U;
    terrain_lower_right_corner = 0U;
    terrain_upper_left_corner = 0U;
    terrain_upper_right_corner = 0U;

    for (y = TERRAIN_VISION_IMAGE_HEIGHT - 6; y > start + 3; y--)
    {
        if (!terrain_lower_left_corner
            && terrain_left_valid[y]
            && !terrain_left_valid[y - 3]
            && (terrain_left_line[y] > 4U))
        {
            terrain_lower_left_corner = 1U;
        }
        if (!terrain_lower_right_corner
            && terrain_right_valid[y]
            && !terrain_right_valid[y - 3]
            && (terrain_right_line[y] < TERRAIN_VISION_IMAGE_WIDTH - 5U))
        {
            terrain_lower_right_corner = 1U;
        }
        if (!terrain_upper_left_corner
            && !terrain_left_valid[y]
            && terrain_left_valid[y - 3])
        {
            terrain_upper_left_corner = 1U;
        }
        if (!terrain_upper_right_corner
            && !terrain_right_valid[y]
            && terrain_right_valid[y - 3])
        {
            terrain_upper_right_corner = 1U;
        }
    }
}

static void terrain_update_line_shape(void)
{
    terrain_left_straight = terrain_line_is_straight(terrain_left_line,
                                                     terrain_left_valid);
    terrain_right_straight = terrain_line_is_straight(terrain_right_line,
                                                      terrain_right_valid);
    terrain_find_line_corners();
}

static void terrain_detect_transverse_bands(const uint8 *image)
{
    uint8 in_band = 0U;
    uint8 band_end = 0U;
    uint8 first_band_start = 0U;
    uint8 last_band_end = 0U;
    uint8 y;

    terrain_step_band_count = 0U;
    terrain_step_band_span = 0U;
    terrain_transverse_row_count = 0U;

    for (y = 8U; y < TERRAIN_VISION_IMAGE_HEIGHT - 5U; y++)
    {
        uint8 x;
        uint8 x1;
        uint8 x2;
        uint8 width;
        uint8 dark_count = 0U;
        uint8 row_hit;

        if (!terrain_get_detect_roi(y, 2U, &x1, &x2))
        {
            if (in_band)
            {
                terrain_step_band_count++;
                last_band_end = y - 1U;
                in_band = 0U;
            }
            continue;
        }
        width = x2 - x1 + 1U;
        if (width < 18U)
        {
            continue;
        }
        for (x = x1; x <= x2; x++)
        {
            if (0U == image[(uint16)y * TERRAIN_VISION_IMAGE_WIDTH + x])
            {
                dark_count++;
            }
        }

        row_hit = ((uint16)dark_count * 100U
                   >= (uint16)width * TERRAIN_VISION_BAND_DARK_PERCENT)
            ? 1U : 0U;
        if (row_hit)
        {
            terrain_transverse_row_count++;
            if (!in_band)
            {
                in_band = 1U;
                if (0U == terrain_step_band_count)
                {
                    first_band_start = y;
                }
            }
            band_end = y;
        }
        else if (in_band)
        {
            terrain_step_band_count++;
            last_band_end = band_end;
            in_band = 0U;
        }
    }

    if (in_band)
    {
        terrain_step_band_count++;
        last_band_end = band_end;
    }
    if ((terrain_step_band_count > 0U)
        && (last_band_end >= first_band_start))
    {
        terrain_step_band_span = last_band_end - first_band_start + 1U;
    }

    terrain_result.step_band_count = terrain_step_band_count;
    terrain_result.transverse_row_count = terrain_transverse_row_count;
}

static void terrain_save_bumpy_strip(uint8 x1,
                                     uint8 x2,
                                     uint8 y1,
                                     uint8 y2)
{
    uint8 width = x2 - x1 + 1U;
    uint8 height = y2 - y1 + 1U;

    if ((width < 6U) || (width > 62U))
    {
        return;
    }
    if ((height < 1U) || (height > 10U))
    {
        return;
    }
    if (width < height * 3U)
    {
        return;
    }
    if (strip_count >= TERRAIN_VISION_STRIP_MAX)
    {
        return;
    }

    strip_x1[strip_count] = x1;
    strip_x2[strip_count] = x2;
    strip_y1[strip_count] = y1;
    strip_y2[strip_count] = y2;
    strip_count++;
}

static void terrain_detect_bumpy_strips(const uint8 *image)
{
    uint8 in_band = 0U;
    uint8 band_x1 = TERRAIN_VISION_IMAGE_WIDTH - 1U;
    uint8 band_x2 = 0U;
    uint8 band_y1 = 0U;
    uint8 band_y2 = 0U;
    uint8 regular_count = 1U;
    uint8 best_regular = 1U;
    uint8 span = 0U;
    uint8 x;
    uint8 y;
    uint8 i;

    strip_count = 0U;
    terrain_result.raw_bumpy = 0U;
    terrain_result.bumpy_strip_count = 0U;
    terrain_result.bumpy_span = 0U;

    for (y = 8U; y < TERRAIN_VISION_IMAGE_HEIGHT - 4U; y++)
    {
        uint8 roi_x1;
        uint8 roi_x2;
        uint8 roi_width;
        uint8 best_x1 = 0U;
        uint8 best_x2 = 0U;
        uint8 best_length = 0U;
        uint8 row_hit;

        if (!terrain_get_detect_roi(y, 2U, &roi_x1, &roi_x2))
        {
            if (in_band)
            {
                terrain_save_bumpy_strip(band_x1, band_x2,
                                          band_y1, band_y2);
                in_band = 0U;
            }
            continue;
        }
        roi_width = roi_x2 - roi_x1 + 1U;
        x = roi_x1;
        while (x <= roi_x2)
        {
            uint8 start;
            uint8 end;
            uint8 length;

            while ((x <= roi_x2)
                   && (1U == image[(uint16)y
                                   * TERRAIN_VISION_IMAGE_WIDTH + x]))
            {
                x++;
            }
            start = x;
            while ((x <= roi_x2)
                   && (0U == image[(uint16)y
                                   * TERRAIN_VISION_IMAGE_WIDTH + x]))
            {
                x++;
            }
            if (x <= start)
            {
                continue;
            }
            end = x - 1U;
            length = end - start + 1U;
            if (length > best_length)
            {
                best_length = length;
                best_x1 = start;
                best_x2 = end;
            }
        }

        row_hit = ((best_length >= 6U)
                   && ((uint16)best_length * 100U
                       >= (uint16)roi_width
                          * TERRAIN_VISION_BUMPY_MIN_WIDTH_PERCENT))
            ? 1U : 0U;
        if (row_hit)
        {
            if (!in_band)
            {
                in_band = 1U;
                band_x1 = best_x1;
                band_x2 = best_x2;
                band_y1 = y;
                band_y2 = y;
            }
            else
            {
                if (best_x1 < band_x1)
                {
                    band_x1 = best_x1;
                }
                if (best_x2 > band_x2)
                {
                    band_x2 = best_x2;
                }
                band_y2 = y;
            }
        }
        else if (in_band)
        {
            terrain_save_bumpy_strip(band_x1, band_x2,
                                      band_y1, band_y2);
            in_band = 0U;
        }
    }

    if (in_band)
    {
        terrain_save_bumpy_strip(band_x1, band_x2, band_y1, band_y2);
    }

    if (strip_count >= 2U)
    {
        span = strip_y2[strip_count - 1U] - strip_y1[0] + 1U;
    }
    for (i = 1U; i < strip_count; i++)
    {
        uint8 previous_middle = (strip_y1[i - 1U] + strip_y2[i - 1U]) / 2U;
        uint8 current_middle = (strip_y1[i] + strip_y2[i]) / 2U;
        uint8 gap = current_middle - previous_middle;

        if ((gap >= 2U) && (gap <= 18U))
        {
            regular_count++;
            if (regular_count > best_regular)
            {
                best_regular = regular_count;
            }
        }
        else
        {
            regular_count = 1U;
        }
    }

    terrain_result.bumpy_strip_count = strip_count;
    terrain_result.bumpy_span = span;
    if ((terrain_result.road_valid_count >= 10U)
        && (strip_count >= TERRAIN_VISION_BUMPY_MIN_STRIPS)
        && ((best_regular >= TERRAIN_VISION_BUMPY_MIN_STRIPS)
            || (span >= 10U)))
    {
        terrain_result.raw_bumpy = 1U;
    }
}

static uint8 terrain_detect_bridge(void)
{
    uint8 left_score = 0U;
    uint8 right_score = 0U;
    uint8 narrow_far = ((terrain_result.road_width_far > 0U)
                        && (terrain_result.road_width_near > 0U)
                        && (terrain_result.road_width_far + 7U
                            < terrain_result.road_width_near))
        ? 1U : 0U;
    uint8 narrow_road = ((terrain_result.road_width_max > 0U)
                         && (terrain_result.road_width_max
                             < TERRAIN_VISION_IMAGE_WIDTH - 12U))
        ? 1U : 0U;
    uint8 enough_line = (terrain_result.road_valid_count >= 10U) ? 1U : 0U;

    if (terrain_left_lost && !terrain_right_lost)
    {
        if (terrain_right_straight)
        {
            left_score += 2U;
        }
        if (terrain_lower_left_corner || terrain_upper_left_corner)
        {
            left_score++;
        }
        if (narrow_far || narrow_road)
        {
            left_score++;
        }
        if (enough_line)
        {
            left_score++;
        }
    }
    if (terrain_right_lost && !terrain_left_lost)
    {
        if (terrain_left_straight)
        {
            right_score += 2U;
        }
        if (terrain_lower_right_corner || terrain_upper_right_corner)
        {
            right_score++;
        }
        if (narrow_far || narrow_road)
        {
            right_score++;
        }
        if (enough_line)
        {
            right_score++;
        }
    }

    if ((left_score >= 4U) && (terrain_transverse_row_count < 5U))
    {
        last_bridge_side = TERRAIN_BRIDGE_SIDE_LEFT;
        return 1U;
    }
    if ((right_score >= 4U) && (terrain_transverse_row_count < 5U))
    {
        last_bridge_side = TERRAIN_BRIDGE_SIDE_RIGHT;
        return 1U;
    }
    return 0U;
}

static uint8 terrain_detect_step(void)
{
    uint8 score = 0U;

    if (terrain_result.raw_bumpy
        || (terrain_result.road_valid_count < 8U)
        || (0U == terrain_step_band_count)
        || (terrain_transverse_row_count < 5U))
    {
        return 0U;
    }
    if ((terrain_end_line >= 28U) || (terrain_lost_line >= 8U))
    {
        score++;
    }
    if ((terrain_result.road_width_far > 0U)
        && (terrain_result.road_width_near > 0U)
        && (terrain_result.road_width_far + 10U
            < terrain_result.road_width_near))
    {
        score++;
    }
    if ((terrain_result.road_width_far > 0U)
        && (terrain_result.road_width_far <= 28U))
    {
        score++;
    }
    score += 2U;
    if (terrain_left_lost && terrain_right_lost)
    {
        score++;
    }
    return (score >= 3U) ? 1U : 0U;
}

static void terrain_detect_shape(void)
{
    terrain_update_line_shape();

    terrain_result.raw_bridge = terrain_detect_bridge();
    terrain_result.raw_step = terrain_detect_step();
    terrain_result.raw_obstacle = 0U;

    if ((terrain_result.road_width_far > 0U)
        && (terrain_result.road_width_far <= 22U)
        && (terrain_result.road_width_near >= 36U)
        && (terrain_transverse_row_count <= 4U)
        && !terrain_result.raw_bridge
        && !terrain_result.raw_step
        && !terrain_result.raw_bumpy)
    {
        terrain_result.raw_obstacle = 1U;
    }
}

static uint8 terrain_update_detection_score(uint8 raw,
                                              uint8 *score,
                                              uint8 *stable,
                                              uint8 confirm,
                                              uint8 release)
{
    if (raw)
    {
        if (*score < TERRAIN_VISION_SCORE_MAX)
        {
            (*score)++;
        }
    }
    else if (*score > 0U)
    {
        (*score)--;
    }

    if (*score >= confirm)
    {
        *stable = 1U;
    }
    else if (*score <= release)
    {
        *stable = 0U;
    }
    return *stable;
}

static void terrain_stabilize_detections(void)
{
    uint8 raw_bridge = terrain_result.raw_bridge;
    uint8 raw_bumpy = terrain_result.raw_bumpy;
    uint8 raw_obstacle = terrain_result.raw_obstacle;
    uint8 raw_step = terrain_result.raw_step;

    if (raw_bumpy)
    {
        raw_bridge = 0U;
        raw_step = 0U;
    }
    if (raw_bridge || raw_bumpy || raw_step)
    {
        raw_obstacle = 0U;
    }

    (void)terrain_update_detection_score(
        raw_bumpy,
        &bumpy_score,
        &bumpy_stable,
        TERRAIN_VISION_BUMPY_CONFIRM_FRAMES,
        0U);
    (void)terrain_update_detection_score(
        raw_step,
        &step_score,
        &step_stable,
        TERRAIN_VISION_STEP_CONFIRM_FRAMES,
        TERRAIN_VISION_RELEASE_SCORE);
    (void)terrain_update_detection_score(
        raw_bridge,
        &bridge_score,
        &bridge_stable,
        TERRAIN_VISION_BRIDGE_CONFIRM_FRAMES,
        TERRAIN_VISION_RELEASE_SCORE);
    (void)terrain_update_detection_score(
        raw_obstacle,
        &obstacle_score,
        &obstacle_stable,
        TERRAIN_VISION_OBSTACLE_CONFIRM_FRAMES,
        TERRAIN_VISION_RELEASE_SCORE);

    terrain_result.bumpy_score = bumpy_score;
    terrain_result.step_score = step_score;
    terrain_result.bridge_score = bridge_score;
    terrain_result.obstacle_score = obstacle_score;
}

static terrain_type_t terrain_raw_state(void)
{
    if (bumpy_stable)
    {
        return TERRAIN_TYPE_BUMPY;
    }
    if (step_stable)
    {
        return TERRAIN_TYPE_STEP;
    }
    if (obstacle_stable)
    {
        return TERRAIN_TYPE_OBSTACLE;
    }
    if (bridge_stable)
    {
        return TERRAIN_TYPE_BRIDGE;
    }
    if ((terrain_lost_line > 22U) || (terrain_end_line > 48U))
    {
        return TERRAIN_TYPE_LOST;
    }
    return TERRAIN_TYPE_NORMAL;
}

static terrain_type_t terrain_filter_state(void)
{
    terrain_type_t state = terrain_raw_state();
    uint8 i;

    for (i = 0U; i < TERRAIN_VISION_STATE_FILTER_LENGTH - 1U; i++)
    {
        terrain_state_filter[i] = terrain_state_filter[i + 1U];
    }
    terrain_state_filter[TERRAIN_VISION_STATE_FILTER_LENGTH - 1U]
        = (uint8)state;

    terrain_result.stable_count = 0U;
    for (i = 0U; i < TERRAIN_VISION_STATE_FILTER_LENGTH; i++)
    {
        if (terrain_state_filter[i] == (uint8)state)
        {
            terrain_result.stable_count++;
        }
    }
    if (terrain_result.stable_count >= 3U)
    {
        return state;
    }
    return terrain_result.type;
}

static uint8 terrain_score_to_confidence(uint8 score)
{
    uint16 confidence = (uint16)score * 100U / TERRAIN_VISION_SCORE_MAX;

    return (confidence > 100U) ? 100U : (uint8)confidence;
}

static void terrain_update_confidence(void)
{
    uint16 confidence;

    switch (terrain_result.type)
    {
        case TERRAIN_TYPE_BUMPY:
            terrain_result.confidence = terrain_score_to_confidence(bumpy_score);
            break;

        case TERRAIN_TYPE_STEP:
            terrain_result.confidence = terrain_score_to_confidence(step_score);
            break;

        case TERRAIN_TYPE_BRIDGE:
            terrain_result.confidence = terrain_score_to_confidence(bridge_score);
            break;

        case TERRAIN_TYPE_OBSTACLE:
            terrain_result.confidence = terrain_score_to_confidence(obstacle_score);
            break;

        case TERRAIN_TYPE_LOST:
            confidence = (uint16)terrain_lost_line * 4U;
            terrain_result.confidence = (confidence > 100U)
                ? 100U : (uint8)confidence;
            break;

        case TERRAIN_TYPE_NORMAL:
        default:
            confidence = (uint16)terrain_result.road_valid_count * 5U;
            terrain_result.confidence = (confidence > 100U)
                ? 100U : (uint8)confidence;
            break;
    }
}

static terrain_vision_status_t terrain_process_compressed(uint32 frame_count)
{
    terrain_make_binary();
    terrain_filter_binary();
    terrain_find_road_lines(terrain_gray[0]);
    terrain_update_road_features();
    terrain_update_path_observation();
    terrain_detect_bumpy_strips(terrain_gray[0]);
    terrain_detect_transverse_bands(terrain_gray[0]);
    terrain_detect_shape();
    terrain_stabilize_detections();
    terrain_result.type = terrain_filter_state();
    terrain_result.bridge_side = (TERRAIN_TYPE_BRIDGE == terrain_result.type)
        ? last_bridge_side : TERRAIN_BRIDGE_SIDE_UNKNOWN;
    terrain_update_confidence();
    terrain_result.frame_count = frame_count;
    terrain_result.status = TERRAIN_VISION_STATUS_OK;
    terrain_publish_result();
    return TERRAIN_VISION_STATUS_OK;
}

terrain_vision_status_t terrain_vision_detector_init(void)
{
    const vision_frame_t *frame;
    vision_frame_status_t frame_status;

    terrain_vision_reset_state();

    if (!TERRAIN_VISION_ENABLE)
    {
        terrain_result.status = TERRAIN_VISION_STATUS_DISABLED;
        terrain_publish_result();
        return terrain_result.status;
    }
    if (!terrain_config_is_valid())
    {
        terrain_result.status = TERRAIN_VISION_STATUS_INVALID_CONFIG;
        terrain_publish_result();
        return terrain_result.status;
    }
    frame_status = vision_frame_init();
    if (VISION_FRAME_STATUS_DISABLED == frame_status)
    {
        terrain_result.status = TERRAIN_VISION_STATUS_DISABLED;
        terrain_publish_result();
        return terrain_result.status;
    }
    if (VISION_FRAME_STATUS_INVALID_CONFIG == frame_status)
    {
        terrain_result.status = TERRAIN_VISION_STATUS_INVALID_CONFIG;
        terrain_publish_result();
        return terrain_result.status;
    }
    if (VISION_FRAME_STATUS_OK != frame_status)
    {
        terrain_result.status = TERRAIN_VISION_STATUS_CAMERA_ERROR;
        terrain_publish_result();
        return terrain_result.status;
    }

    frame = vision_frame_get_latest();
    terrain_result.enabled = 1U;
    terrain_result.status = TERRAIN_VISION_STATUS_OK;
    terrain_result.exposure = frame->exposure;
    terrain_result.dropped_frame_count = frame->dropped_frame_count;
    terrain_publish_result();
    return terrain_result.status;
}

terrain_vision_status_t terrain_vision_init(void)
{
    vision_pipeline_status_t status = vision_pipeline_init();

    switch (status)
    {
        case VISION_PIPELINE_STATUS_DISABLED:
            return TERRAIN_VISION_STATUS_DISABLED;

        case VISION_PIPELINE_STATUS_INVALID_CONFIG:
            return TERRAIN_VISION_STATUS_INVALID_CONFIG;

        case VISION_PIPELINE_STATUS_CAMERA_ERROR:
            return TERRAIN_VISION_STATUS_CAMERA_ERROR;

        case VISION_PIPELINE_STATUS_OBSERVER_ERROR:
        case VISION_PIPELINE_STATUS_NOT_INITIALIZED:
            return TERRAIN_VISION_STATUS_CAMERA_ERROR;

        case VISION_PIPELINE_STATUS_OK:
        case VISION_PIPELINE_STATUS_WAITING_FOR_FRAME:
        default:
            return terrain_result.status;
    }
}

terrain_vision_status_t terrain_vision_process_shared_frame(
    const vision_frame_t *frame)
{
    if ((0 == frame) || !frame->enabled
        || (VISION_FRAME_STATUS_OK != frame->status)
        || (0U == frame->frame_count))
    {
        terrain_result.status = TERRAIN_VISION_STATUS_INVALID_ARGUMENT;
        terrain_publish_result();
        return terrain_result.status;
    }
    if (frame->frame_count == terrain_last_source_frame_count)
    {
        return terrain_result.status;
    }

    memcpy(terrain_gray, frame->gray, sizeof(terrain_gray));
    terrain_last_source_frame_count = frame->frame_count;
    terrain_result.dropped_frame_count = frame->dropped_frame_count;
    terrain_result.exposure = frame->exposure;
    return terrain_process_compressed(frame->frame_count);
}

void terrain_vision_task(void)
{
    (void)vision_pipeline_task();
}

terrain_vision_status_t terrain_vision_process_frame(const uint8 *image,
                                                     uint16 width,
                                                     uint16 height)
{
    vision_frame_status_t frame_status;

    if ((0 == image) || (0U == width) || (0U == height))
    {
        terrain_result.status = TERRAIN_VISION_STATUS_INVALID_ARGUMENT;
        terrain_publish_result();
        return terrain_result.status;
    }

    frame_status = vision_frame_resample(image,
                                         width,
                                         height,
                                         terrain_gray[0],
                                         0);
    if (VISION_FRAME_STATUS_OK != frame_status)
    {
        terrain_result.status = TERRAIN_VISION_STATUS_INVALID_ARGUMENT;
        terrain_publish_result();
        return terrain_result.status;
    }
    return terrain_process_compressed(terrain_result.frame_count + 1U);
}

const terrain_vision_result_t *terrain_vision_get_result(void)
{
    return &terrain_published_result[terrain_published_index];
}

uint8 terrain_vision_get_snapshot(terrain_vision_result_t *result)
{
    uint8 published_index;

    if (0 == result)
    {
        return 0U;
    }
    published_index = terrain_published_index;
    *result = terrain_published_result[published_index];
    return 1U;
}
