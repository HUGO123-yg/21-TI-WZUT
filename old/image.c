#include "zf_common_headfile.h"

extern vuint8 mt9v03x_finish_flag;

float final_error = 0.0f;
Road_State_e Road_State = ROAD_NORMAL;
image_runtime_struct Image_Runtime = {0};

static uint8 Deal_Image_Gray[DEAL_IMAGE_H][DEAL_IMAGE_W];
static uint8 Deal_Image_Binary[DEAL_IMAGE_H][DEAL_IMAGE_W];
static uint8 Show_Image_Gray[MT9V03X_H][MT9V03X_W];
static uint8 Deal_Image_Left_Line[DEAL_IMAGE_H];
static uint8 Deal_Image_Right_Line[DEAL_IMAGE_H];
static uint8 Deal_Image_Mid_Line[DEAL_IMAGE_H];
static uint8 Deal_Image_Wide[DEAL_IMAGE_H];
static uint8 Deal_Image_Left_Flag[DEAL_IMAGE_H];
static uint8 Deal_Image_Right_Flag[DEAL_IMAGE_H];
static uint8 Detect_Strip_X1[8];
static uint8 Detect_Strip_X2[8];
static uint8 Detect_Strip_Y1[8];
static uint8 Detect_Strip_Y2[8];
static uint8 Detect_Strip_Count;

static uint8 image_row_mid = DEAL_IMAGE_W / 2 - 1;
static uint8 bottom_left_line;
static uint8 bottom_right_line;
static uint8 End_Line = 15;
static uint8 Lost_Line;
static uint8 Left_Lost_Line;
static uint8 Right_Lost_Line;
static uint8 Border_Interval = 5;
static uint8 road_state_filter[5] = {0};
static uint8 Road_Valid_Count;
static uint8 Road_Left_Lost_Flag;
static uint8 Road_Right_Lost_Flag;
static uint8 Bridge_Detect_Score;
static uint8 Bumpy_Detect_Score;
static uint8 Obstacle_Detect_Score;
static uint8 Step_Detect_Score;
static uint8 Bridge_Stable_Flag;
static uint8 Bumpy_Stable_Flag;
static uint8 Obstacle_Stable_Flag;
static uint8 Step_Stable_Flag;
static uint8 Left_Straight_Flag;
static uint8 Right_Straight_Flag;
static uint8 Lower_Left_Corner_Flag;
static uint8 Lower_Right_Corner_Flag;
static uint8 Upper_Left_Corner_Flag;
static uint8 Upper_Right_Corner_Flag;
static uint8 Step_Band_Count;
static uint8 Step_Band_Span;
static uint8 Transverse_Row_Count;

typedef struct
{
    uint8 used;
    uint8 x1;
    uint8 x2;
    uint8 y1;
    uint8 y2;
    uint8 last_y;
    uint16 area;
} image_block_struct;

static uint8 Image_Threshold(uint8 *image, uint16 width, uint16 height, uint8 max_threshold)
{
    uint16 pixel_count[256] = {0};
    float pixel_pro[256] = {0};
    uint32 gray_sum = 0;
    uint32 pixel_sum = (uint32)width * (uint32)height;
    uint8 threshold = 0;
    uint16 i;
    uint16 j;
    float w0 = 0.0f;
    float u0tmp = 0.0f;
    float delta_max = 0.0f;

    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            uint8 gray = image[i * width + j];
            pixel_count[gray]++;
            gray_sum += gray;
        }
    }

    Image_Runtime.avg_gray = (uint16)(gray_sum / pixel_sum);

    for (i = 0; i < 256; i++)
    {
        pixel_pro[i] = (float)pixel_count[i] / (float)pixel_sum;
    }

    for (j = 0; j < max_threshold; j++)
    {
        float w1;
        float u1tmp;
        float u0;
        float u1;
        float u;
        float delta_tmp;

        w0 += pixel_pro[j];
        u0tmp += (float)j * pixel_pro[j];
        w1 = 1.0f - w0;
        u1tmp = (float)gray_sum / (float)pixel_sum - u0tmp;

        if (w0 <= 0.0f || w1 <= 0.0f)
        {
            continue;
        }

        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        u = u0tmp + u1tmp;
        delta_tmp = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u);

        if (delta_tmp > delta_max)
        {
            delta_max = delta_tmp;
            threshold = (uint8)j;
        }
    }

    return threshold;
}

static void Image_Compress(uint8 *in_image, uint8 *out_image)
{
    uint16 y;
    uint16 x;
    float h_ratio = (float)MT9V03X_H / (float)DEAL_IMAGE_H;
    float w_ratio = (float)MT9V03X_W / (float)DEAL_IMAGE_W;

    for (y = 0; y < DEAL_IMAGE_H; y++)
    {
        uint16 src_y = (uint16)((float)y * h_ratio);
        for (x = 0; x < DEAL_IMAGE_W; x++)
        {
            uint16 src_x = (uint16)((float)x * w_ratio);
            out_image[y * DEAL_IMAGE_W + x] = in_image[src_y * MT9V03X_W + src_x];
        }
    }
}

static void Image_Binary(uint8 *in_image, uint8 *out_image)
{
    uint8 x;
    uint8 y;
    uint8 threshold = Image_Threshold(in_image, DEAL_IMAGE_W, DEAL_IMAGE_H, 210);
    uint8 threshold_floor;

    threshold_floor = (Image_Runtime.avg_gray < 70) ? 75 : 92;
    if (threshold < threshold_floor)
        threshold = threshold_floor;
    Image_Runtime.threshold = threshold;

    for (y = 0; y < DEAL_IMAGE_H; y++)
    {
        for (x = 0; x < DEAL_IMAGE_W; x++)
        {
            int16 final_threshold;

            if (y < 12)
                final_threshold = (int16)threshold + 18;
            else if (y < 38)
                final_threshold = (int16)threshold;
            else
                final_threshold = (int16)threshold - 12;

            if (x < 8 || x > DEAL_IMAGE_W - 9)
                final_threshold -= 8;

            if (final_threshold < 65)
                final_threshold = 65;
            if (final_threshold > 220)
                final_threshold = 220;

            out_image[y * DEAL_IMAGE_W + x] =
                (in_image[y * DEAL_IMAGE_W + x] > final_threshold) ? 1 : 0;
        }
    }
}

static void Image_Filter(uint8 *image)
{
    uint8 x;
    uint8 y;

    for (y = 2; y < DEAL_IMAGE_H - 2; y++)
    {
        for (x = 2; x < DEAL_IMAGE_W - 2; x++)
        {
            uint8 near =
                image[DEAL_IMAGE_W * (y - 1) + x] +
                image[DEAL_IMAGE_W * (y + 1) + x] +
                image[DEAL_IMAGE_W * y + x + 1] +
                image[DEAL_IMAGE_W * y + x - 1];

            if (image[DEAL_IMAGE_W * y + x] == 0 && near >= 3)
                image[DEAL_IMAGE_W * y + x] = 1;
            else if (image[DEAL_IMAGE_W * y + x] == 1 && near <= 1)
                image[DEAL_IMAGE_W * y + x] = 0;
        }
    }
}

static uint8 Image_Find_Start_Line(uint8 *image, uint8 *start_y, uint8 *left, uint8 *right)
{
    int16 y;
    uint8 best_y = DEAL_IMAGE_H - 2;
    uint8 best_left = 1;
    uint8 best_right = DEAL_IMAGE_W - 2;
    int16 best_score = -32768;

    for (y = DEAL_IMAGE_H - 2; y > DEAL_IMAGE_H - 14; y--)
    {
        uint8 x = 2;

        while (x < DEAL_IMAGE_W - 2)
        {
            uint8 run_start;
            uint8 run_end;
            uint8 len;
            uint8 center;
            int16 dist;
            int16 score;

            while (x < DEAL_IMAGE_W - 2 && image[y * DEAL_IMAGE_W + x] == 0)
                x++;
            run_start = x;
            while (x < DEAL_IMAGE_W - 2 && image[y * DEAL_IMAGE_W + x] == 1)
                x++;
            if (x <= run_start)
                continue;

            run_end = x - 1;
            len = run_end - run_start + 1;
            center = (run_start + run_end) / 2;
            dist = (center > image_row_mid) ? (center - image_row_mid) : (image_row_mid - center);
            score = (int16)len * 3 - dist * 2 - (int16)(DEAL_IMAGE_H - 2 - y);

            if (len >= 14 && score > best_score)
            {
                best_score = score;
                best_y = (uint8)y;
                best_left = (run_start > 1) ? (run_start - 1) : 1;
                best_right = (run_end < DEAL_IMAGE_W - 2) ? (run_end + 1) : DEAL_IMAGE_W - 2;
            }
        }
    }

    if (best_score == -32768)
        return 0;

    *start_y = best_y;
    *left = best_left;
    *right = best_right;
    return 1;
}

static void Image_Add_Block(image_block_struct block[], uint8 *block_num, uint8 x1, uint8 x2, uint8 y)
{
    uint8 i;
    uint8 best = 255;
    uint8 best_gap = 255;

    for (i = 0; i < *block_num; i++)
    {
        uint8 overlap = !(x2 < block[i].x1 || x1 > block[i].x2);
        uint8 near_x = (x1 > block[i].x2) ? (x1 - block[i].x2) : ((block[i].x1 > x2) ? (block[i].x1 - x2) : 0);
        uint8 near_y = y - block[i].last_y;

        if (block[i].used && near_y <= 2 && (overlap || near_x <= 3) && near_x < best_gap)
        {
            best = i;
            best_gap = near_x;
        }
    }

    if (best != 255)
    {
        if (x1 < block[best].x1)
            block[best].x1 = x1;
        if (x2 > block[best].x2)
            block[best].x2 = x2;
        block[best].y2 = y;
        block[best].last_y = y;
        block[best].area += (uint16)(x2 - x1 + 1);
    }
    else if (*block_num < 12)
    {
        block[*block_num].used = 1;
        block[*block_num].x1 = x1;
        block[*block_num].x2 = x2;
        block[*block_num].y1 = y;
        block[*block_num].y2 = y;
        block[*block_num].last_y = y;
        block[*block_num].area = (uint16)(x2 - x1 + 1);
        (*block_num)++;
    }
}

static uint8 Image_Get_Road_Roi(uint8 y, uint8 margin, uint8 *x1, uint8 *x2)
{
    uint8 left;
    uint8 right;

    if (y >= DEAL_IMAGE_H)
        return 0;
    if (!Deal_Image_Left_Flag[y] || !Deal_Image_Right_Flag[y])
        return 0;
    if (Deal_Image_Left_Line[y] >= Deal_Image_Right_Line[y])
        return 0;
    if (Deal_Image_Wide[y] < 10 || Deal_Image_Wide[y] > DEAL_IMAGE_W - 6)
        return 0;

    left = (Deal_Image_Left_Line[y] > margin) ? (Deal_Image_Left_Line[y] - margin) : 1;
    right = Deal_Image_Right_Line[y] + margin;
    if (right > DEAL_IMAGE_W - 2)
        right = DEAL_IMAGE_W - 2;
    if (left >= right)
        return 0;

    *x1 = left;
    *x2 = right;
    return 1;
}

static uint8 Image_Get_Detect_Roi(uint8 y, uint8 margin, uint8 *x1, uint8 *x2)
{
    uint8 half_width;
    uint8 left;
    uint8 right;

    if (Image_Get_Road_Roi(y, margin, x1, x2))
        return 1;

    if (y >= DEAL_IMAGE_H)
        return 0;

    half_width = 8 + y / 2;
    if (half_width > 36)
        half_width = 36;

    left = (DEAL_IMAGE_W / 2 > half_width + margin) ?
        (DEAL_IMAGE_W / 2 - half_width - margin) : 1;
    right = DEAL_IMAGE_W / 2 + half_width + margin;
    if (right > DEAL_IMAGE_W - 2)
        right = DEAL_IMAGE_W - 2;
    if (left >= right)
        return 0;

    *x1 = left;
    *x2 = right;
    return 1;
}

static void Image_Detect_White_Blocks(uint8 *image)
{
    image_block_struct block[12] = {0};
    uint8 block_num = 0;
    uint8 x;
    uint8 y;
    uint8 i;

    Image_Runtime.white_block_count = 0;

    for (y = 6; y < DEAL_IMAGE_H - 4; y++)
    {
        uint8 roi_x1;
        uint8 roi_x2;

        if (!Image_Get_Detect_Roi(y, 4, &roi_x1, &roi_x2))
            continue;

        x = roi_x1;
        while (x <= roi_x2)
        {
            uint8 start;
            uint8 end;
            uint8 len;

            while (x <= roi_x2 && image[y * DEAL_IMAGE_W + x] == 0)
                x++;
            start = x;
            while (x <= roi_x2 && image[y * DEAL_IMAGE_W + x] == 1)
                x++;
            end = (x > start) ? (x - 1) : start;
            len = end - start + 1;

            if (len >= 3 && start <= roi_x2)
                Image_Add_Block(block, &block_num, start, end, y);
        }
    }

    for (i = 0; i < block_num; i++)
    {
        uint8 w = block[i].x2 - block[i].x1 + 1;
        uint8 h = block[i].y2 - block[i].y1 + 1;
        uint8 road_surface_block = 0;

        if (w > 38 || h > 22 || block[i].area > 520)
            road_surface_block = 1;
        if (block[i].y2 > DEAL_IMAGE_H - 10 && w > 18)
            road_surface_block = 1;
        if (block[i].x1 <= 4 || block[i].x2 >= DEAL_IMAGE_W - 5)
            road_surface_block = 1;

        if (w >= 5 && h >= 3 && block[i].area >= 18 && !road_surface_block)
        {
            Image_Runtime.white_block_count++;
        }
    }
}

static uint8 Image_Abs_Diff(uint8 a, uint8 b)
{
    return (a > b) ? (a - b) : (b - a);
}

static uint8 Image_Line_Is_Straight(uint8 line[DEAL_IMAGE_H], uint8 flag[DEAL_IMAGE_H])
{
    int16 y;
    uint8 valid = 0;
    uint8 smooth = 0;
    uint8 big_jump = 0;
    uint8 last = 0;
    uint8 have_last = 0;
    int16 start = (End_Line > 8) ? End_Line + 4 : 12;

    for (y = DEAL_IMAGE_H - 5; y > start; y--)
    {
        if (!flag[y])
        {
            have_last = 0;
            continue;
        }

        valid++;
        if (have_last)
        {
            uint8 diff = Image_Abs_Diff(line[y], last);

            if (diff <= 2)
                smooth++;
            else if (diff >= 6)
                big_jump++;
        }
        last = line[y];
        have_last = 1;
    }

    if (valid < 16)
        return 0;
    if (big_jump > 3)
        return 0;
    return (smooth >= valid / 2) ? 1 : 0;
}

static void Image_Find_Line_Corners(void)
{
    int16 y;
    int16 start = (End_Line > 6) ? End_Line + 4 : 10;

    Lower_Left_Corner_Flag = 0;
    Lower_Right_Corner_Flag = 0;
    Upper_Left_Corner_Flag = 0;
    Upper_Right_Corner_Flag = 0;

    for (y = DEAL_IMAGE_H - 6; y > start + 3; y--)
    {
        if (!Lower_Left_Corner_Flag &&
            Deal_Image_Left_Flag[y] &&
            !Deal_Image_Left_Flag[y - 3] &&
            Deal_Image_Left_Line[y] > 4)
        {
            Lower_Left_Corner_Flag = 1;
        }

        if (!Lower_Right_Corner_Flag &&
            Deal_Image_Right_Flag[y] &&
            !Deal_Image_Right_Flag[y - 3] &&
            Deal_Image_Right_Line[y] < DEAL_IMAGE_W - 5)
        {
            Lower_Right_Corner_Flag = 1;
        }

        if (!Upper_Left_Corner_Flag &&
            !Deal_Image_Left_Flag[y] &&
            Deal_Image_Left_Flag[y - 3])
        {
            Upper_Left_Corner_Flag = 1;
        }

        if (!Upper_Right_Corner_Flag &&
            !Deal_Image_Right_Flag[y] &&
            Deal_Image_Right_Flag[y - 3])
        {
            Upper_Right_Corner_Flag = 1;
        }
    }
}

static void Image_Update_Line_Shape_Feature(void)
{
    Left_Straight_Flag = Image_Line_Is_Straight(Deal_Image_Left_Line, Deal_Image_Left_Flag);
    Right_Straight_Flag = Image_Line_Is_Straight(Deal_Image_Right_Line, Deal_Image_Right_Flag);
    Image_Find_Line_Corners();

    Image_Runtime.left_lost_flag = Road_Left_Lost_Flag;
    Image_Runtime.right_lost_flag = Road_Right_Lost_Flag;
    Image_Runtime.left_straight_flag = Left_Straight_Flag;
    Image_Runtime.right_straight_flag = Right_Straight_Flag;
    Image_Runtime.lower_left_corner = Lower_Left_Corner_Flag;
    Image_Runtime.lower_right_corner = Lower_Right_Corner_Flag;
}

static void Image_Detect_Transverse_Bands(uint8 *image)
{
    uint8 y;
    uint8 in_band = 0;
    uint8 band_end = 0;
    uint8 first_band_start = 0;
    uint8 last_band_end = 0;

    Step_Band_Count = 0;
    Step_Band_Span = 0;
    Transverse_Row_Count = 0;

    for (y = 8; y < DEAL_IMAGE_H - 5; y++)
    {
        uint8 x;
        uint8 x1;
        uint8 x2;
        uint8 width;
        uint8 dark_count = 0;
        uint8 row_hit;

        if (!Image_Get_Detect_Roi(y, 2, &x1, &x2))
        {
            if (in_band)
            {
                Step_Band_Count++;
                band_end = y - 1;
                last_band_end = band_end;
                in_band = 0;
            }
            continue;
        }

        width = x2 - x1 + 1;
        if (width < 18)
            continue;

        for (x = x1; x <= x2; x++)
        {
            if (image[y * DEAL_IMAGE_W + x] == 0)
                dark_count++;
        }

        row_hit = ((uint16)dark_count * 100U >= (uint16)width * 62U) ? 1 : 0;
        if (row_hit)
        {
            Transverse_Row_Count++;
            if (!in_band)
            {
                in_band = 1;
                if (Step_Band_Count == 0)
                    first_band_start = y;
            }
            band_end = y;
        }
        else if (in_band)
        {
            Step_Band_Count++;
            last_band_end = band_end;
            in_band = 0;
        }
    }

    if (in_band)
    {
        Step_Band_Count++;
        last_band_end = band_end;
    }
    if (Step_Band_Count > 0 && last_band_end >= first_band_start)
        Step_Band_Span = last_band_end - first_band_start + 1;

    Image_Runtime.step_band_count = Step_Band_Count;
    Image_Runtime.step_band_span = Step_Band_Span;
    Image_Runtime.transverse_row_count = Transverse_Row_Count;
}

static uint8 Image_Detect_Bridge_By_Feature(void)
{
    uint8 left_bridge = 0;
    uint8 right_bridge = 0;
    uint8 narrow_far =
        (Image_Runtime.road_width_far > 0 &&
         Image_Runtime.road_width_near > 0 &&
         Image_Runtime.road_width_far + 7 < Image_Runtime.road_width_near) ? 1 : 0;
    uint8 narrow_road =
        (Image_Runtime.road_width_max > 0 &&
         Image_Runtime.road_width_max < DEAL_IMAGE_W - 12) ? 1 : 0;
    uint8 enough_line = (Road_Valid_Count >= 10) ? 1 : 0;

    Image_Runtime.bridge_side = 0;

    if (Road_Left_Lost_Flag && !Road_Right_Lost_Flag)
    {
        if (Right_Straight_Flag)
            left_bridge += 2;
        if (Lower_Left_Corner_Flag || Upper_Left_Corner_Flag)
            left_bridge++;
        if (narrow_far || narrow_road)
            left_bridge++;
        if (enough_line)
            left_bridge++;
    }

    if (Road_Right_Lost_Flag && !Road_Left_Lost_Flag)
    {
        if (Left_Straight_Flag)
            right_bridge += 2;
        if (Lower_Right_Corner_Flag || Upper_Right_Corner_Flag)
            right_bridge++;
        if (narrow_far || narrow_road)
            right_bridge++;
        if (enough_line)
            right_bridge++;
    }

    if (left_bridge >= 4 && Transverse_Row_Count < 5)
    {
        Image_Runtime.bridge_side = 1;
        return 1;
    }
    if (right_bridge >= 4 && Transverse_Row_Count < 5)
    {
        Image_Runtime.bridge_side = 2;
        return 1;
    }
    return 0;
}

static uint8 Image_Detect_Step_By_Feature(void)
{
    uint8 score = 0;

    if (Image_Runtime.bumpy_candidate)
        return 0;

    if (End_Line >= 28 || Lost_Line >= 8)
        score++;
    if (Image_Runtime.road_width_far > 0 &&
        Image_Runtime.road_width_near > 0 &&
        Image_Runtime.road_width_far + 10 < Image_Runtime.road_width_near)
        score++;
    if (Image_Runtime.road_width_far > 0 && Image_Runtime.road_width_far <= 28)
        score++;
    if (Step_Band_Count >= 2 || Transverse_Row_Count >= 5)
        score += 2;
    if (Road_Left_Lost_Flag && Road_Right_Lost_Flag)
        score++;

    return (score >= 4) ? 1 : 0;
}

static void Image_Detect_Road_Shape(void)
{
    Image_Update_Line_Shape_Feature();

    Image_Runtime.bridge_candidate = 0;
    Image_Runtime.obstacle_candidate = 0;
    Image_Runtime.step_candidate = 0;

    Image_Runtime.bridge_candidate = Image_Detect_Bridge_By_Feature();
    Image_Runtime.step_candidate = Image_Detect_Step_By_Feature();

    if (Image_Runtime.road_width_far > 0 &&
        Image_Runtime.road_width_far <= 22 &&
        Image_Runtime.road_width_near >= 36 &&
        Transverse_Row_Count <= 3 &&
        !Image_Runtime.bridge_candidate &&
        !Image_Runtime.step_candidate &&
        !Image_Runtime.bumpy_candidate)
    {
        Image_Runtime.obstacle_candidate = 1;
    }
}

static uint8 Image_Update_Detect_Score(uint8 raw, uint8 *score, uint8 *stable, uint8 confirm, uint8 release)
{
    if (raw)
    {
        if (*score < 8)
            (*score)++;
    }
    else if (*score > 0)
    {
        (*score)--;
    }

    if (*score >= confirm)
        *stable = 1;
    else if (*score <= release)
        *stable = 0;

    return *stable;
}

static void Image_Stabilize_Detections(void)
{
    uint8 raw_bridge = Image_Runtime.bridge_candidate;
    uint8 raw_bumpy = Image_Runtime.bumpy_candidate;
    uint8 raw_obstacle = Image_Runtime.obstacle_candidate;
    uint8 raw_step = Image_Runtime.step_candidate;

    if (raw_bumpy)
    {
        raw_bridge = 0;
        raw_step = 0;
    }
    if (raw_bridge || raw_bumpy || raw_step)
        raw_obstacle = 0;

    Image_Runtime.bumpy_candidate =
        Image_Update_Detect_Score(raw_bumpy, &Bumpy_Detect_Score, &Bumpy_Stable_Flag, 2, 0);
    Image_Runtime.step_candidate =
        Image_Update_Detect_Score(raw_step, &Step_Detect_Score, &Step_Stable_Flag, 3, 1);
    Image_Runtime.bridge_candidate =
        Image_Update_Detect_Score(raw_bridge, &Bridge_Detect_Score, &Bridge_Stable_Flag, 3, 1);
    Image_Runtime.obstacle_candidate =
        Image_Update_Detect_Score(raw_obstacle, &Obstacle_Detect_Score, &Obstacle_Stable_Flag, 3, 1);

    if (Image_Runtime.bumpy_candidate)
    {
        Image_Runtime.step_candidate = 0;
        Image_Runtime.bridge_candidate = 0;
        Image_Runtime.obstacle_candidate = 0;
    }
    else if (Image_Runtime.step_candidate)
    {
        Image_Runtime.bridge_candidate = 0;
        Image_Runtime.obstacle_candidate = 0;
    }
    else if (Image_Runtime.bridge_candidate)
    {
        Image_Runtime.obstacle_candidate = 0;
    }
    else
    {
        Image_Runtime.bridge_side = 0;
    }

    Image_Runtime.bumpy_score = Bumpy_Detect_Score;
    Image_Runtime.step_score = Step_Detect_Score;
    Image_Runtime.bridge_score = Bridge_Detect_Score;
    Image_Runtime.obstacle_score = Obstacle_Detect_Score;
}

static void Image_Save_Bumpy_Strip(uint8 x1, uint8 x2, uint8 y1, uint8 y2)
{
    uint8 w = x2 - x1 + 1;
    uint8 h = y2 - y1 + 1;

    if (w < 6 || w > 62)
        return;
    if (h < 1 || h > 10)
        return;
    if (w < h * 3)
        return;

    if (Detect_Strip_Count < 8)
    {
        Detect_Strip_X1[Detect_Strip_Count] = x1;
        Detect_Strip_X2[Detect_Strip_Count] = x2;
        Detect_Strip_Y1[Detect_Strip_Count] = y1;
        Detect_Strip_Y2[Detect_Strip_Count] = y2;
        Detect_Strip_Count++;
    }
}

static void Image_Detect_Bumpy_Strips(uint8 *image)
{
    uint8 x;
    uint8 y;
    uint8 i;
    uint8 in_band = 0;
    uint8 band_x1 = DEAL_IMAGE_W - 1;
    uint8 band_x2 = 0;
    uint8 band_y1 = 0;
    uint8 band_y2 = 0;
    uint8 regular_count = 1;
    uint8 best_regular = 1;
    uint8 span = 0;
    uint8 row_hit_count = 0;

    Detect_Strip_Count = 0;
    Image_Runtime.bumpy_candidate = 0;
    Image_Runtime.bumpy_strip_count = 0;
    Image_Runtime.bumpy_span = 0;

    for (y = 8; y < DEAL_IMAGE_H - 4; y++)
    {
        uint8 roi_x1;
        uint8 roi_x2;
        uint8 best_x1 = 0;
        uint8 best_x2 = 0;
        uint8 best_len = 0;
        uint8 dark_total = 0;
        uint8 row_hit;

        if (!Image_Get_Detect_Roi(y, 8, &roi_x1, &roi_x2))
        {
            if (in_band)
            {
                Image_Save_Bumpy_Strip(band_x1, band_x2, band_y1, band_y2);
                in_band = 0;
            }
            continue;
        }

        x = roi_x1;
        while (x <= roi_x2)
        {
            uint8 start;
            uint8 end;
            uint8 len;

            while (x <= roi_x2 && image[y * DEAL_IMAGE_W + x] == 1)
                x++;
            start = x;
            while (x <= roi_x2 && image[y * DEAL_IMAGE_W + x] == 0)
                x++;
            end = (x > start) ? (x - 1) : start;
            len = end - start + 1;

            if (len >= 4)
                dark_total += len;
            if (len > best_len)
            {
                best_len = len;
                best_x1 = start;
                best_x2 = end;
            }
        }

        row_hit = (best_len >= 6 && best_len <= 62 && dark_total <= 70) ? 1 : 0;

        if (row_hit)
        {
            row_hit_count++;
            if (!in_band)
            {
                in_band = 1;
                band_x1 = best_x1;
                band_x2 = best_x2;
                band_y1 = y;
                band_y2 = y;
            }
            else
            {
                if (best_x1 < band_x1)
                    band_x1 = best_x1;
                if (best_x2 > band_x2)
                    band_x2 = best_x2;
                band_y2 = y;
            }
        }
        else if (in_band)
        {
            Image_Save_Bumpy_Strip(band_x1, band_x2, band_y1, band_y2);
            in_band = 0;
        }
    }

    if (in_band)
        Image_Save_Bumpy_Strip(band_x1, band_x2, band_y1, band_y2);

    Image_Runtime.bumpy_strip_count = Detect_Strip_Count;

    if (Detect_Strip_Count >= 2)
        span = Detect_Strip_Y2[Detect_Strip_Count - 1] - Detect_Strip_Y1[0] + 1;
    Image_Runtime.bumpy_span = span;
    if (Detect_Strip_Count == 0 && row_hit_count >= 6)
        Image_Runtime.bumpy_strip_count = row_hit_count;

    for (i = 1; i < Detect_Strip_Count; i++)
    {
        uint8 prev_mid = (Detect_Strip_Y1[i - 1] + Detect_Strip_Y2[i - 1]) / 2;
        uint8 now_mid = (Detect_Strip_Y1[i] + Detect_Strip_Y2[i]) / 2;
        uint8 gap = now_mid - prev_mid;

        if (gap >= 2 && gap <= 18)
        {
            regular_count++;
            if (regular_count > best_regular)
                best_regular = regular_count;
        }
        else
        {
            regular_count = 1;
        }
    }

    if (Detect_Strip_Count >= 4 || best_regular >= 3 ||
        row_hit_count >= 6 ||
        (Detect_Strip_Count >= 2 && span >= 14))
        Image_Runtime.bumpy_candidate = 1;
}

static void Image_Find_Line(uint8 *image)
{
    uint8 x;
    int16 y;
    uint8 start_y;
    uint8 start_left;
    uint8 start_right;
    uint8 *row_addr;

    End_Line = 20;
    Lost_Line = 0;
    Left_Lost_Line = 0;
    Right_Lost_Line = 0;
    Road_Valid_Count = 0;
    Road_Left_Lost_Flag = 0;
    Road_Right_Lost_Flag = 0;
    Image_Runtime.road_width_max = 0;
    Image_Runtime.road_width_near = 0;
    Image_Runtime.road_width_far = 0;

    for (y = 0; y < DEAL_IMAGE_H; y++)
    {
        Deal_Image_Left_Line[y] = 0;
        Deal_Image_Right_Line[y] = DEAL_IMAGE_W - 1;
        Deal_Image_Mid_Line[y] = DEAL_IMAGE_W / 2;
        Deal_Image_Wide[y] = DEAL_IMAGE_W - 1;
        Deal_Image_Left_Flag[y] = 0;
        Deal_Image_Right_Flag[y] = 0;
    }

    if (Image_Find_Start_Line(image, &start_y, &start_left, &start_right))
    {
        bottom_left_line = start_left;
        bottom_right_line = start_right;
    }
    else
    {
        row_addr = image + DEAL_IMAGE_W * (DEAL_IMAGE_H - 2);
        start_y = DEAL_IMAGE_H - 2;
        bottom_left_line = 0;
        bottom_right_line = DEAL_IMAGE_W - 1;

        for (x = image_row_mid; x > 1; x--)
        {
            if (row_addr[x - 1] == 0 && row_addr[x] == 0)
            {
                bottom_left_line = x;
                break;
            }
        }
        for (x = image_row_mid; x < DEAL_IMAGE_W - 2; x++)
        {
            if (row_addr[x + 1] == 0 && row_addr[x] == 0)
            {
                bottom_right_line = x;
                break;
            }
        }
    }

    for (y = start_y; y < DEAL_IMAGE_H; y++)
    {
        Deal_Image_Left_Line[y] = bottom_left_line;
        Deal_Image_Right_Line[y] = bottom_right_line;
        Deal_Image_Mid_Line[y] = (bottom_left_line + bottom_right_line) / 2;
        Deal_Image_Wide[y] = bottom_right_line - bottom_left_line;
        Deal_Image_Left_Flag[y] = 1;
        Deal_Image_Right_Flag[y] = 1;
    }

    for (y = (int16)start_y - 1; y > End_Line; y--)
    {
        uint8 left_border;
        uint8 right_border;

        row_addr = image + DEAL_IMAGE_W * y;

        left_border = (Deal_Image_Left_Line[y + 1] > Border_Interval) ?
            Deal_Image_Left_Line[y + 1] - Border_Interval : 1;
        right_border = Deal_Image_Left_Line[y + 1] + Border_Interval;
        if (right_border > DEAL_IMAGE_W - 2)
            right_border = DEAL_IMAGE_W - 2;

        for (x = left_border; x <= right_border; x++)
        {
            if (row_addr[x] == 0 && row_addr[x + 1] == 1)
            {
                Deal_Image_Left_Line[y] = x;
                Deal_Image_Left_Flag[y] = 1;
                break;
            }
        }
        if (Deal_Image_Left_Flag[y] == 0)
        {
            Deal_Image_Left_Line[y] = Deal_Image_Left_Line[y + 1];
            Left_Lost_Line++;
        }

        left_border = (Deal_Image_Right_Line[y + 1] > Border_Interval) ?
            Deal_Image_Right_Line[y + 1] - Border_Interval : 1;
        right_border = Deal_Image_Right_Line[y + 1] + Border_Interval;
        if (right_border > DEAL_IMAGE_W - 2)
            right_border = DEAL_IMAGE_W - 2;

        for (x = right_border; x > left_border; x--)
        {
            if (row_addr[x] == 0 && row_addr[x - 1] == 1)
            {
                Deal_Image_Right_Line[y] = x;
                Deal_Image_Right_Flag[y] = 1;
                break;
            }
        }
        if (Deal_Image_Right_Flag[y] == 0)
        {
            Deal_Image_Right_Line[y] = Deal_Image_Right_Line[y + 1];
            Right_Lost_Line++;
        }

        if (Deal_Image_Left_Flag[y] == 0 && Deal_Image_Right_Flag[y] == 0)
            Lost_Line++;

        if (Deal_Image_Right_Line[y] <= Deal_Image_Left_Line[y] + 6)
        {
            End_Line = (uint8)y + 1;
            break;
        }

        Deal_Image_Mid_Line[y] = (Deal_Image_Left_Line[y] + Deal_Image_Right_Line[y]) / 2;
        Deal_Image_Wide[y] = Deal_Image_Right_Line[y] - Deal_Image_Left_Line[y];
    }
}

static void Image_Update_Road_Feature(void)
{
    uint8 y;
    uint8 valid_count = 0;
    uint8 max_width = 0;
    uint16 near_sum = 0;
    uint16 far_sum = 0;
    uint8 near_count = 0;
    uint8 far_count = 0;

    Road_Left_Lost_Flag = (Left_Lost_Line >= 10) ? 1 : 0;
    Road_Right_Lost_Flag = (Right_Lost_Line >= 10) ? 1 : 0;

    for (y = End_Line + 1; y < DEAL_IMAGE_H - 2; y++)
    {
        uint8 width;

        if (!Deal_Image_Left_Flag[y] || !Deal_Image_Right_Flag[y])
            continue;
        if (Deal_Image_Right_Line[y] <= Deal_Image_Left_Line[y])
            continue;

        width = Deal_Image_Wide[y];
        if (width < 10 || width > DEAL_IMAGE_W - 4)
            continue;

        valid_count++;
        if (width > max_width)
            max_width = width;

        if (y >= DEAL_IMAGE_H - 18)
        {
            near_sum += width;
            near_count++;
        }
        else if (y >= 22 && y <= 34)
        {
            far_sum += width;
            far_count++;
        }
    }

    Road_Valid_Count = valid_count;
    Image_Runtime.road_valid_count = valid_count;
    Image_Runtime.road_width_max = max_width;
    Image_Runtime.road_width_near = (near_count > 0) ? (uint8)(near_sum / near_count) : 0;
    Image_Runtime.road_width_far = (far_count > 0) ? (uint8)(far_sum / far_count) : 0;
}

static float Line_Curvature(uint8 in_line[DEAL_IMAGE_H], uint8 start, uint8 end)
{
    uint8 i;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float avr_x;
    float avr_y;
    float up = 0.0f;
    float down = 0.0f;
    float count = 0.0f;

    if (end > DEAL_IMAGE_H - 5)
        end = DEAL_IMAGE_H - 5;
    if (start >= end)
        return 0.0f;

    for (i = start; i <= end; i++)
    {
        count += 1.0f;
        sum_x += i;
        sum_y += in_line[i];
    }

    avr_x = sum_x / count;
    avr_y = sum_y / count;

    for (i = start; i <= end; i++)
    {
        up += ((float)in_line[i] - avr_y) * ((float)i - avr_x);
        down += ((float)i - avr_x) * ((float)i - avr_x);
    }

    if (down == 0.0f)
        return 0.0f;
    return up * 16.0f / down;
}

static Road_State_e Road_Check(void)
{
    uint8 i;
    Road_State_e state = ROAD_NORMAL;

    if (Image_Runtime.bumpy_candidate)
    {
        state = ROAD_BUMPY;
    }
    else if (Image_Runtime.step_candidate)
    {
        state = ROAD_STEP;
    }
    else if (Image_Runtime.obstacle_candidate)
    {
        state = ROAD_OBSTACLE;
    }
    else if (Image_Runtime.bridge_candidate)
    {
        state = ROAD_BRIDGE;
    }
    else if (Lost_Line > 22 || End_Line > 48)
    {
        state = ROAD_LOST;
    }

    road_state_filter[0] = road_state_filter[1];
    road_state_filter[1] = road_state_filter[2];
    road_state_filter[2] = road_state_filter[3];
    road_state_filter[3] = road_state_filter[4];
    road_state_filter[4] = (uint8)state;

    Image_Runtime.stable_count = 0;
    for (i = 0; i < 5; i++)
    {
        if (road_state_filter[i] == (uint8)state)
            Image_Runtime.stable_count++;
    }

    if (Image_Runtime.stable_count >= 3)
        return state;

    return Road_State;
}

static void Image_Update_Exposure(void)
{
#if (MT9V03X_AUTO_EXP_DEF == 0)
    static uint8 div = 0;

    div++;
    if (div < 10)
        return;
    div = 0;

    if (Image_Runtime.avg_gray > 165 && Image_Runtime.exposure > 40)
    {
        Image_Runtime.exposure -= 10;
        mt9v03x_set_exposure_time(Image_Runtime.exposure);
    }
    else if (Image_Runtime.avg_gray < 75 && Image_Runtime.exposure < 650)
    {
        Image_Runtime.exposure += 10;
        mt9v03x_set_exposure_time(Image_Runtime.exposure);
    }
#endif
}

static float Image_Deal(uint8 prospect, float error_offset)
{
    float error;

    Image_Compress(mt9v03x_image[0], Deal_Image_Gray[0]);
    Image_Binary(Deal_Image_Gray[0], Deal_Image_Binary[0]);
    Image_Filter(Deal_Image_Binary[0]);
    Image_Find_Line(Deal_Image_Binary[0]);
    Image_Update_Road_Feature();
    Image_Detect_White_Blocks(Deal_Image_Binary[0]);
    Image_Detect_Bumpy_Strips(Deal_Image_Binary[0]);
    Image_Detect_Transverse_Bands(Deal_Image_Binary[0]);
    Image_Detect_Road_Shape();
    Image_Stabilize_Detections();
    Road_State = Road_Check();

    if (prospect > DEAL_IMAGE_H - 5)
        prospect = DEAL_IMAGE_H - 5;
    if (prospect < End_Line)
        prospect = End_Line;

    error = 1.1f * ((float)Deal_Image_Mid_Line[prospect] - (float)DEAL_IMAGE_W / 2.0f)
          - 0.75f * Line_Curvature(Deal_Image_Mid_Line, End_Line, DEAL_IMAGE_H - 5)
          + error_offset;

    if (End_Line > 20)
        Image_Runtime.obstacle_distance_mm = (float)(DEAL_IMAGE_H - End_Line) * 18.0f;
    else
        Image_Runtime.obstacle_distance_mm = 900.0f;

    Image_Update_Exposure();
    return error;
}

void Image_Init(void)
{
    Image_Runtime.enable = 0;
    Image_Runtime.show_enable = 0;
    Image_Runtime.exposure = MT9V03X_EXP_TIME_DEF;

    if (mt9v03x_init() == 0)
    {
        Image_Set_Outdoor_Mode();
        Image_Runtime.enable = 1;
    }
}

void Image_Set_Outdoor_Mode(void)
{
    Image_Runtime.exposure = MT9V03X_EXP_TIME_DEF;

#if (MT9V03X_AUTO_EXP_DEF == 0)
    mt9v03x_set_exposure_time(Image_Runtime.exposure);
#endif
}

void Image_Set_Show(uint8 enable)
{
    Image_Runtime.show_enable = enable;
}

static void Image_Draw_Frame(uint16 x0, uint16 y0, uint16 width, uint16 height, uint16 color)
{
    ips200_draw_line(x0, y0, x0 + width - 1, y0, color);
    ips200_draw_line(x0, y0 + height - 1, x0 + width - 1, y0 + height - 1, color);
    ips200_draw_line(x0, y0, x0, y0 + height - 1, color);
    ips200_draw_line(x0 + width - 1, y0, x0 + width - 1, y0 + height - 1, color);
}

static uint8 Image_Row_Is_Valid(uint8 row)
{
    if (row >= DEAL_IMAGE_H)
        return 0;
    if (!Deal_Image_Left_Flag[row] || !Deal_Image_Right_Flag[row])
        return 0;
    if (Deal_Image_Left_Line[row] >= Deal_Image_Right_Line[row])
        return 0;
    if (Deal_Image_Wide[row] < 10 || Deal_Image_Wide[row] > DEAL_IMAGE_W - 4)
        return 0;
    return 1;
}

static void Image_Draw_Scale_Line(uint16 x0, uint16 y0, uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 scale, uint16 color)
{
    uint16 sx1 = x0 + (uint16)x1 * scale;
    uint16 sy1 = y0 + (uint16)y1 * scale;
    uint16 sx2 = x0 + (uint16)x2 * scale;
    uint16 sy2 = y0 + (uint16)y2 * scale;

    ips200_draw_line(sx1, sy1, sx2, sy2, color);
    ips200_draw_line(sx1, sy1 + 1, sx2, sy2 + 1, color);
}

static void Image_Draw_Binary_View(uint16 x0, uint16 y0, uint8 scale)
{
    ips200_show_gray_image(x0, y0, Deal_Image_Binary[0], DEAL_IMAGE_W, DEAL_IMAGE_H, DEAL_IMAGE_W * scale, DEAL_IMAGE_H * scale, 1);
}

static void Image_Draw_Scale_Point(uint16 x0, uint16 y0, uint8 x, uint8 y, uint8 scale, uint16 color)
{
    uint8 dx;
    uint8 dy;

    for (dy = 0; dy < scale; dy++)
    {
        for (dx = 0; dx < scale; dx++)
        {
            ips200_draw_point(x0 + (uint16)x * scale + dx, y0 + (uint16)y * scale + dy, color);
        }
    }
}

static void Image_Draw_Camera_Detect_View(uint16 x0, uint16 y0)
{
    uint8 i;
    uint16 x_ratio = (uint16)((MT9V03X_W << 8) / DEAL_IMAGE_W);
    uint16 y_ratio = (uint16)((MT9V03X_H << 8) / DEAL_IMAGE_H);

    for (i = 0; i < Detect_Strip_Count; i++)
    {
        uint16 x1 = x0 + (((uint16)Detect_Strip_X1[i] * x_ratio) >> 8);
        uint16 x2 = x0 + (((uint16)(Detect_Strip_X2[i] + 1) * x_ratio) >> 8);
        uint16 y1 = y0 + (((uint16)Detect_Strip_Y1[i] * y_ratio) >> 8);
        uint16 y2 = y0 + (((uint16)(Detect_Strip_Y2[i] + 1) * y_ratio) >> 8);

        ips200_draw_line(x1, y1, x2, y1, RGB565_YELLOW);
        ips200_draw_line(x1, y2, x2, y2, RGB565_YELLOW);
        ips200_draw_line(x1, y1, x1, y2, RGB565_YELLOW);
        ips200_draw_line(x2, y1, x2, y2, RGB565_YELLOW);
    }
}

static void Image_Make_Bright_View(void)
{
    uint16 x;
    uint16 y;
    uint8 boost = (Image_Runtime.avg_gray < 70) ? 90 : ((Image_Runtime.avg_gray < 105) ? 45 : 0);

    for (y = 0; y < MT9V03X_H; y++)
    {
        for (x = 0; x < MT9V03X_W; x++)
        {
            uint16 gray = mt9v03x_image[y][x];

            if (boost != 0)
            {
                gray = gray + boost;
                if (gray > 255)
                    gray = 255;
            }
            Show_Image_Gray[y][x] = (uint8)gray;
        }
    }
}

static void Image_Draw_Reference_View(uint16 x0, uint16 y0, uint8 scale)
{
    uint8 i;
    uint8 y;
    uint8 last_left_y = 0;
    uint8 last_right_y = 0;
    uint8 last_mid_y = 0;
    uint8 last_left_x = 0;
    uint8 last_right_x = 0;
    uint8 last_mid_x = 0;
    uint8 have_left = 0;
    uint8 have_right = 0;
    uint8 have_mid = 0;

    Image_Draw_Binary_View(x0, y0, scale);

    for (i = 0; i < Detect_Strip_Count; i++)
    {
        Image_Draw_Scale_Line(x0, y0, Detect_Strip_X1[i], Detect_Strip_Y1[i], Detect_Strip_X2[i], Detect_Strip_Y1[i], scale, RGB565_YELLOW);
        Image_Draw_Scale_Line(x0, y0, Detect_Strip_X1[i], Detect_Strip_Y2[i], Detect_Strip_X2[i], Detect_Strip_Y2[i], scale, RGB565_YELLOW);
        Image_Draw_Scale_Line(x0, y0, Detect_Strip_X1[i], Detect_Strip_Y1[i], Detect_Strip_X1[i], Detect_Strip_Y2[i], scale, RGB565_YELLOW);
        Image_Draw_Scale_Line(x0, y0, Detect_Strip_X2[i], Detect_Strip_Y1[i], Detect_Strip_X2[i], Detect_Strip_Y2[i], scale, RGB565_YELLOW);
    }

    Image_Runtime.target_found = (Road_Valid_Count >= 12) ? 1 : 0;

    for (y = End_Line; y < DEAL_IMAGE_H - 1; y++)
    {
        if (Deal_Image_Left_Flag[y] && Deal_Image_Left_Line[y] < DEAL_IMAGE_W)
        {
            if (have_left && y == last_left_y + 1)
                Image_Draw_Scale_Line(x0, y0, last_left_x, last_left_y, Deal_Image_Left_Line[y], y, scale, RGB565_RED);
            Image_Draw_Scale_Point(x0, y0, Deal_Image_Left_Line[y], y, scale, RGB565_RED);
            last_left_x = Deal_Image_Left_Line[y];
            last_left_y = y;
            have_left = 1;
        }

        if (Deal_Image_Right_Flag[y] && Deal_Image_Right_Line[y] < DEAL_IMAGE_W)
        {
            if (have_right && y == last_right_y + 1)
                Image_Draw_Scale_Line(x0, y0, last_right_x, last_right_y, Deal_Image_Right_Line[y], y, scale, RGB565_BLUE);
            Image_Draw_Scale_Point(x0, y0, Deal_Image_Right_Line[y], y, scale, RGB565_BLUE);
            last_right_x = Deal_Image_Right_Line[y];
            last_right_y = y;
            have_right = 1;
        }

        if (Image_Row_Is_Valid(y))
        {
            if (have_mid && y == last_mid_y + 1)
                Image_Draw_Scale_Line(x0, y0, last_mid_x, last_mid_y, Deal_Image_Mid_Line[y], y, scale, RGB565_GREEN);
            Image_Draw_Scale_Point(x0, y0, Deal_Image_Mid_Line[y], y, scale, RGB565_GREEN);
            last_mid_x = Deal_Image_Mid_Line[y];
            last_mid_y = y;
            have_mid = 1;
        }
    }

    if (!Image_Runtime.target_found)
    {
        ips_show_string(x0 + 8 * 8, y0 + 16 * 2, "NO");
    }
}

void Image_Show_View(uint16 x0, uint16 y0)
{
    static uint8 view_inited = 0;
    const uint16 camera_w = MT9V03X_W;
    const uint16 camera_h = MT9V03X_H;
    const uint8 line_scale = 2;
    const uint16 line_w = DEAL_IMAGE_W * line_scale;
    const uint16 line_h = DEAL_IMAGE_H * line_scale;
    const uint16 line_y = y0 + camera_h + 8;
    const uint16 info_x = x0 + line_w + 8;

    if (!view_inited)
    {
        Image_Draw_Frame(x0, y0, camera_w, camera_h, RGB565_WHITE);
        Image_Draw_Frame(x0, line_y, line_w, line_h, RGB565_WHITE);
        ips_show_string(x0 + camera_w + 4, y0 + 0, "CAM");
        ips_show_string(info_x, line_y + 0, "LINE");
        view_inited = 1;
    }

    if (!Image_Runtime.frame_ready)
        return;
    Image_Runtime.frame_ready = 0;

    Image_Make_Bright_View();
    ips200_show_gray_image(x0, y0, Show_Image_Gray[0], MT9V03X_W, MT9V03X_H, camera_w, camera_h, 0);
    Image_Draw_Camera_Detect_View(x0, y0);
    Image_Draw_Frame(x0, y0, camera_w, camera_h, RGB565_WHITE);

    Image_Draw_Reference_View(x0, line_y, line_scale);
    Image_Draw_Frame(x0, line_y, line_w, line_h, RGB565_WHITE);

    ips_show_string(info_x, line_y + 16, "R");  ips_show_int(info_x + 8 * 3, line_y + 16, Road_State, 3);
    ips_show_string(info_x, line_y + 32, "E");  ips_show_int(info_x + 8 * 3, line_y + 32, (int32)final_error, 4);
    ips_show_string(info_x, line_y + 48, "Th"); ips_show_int(info_x + 8 * 3, line_y + 48, Image_Runtime.threshold, 3);
    ips_show_string(info_x, line_y + 64, "Ex"); ips_show_int(info_x + 8 * 3, line_y + 64, Image_Runtime.exposure, 4);
    ips_show_string(info_x, line_y + 80, "F");  ips_show_int(info_x + 8 * 3, line_y + 80, Image_Runtime.target_found, 3);
    ips_show_string(info_x, line_y + 96, "B");  ips_show_int(info_x + 8 * 3, line_y + 96, Image_Runtime.bridge_candidate, 3);
    ips_show_string(info_x, line_y + 112, "D"); ips_show_int(info_x + 8 * 3, line_y + 112, Image_Runtime.bumpy_candidate, 3);
    ips_show_string(info_x, line_y + 128, "T"); ips_show_int(info_x + 8 * 3, line_y + 128, Image_Runtime.step_candidate, 3);
    ips_show_string(info_x, line_y + 144, "O"); ips_show_int(info_x + 8 * 3, line_y + 144, Image_Runtime.obstacle_candidate, 3);
    ips_show_string(info_x, line_y + 160, "S"); ips_show_int(info_x + 8 * 3, line_y + 160, Image_Runtime.bumpy_strip_count, 3);
    ips_show_string(info_x, line_y + 176, "W"); ips_show_int(info_x + 8 * 3, line_y + 176, Image_Runtime.road_width_max, 3);
}
void Image_Show_State(void)
{
    ips_show_string(0, 16 * 0, "CAM");
    ips_show_string(0, 16 * 1, "En");    ips_show_int(8 * 8, 16 * 1, Image_Runtime.enable, 3);
    ips_show_string(0, 16 * 2, "Road");  ips_show_int(8 * 8, 16 * 2, Road_State, 3);
    ips_show_string(0, 16 * 3, "Err");   ips_show_float(8 * 8, 16 * 3, final_error, 5, 1);
    ips_show_string(0, 16 * 4, "Gray");  ips_show_int(8 * 8, 16 * 4, Image_Runtime.avg_gray, 4);
    ips_show_string(0, 16 * 5, "Th");    ips_show_int(8 * 8, 16 * 5, Image_Runtime.threshold, 4);
    ips_show_string(0, 16 * 6, "Exp");   ips_show_int(8 * 8, 16 * 6, Image_Runtime.exposure, 4);
    ips_show_string(0, 16 * 7, "Dist");  ips_show_float(8 * 8, 16 * 7, Image_Runtime.obstacle_distance_mm, 5, 1);
    ips_show_string(0, 16 * 8, "D");     ips_show_int(8 * 8, 16 * 8, Image_Runtime.bumpy_candidate, 3);
    ips_show_string(0, 16 * 9, "Strip"); ips_show_int(8 * 8, 16 * 9, Image_Runtime.bumpy_strip_count, 3);
    ips_show_string(0, 16 * 10, "Span"); ips_show_int(8 * 8, 16 * 10, Image_Runtime.bumpy_span, 3);
    ips_show_string(0, 16 * 11, "Width"); ips_show_int(8 * 8, 16 * 11, Image_Runtime.road_width_max, 3);
    ips_show_string(0, 16 * 12, "T");     ips_show_int(8 * 8, 16 * 12, Image_Runtime.step_candidate, 3);
}

void Image_Task(void)
{
    if (!Image_Runtime.enable)
        return;

    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        Image_Runtime.frame_ready = 1;
        final_error = Image_Deal(30, 0.0f);

        if (Road_State == ROAD_OBSTACLE)
        {
            jump_vision_update(Image_Runtime.obstacle_distance_mm);
        }
    }
}
