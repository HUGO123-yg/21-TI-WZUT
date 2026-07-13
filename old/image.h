#ifndef __IMAGE_H
#define __IMAGE_H

#define DEAL_IMAGE_W     80
#define DEAL_IMAGE_H     60

typedef enum
{
    ROAD_NORMAL = 0,
    ROAD_STEP,
    ROAD_BRIDGE,
    ROAD_BUMPY,
    ROAD_OBSTACLE,
    ROAD_LOST,
} Road_State_e;

typedef struct
{
    uint8 enable;
    uint8 show_enable;
    uint8 frame_ready;
    uint8 stable_count;
    uint8 target_found;
    uint8 white_block_count;
    uint8 bridge_candidate;
    uint8 obstacle_candidate;
    uint8 step_candidate;
    uint8 bumpy_candidate;
    uint8 bumpy_strip_count;
    uint8 bumpy_span;
    uint8 road_width_max;
    uint8 road_width_near;
    uint8 road_width_far;
    uint8 road_valid_count;
    uint8 bridge_side;
    uint8 left_lost_flag;
    uint8 right_lost_flag;
    uint8 left_straight_flag;
    uint8 right_straight_flag;
    uint8 lower_left_corner;
    uint8 lower_right_corner;
    uint8 step_band_count;
    uint8 step_band_span;
    uint8 transverse_row_count;
    uint8 bridge_score;
    uint8 bumpy_score;
    uint8 obstacle_score;
    uint8 step_score;
    uint8 threshold;
    uint16 avg_gray;
    uint16 exposure;
    float obstacle_distance_mm;
} image_runtime_struct;

extern float final_error;
extern Road_State_e Road_State;
extern image_runtime_struct Image_Runtime;

void Image_Init(void);
void Image_Task(void);
void Image_Show_State(void);
void Image_Show_View(uint16 x, uint16 y);
void Image_Set_Show(uint8 enable);
void Image_Set_Outdoor_Mode(void);

#endif
