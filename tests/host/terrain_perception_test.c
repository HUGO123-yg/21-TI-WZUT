#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Cone_vision.h"
#include "Minefield_vision.h"
#include "Mission_perception.h"
#include "Perception_fusion.h"
#include "Terrain_vision.h"
#include "Vision_frame.h"
#include "Vision_pipeline.h"
#include "zf_device_mt9v03x.h"

volatile uint8 mt9v03x_finish_flag;
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];

uint8 mt9v03x_set_exposure_time(uint16 light)
{
    (void)light;
    return 0U;
}

uint8 mt9v03x_init(void)
{
    return 0U;
}

static void test_shared_camera_frontend(void)
{
    const vision_frame_t *frame;
    terrain_vision_result_t terrain;
    cone_vision_result_t cone;
    minefield_vision_result_t minefield;
    const vision_pipeline_state_t *pipeline;
    mission_perception_context_t context;
    navigation_state_t navigation;
    const mission_perception_state_t *mission;

    memset(mt9v03x_image, 123, sizeof(mt9v03x_image));
    assert(VISION_FRAME_STATUS_OK == vision_frame_init());
    mt9v03x_finish_flag = 1U;
    assert(VISION_FRAME_STATUS_OK == vision_frame_task());

    frame = vision_frame_get_latest();
    assert(frame->enabled);
    assert(1U == frame->frame_count);
    assert(123U == frame->average_gray);
    assert(123U == frame->gray[0][0]);
    assert(123U == frame->gray[VISION_FRAME_HEIGHT - 1U]
                                   [VISION_FRAME_WIDTH - 1U]);

    // Terrain_vision must reuse the already initialized front-end and consume
    // exactly one immutable publication rather than reading the camera buffer.
    assert(TERRAIN_VISION_STATUS_OK == terrain_vision_init());
    terrain_vision_task();
    assert(terrain_vision_get_snapshot(&terrain));
    assert(1U == terrain.frame_count);
    assert(123U == terrain.average_gray);
    assert(cone_vision_get_snapshot(&cone));
    assert(minefield_vision_get_snapshot(&minefield));
    assert(1U == cone.frame_count);
    assert(1U == minefield.frame_count);
    pipeline = vision_pipeline_get_state();
    assert(VISION_PIPELINE_STATUS_OK == pipeline->status);
    assert(1U == pipeline->processed_frame_count);
    assert(pipeline->terrain_enabled);
    assert(pipeline->cone_enabled);
    assert(pipeline->minefield_enabled);
    terrain_vision_task();
    assert(terrain_vision_get_snapshot(&terrain));
    assert(1U == terrain.frame_count);
    assert(1U == vision_pipeline_get_state()->processed_frame_count);

    memset(mt9v03x_image, 77, sizeof(mt9v03x_image));
    mt9v03x_finish_flag = 1U;
    terrain_vision_task();
    assert(terrain_vision_get_snapshot(&terrain));
    assert(cone_vision_get_snapshot(&cone));
    assert(minefield_vision_get_snapshot(&minefield));
    assert(2U == terrain.frame_count);
    assert(2U == cone.frame_count);
    assert(2U == minefield.frame_count);
    assert(77U == terrain.average_gray);
    assert(2U == vision_pipeline_get_state()->processed_frame_count);

    memset(&context, 0, sizeof(context));
    memset(&navigation, 0, sizeof(navigation));
    context.task = MISSION_PERCEPTION_TASK_CONE_SLALOM;
    context.distance_source = MISSION_PERCEPTION_DISTANCE_TRAVELED;
    context.navigation_window_start_m = 1.0f;
    context.navigation_window_end_m = 3.0f;
    navigation.traveled_distance_m = 2.0f;
    assert(MISSION_PERCEPTION_STATUS_IDLE == mission_perception_init());
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_start(&context));
    assert(MISSION_PERCEPTION_STATUS_UNCALIBRATED
           == mission_perception_service(&navigation, 10U));
    mission = mission_perception_get_state();
    assert(2U == mission->last_visual_frame_count);
    assert(mission->visual_fresh);
    assert(!mission->advisory_valid);
}

static void draw_cone(vision_frame_t *frame,
                      uint8 center_x,
                      uint8 top_y,
                      uint8 bottom_y,
                      uint8 gray)
{
    uint8 y;

    for (y = top_y; y <= bottom_y; y++)
    {
        uint8 half_width = (uint8)(1U
            + (uint16)(y - top_y) * 5U / (bottom_y - top_y));
        uint8 x;

        for (x = center_x - half_width; x <= center_x + half_width; x++)
        {
            frame->gray[y][x] = gray;
        }
    }
}

static void update_frame_average(vision_frame_t *frame)
{
    uint32 sum = 0U;
    uint8 x;
    uint8 y;

    for (y = 0U; y < VISION_FRAME_HEIGHT; y++)
    {
        for (x = 0U; x < VISION_FRAME_WIDTH; x++)
        {
            sum += frame->gray[y][x];
        }
    }
    frame->average_gray = (uint16)(sum
        / ((uint32)VISION_FRAME_WIDTH * VISION_FRAME_HEIGHT));
}

static void test_cone_observation(void)
{
    vision_frame_t frame;
    cone_vision_result_t result;

    memset(&frame, 0, sizeof(frame));
    memset(frame.gray, 120, sizeof(frame.gray));
    frame.status = VISION_FRAME_STATUS_OK;
    frame.enabled = 1U;
    frame.frame_count = 10U;
    draw_cone(&frame, 24U, 20U, 50U, 30U);
    draw_cone(&frame, 55U, 20U, 50U, 30U);
    update_frame_average(&frame);

    assert(CONE_VISION_STATUS_WAITING_FOR_FRAME == cone_vision_init());
    assert(CONE_VISION_STATUS_OK == cone_vision_process_frame(&frame));
    assert(cone_vision_get_snapshot(&result));
    assert(!result.calibrated);
    assert(2U == result.candidate_count);
    assert(result.gap_valid);
    assert(result.gap_quality >= 50U);
    assert(fabsf(result.gap_center_error_norm) < 0.08f);
    assert(result.candidate[0].bottom_width_px
           >= result.candidate[0].top_width_px);
    assert(CONE_VISION_POLARITY_DARK == result.candidate[0].polarity);

    memset(frame.gray, 120, sizeof(frame.gray));
    frame.frame_count++;
    draw_cone(&frame, 39U, 20U, 50U, 30U);
    draw_cone(&frame, 68U, 20U, 50U, 30U);
    update_frame_average(&frame);
    assert(CONE_VISION_STATUS_OK == cone_vision_process_frame(&frame));
    assert(cone_vision_get_snapshot(&result));
    assert(result.gap_valid);
    assert(result.gap_center_error_norm > 0.20f);

    memset(frame.gray, 70, sizeof(frame.gray));
    frame.frame_count++;
    draw_cone(&frame, 24U, 20U, 50U, 230U);
    draw_cone(&frame, 55U, 20U, 50U, 230U);
    update_frame_average(&frame);
    assert(CONE_VISION_STATUS_OK == cone_vision_process_frame(&frame));
    assert(cone_vision_get_snapshot(&result));
    assert(2U == result.candidate_count);
    assert(CONE_VISION_POLARITY_BRIGHT == result.candidate[0].polarity);
}

static void draw_minefield_frame(vision_frame_t *frame,
                                 uint8 top_left,
                                 uint8 top_right,
                                 uint8 bottom_left,
                                 uint8 bottom_right)
{
    const uint8 top_y = 15U;
    const uint8 bottom_y = 50U;
    uint8 y;

    for (y = top_y; y <= bottom_y; y++)
    {
        uint8 left = (uint8)(top_left
            + (int16)(bottom_left - top_left) * (y - top_y)
              / (bottom_y - top_y));
        uint8 right = (uint8)(top_right
            + (int16)(bottom_right - top_right) * (y - top_y)
              / (bottom_y - top_y));

        frame->gray[y][left] = 235U;
        frame->gray[y][left + 1U] = 235U;
        frame->gray[y][right - 1U] = 235U;
        frame->gray[y][right] = 235U;
        if ((y <= top_y + 1U) || (y >= bottom_y - 1U))
        {
            uint8 x;

            for (x = left; x <= right; x++)
            {
                frame->gray[y][x] = 235U;
            }
        }
    }
}

static void test_minefield_observation(void)
{
    vision_frame_t frame;
    minefield_vision_result_t result;
    uint8 x;

    memset(&frame, 0, sizeof(frame));
    memset(frame.gray, 70, sizeof(frame.gray));
    frame.status = VISION_FRAME_STATUS_OK;
    frame.enabled = 1U;
    frame.frame_count = 20U;
    draw_minefield_frame(&frame, 25U, 54U, 11U, 68U);
    update_frame_average(&frame);

    assert(MINEFIELD_VISION_STATUS_WAITING_FOR_FRAME
           == minefield_vision_init());
    assert(MINEFIELD_VISION_STATUS_OK
           == minefield_vision_process_frame(&frame));
    assert(minefield_vision_get_snapshot(&result));
    assert(!result.calibrated);
    assert(result.frame_candidate);
    assert(result.center_valid);
    assert(result.boundary_visible);
    assert(result.boundary_warning);
    assert(result.horizontal_band_count >= 2U);
    assert(result.confidence >= 80U);
    assert(fabsf(result.center_error_norm) < 0.08f);

    memset(frame.gray, 70, sizeof(frame.gray));
    frame.frame_count++;
    draw_minefield_frame(&frame, 33U, 61U, 21U, 75U);
    update_frame_average(&frame);
    assert(MINEFIELD_VISION_STATUS_OK
           == minefield_vision_process_frame(&frame));
    assert(minefield_vision_get_snapshot(&result));
    assert(result.frame_candidate);
    assert(result.center_error_norm > 0.10f);

    // An isolated white line is intentionally not accepted as a minefield
    // enclosure, matching the official warning about unrelated white lines.
    memset(frame.gray, 70, sizeof(frame.gray));
    frame.frame_count++;
    for (x = 10U; x <= 69U; x++)
    {
        frame.gray[50U][x] = 235U;
        frame.gray[51U][x] = 235U;
    }
    update_frame_average(&frame);
    assert(MINEFIELD_VISION_STATUS_OK
           == minefield_vision_process_frame(&frame));
    assert(minefield_vision_get_snapshot(&result));
    assert(!result.frame_candidate);
    assert(!result.center_valid);
    assert(result.boundary_visible);
    assert(result.boundary_warning);
}

static float wrap_test_yaw(float yaw_rad)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;

    while (yaw_rad > pi)
    {
        yaw_rad -= two_pi;
    }
    while (yaw_rad <= -pi)
    {
        yaw_rad += two_pi;
    }
    return yaw_rad;
}

static void test_mission_cone_context_and_timeout(void)
{
    mission_perception_context_t context;
    cone_vision_result_t cone;
    minefield_vision_result_t minefield;
    perception_fusion_state_t fusion;
    navigation_state_t navigation;
    const mission_perception_state_t *mission;

    memset(&context, 0, sizeof(context));
    memset(&cone, 0, sizeof(cone));
    memset(&minefield, 0, sizeof(minefield));
    memset(&fusion, 0, sizeof(fusion));
    memset(&navigation, 0, sizeof(navigation));
    context.task = MISSION_PERCEPTION_TASK_CONE_SLALOM;
    context.distance_source = MISSION_PERCEPTION_DISTANCE_TRAVELED;
    context.navigation_window_start_m = 1.0f;
    context.navigation_window_end_m = 3.0f;
    cone.status = CONE_VISION_STATUS_OK;
    cone.enabled = 1U;
    cone.frame_count = 1U;
    cone.gap_valid = 1U;
    cone.gap_quality = 90U;
    cone.gap_center_error_norm = 0.25f;
    navigation.target_yaw_rate_rad_s = 0.10f;

    assert(MISSION_PERCEPTION_STATUS_IDLE == mission_perception_init());
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_start(&context));
    assert(MISSION_PERCEPTION_STATUS_WAITING_FOR_NAVIGATION
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        0U));
    navigation.traveled_distance_m = 2.0f;
    assert(MISSION_PERCEPTION_STATUS_UNCALIBRATED
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        5U));
    mission = mission_perception_get_state();
    assert(!mission->advisory_valid);

    cone.calibrated = 1U;
    cone.frame_count++;
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        10U));
    mission = mission_perception_get_state();
    assert(mission->navigation_window_active);
    assert(mission->visual_fresh);
    assert(mission->advisory_valid);
    assert(mission->visual_yaw_rate_correction_rad_s > 0.0f);
    assert(mission->suggested_yaw_rate_rad_s
           > navigation.target_yaw_rate_rad_s);

    assert(MISSION_PERCEPTION_STATUS_VISION_STALE
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        300U));
    mission = mission_perception_get_state();
    assert(!mission->advisory_valid);
    assert(fabsf(mission->suggested_yaw_rate_rad_s
                 - navigation.target_yaw_rate_rad_s) < 0.0001f);

    mission_perception_stop();
    assert(MISSION_PERCEPTION_STATUS_IDLE
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        305U));
}

static void test_mission_minefield_rotation_progress(void)
{
    mission_perception_context_t context;
    cone_vision_result_t cone;
    minefield_vision_result_t minefield;
    perception_fusion_state_t fusion;
    navigation_state_t navigation;
    const mission_perception_state_t *mission;
    float yaw = 3.0f;
    float progress_before_jump;
    uint16 step;

    memset(&context, 0, sizeof(context));
    memset(&cone, 0, sizeof(cone));
    memset(&minefield, 0, sizeof(minefield));
    memset(&fusion, 0, sizeof(fusion));
    memset(&navigation, 0, sizeof(navigation));
    context.task = MISSION_PERCEPTION_TASK_MINEFIELD;
    context.distance_source = MISSION_PERCEPTION_DISTANCE_TRAVELED;
    context.rotation_direction = MISSION_PERCEPTION_ROTATION_CCW;
    context.navigation_window_start_m = 1.0f;
    context.navigation_window_end_m = 3.0f;
    context.mine_target_turns = 2.0f;
    minefield.status = MINEFIELD_VISION_STATUS_OK;
    minefield.enabled = 1U;
    minefield.calibrated = 1U;
    minefield.frame_count = 1U;
    minefield.frame_candidate = 1U;
    minefield.center_valid = 1U;
    minefield.boundary_visible = 1U;
    minefield.boundary_warning = 1U;
    minefield.confidence = 90U;
    minefield.center_error_norm = -0.10f;
    minefield.boundary_proximity_norm = 0.85f;
    navigation.traveled_distance_m = 2.0f;
    navigation.yaw_rad = yaw;

    assert(MISSION_PERCEPTION_STATUS_IDLE == mission_perception_init());
    context.mine_target_turns = 1.9f;
    assert(MISSION_PERCEPTION_STATUS_INVALID_ARGUMENT
           == mission_perception_start(&context));
    context.mine_target_turns = 2.0f;
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_start(&context));
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        0U));

    for (step = 1U; step <= 130U; step++)
    {
        yaw = wrap_test_yaw(yaw + 0.10f);
        navigation.yaw_rad = yaw;
        minefield.frame_count++;
        assert(MISSION_PERCEPTION_STATUS_OK
               == mission_perception_update(&cone,
                                            &minefield,
                                            &fusion,
                                            &navigation,
                                            (uint32)step * 5U));
    }
    mission = mission_perception_get_state();
    assert(mission->mine_rotation_progress_valid);
    assert(mission->mine_rotation_complete);
    assert(mission->mine_rotation_progress_turns >= 2.0f);
    assert(mission->mine_boundary_guard_valid);
    assert(mission->mine_boundary_warning);
    assert(mission->mine_center_valid);

    progress_before_jump = mission->mine_rotation_progress_rad;
    navigation.yaw_rad = wrap_test_yaw(navigation.yaw_rad + 0.50f);
    minefield.frame_count++;
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        660U));
    mission = mission_perception_get_state();
    assert(1U == mission->rotation_discontinuity_count);
    assert(fabsf(mission->mine_rotation_progress_rad
                 - progress_before_jump) < 0.0001f);

    minefield.calibrated = 0U;
    minefield.frame_count++;
    assert(MISSION_PERCEPTION_STATUS_UNCALIBRATED
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        665U));
    mission = mission_perception_get_state();
    assert(!mission->mine_boundary_guard_valid);
    assert(mission->mine_rotation_complete);
}

static void test_mission_terrain_calibration_gate(void)
{
    mission_perception_context_t context;
    cone_vision_result_t cone;
    minefield_vision_result_t minefield;
    perception_fusion_state_t fusion;
    navigation_state_t navigation;
    const mission_perception_state_t *mission;

    memset(&context, 0, sizeof(context));
    memset(&cone, 0, sizeof(cone));
    memset(&minefield, 0, sizeof(minefield));
    memset(&fusion, 0, sizeof(fusion));
    memset(&navigation, 0, sizeof(navigation));
    context.task = MISSION_PERCEPTION_TASK_TERRAIN;
    context.distance_source = MISSION_PERCEPTION_DISTANCE_TRAVELED;
    context.navigation_window_start_m = 1.0f;
    context.navigation_window_end_m = 3.0f;
    navigation.traveled_distance_m = 2.0f;
    fusion.last_vision_frame_count = 3U;
    fusion.accepted_vision_count = 3U;
    fusion.vision_fresh = 1U;
    fusion.path_valid = 1U;
    fusion.guidance_valid = 1U;
    fusion.fused_confidence = 90U;
    fusion.suggested_yaw_rate_rad_s = 0.4f;

    assert(MISSION_PERCEPTION_STATUS_IDLE == mission_perception_init());
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_start(&context));
    assert(MISSION_PERCEPTION_STATUS_UNCALIBRATED
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        0U));
    mission = mission_perception_get_state();
    assert(!mission->advisory_valid);
    assert(!mission->visual_calibrated);
}

static void test_mission_route_lifecycle(void)
{
    cone_vision_result_t cone;
    minefield_vision_result_t minefield;
    perception_fusion_state_t fusion;
    navigation_state_t navigation;
    const mission_perception_state_t *mission;

    memset(&cone, 0, sizeof(cone));
    memset(&minefield, 0, sizeof(minefield));
    memset(&fusion, 0, sizeof(fusion));
    memset(&navigation, 0, sizeof(navigation));
    navigation.mode = NAVIGATION_MODE_REPLAYING;
    navigation.route_distance_m = 2.0f;
    minefield.status = MINEFIELD_VISION_STATUS_OK;
    minefield.enabled = 1U;
    minefield.calibrated = 1U;
    minefield.frame_count = 1U;
    minefield.frame_candidate = 1U;
    minefield.center_valid = 1U;
    minefield.boundary_visible = 1U;
    minefield.confidence = 90U;

    assert(MISSION_PERCEPTION_STATUS_IDLE == mission_perception_init());
    assert(MISSION_PERCEPTION_STATUS_INVALID_ARGUMENT
           == mission_perception_start_route(4U));
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_start_route(2U));
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        100U));
    mission = mission_perception_get_state();
    assert(2U == mission->route_id);
    assert(MISSION_PERCEPTION_TASK_MINEFIELD == mission->task);
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_configure_mine_rotation(
               MISSION_PERCEPTION_ROTATION_CW,
               2.5f));
    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        105U));
    mission = mission_perception_get_state();
    assert(MISSION_PERCEPTION_ROTATION_CW == mission->rotation_direction);
    assert(fabsf(mission->mine_target_turns - 2.5f) < 0.0001f);
    assert(fabsf(mission->mine_rotation_progress_turns) < 0.0001f);

    mission_perception_stop();
    assert(MISSION_PERCEPTION_STATUS_IDLE
           == mission_perception_update(&cone,
                                        &minefield,
                                        &fusion,
                                        &navigation,
                                        110U));
    mission = mission_perception_get_state();
    assert(0U == mission->route_id);
    assert(MISSION_PERCEPTION_TASK_NONE == mission->task);

    assert(MISSION_PERCEPTION_STATUS_OK
           == mission_perception_start_route(1U));
    assert(MISSION_PERCEPTION_STATUS_INVALID_ARGUMENT
           == mission_perception_configure_mine_rotation(
               MISSION_PERCEPTION_ROTATION_CCW,
               2.0f));
}

static void make_lane(uint8 *image, float near_center, float far_center)
{
    uint16 x;
    uint16 y;

    memset(image, 20, TERRAIN_VISION_IMAGE_WIDTH
                       * TERRAIN_VISION_IMAGE_HEIGHT);
    for (y = 0U; y < TERRAIN_VISION_IMAGE_HEIGHT; y++)
    {
        float progress = (float)y
                         / (float)(TERRAIN_VISION_IMAGE_HEIGHT - 1U);
        float center = far_center + progress * (near_center - far_center);
        float half_width = 8.0f + 0.43f * (float)y;
        int16 left = (int16)(center - half_width);
        int16 right = (int16)(center + half_width);

        if (left < 1)
        {
            left = 1;
        }
        if (right > (int16)TERRAIN_VISION_IMAGE_WIDTH - 2)
        {
            right = (int16)TERRAIN_VISION_IMAGE_WIDTH - 2;
        }
        for (x = (uint16)left; x <= (uint16)right; x++)
        {
            image[y * TERRAIN_VISION_IMAGE_WIDTH + x] = 220U;
        }
    }
}

static terrain_vision_result_t process_lane(float near_center,
                                            float far_center)
{
    uint8 image[TERRAIN_VISION_IMAGE_HEIGHT][TERRAIN_VISION_IMAGE_WIDTH];
    terrain_vision_result_t result;
    uint8 frame;

    make_lane(image[0], near_center, far_center);
    for (frame = 0U; frame < 5U; frame++)
    {
        assert(TERRAIN_VISION_STATUS_OK
               == terrain_vision_process_frame(image[0],
                                                TERRAIN_VISION_IMAGE_WIDTH,
                                                TERRAIN_VISION_IMAGE_HEIGHT));
    }
    assert(terrain_vision_get_snapshot(&result));
    return result;
}

static void make_normal_terrain(uint8 image[60][80])
{
    uint8 x;
    uint8 y;

    memset(image, 0, 60U * 80U);
    for (y = 0U; y < 60U; y++)
    {
        uint8 left = (uint8)(32U - (uint16)y * 24U / 59U);
        uint8 right = (uint8)(47U + (uint16)y * 24U / 59U);

        for (x = left + 1U; x < right; x++)
        {
            image[y][x] = 255U;
        }
    }
}

static void add_bumpy_strips(uint8 image[60][80])
{
    static const uint8 rows[3] = {30U, 36U, 42U};
    uint8 i;

    for (i = 0U; i < 3U; i++)
    {
        uint8 y = rows[i];
        uint8 left = (uint8)(32U - (uint16)y * 24U / 59U);
        uint8 right = (uint8)(47U + (uint16)y * 24U / 59U);
        uint8 x;

        for (x = left + 2U; x < right - 1U; x++)
        {
            image[y][x] = 0U;
        }
    }
}

static void add_step_band(uint8 image[60][80])
{
    uint8 x;
    uint8 y;

    for (y = 31U; y <= 39U; y++)
    {
        uint8 left = (uint8)(32U - (uint16)y * 24U / 59U);
        uint8 right = (uint8)(47U + (uint16)y * 24U / 59U);

        for (x = left + 2U; x < right - 1U; x++)
        {
            image[y][x] = 0U;
        }
    }
}

static void make_obstacle_shape(uint8 image[60][80])
{
    uint8 x;
    uint8 y;

    memset(image, 0, 60U * 80U);
    for (y = 0U; y < 60U; y++)
    {
        uint8 half = (uint8)(4U + (uint32)y * y * 28U
                            / (59U * 59U));
        uint8 left = 39U - half;
        uint8 right = 40U + half;

        for (x = left + 1U; x < right; x++)
        {
            image[y][x] = 255U;
        }
    }
}

static void make_left_bridge(uint8 image[60][80])
{
    uint8 x;
    uint8 y;

    make_normal_terrain(image);
    for (y = 25U; y <= 42U; y++)
    {
        uint8 right = (uint8)(47U + (uint16)y * 24U / 59U);

        for (x = 0U; x < right; x++)
        {
            image[y][x] = 255U;
        }
    }
}

static void run_terrain_case(uint8 frame[60][80],
                             terrain_type_t expected)
{
    terrain_vision_result_t result;
    uint8 i;

    assert(TERRAIN_VISION_STATUS_OK == terrain_vision_init());
    for (i = 0U; i < 10U; i++)
    {
        assert(TERRAIN_VISION_STATUS_OK
               == terrain_vision_process_frame(frame[0], 80U, 60U));
    }
    assert(terrain_vision_get_snapshot(&result));
    assert(expected == result.type);
}

static void test_terrain_regression(void)
{
    uint8 frame[60][80];

    make_normal_terrain(frame);
    run_terrain_case(frame, TERRAIN_TYPE_NORMAL);

    make_normal_terrain(frame);
    add_bumpy_strips(frame);
    run_terrain_case(frame, TERRAIN_TYPE_BUMPY);

    make_normal_terrain(frame);
    add_step_band(frame);
    run_terrain_case(frame, TERRAIN_TYPE_STEP);

    make_obstacle_shape(frame);
    run_terrain_case(frame, TERRAIN_TYPE_OBSTACLE);

    make_left_bridge(frame);
    run_terrain_case(frame, TERRAIN_TYPE_BRIDGE);

    memset(frame, 0, sizeof(frame));
    run_terrain_case(frame, TERRAIN_TYPE_LOST);
}

static void test_path_observation(void)
{
    terrain_vision_result_t centered;
    terrain_vision_result_t right_path;

    assert(TERRAIN_VISION_STATUS_OK == terrain_vision_init());
    centered = process_lane(39.5f, 39.5f);
    assert(centered.path_valid);
    assert(centered.path_quality >= 80U);
    assert(fabsf(centered.path_center_error_norm) < 0.08f);
    assert(fabsf(centered.path_heading_error_norm) < 0.08f);

    right_path = process_lane(46.0f, 52.0f);
    assert(right_path.path_valid);
    assert(right_path.path_center_error_norm > 0.08f);
    assert(right_path.path_heading_error_norm > 0.05f);
}

static void test_fusion_freshness_and_imu_support(void)
{
    terrain_vision_result_t vision;
    imu_data_t imu;
    navigation_state_t navigation;
    const perception_fusion_state_t *fusion;

    memset(&vision, 0, sizeof(vision));
    memset(&imu, 0, sizeof(imu));
    memset(&navigation, 0, sizeof(navigation));
    vision.status = TERRAIN_VISION_STATUS_OK;
    vision.enabled = 1U;
    vision.frame_count = 1U;
    vision.type = TERRAIN_TYPE_BRIDGE;
    vision.bridge_side = TERRAIN_BRIDGE_SIDE_LEFT;
    vision.confidence = 60U;
    vision.path_valid = 1U;
    vision.path_quality = 100U;
    vision.path_center_error_norm = 0.20f;
    vision.path_heading_error_norm = 0.10f;
    imu.acc_norm_g = 1.0f;
    imu.roll_deg = 3.0f;
    navigation.target_yaw_rate_rad_s = 0.10f;

    assert(PERCEPTION_FUSION_STATUS_WAITING_FOR_VISION
           == perception_fusion_init());
    assert(PERCEPTION_FUSION_STATUS_OK
           == perception_fusion_update(&vision, &imu, &navigation, 100U));
    fusion = perception_fusion_get_state();
    assert(fusion->vision_fresh);
    assert(fusion->guidance_valid);
    assert(fusion->terrain_imu_supported);
    assert(75U == fusion->fused_confidence);
    assert(fusion->suggested_yaw_rate_rad_s
           > navigation.target_yaw_rate_rad_s);

    assert(PERCEPTION_FUSION_STATUS_VISION_STALE
           == perception_fusion_update(&vision, &imu, &navigation, 400U));
    fusion = perception_fusion_get_state();
    assert(!fusion->vision_fresh);
    assert(!fusion->guidance_valid);
    assert(fabsf(fusion->suggested_yaw_rate_rad_s
                 - navigation.target_yaw_rate_rad_s) < 0.0001f);

    vision.frame_count++;
    vision.path_center_error_norm = -0.20f;
    vision.path_heading_error_norm = -0.10f;
    assert(PERCEPTION_FUSION_STATUS_OK
           == perception_fusion_update(&vision, &imu, &navigation, 405U));
    fusion = perception_fusion_get_state();
    assert(fusion->visual_yaw_rate_correction_rad_s < 0.0f);
}

int main(void)
{
    test_shared_camera_frontend();
    test_cone_observation();
    test_minefield_observation();
    test_mission_cone_context_and_timeout();
    test_mission_minefield_rotation_progress();
    test_mission_terrain_calibration_gate();
    test_mission_route_lifecycle();
    test_terrain_regression();
    test_path_observation();
    test_fusion_freshness_and_imu_support();
    puts("terrain_perception_test: PASS");
    return 0;
}
