#include "Navigation.h"

#include <math.h>
#include <string.h>

#include "Flash.h"
#include "config.h"

#define NAVIGATION_PI             (3.14159265358979323846f)
#define NAVIGATION_DEG_TO_RAD     (0.017453292519943295f)
#define NAVIGATION_RAD_TO_DEG     (57.29577951308232f)

static navigation_state_t navigation_state;
static float route_start_yaw_rad;
static uint8 navigation_initialized;

static float navigation_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float navigation_wrap_angle(float angle_rad)
{
    while (angle_rad > NAVIGATION_PI)
    {
        angle_rad -= 2.0f * NAVIGATION_PI;
    }
    while (angle_rad <= -NAVIGATION_PI)
    {
        angle_rad += 2.0f * NAVIGATION_PI;
    }
    return angle_rad;
}

static uint8 navigation_config_is_valid(void)
{
    if ((NAVIGATION_WHEEL_YAW_RATE_WEIGHT < 0.0f)
        || (NAVIGATION_WHEEL_YAW_RATE_WEIGHT > 1.0f)
        || (NAVIGATION_HEADING_KP < 0.0f)
        || (NAVIGATION_MAX_YAW_RATE_RAD_S <= 0.0f)
        || (NAV_FLASH_SAMPLE_DISTANCE_M <= 0.0f))
    {
        return 0U;
    }
#if NAVIGATION_USE_WHEEL_YAW_CORRECTION
    if (NAVIGATION_WHEEL_TRACK_WIDTH_M <= 0.0f)
    {
        return 0U;
    }
#endif
    return 1U;
}

static navigation_status_t navigation_storage_error(void)
{
    navigation_state.status = NAVIGATION_STATUS_STORAGE_ERROR;
    navigation_state.mode = NAVIGATION_MODE_ERROR;
    navigation_state.storage_error_count++;
    navigation_state.target_yaw_rate_rad_s = 0.0f;
    return navigation_state.status;
}

static navigation_status_t navigation_update_replay(void)
{
    float sample_position;
    float yaw_0_deg;
    float yaw_1_deg;
    float yaw_delta_deg;
    float fraction;
    float relative_yaw_rad;
    uint32 index_0;
    uint32 index_1;

    if (0U == navigation_state.route_sample_count)
    {
        return navigation_storage_error();
    }

    if (navigation_state.route_distance_m <= NAV_FLASH_SAMPLE_DISTANCE_M)
    {
        sample_position = 0.0f;
    }
    else
    {
        sample_position = navigation_state.route_distance_m
                          / NAV_FLASH_SAMPLE_DISTANCE_M - 1.0f;
    }

    index_0 = (uint32)sample_position;
    if (index_0 >= navigation_state.route_sample_count - 1U)
    {
        index_0 = navigation_state.route_sample_count - 1U;
        index_1 = index_0;
        fraction = 0.0f;
    }
    else
    {
        index_1 = index_0 + 1U;
        fraction = sample_position - (float)index_0;
    }

    if (NAV_FLASH_STATUS_OK != nav_flash_get_sample(index_0, &yaw_0_deg))
    {
        return navigation_storage_error();
    }
    yaw_1_deg = yaw_0_deg;
    if ((index_1 != index_0)
        && (NAV_FLASH_STATUS_OK
            != nav_flash_get_sample(index_1, &yaw_1_deg)))
    {
        return navigation_storage_error();
    }

    yaw_delta_deg = navigation_wrap_angle(
        (yaw_1_deg - yaw_0_deg) * NAVIGATION_DEG_TO_RAD)
        * NAVIGATION_RAD_TO_DEG;
    relative_yaw_rad = (yaw_0_deg + fraction * yaw_delta_deg)
                       * NAVIGATION_DEG_TO_RAD;
    navigation_state.route_heading_reference_rad = navigation_wrap_angle(
        route_start_yaw_rad + relative_yaw_rad);
    navigation_state.route_heading_error_rad = navigation_wrap_angle(
        navigation_state.route_heading_reference_rad - navigation_state.yaw_rad);
    navigation_state.target_yaw_rate_rad_s = navigation_clamp(
        NAVIGATION_HEADING_KP * navigation_state.route_heading_error_rad,
        -NAVIGATION_MAX_YAW_RATE_RAD_S,
        NAVIGATION_MAX_YAW_RATE_RAD_S);

    if (navigation_state.route_distance_m
        >= (float)navigation_state.route_sample_count
           * NAV_FLASH_SAMPLE_DISTANCE_M)
    {
        navigation_state.mode = NAVIGATION_MODE_REPLAY_COMPLETE;
        navigation_state.target_yaw_rate_rad_s = 0.0f;
    }
    return NAVIGATION_STATUS_OK;
}

navigation_status_t navigation_init(const imu_data_t *imu)
{
    memset(&navigation_state, 0, sizeof(navigation_state));
    route_start_yaw_rad = 0.0f;
    navigation_initialized = 0U;

    if (!navigation_config_is_valid())
    {
        navigation_state.status = NAVIGATION_STATUS_INVALID_CONFIG;
        navigation_state.mode = NAVIGATION_MODE_ERROR;
        return navigation_state.status;
    }
    if (0 == imu)
    {
        navigation_state.status = NAVIGATION_STATUS_INVALID_ARGUMENT;
        navigation_state.mode = NAVIGATION_MODE_ERROR;
        return navigation_state.status;
    }

    navigation_state.yaw_rad = imu->yaw_deg * NAVIGATION_DEG_TO_RAD;
    navigation_state.route_heading_reference_rad = navigation_state.yaw_rad;
    navigation_state.status = NAVIGATION_STATUS_OK;
    navigation_state.mode = NAVIGATION_MODE_IDLE;
    navigation_initialized = 1U;
    return NAVIGATION_STATUS_OK;
}

navigation_status_t navigation_reset_pose(float x_m,
                                          float y_m,
                                          float yaw_rad)
{
    if (!navigation_initialized)
    {
        return NAVIGATION_STATUS_NOT_INITIALIZED;
    }
    if ((x_m != x_m) || (y_m != y_m) || (yaw_rad != yaw_rad))
    {
        return NAVIGATION_STATUS_INVALID_ARGUMENT;
    }
    if ((NAVIGATION_MODE_RECORDING == navigation_state.mode)
        || (NAVIGATION_MODE_REPLAYING == navigation_state.mode))
    {
        return NAVIGATION_STATUS_INVALID_ARGUMENT;
    }

    navigation_state.x_m = x_m;
    navigation_state.y_m = y_m;
    navigation_state.yaw_rad = navigation_wrap_angle(yaw_rad);
    navigation_state.forward_speed_m_s = 0.0f;
    navigation_state.traveled_distance_m = 0.0f;
    navigation_state.route_distance_m = 0.0f;
    navigation_state.route_heading_reference_rad = navigation_state.yaw_rad;
    navigation_state.route_heading_error_rad = 0.0f;
    navigation_state.target_yaw_rate_rad_s = 0.0f;
    navigation_state.mode = NAVIGATION_MODE_IDLE;
    navigation_state.status = NAVIGATION_STATUS_OK;
    return NAVIGATION_STATUS_OK;
}

navigation_status_t navigation_update(const imu_data_t *imu,
                                      const wheel_feedback_t *wheel,
                                      uint8 wheel_feedback_valid,
                                      float dt_s)
{
    float wheel_circumference_m;
    float left_speed_m_s = 0.0f;
    float right_speed_m_s = 0.0f;
    float previous_yaw_rad;
    float forward_distance_delta_m = 0.0f;
    float route_yaw_deg;

    if (!navigation_initialized)
    {
        return NAVIGATION_STATUS_NOT_INITIALIZED;
    }
    if ((0 == imu) || (0 == wheel) || (dt_s <= 0.0f))
    {
        navigation_state.status = NAVIGATION_STATUS_INVALID_ARGUMENT;
        return navigation_state.status;
    }
    if (!NAVIGATION_ENABLE)
    {
        return NAVIGATION_STATUS_OK;
    }

    wheel_circumference_m = NAVIGATION_PI * WHEEL_DIAMETER_M;
    if (wheel_feedback_valid)
    {
        left_speed_m_s = (float)wheel->left_rpm
                         * wheel_circumference_m / 60.0f;
        right_speed_m_s = (float)wheel->right_rpm
                          * wheel_circumference_m / 60.0f;
        navigation_state.forward_speed_m_s
            = 0.5f * (left_speed_m_s + right_speed_m_s);
        navigation_state.wheel_feedback_used = 1U;
    }
    else
    {
        navigation_state.forward_speed_m_s = 0.0f;
        navigation_state.wheel_feedback_used = 0U;
    }

    navigation_state.imu_yaw_rate_rad_s
        = imu->attitude_rate_dps[2] * NAVIGATION_DEG_TO_RAD;
    navigation_state.gyro_bias_z_rad_s
        = imu->online_gyro_bias_dps[2] * NAVIGATION_DEG_TO_RAD;

    navigation_state.wheel_yaw_rate_rad_s = 0.0f;
    navigation_state.fused_yaw_rate_rad_s
        = navigation_state.imu_yaw_rate_rad_s;
#if NAVIGATION_USE_WHEEL_YAW_CORRECTION
    if (wheel_feedback_valid)
    {
        navigation_state.wheel_yaw_rate_rad_s
            = (right_speed_m_s - left_speed_m_s)
              / NAVIGATION_WHEEL_TRACK_WIDTH_M;
        navigation_state.fused_yaw_rate_rad_s
            = (1.0f - NAVIGATION_WHEEL_YAW_RATE_WEIGHT)
              * navigation_state.imu_yaw_rate_rad_s
              + NAVIGATION_WHEEL_YAW_RATE_WEIGHT
                * navigation_state.wheel_yaw_rate_rad_s;
    }
#endif

    previous_yaw_rad = navigation_state.yaw_rad;
    navigation_state.yaw_rad = navigation_wrap_angle(
        navigation_state.yaw_rad
        + navigation_state.fused_yaw_rate_rad_s * dt_s);
    navigation_state.x_m += navigation_state.forward_speed_m_s
        * cosf(previous_yaw_rad
               + 0.5f * navigation_state.fused_yaw_rate_rad_s * dt_s)
        * dt_s;
    navigation_state.y_m += navigation_state.forward_speed_m_s
        * sinf(previous_yaw_rad
               + 0.5f * navigation_state.fused_yaw_rate_rad_s * dt_s)
        * dt_s;
    navigation_state.traveled_distance_m +=
        fabsf(navigation_state.forward_speed_m_s) * dt_s;
    if (navigation_state.forward_speed_m_s > 0.0f)
    {
        forward_distance_delta_m
            = navigation_state.forward_speed_m_s * dt_s;
    }
    navigation_state.update_count++;

    if (NAVIGATION_MODE_RECORDING == navigation_state.mode)
    {
        route_yaw_deg = navigation_wrap_angle(
            navigation_state.yaw_rad - route_start_yaw_rad)
            * NAVIGATION_RAD_TO_DEG;
        if (NAV_FLASH_STATUS_OK
            != nav_flash_record_sample(route_yaw_deg,
                                       forward_distance_delta_m))
        {
            return navigation_storage_error();
        }
        navigation_state.route_distance_m += forward_distance_delta_m;
        navigation_state.route_sample_count = (uint32)
            (navigation_state.route_distance_m
             / NAV_FLASH_SAMPLE_DISTANCE_M);
    }
    else if (NAVIGATION_MODE_REPLAYING == navigation_state.mode)
    {
        navigation_state.route_distance_m += forward_distance_delta_m;
        if (NAVIGATION_STATUS_OK != navigation_update_replay())
        {
            return navigation_state.status;
        }
    }

    navigation_state.status = NAVIGATION_STATUS_OK;
    return NAVIGATION_STATUS_OK;
}

navigation_status_t navigation_start_recording(uint8 route_id)
{
    if (!navigation_initialized)
    {
        return NAVIGATION_STATUS_NOT_INITIALIZED;
    }
    if ((NAVIGATION_MODE_RECORDING == navigation_state.mode)
        || (NAVIGATION_MODE_REPLAYING == navigation_state.mode))
    {
        return NAVIGATION_STATUS_INVALID_ARGUMENT;
    }
    if (NAV_FLASH_STATUS_OK != nav_flash_record_start(route_id))
    {
        return navigation_storage_error();
    }

    route_start_yaw_rad = navigation_state.yaw_rad;
    navigation_state.route_id = route_id;
    navigation_state.route_distance_m = 0.0f;
    navigation_state.route_sample_count = 0U;
    navigation_state.route_heading_reference_rad = navigation_state.yaw_rad;
    navigation_state.route_heading_error_rad = 0.0f;
    navigation_state.target_yaw_rate_rad_s = 0.0f;
    navigation_state.status = NAVIGATION_STATUS_OK;
    navigation_state.mode = NAVIGATION_MODE_RECORDING;
    return NAVIGATION_STATUS_OK;
}

navigation_status_t navigation_stop_recording(void)
{
    if (!navigation_initialized)
    {
        return NAVIGATION_STATUS_NOT_INITIALIZED;
    }
    if (NAVIGATION_MODE_RECORDING != navigation_state.mode)
    {
        return NAVIGATION_STATUS_INVALID_ARGUMENT;
    }
    if (NAV_FLASH_STATUS_OK != nav_flash_record_stop())
    {
        return navigation_storage_error();
    }

    navigation_state.mode = NAVIGATION_MODE_IDLE;
    navigation_state.route_id = 0U;
    navigation_state.target_yaw_rate_rad_s = 0.0f;
    return NAVIGATION_STATUS_OK;
}

navigation_status_t navigation_abort_recording(void)
{
    if (!navigation_initialized)
    {
        return NAVIGATION_STATUS_NOT_INITIALIZED;
    }
    if (NAVIGATION_MODE_RECORDING != navigation_state.mode)
    {
        return NAVIGATION_STATUS_INVALID_ARGUMENT;
    }
    if (NAV_FLASH_STATUS_OK != nav_flash_record_abort())
    {
        return navigation_storage_error();
    }

    navigation_state.mode = NAVIGATION_MODE_IDLE;
    navigation_state.route_id = 0U;
    navigation_state.route_distance_m = 0.0f;
    navigation_state.route_sample_count = 0U;
    navigation_state.target_yaw_rate_rad_s = 0.0f;
    navigation_state.status = NAVIGATION_STATUS_OK;
    return NAVIGATION_STATUS_OK;
}

navigation_status_t navigation_start_replay(uint8 route_id)
{
    if (!navigation_initialized)
    {
        return NAVIGATION_STATUS_NOT_INITIALIZED;
    }
    if ((NAVIGATION_MODE_RECORDING == navigation_state.mode)
        || (NAVIGATION_MODE_REPLAYING == navigation_state.mode))
    {
        return NAVIGATION_STATUS_INVALID_ARGUMENT;
    }
    if (NAV_FLASH_STATUS_OK != nav_flash_load_route(route_id))
    {
        return navigation_storage_error();
    }

    navigation_state.route_sample_count = nav_flash_get_loaded_sample_count();
    if (0U == navigation_state.route_sample_count)
    {
        return navigation_storage_error();
    }
    route_start_yaw_rad = navigation_state.yaw_rad;
    navigation_state.route_id = route_id;
    navigation_state.route_distance_m = 0.0f;
    navigation_state.route_heading_reference_rad = navigation_state.yaw_rad;
    navigation_state.route_heading_error_rad = 0.0f;
    navigation_state.target_yaw_rate_rad_s = 0.0f;
    navigation_state.status = NAVIGATION_STATUS_OK;
    navigation_state.mode = NAVIGATION_MODE_REPLAYING;
    return NAVIGATION_STATUS_OK;
}

void navigation_stop_replay(void)
{
    if (navigation_initialized
        && ((NAVIGATION_MODE_REPLAYING == navigation_state.mode)
            || (NAVIGATION_MODE_REPLAY_COMPLETE == navigation_state.mode)))
    {
        navigation_state.mode = NAVIGATION_MODE_IDLE;
        navigation_state.route_id = 0U;
        navigation_state.route_distance_m = 0.0f;
        navigation_state.route_heading_error_rad = 0.0f;
        navigation_state.target_yaw_rate_rad_s = 0.0f;
    }
}

const navigation_state_t *navigation_get_state(void)
{
    return &navigation_state;
}

uint8 navigation_is_initialized(void)
{
    return navigation_initialized;
}
