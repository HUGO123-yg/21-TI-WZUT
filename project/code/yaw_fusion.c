#include "zf_common_headfile.h"
#include "yaw_fusion.h"

static float  s_offset       = 0.0f;
static bool   s_calibrated   = false;
static uint32 s_last_tick    = 0;

float yaw_fusion_gyro_bias = 0.0f;

static float norm_angle(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void yaw_fusion_init(void)
{
    s_offset              = 0.0f;
    s_calibrated          = false;
    s_last_tick           = 0;
    yaw_fusion_gyro_bias  = 0.0f;
}

bool yaw_fusion_is_calibrated(void)
{
    return s_calibrated;
}

void yaw_fusion_update(void)
{
    if (sys_times - s_last_tick < YAW_FUSION_UPDATE_MS) return;
    s_last_tick = sys_times;

    float imu_yaw     = roll_balance_cascade.posture_value.yaw;
    float gps_heading = 0.0f;
    float kp          = 0.0f;
    bool  valid       = false;

    if (gnss.antenna_direction_state == 1)
    {
        gps_heading = gnss.antenna_direction;
        kp = (gnss.satellite_used >= 12)
            ? YAW_FUSION_KP_ANTENNA_HIGH : YAW_FUSION_KP_ANTENNA;
        valid = true;
    }
    else if (gnss.state == 1 && gnss.speed > YAW_FUSION_MIN_SPEED)
    {
        gps_heading = gnss.direction;
        kp          = YAW_FUSION_KP_MOTION;
        valid       = true;
    }

    if (!valid)
    {
        roll_balance_cascade.posture_value.yaw = norm_angle(imu_yaw + s_offset);
        return;
    }

    float gps_norm = (gps_heading > 180.0f) ? (gps_heading - 360.0f) : gps_heading;
    float diff     = norm_angle(gps_norm - imu_yaw);

    if (!s_calibrated)
    {
        s_offset     = diff;
        s_calibrated = true;
    }
    else
    {
        s_offset = s_offset * (1.0f - kp) + diff * kp;
        // GPS航向误差 → 陀螺仪零偏慢速校准
        yaw_fusion_gyro_bias += diff * 0.05f;
    }

    roll_balance_cascade.posture_value.yaw = norm_angle(imu_yaw + s_offset);
}
