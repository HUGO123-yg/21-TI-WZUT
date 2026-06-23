#include "zf_common_headfile.h"
#include "yaw_fusion.h"
#include <math.h>

static float  s_offset       = 0.0f;
static bool   s_calibrated   = false;
static bool   s_bias_ready   = false;
static uint32 s_last_tick    = 0;
static float  s_bias_sum     = 0.0f;
static uint16 s_bias_count   = 0;

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
    s_bias_ready          = false;
    s_last_tick           = 0;
    s_bias_sum            = 0.0f;
    s_bias_count          = 0;
    yaw_fusion_gyro_bias  = 0.0f;
}

bool yaw_fusion_is_calibrated(void)
{
    return s_calibrated || s_bias_ready;
}

bool yaw_fusion_is_gyro_bias_ready(void)
{
    return s_bias_ready;
}

void yaw_fusion_calibrate_gyro_bias(int16 gyro_z_raw)
{
    float ax = (float)ACC_DATA_X / ACC_TRANSITION_FACTOR;
    float ay = (float)ACC_DATA_Y / ACC_TRANSITION_FACTOR;
    float az = (float)ACC_DATA_Z / ACC_TRANSITION_FACTOR;
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    bool speed_stopped = (func_abs(car_speed) <= (int16)YAW_FUSION_STATIC_SPEED_MAX)
                      && (func_abs((int16)target_speed) <= (int16)YAW_FUSION_STATIC_SPEED_MAX);
    bool imu_still = (func_abs(gyro_z_raw) <= YAW_FUSION_STATIC_GYRO_MAX)
                  && (acc_norm >= YAW_FUSION_STATIC_ACC_MIN)
                  && (acc_norm <= YAW_FUSION_STATIC_ACC_MAX);

    if (!speed_stopped || !imu_still)
    {
        if (!s_bias_ready)
        {
            s_bias_sum   = 0.0f;
            s_bias_count = 0;
        }
        return;
    }

    if (!s_bias_ready)
    {
        s_bias_sum += (float)gyro_z_raw;
        if (++s_bias_count >= YAW_FUSION_STATIC_CALIB_MS)
        {
            yaw_fusion_gyro_bias = func_limit_ab(s_bias_sum / (float)s_bias_count,
                                                 -YAW_FUSION_GYRO_BIAS_MAX,
                                                  YAW_FUSION_GYRO_BIAS_MAX);
            s_bias_ready = true;
        }
        return;
    }

    // 静止时缓慢跟踪温漂；运动时保持上一次可信零偏。
    yaw_fusion_gyro_bias = yaw_fusion_gyro_bias * 0.995f + (float)gyro_z_raw * 0.005f;
    yaw_fusion_gyro_bias = func_limit_ab(yaw_fusion_gyro_bias,
                                         -YAW_FUSION_GYRO_BIAS_MAX,
                                          YAW_FUSION_GYRO_BIAS_MAX);
}

void yaw_fusion_update(void)
{
    float imu_yaw     = roll_balance_cascade.posture_value.yaw;
    float gps_heading = 0.0f;
    float kp          = 0.0f;
    bool  valid       = false;

    if (sys_times - s_last_tick < YAW_FUSION_UPDATE_MS)
    {
        roll_balance_cascade.posture_value.yaw = norm_angle(imu_yaw + s_offset);
        return;
    }
    s_last_tick = sys_times;

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
        yaw_fusion_gyro_bias += func_limit_ab(diff * 0.01f, -0.5f, 0.5f);
        yaw_fusion_gyro_bias = func_limit_ab(yaw_fusion_gyro_bias,
                                             -YAW_FUSION_GYRO_BIAS_MAX,
                                              YAW_FUSION_GYRO_BIAS_MAX);
    }

    roll_balance_cascade.posture_value.yaw = norm_angle(imu_yaw + s_offset);
}
