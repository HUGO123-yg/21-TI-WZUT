#include "Imu.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "zf_device_imu660rb.h"
#include "zf_driver_delay.h"

#define IMU_AXIS_COUNT                  (3U)
#define IMU_X                           (0U)
#define IMU_Y                           (1U)
#define IMU_Z                           (2U)
#define IMU_RAD_TO_DEG                  (57.29577951308232f)

typedef struct
{
    float angle;
    float bias;
    float covariance[2][2];
} imu_kalman_filter_t;

static imu_data_t imu_data;
static imu_kalman_filter_t roll_filter;
static imu_kalman_filter_t pitch_filter;
static float startup_gyro_bias_dps[IMU_AXIS_COUNT];
static uint8 imu_initialized;

static float imu_wrap_angle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static uint8 imu_config_is_valid(void)
{
    uint8 axis_mask;

    if ((0U == IMU_UPDATE_INTERVAL_TICKS)
        || (IMU_SCHEDULER_TICK_PERIOD_S <= 0.0f)
        || (IMU_UPDATE_PERIOD_S <= 0.0f)
        || (IMU_ACCEL_LSB_PER_G <= 0.0f)
        || (IMU_GYRO_LSB_PER_DPS <= 0.0f)
        || (IMU_STARTUP_CALIBRATION_SAMPLES == 0U)
        || (IMU_STARTUP_CALIBRATION_MIN_VALID == 0U)
        || (IMU_STARTUP_CALIBRATION_MIN_VALID
            > IMU_STARTUP_CALIBRATION_SAMPLES)
        || (IMU_STARTUP_CALIBRATION_MAX_GYRO_DPS <= 0.0f)
        || (IMU_STARTUP_CALIBRATION_MIN_G < 0.0f)
        || (IMU_STARTUP_CALIBRATION_MAX_G
            <= IMU_STARTUP_CALIBRATION_MIN_G)
        || (IMU_KALMAN_Q_ANGLE < 0.0f)
        || (IMU_KALMAN_Q_BIAS < 0.0f)
        || (IMU_KALMAN_R_MEASUREMENT <= 0.0f)
        || (IMU_KALMAN_INITIAL_VARIANCE < 0.0f)
        || (IMU_ACCEL_CORRECTION_MIN_G < 0.0f)
        || (IMU_ACCEL_CORRECTION_MAX_G <= IMU_ACCEL_CORRECTION_MIN_G)
        || (IMU_BODY_X_SOURCE_AXIS >= IMU_AXIS_COUNT)
        || (IMU_BODY_Y_SOURCE_AXIS >= IMU_AXIS_COUNT)
        || (IMU_BODY_Z_SOURCE_AXIS >= IMU_AXIS_COUNT))
    {
        return 0;
    }

    axis_mask = (uint8)((1U << IMU_BODY_X_SOURCE_AXIS)
                        | (1U << IMU_BODY_Y_SOURCE_AXIS)
                        | (1U << IMU_BODY_Z_SOURCE_AXIS));
    return (uint8)(axis_mask == 0x07U);
}

static void imu_kalman_init(imu_kalman_filter_t *filter, float initial_angle)
{
    memset(filter, 0, sizeof(*filter));
    filter->angle = initial_angle;
    filter->covariance[0][0] = IMU_KALMAN_INITIAL_VARIANCE;
    filter->covariance[1][1] = IMU_KALMAN_INITIAL_VARIANCE;
}

static float imu_kalman_update(imu_kalman_filter_t *filter,
                               float measured_angle,
                               float measured_rate,
                               uint8 measurement_valid)
{
    float rate;
    float covariance_00;
    float covariance_01;
    float innovation;
    float innovation_covariance;
    float gain_0;
    float gain_1;

    rate = measured_rate - filter->bias;
    filter->angle = imu_wrap_angle(filter->angle + IMU_UPDATE_PERIOD_S * rate);

    filter->covariance[0][0] += IMU_UPDATE_PERIOD_S
        * (IMU_UPDATE_PERIOD_S * filter->covariance[1][1]
           - filter->covariance[0][1]
           - filter->covariance[1][0]
           + IMU_KALMAN_Q_ANGLE);
    filter->covariance[0][1] -= IMU_UPDATE_PERIOD_S * filter->covariance[1][1];
    filter->covariance[1][0] -= IMU_UPDATE_PERIOD_S * filter->covariance[1][1];
    filter->covariance[1][1] += IMU_KALMAN_Q_BIAS * IMU_UPDATE_PERIOD_S;

    if (!measurement_valid)
    {
        return filter->angle;
    }

    innovation = imu_wrap_angle(measured_angle - filter->angle);
    innovation_covariance = filter->covariance[0][0]
                            + IMU_KALMAN_R_MEASUREMENT;
    gain_0 = filter->covariance[0][0] / innovation_covariance;
    gain_1 = filter->covariance[1][0] / innovation_covariance;

    filter->angle = imu_wrap_angle(filter->angle + gain_0 * innovation);
    filter->bias += gain_1 * innovation;

    covariance_00 = filter->covariance[0][0];
    covariance_01 = filter->covariance[0][1];
    filter->covariance[0][0] -= gain_0 * covariance_00;
    filter->covariance[0][1] -= gain_0 * covariance_01;
    filter->covariance[1][0] -= gain_1 * covariance_00;
    filter->covariance[1][1] -= gain_1 * covariance_01;

    return filter->angle;
}

static void imu_read_sample(void)
{
    int16 sensor_acc[IMU_AXIS_COUNT];
    int16 sensor_gyro[IMU_AXIS_COUNT];
    const uint8 source_axis[IMU_AXIS_COUNT] =
    {
        IMU_BODY_X_SOURCE_AXIS,
        IMU_BODY_Y_SOURCE_AXIS,
        IMU_BODY_Z_SOURCE_AXIS
    };
    const float direction[IMU_AXIS_COUNT] =
    {
        IMU_BODY_X_DIRECTION,
        IMU_BODY_Y_DIRECTION,
        IMU_BODY_Z_DIRECTION
    };
    const float acc_bias[IMU_AXIS_COUNT] =
    {
        IMU_ACCEL_BIAS_X_G,
        IMU_ACCEL_BIAS_Y_G,
        IMU_ACCEL_BIAS_Z_G
    };
    const float gyro_bias[IMU_AXIS_COUNT] =
    {
        IMU_GYRO_BIAS_X_DPS,
        IMU_GYRO_BIAS_Y_DPS,
        IMU_GYRO_BIAS_Z_DPS
    };
    uint8 axis;

    imu660rb_get_gyro();
    imu660rb_get_acc();

    sensor_acc[IMU_X] = imu660rb_acc_x;
    sensor_acc[IMU_Y] = imu660rb_acc_y;
    sensor_acc[IMU_Z] = imu660rb_acc_z;
    sensor_gyro[IMU_X] = imu660rb_gyro_x;
    sensor_gyro[IMU_Y] = imu660rb_gyro_y;
    sensor_gyro[IMU_Z] = imu660rb_gyro_z;

    for (axis = 0; axis < IMU_AXIS_COUNT; axis++)
    {
        imu_data.raw_acc[axis] = sensor_acc[source_axis[axis]];
        imu_data.raw_gyro[axis] = sensor_gyro[source_axis[axis]];
        imu_data.acc_g[axis] = direction[axis]
            * ((float)imu_data.raw_acc[axis] / IMU_ACCEL_LSB_PER_G)
            - acc_bias[axis];
        imu_data.gyro_dps[axis] = direction[axis]
            * ((float)imu_data.raw_gyro[axis] / IMU_GYRO_LSB_PER_DPS)
            - gyro_bias[axis]
            - startup_gyro_bias_dps[axis];
    }

    imu_data.acc_norm_g = sqrtf(imu_data.acc_g[IMU_X] * imu_data.acc_g[IMU_X]
                                + imu_data.acc_g[IMU_Y] * imu_data.acc_g[IMU_Y]
                                + imu_data.acc_g[IMU_Z] * imu_data.acc_g[IMU_Z]);
}

static uint8 imu_sample_is_stationary(void)
{
    uint8 axis;

    if ((imu_data.acc_norm_g < IMU_STARTUP_CALIBRATION_MIN_G)
        || (imu_data.acc_norm_g > IMU_STARTUP_CALIBRATION_MAX_G))
    {
        return 0U;
    }

    for (axis = 0U; axis < IMU_AXIS_COUNT; axis++)
    {
        if (fabsf(imu_data.gyro_dps[axis])
            > IMU_STARTUP_CALIBRATION_MAX_GYRO_DPS)
        {
            return 0U;
        }
    }
    return 1U;
}

static imu_status_t imu_calibrate_gyro(void)
{
    float bias_sum[IMU_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
    uint32 sample;
    uint32 valid_sample_count = 0U;
    uint8 axis;

    memset(startup_gyro_bias_dps, 0, sizeof(startup_gyro_bias_dps));
    if (!IMU_STARTUP_CALIBRATION_ENABLE)
    {
        imu_data.calibration_sample_count = 0U;
        return IMU_STATUS_OK;
    }

    for (sample = 0U; sample < IMU_STARTUP_CALIBRATION_SAMPLES; sample++)
    {
        imu_read_sample();
        if (imu_sample_is_stationary())
        {
            for (axis = 0U; axis < IMU_AXIS_COUNT; axis++)
            {
                bias_sum[axis] += imu_data.gyro_dps[axis];
            }
            valid_sample_count++;
        }
        if ((sample + 1U) < IMU_STARTUP_CALIBRATION_SAMPLES)
        {
            system_delay_ms(IMU_STARTUP_CALIBRATION_DELAY_MS);
        }
    }

    imu_data.calibration_sample_count = valid_sample_count;
    if (valid_sample_count < IMU_STARTUP_CALIBRATION_MIN_VALID)
    {
        return IMU_STATUS_CALIBRATION_ERROR;
    }

    for (axis = 0U; axis < IMU_AXIS_COUNT; axis++)
    {
        startup_gyro_bias_dps[axis]
            = bias_sum[axis] / (float)valid_sample_count;
        imu_data.startup_gyro_bias_dps[axis] = startup_gyro_bias_dps[axis];
    }
    imu_read_sample();
    return IMU_STATUS_OK;
}

static void imu_get_acc_angles(float *roll_deg, float *pitch_deg)
{
    float yz_norm;

    yz_norm = sqrtf(imu_data.acc_g[IMU_Y] * imu_data.acc_g[IMU_Y]
                    + imu_data.acc_g[IMU_Z] * imu_data.acc_g[IMU_Z]);
    *roll_deg = atan2f(imu_data.acc_g[IMU_Y], imu_data.acc_g[IMU_Z])
                * IMU_RAD_TO_DEG;
    *pitch_deg = atan2f(-imu_data.acc_g[IMU_X], yz_norm) * IMU_RAD_TO_DEG;
}

imu_status_t imu_init(void)
{
    imu_status_t status;

    imu_initialized = 0;
    memset(&imu_data, 0, sizeof(imu_data));
    memset(&roll_filter, 0, sizeof(roll_filter));
    memset(&pitch_filter, 0, sizeof(pitch_filter));
    memset(startup_gyro_bias_dps, 0, sizeof(startup_gyro_bias_dps));

    if (!imu_config_is_valid())
    {
        return IMU_STATUS_INVALID_CONFIG;
    }
    if (imu660rb_init())
    {
        return IMU_STATUS_DRIVER_ERROR;
    }

    status = imu_calibrate_gyro();
    if (IMU_STATUS_OK != status)
    {
        return status;
    }
    imu_initialized = 1;
    status = imu_reset_attitude(0.0f);
    if (IMU_STATUS_OK != status)
    {
        imu_initialized = 0;
    }
    return status;
}

imu_status_t imu_update(void)
{
    float acc_roll;
    float acc_pitch;

    if (!imu_initialized)
    {
        return IMU_STATUS_NOT_INITIALIZED;
    }

    imu_read_sample();
    imu_get_acc_angles(&acc_roll, &acc_pitch);
    imu_data.accel_correction_used = (uint8)
        ((imu_data.acc_norm_g >= IMU_ACCEL_CORRECTION_MIN_G)
         && (imu_data.acc_norm_g <= IMU_ACCEL_CORRECTION_MAX_G));

    imu_data.roll_deg = imu_kalman_update(&roll_filter,
                                          acc_roll,
                                          imu_data.gyro_dps[IMU_X],
                                          imu_data.accel_correction_used);
    imu_data.pitch_deg = imu_kalman_update(&pitch_filter,
                                           acc_pitch,
                                           imu_data.gyro_dps[IMU_Y],
                                           imu_data.accel_correction_used);
    imu_data.yaw_deg = imu_wrap_angle(imu_data.yaw_deg
                                      + imu_data.gyro_dps[IMU_Z]
                                      * IMU_UPDATE_PERIOD_S);
    imu_data.sample_count++;

    return IMU_STATUS_OK;
}

imu_status_t imu_reset_attitude(float yaw_deg)
{
    float acc_roll;
    float acc_pitch;

    if (!imu_initialized)
    {
        return IMU_STATUS_NOT_INITIALIZED;
    }

    imu_get_acc_angles(&acc_roll, &acc_pitch);
    imu_kalman_init(&roll_filter, acc_roll);
    imu_kalman_init(&pitch_filter, acc_pitch);
    imu_data.roll_deg = acc_roll;
    imu_data.pitch_deg = acc_pitch;
    imu_data.yaw_deg = imu_wrap_angle(yaw_deg);
    imu_data.sample_count = 0;
    imu_data.accel_correction_used = 1;

    return IMU_STATUS_OK;
}

const imu_data_t *imu_get_data(void)
{
    return &imu_data;
}

uint8 imu_is_initialized(void)
{
    return imu_initialized;
}
