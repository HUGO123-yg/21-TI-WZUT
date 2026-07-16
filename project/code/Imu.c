#include "Imu.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

#if (IMU_SENSOR_TYPE == IMU_SENSOR_TYPE_660RA)
#include "zf_device_imu660ra.h"
#define imu_driver_init          imu660ra_init
#define imu_driver_get_acc       imu660ra_get_acc
#define imu_driver_get_gyro      imu660ra_get_gyro
#define imu_driver_acc_x         imu660ra_acc_x
#define imu_driver_acc_y         imu660ra_acc_y
#define imu_driver_acc_z         imu660ra_acc_z
#define imu_driver_gyro_x        imu660ra_gyro_x
#define imu_driver_gyro_y        imu660ra_gyro_y
#define imu_driver_gyro_z        imu660ra_gyro_z
#elif (IMU_SENSOR_TYPE == IMU_SENSOR_TYPE_660RB)
#include "zf_device_imu660rb.h"
#define imu_driver_init          imu660rb_init
#define imu_driver_get_acc       imu660rb_get_acc
#define imu_driver_get_gyro      imu660rb_get_gyro
#define imu_driver_acc_x         imu660rb_acc_x
#define imu_driver_acc_y         imu660rb_acc_y
#define imu_driver_acc_z         imu660rb_acc_z
#define imu_driver_gyro_x        imu660rb_gyro_x
#define imu_driver_gyro_y        imu660rb_gyro_y
#define imu_driver_gyro_z        imu660rb_gyro_z
#endif

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
static float runtime_gyro_bias_z_dps;
static uint32 stationary_candidate_count;
static uint8 imu_initialized;

#if ((IMU_SENSOR_TYPE == IMU_SENSOR_TYPE_660RA) && IMU660RA_USE_SOFT_IIC)
#error "This project requires the SPI IMU660RA interface for ODR and range verification"
#elif ((IMU_SENSOR_TYPE == IMU_SENSOR_TYPE_660RB) && IMU660RB_USE_SOFT_IIC)
#error "This project requires the SPI IMU660RB interface for LSM6DSR identity and ODR verification"
#endif

#if (IMU_SENSOR_TYPE == IMU_SENSOR_TYPE_660RA)
static uint8 imu_ra_read_register(uint8 reg)
{
    uint8 data[2];

    IMU660RA_CS(0);
    spi_read_8bit_registers(
        IMU660RA_SPI,
        reg | IMU660RA_SPI_R,
        data,
        2U);
    IMU660RA_CS(1);
    return data[1];
}
#endif

static uint8 imu_driver_configure_and_verify(void)
{
#if (IMU_SENSOR_TYPE == IMU_SENSOR_TYPE_660RA)
    uint8 acc_conf;
    uint8 gyr_conf;
    uint8 acc_range;
    uint8 gyr_range;

    IMU660RA_CS(0);
    spi_write_8bit_register(
        IMU660RA_SPI,
        IMU660RA_ACC_CONF | IMU660RA_SPI_W,
        IMU660RA_ACC_CONF_200HZ);
    IMU660RA_CS(1);
    IMU660RA_CS(0);
    spi_write_8bit_register(
        IMU660RA_SPI,
        IMU660RA_GYR_CONF | IMU660RA_SPI_W,
        IMU660RA_GYR_CONF_200HZ);
    IMU660RA_CS(1);
    IMU660RA_CS(0);
    spi_write_8bit_register(
        IMU660RA_SPI,
        IMU660RA_ACC_RANGE | IMU660RA_SPI_W,
        IMU660RA_ACC_RANGE_8G);
    IMU660RA_CS(1);
    IMU660RA_CS(0);
    spi_write_8bit_register(
        IMU660RA_SPI,
        IMU660RA_GYR_RANGE | IMU660RA_SPI_W,
        IMU660RA_GYR_RANGE_2000DPS);
    IMU660RA_CS(1);
    system_delay_ms(2U);

    acc_conf = imu_ra_read_register(IMU660RA_ACC_CONF);
    gyr_conf = imu_ra_read_register(IMU660RA_GYR_CONF);
    acc_range = imu_ra_read_register(IMU660RA_ACC_RANGE);
    gyr_range = imu_ra_read_register(IMU660RA_GYR_RANGE);

    return (uint8)(
        (IMU660RA_ACC_CONF_200HZ == acc_conf)
        && (IMU660RA_GYR_CONF_200HZ == gyr_conf)
        && (IMU660RA_ACC_RANGE_8G == acc_range)
        && (IMU660RA_GYR_RANGE_2000DPS == gyr_range));
#else
    uint8 who_am_i;
    uint8 ctrl1_xl;
    uint8 ctrl2_g;

    IMU660RB_CS(0);
    who_am_i = spi_read_8bit_register(
        IMU660RB_SPI,
        IMU660RB_CHIP_ID | IMU660RB_SPI_R);
    IMU660RB_CS(1);
    if (IMU_LSM6DSR_EXPECTED_WHO_AM_I != who_am_i)
    {
        return 0U;
    }

    IMU660RB_CS(0);
    spi_write_8bit_register(
        IMU660RB_SPI,
        IMU660RB_CTRL1_XL | IMU660RB_SPI_W,
        IMU_LSM6DSR_CTRL1_XL_208HZ_8G);
    IMU660RB_CS(1);

    IMU660RB_CS(0);
    ctrl1_xl = spi_read_8bit_register(
        IMU660RB_SPI,
        IMU660RB_CTRL1_XL | IMU660RB_SPI_R);
    IMU660RB_CS(1);
    IMU660RB_CS(0);
    ctrl2_g = spi_read_8bit_register(
        IMU660RB_SPI,
        IMU660RB_CTRL2_G | IMU660RB_SPI_R);
    IMU660RB_CS(1);

    return (uint8)(
        (IMU_LSM6DSR_CTRL1_XL_208HZ_8G == ctrl1_xl)
        && (IMU_LSM6DSR_CTRL2_G_208HZ_2000DPS == ctrl2_g));
#endif
}

static float imu_clamp(float value, float minimum, float maximum)
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
        || (IMU_KALMAN_MAX_BIAS_DPS <= 0.0f)
        || (IMU_ACCEL_CORRECTION_MIN_G < 0.0f)
        || (IMU_ACCEL_CORRECTION_MAX_G <= IMU_ACCEL_CORRECTION_MIN_G)
        || (IMU_ACCEL_MAX_INNOVATION_DEG <= 0.0f)
        || (IMU_RUNTIME_BIAS_STATIONARY_SAMPLES == 0U)
        || (IMU_RUNTIME_BIAS_MAX_GYRO_DPS <= 0.0f)
        || (IMU_RUNTIME_BIAS_MAX_WHEEL_SPEED_M_S < 0.0f)
        || (IMU_RUNTIME_Z_BIAS_LEARNING_RATE < 0.0f)
        || (IMU_RUNTIME_Z_BIAS_LEARNING_RATE > 1.0f)
        || (IMU_RUNTIME_Z_BIAS_MAX_DPS <= 0.0f)
        || (IMU_BODY_X_SOURCE_AXIS >= IMU_AXIS_COUNT)
        || (IMU_BODY_Y_SOURCE_AXIS >= IMU_AXIS_COUNT)
        || (IMU_BODY_Z_SOURCE_AXIS >= IMU_AXIS_COUNT)
        || (fabsf(fabsf(IMU_BODY_X_DIRECTION) - 1.0f) > 0.0001f)
        || (fabsf(fabsf(IMU_BODY_Y_DIRECTION) - 1.0f) > 0.0001f)
        || (fabsf(fabsf(IMU_BODY_Z_DIRECTION) - 1.0f) > 0.0001f))
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
                               uint8 measurement_valid,
                               uint8 bias_learning_allowed,
                               uint8 *measurement_used)
{
    float rate;
    float covariance_00;
    float covariance_01;
    float covariance_10;
    float covariance_11;
    float innovation;
    float innovation_covariance;
    float gain_0;
    float gain_1;
    float one_minus_gain_0;

    if (0 != measurement_used)
    {
        *measurement_used = 0U;
    }

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
    if (fabsf(innovation) > IMU_ACCEL_MAX_INNOVATION_DEG)
    {
        return filter->angle;
    }
    innovation_covariance = filter->covariance[0][0]
                            + IMU_KALMAN_R_MEASUREMENT;
    gain_0 = filter->covariance[0][0] / innovation_covariance;
    gain_1 = bias_learning_allowed
        ? filter->covariance[1][0] / innovation_covariance
        : 0.0f;

    filter->angle = imu_wrap_angle(filter->angle + gain_0 * innovation);
    if (bias_learning_allowed)
    {
        filter->bias = imu_clamp(filter->bias + gain_1 * innovation,
                                 -IMU_KALMAN_MAX_BIAS_DPS,
                                 IMU_KALMAN_MAX_BIAS_DPS);
    }

    covariance_00 = filter->covariance[0][0];
    covariance_01 = filter->covariance[0][1];
    covariance_10 = filter->covariance[1][0];
    covariance_11 = filter->covariance[1][1];
    one_minus_gain_0 = 1.0f - gain_0;
    // Joseph-form covariance update remains symmetric and non-negative even
    // when the bias gain is deliberately frozen during vehicle motion.
    filter->covariance[0][0]
        = one_minus_gain_0 * one_minus_gain_0 * covariance_00
          + gain_0 * gain_0 * IMU_KALMAN_R_MEASUREMENT;
    filter->covariance[0][1]
        = one_minus_gain_0 * (covariance_01 - gain_1 * covariance_00)
          + gain_0 * gain_1 * IMU_KALMAN_R_MEASUREMENT;
    filter->covariance[1][0]
        = one_minus_gain_0 * (covariance_10 - gain_1 * covariance_00)
          + gain_0 * gain_1 * IMU_KALMAN_R_MEASUREMENT;
    filter->covariance[1][1]
        = covariance_11
          - gain_1 * (covariance_01 + covariance_10)
          + gain_1 * gain_1
            * (covariance_00 + IMU_KALMAN_R_MEASUREMENT);

    if (0 != measurement_used)
    {
        *measurement_used = 1U;
    }

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

    imu_driver_get_gyro();
    imu_driver_get_acc();

    sensor_acc[IMU_X] = imu_driver_acc_x;
    sensor_acc[IMU_Y] = imu_driver_acc_y;
    sensor_acc[IMU_Z] = imu_driver_acc_z;
    sensor_gyro[IMU_X] = imu_driver_gyro_x;
    sensor_gyro[IMU_Y] = imu_driver_gyro_y;
    sensor_gyro[IMU_Z] = imu_driver_gyro_z;

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

static uint8 imu_update_stationary_state(uint8 platform_stationary)
{
    const float online_bias_dps[IMU_AXIS_COUNT] =
    {
        roll_filter.bias,
        pitch_filter.bias,
        runtime_gyro_bias_z_dps
    };
    uint8 stationary_sample;
    uint8 axis;

    stationary_sample = (uint8)(platform_stationary
        && (imu_data.acc_norm_g >= IMU_STARTUP_CALIBRATION_MIN_G)
        && (imu_data.acc_norm_g <= IMU_STARTUP_CALIBRATION_MAX_G));
    for (axis = 0U; stationary_sample && (axis < IMU_AXIS_COUNT); axis++)
    {
        if (fabsf(imu_data.gyro_dps[axis] - online_bias_dps[axis])
            > IMU_RUNTIME_BIAS_MAX_GYRO_DPS)
        {
            stationary_sample = 0U;
        }
    }

    if (stationary_sample)
    {
        if (stationary_candidate_count
            < IMU_RUNTIME_BIAS_STATIONARY_SAMPLES)
        {
            stationary_candidate_count++;
        }
    }
    else
    {
        stationary_candidate_count = 0U;
    }

    imu_data.stationary_sample_count = stationary_candidate_count;
    imu_data.stationary_confirmed = (uint8)(stationary_candidate_count
        >= IMU_RUNTIME_BIAS_STATIONARY_SAMPLES);
    return imu_data.stationary_confirmed;
}

static void imu_update_runtime_z_bias(uint8 stationary_confirmed)
{
    imu_data.runtime_z_bias_learning = 0U;
    if (IMU_RUNTIME_Z_BIAS_ENABLE && stationary_confirmed)
    {
        runtime_gyro_bias_z_dps = imu_clamp(
            runtime_gyro_bias_z_dps
            + IMU_RUNTIME_Z_BIAS_LEARNING_RATE
              * (imu_data.gyro_dps[IMU_Z]
                 - runtime_gyro_bias_z_dps),
            -IMU_RUNTIME_Z_BIAS_MAX_DPS,
            IMU_RUNTIME_Z_BIAS_MAX_DPS);
        imu_data.runtime_z_bias_update_count++;
        imu_data.runtime_z_bias_learning = 1U;
    }
    imu_data.online_gyro_bias_dps[IMU_Z] = runtime_gyro_bias_z_dps;
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
    runtime_gyro_bias_z_dps = 0.0f;
    stationary_candidate_count = 0U;

    if (!imu_config_is_valid())
    {
        return IMU_STATUS_INVALID_CONFIG;
    }
    if (imu_driver_init())
    {
        return IMU_STATUS_DRIVER_ERROR;
    }
    if (!imu_driver_configure_and_verify())
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

imu_status_t imu_update(uint8 platform_stationary)
{
    float acc_roll;
    float acc_pitch;
    uint8 accel_measurement_valid;
    uint8 roll_measurement_used;
    uint8 pitch_measurement_used;
    uint8 bias_learning_allowed;

    if (!imu_initialized)
    {
        return IMU_STATUS_NOT_INITIALIZED;
    }

    imu_read_sample();
    imu_data.runtime_xy_bias_learning = 0U;
    bias_learning_allowed = imu_update_stationary_state(
        platform_stationary);
    imu_update_runtime_z_bias(bias_learning_allowed);
    imu_get_acc_angles(&acc_roll, &acc_pitch);
    accel_measurement_valid = (uint8)
        ((imu_data.acc_norm_g >= IMU_ACCEL_CORRECTION_MIN_G)
         && (imu_data.acc_norm_g <= IMU_ACCEL_CORRECTION_MAX_G));
    imu_data.roll_deg = imu_kalman_update(&roll_filter,
                                          acc_roll,
                                          imu_data.gyro_dps[IMU_X],
                                          accel_measurement_valid,
                                          bias_learning_allowed,
                                          &roll_measurement_used);
    imu_data.pitch_deg = imu_kalman_update(&pitch_filter,
                                           acc_pitch,
                                           imu_data.gyro_dps[IMU_Y],
                                           accel_measurement_valid,
                                           bias_learning_allowed,
                                           &pitch_measurement_used);
    imu_data.accel_correction_mask = (uint8)(
        (roll_measurement_used ? IMU_ACCEL_CORRECTION_ROLL : 0U)
        | (pitch_measurement_used ? IMU_ACCEL_CORRECTION_PITCH : 0U));
    imu_data.accel_correction_used = (uint8)(
        IMU_ACCEL_CORRECTION_NONE != imu_data.accel_correction_mask);
    if (accel_measurement_valid && !roll_measurement_used)
    {
        imu_data.accel_innovation_reject_count[IMU_X]++;
    }
    if (accel_measurement_valid && !pitch_measurement_used)
    {
        imu_data.accel_innovation_reject_count[IMU_Y]++;
    }
    if (bias_learning_allowed && imu_data.accel_correction_used)
    {
        imu_data.runtime_xy_bias_update_count++;
        imu_data.runtime_xy_bias_learning = 1U;
    }
    imu_data.online_gyro_bias_dps[IMU_X] = roll_filter.bias;
    imu_data.online_gyro_bias_dps[IMU_Y] = pitch_filter.bias;
    imu_data.attitude_rate_dps[IMU_X] = imu_data.gyro_dps[IMU_X]
                                          - roll_filter.bias;
    imu_data.attitude_rate_dps[IMU_Y] = imu_data.gyro_dps[IMU_Y]
                                          - pitch_filter.bias;
    imu_data.online_gyro_bias_dps[IMU_Z] = runtime_gyro_bias_z_dps;
    imu_data.attitude_rate_dps[IMU_Z] = imu_data.gyro_dps[IMU_Z]
                                          - runtime_gyro_bias_z_dps;
    imu_data.yaw_deg = imu_wrap_angle(imu_data.yaw_deg
                                      + imu_data.attitude_rate_dps[IMU_Z]
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
    imu_data.online_gyro_bias_dps[IMU_X] = 0.0f;
    imu_data.online_gyro_bias_dps[IMU_Y] = 0.0f;
    imu_data.online_gyro_bias_dps[IMU_Z] = runtime_gyro_bias_z_dps;
    imu_data.attitude_rate_dps[IMU_X] = imu_data.gyro_dps[IMU_X];
    imu_data.attitude_rate_dps[IMU_Y] = imu_data.gyro_dps[IMU_Y];
    imu_data.attitude_rate_dps[IMU_Z] = imu_data.gyro_dps[IMU_Z]
                                          - runtime_gyro_bias_z_dps;
    imu_data.sample_count = 0;
    imu_data.accel_correction_mask = (uint8)(
        IMU_ACCEL_CORRECTION_ROLL | IMU_ACCEL_CORRECTION_PITCH);
    imu_data.accel_correction_used = 1;
    imu_data.runtime_xy_bias_learning = 0U;
    imu_data.runtime_z_bias_learning = 0U;
    imu_data.stationary_sample_count = 0U;
    imu_data.stationary_confirmed = 0U;
    stationary_candidate_count = 0U;

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
