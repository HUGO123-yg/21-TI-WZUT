#ifndef PROJECT_IMU_H
#define PROJECT_IMU_H

#include "zf_common_typedef.h"

typedef enum
{
    IMU_STATUS_OK = 0,
    IMU_STATUS_NOT_INITIALIZED,
    IMU_STATUS_DRIVER_ERROR,
    IMU_STATUS_INVALID_CONFIG,
    IMU_STATUS_CALIBRATION_ERROR
} imu_status_t;

typedef enum
{
    IMU_ACCEL_CORRECTION_NONE = 0U,
    IMU_ACCEL_CORRECTION_ROLL = 1U << 0,
    IMU_ACCEL_CORRECTION_PITCH = 1U << 1
} imu_accel_correction_t;

typedef struct
{
    int16 raw_acc[3];
    int16 raw_gyro[3];
    float acc_g[3];
    // Axis-mapped gyro data after configured and startup bias removal.
    float gyro_dps[3];
    // Rates consumed by attitude control after online residual-bias removal.
    // X/Y use the Kalman bias; Z learns only while the platform is stationary.
    float attitude_rate_dps[3];
    float online_gyro_bias_dps[3];
    float acc_norm_g;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float startup_gyro_bias_dps[3];
    uint32 sample_count;
    uint32 calibration_sample_count;
    uint32 runtime_xy_bias_update_count;
    uint32 runtime_z_bias_update_count;
    uint32 accel_innovation_reject_count[2];
    uint32 stationary_sample_count;
    uint8 accel_correction_mask;
    uint8 accel_correction_used;
    uint8 runtime_xy_bias_learning;
    uint8 runtime_z_bias_learning;
    uint8 stationary_confirmed;
} imu_data_t;

// Initialize the 660RA/660RB module selected in config.h and seed roll/pitch
// from gravity. The project layer verifies the selected sensor's required
// output data rates and ranges after the Seekfree driver has initialized it.
imu_status_t imu_init(void);

// Read the driver and advance the estimator by IMU_UPDATE_PERIOD_S.
// platform_stationary must come from an independent source such as valid wheel
// feedback. IMU gates it with acceleration and all gyro axes before learning.
imu_status_t imu_update(uint8 platform_stationary);

// Reseed roll/pitch from the latest acceleration and assign a yaw reference.
imu_status_t imu_reset_attitude(float yaw_deg);

// The returned storage is module-owned and updated by imu_update.
const imu_data_t *imu_get_data(void);
uint8 imu_is_initialized(void);

#endif
