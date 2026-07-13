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

typedef struct
{
    int16 raw_acc[3];
    int16 raw_gyro[3];
    float acc_g[3];
    float gyro_dps[3];
    float acc_norm_g;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float startup_gyro_bias_dps[3];
    uint32 sample_count;
    uint32 calibration_sample_count;
    uint8 accel_correction_used;
} imu_data_t;

// Initialize the 660RB driver and seed roll/pitch from gravity.
imu_status_t imu_init(void);

// Read the driver and advance the estimator by IMU_UPDATE_PERIOD_S.
imu_status_t imu_update(void);

// Reseed roll/pitch from the latest acceleration and assign a yaw reference.
imu_status_t imu_reset_attitude(float yaw_deg);

// The returned storage is module-owned and updated by imu_update().
const imu_data_t *imu_get_data(void);
uint8 imu_is_initialized(void);

#endif
