#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// PIT_CH0 runs every 1 ms; five ticks produce a 200 Hz IMU update. The filter
// period is derived from these values so scheduler and estimator cannot drift.
#define IMU_SCHEDULER_TICK_PERIOD_S      (0.001f)
#define IMU_UPDATE_INTERVAL_TICKS        (5U)
#define IMU_UPDATE_PERIOD_S              \
    (IMU_SCHEDULER_TICK_PERIOD_S * (float)IMU_UPDATE_INTERVAL_TICKS)

// IMU660RB ranges selected by zf_device_imu660rb.h: +/-8 g and +/-2000 dps.
#define IMU_ACCEL_LSB_PER_G              (4098.0f)
#define IMU_GYRO_LSB_PER_DPS             (14.3f)

// Sensor-axis to vehicle-axis mapping. Axis indices are X=0, Y=1, Z=2.
#define IMU_BODY_X_SOURCE_AXIS           (0U)
#define IMU_BODY_Y_SOURCE_AXIS           (1U)
#define IMU_BODY_Z_SOURCE_AXIS           (2U)
#define IMU_BODY_X_DIRECTION             (1.0f)
#define IMU_BODY_Y_DIRECTION             (-1.0f)
#define IMU_BODY_Z_DIRECTION             (-1.0f)

// Additional software calibration in physical units. The 660RB driver already
// applies its fixed raw-count gyro offsets (-7, +6, +2).
#define IMU_ACCEL_BIAS_X_G               (0.0f)
#define IMU_ACCEL_BIAS_Y_G               (0.0f)
#define IMU_ACCEL_BIAS_Z_G               (0.0f)
#define IMU_GYRO_BIAS_X_DPS              (0.0f)
#define IMU_GYRO_BIAS_Y_DPS              (0.0f)
#define IMU_GYRO_BIAS_Z_DPS              (0.0f)

// Two-state Kalman filter parameters: angle and gyro bias.
#define IMU_KALMAN_Q_ANGLE               (0.001f)
#define IMU_KALMAN_Q_BIAS                (0.003f)
#define IMU_KALMAN_R_MEASUREMENT         (0.030f)
#define IMU_KALMAN_INITIAL_VARIANCE      (1.0f)

// Ignore accelerometer angle correction during strong dynamic acceleration.
#define IMU_ACCEL_CORRECTION_MIN_G       (0.80f)
#define IMU_ACCEL_CORRECTION_MAX_G       (1.20f)

#endif
