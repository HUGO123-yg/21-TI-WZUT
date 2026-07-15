#ifndef PROJECT_NAVIGATION_H
#define PROJECT_NAVIGATION_H

#include "Imu.h"
#include "Wheel_driver.h"
#include "zf_common_typedef.h"

typedef enum
{
    NAVIGATION_STATUS_OK = 0,
    NAVIGATION_STATUS_NOT_INITIALIZED,
    NAVIGATION_STATUS_INVALID_ARGUMENT,
    NAVIGATION_STATUS_INVALID_CONFIG,
    NAVIGATION_STATUS_STORAGE_ERROR
} navigation_status_t;

typedef enum
{
    NAVIGATION_MODE_IDLE = 0,
    NAVIGATION_MODE_RECORDING,
    NAVIGATION_MODE_REPLAYING,
    NAVIGATION_MODE_REPLAY_COMPLETE,
    NAVIGATION_MODE_ERROR
} navigation_mode_t;

typedef struct
{
    navigation_status_t status;
    navigation_mode_t mode;
    float x_m;
    float y_m;
    float yaw_rad;
    float forward_speed_m_s;
    float imu_yaw_rate_rad_s;
    float wheel_yaw_rate_rad_s;
    float fused_yaw_rate_rad_s;
    float gyro_bias_z_rad_s;
    float traveled_distance_m;
    float route_distance_m;
    float route_heading_reference_rad;
    float route_heading_error_rad;
    float target_yaw_rate_rad_s;
    uint32 update_count;
    uint32 storage_error_count;
    uint32 route_sample_count;
    uint8 wheel_feedback_used;
    uint8 route_id;
} navigation_state_t;

navigation_status_t navigation_init(const imu_data_t *imu);
navigation_status_t navigation_reset_pose(float x_m,
                                          float y_m,
                                          float yaw_rad);

// Run from the 200 Hz control step. Flash recording only writes to a RAM
// buffer here; blocking page writes remain in nav_flash_service().
navigation_status_t navigation_update(const imu_data_t *imu,
                                      const wheel_feedback_t *wheel,
                                      uint8 wheel_feedback_valid,
                                      float dt_s);

// Route start/load functions must be called from the main context, not an ISR.
navigation_status_t navigation_start_recording(uint8 route_id);
navigation_status_t navigation_stop_recording(void);
navigation_status_t navigation_abort_recording(void);
navigation_status_t navigation_start_replay(uint8 route_id);
void navigation_stop_replay(void);

const navigation_state_t *navigation_get_state(void);
uint8 navigation_is_initialized(void);

#endif
