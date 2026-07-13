#ifndef PROJECT_ROTATION_CTRL_H
#define PROJECT_ROTATION_CTRL_H

#include "Balance_ctrl.h"
#include "Imu.h"
#include "zf_common_typedef.h"

typedef enum
{
    ROTATION_CTRL_STATUS_OK = 0,
    ROTATION_CTRL_STATUS_DISABLED,
    ROTATION_CTRL_STATUS_INVALID_ARGUMENT,
    ROTATION_CTRL_STATUS_INVALID_CONFIG,
    ROTATION_CTRL_STATUS_BUSY
} rotation_ctrl_status_t;

typedef enum
{
    ROTATION_DIR_CW = 0,
    ROTATION_DIR_CCW
} rotation_dir_t;

typedef enum
{
    ROTATION_PHASE_IDLE = 0,
    ROTATION_PHASE_RUNNING,
    ROTATION_PHASE_SETTLING,
    ROTATION_PHASE_HOLDING
} rotation_phase_t;

typedef enum
{
    ROTATION_RESULT_IDLE = 0,
    ROTATION_RESULT_RUNNING,
    ROTATION_RESULT_COMPLETED,
    ROTATION_RESULT_ABORTED,
    ROTATION_RESULT_TIMEOUT,
    ROTATION_RESULT_FAULT
} rotation_result_t;

typedef struct
{
    rotation_ctrl_status_t status;
    rotation_phase_t phase;
    rotation_result_t result;
    rotation_dir_t direction;
    uint8 enabled;
    int8 yaw_sign;
    uint32 elapsed_steps;
    uint32 timeout_steps;
    uint32 settle_steps;
    float target_angle_deg;
    float progress_angle_deg;
    float remaining_angle_deg;
    float last_yaw_deg;
    float target_yaw_rate_rad_s;
    float measured_yaw_rate_rad_s;
} rotation_ctrl_state_t;

rotation_ctrl_status_t rotation_ctrl_init(void);

// Starts a zero-forward-speed turn. The first yaw sample seeds signed angle
// accumulation so crossing +/-180 degrees does not lose progress.
rotation_ctrl_status_t rotation_ctrl_start(rotation_dir_t direction,
                                           float turns,
                                           const imu_data_t *imu);

// Called at CONTROL_FAST_PERIOD_S. While the module owns the command, forward
// speed is forced to zero and only the yaw-rate target is replaced.
rotation_ctrl_status_t rotation_ctrl_update(
    const imu_data_t *imu,
    const balance_command_t *requested,
    balance_command_t *shaped);

// Abort releases command ownership immediately. Release is used after the
// holding phase so stale requested commands cannot resume at completion.
rotation_ctrl_status_t rotation_ctrl_abort(void);
rotation_ctrl_status_t rotation_ctrl_release(void);

// Active includes the completed/timeout holding phase because it still owns a
// zero-speed command until rotation_ctrl_release() is called.
uint8 rotation_ctrl_is_active(void);
const rotation_ctrl_state_t *rotation_ctrl_get_state(void);

#endif
