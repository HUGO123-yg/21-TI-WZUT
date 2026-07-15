#ifndef PROJECT_BRIDGE_CTRL_H
#define PROJECT_BRIDGE_CTRL_H

#include "Balance_ctrl.h"
#include "zf_common_typedef.h"

typedef enum
{
    BRIDGE_CTRL_STATUS_OK = 0,
    BRIDGE_CTRL_STATUS_DISABLED,
    BRIDGE_CTRL_STATUS_INVALID_ARGUMENT,
    BRIDGE_CTRL_STATUS_INVALID_CONFIG
} bridge_ctrl_status_t;

typedef enum
{
    BRIDGE_PHASE_IDLE = 0,
    BRIDGE_PHASE_DETECTING,
    BRIDGE_PHASE_ENTERING,
    BRIDGE_PHASE_CROSSING,
    BRIDGE_PHASE_EXITING,
    BRIDGE_PHASE_RECOVERING
} bridge_phase_t;

typedef enum
{
    BRIDGE_RESULT_IDLE = 0,
    BRIDGE_RESULT_RUNNING,
    BRIDGE_RESULT_COMPLETED,
    BRIDGE_RESULT_TIMEOUT,
    BRIDGE_RESULT_FAULT,
    BRIDGE_RESULT_ABORTED
} bridge_ctrl_result_t;

typedef struct
{
    bridge_ctrl_status_t status;
    bridge_ctrl_result_t result;
    bridge_phase_t phase;
    uint32 phase_elapsed_steps;
    uint32 detect_count;
    uint32 recover_count;
    int8 roll_sign;
    uint8 enabled;
    uint8 forced_entry;
    float entry_distance_m;
    float crossing_distance_m;
    float latched_feedforward_m;
    float roll_feedback_m;
    // Positive means left leg +z and right leg -z.
    float differential_leg_offset_m;
    float shaped_speed_m_s;
    float shaped_yaw_rate_rad_s;
} bridge_ctrl_state_t;

bridge_ctrl_status_t bridge_ctrl_init(void);
bridge_ctrl_status_t bridge_ctrl_set_enabled(uint8 enabled);

// roll_sign is +1 or -1 and is used by a route/mileage trigger when the bridge
// side is known before IMU roll exceeds the automatic detection threshold.
bridge_ctrl_status_t bridge_ctrl_force_enter(int8 roll_sign,
                                             float traveled_distance_m);
bridge_ctrl_status_t bridge_ctrl_abort(void);

// Called at CONTROL_FAST_PERIOD_S. The requested command is copied and shaped;
// wheel and leg outputs remain owned by Balance_ctrl and Leg_ctrl.
bridge_ctrl_status_t bridge_ctrl_update(const imu_data_t *imu,
                                        float traveled_distance_m,
                                        const balance_command_t *requested,
                                        balance_command_t *shaped);

uint8 bridge_ctrl_is_active(void);
const bridge_ctrl_state_t *bridge_ctrl_get_state(void);

#endif
