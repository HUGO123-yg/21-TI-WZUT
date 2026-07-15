#ifndef PROJECT_CONTROL_SYSTEM_H
#define PROJECT_CONTROL_SYSTEM_H

#include "Balance_ctrl.h"
#include "Bridge_ctrl.h"
#include "Bumpy_ctrl.h"
#include "Jump.h"
#include "Leg_ctrl.h"
#include "Rotation_ctrl.h"
#include "zf_common_typedef.h"

typedef enum
{
    CONTROL_STATUS_OK = 0,
    CONTROL_STATUS_IMU_ERROR,
    CONTROL_STATUS_NOT_READY,
    CONTROL_STATUS_JUMP_ERROR,
    CONTROL_STATUS_LEG_ERROR,
    CONTROL_STATUS_WHEEL_CONFIG_ERROR,
    CONTROL_STATUS_BALANCE_CONFIG_ERROR,
    CONTROL_STATUS_EMERGENCY_STOP
} control_status_t;

typedef enum
{
    CONTROL_FAULT_NONE = 0U,
    CONTROL_FAULT_EMERGENCY_STOP = 1U << 0,
    CONTROL_FAULT_IMU = 1U << 1,
    CONTROL_FAULT_BALANCE = 1U << 2,
    CONTROL_FAULT_LEG = 1U << 3,
    CONTROL_FAULT_JUMP = 1U << 4,
    CONTROL_FAULT_WHEEL = 1U << 5
} control_fault_t;

typedef enum
{
    CONTROL_STARTUP_DISABLED = 0,
    CONTROL_STARTUP_WAITING_DELAY,
    CONTROL_STARTUP_WAITING_WHEEL_FEEDBACK,
    CONTROL_STARTUP_WAITING_UPRIGHT,
    CONTROL_STARTUP_STANDING,
    CONTROL_STARTUP_FAULT,
    CONTROL_STARTUP_EMERGENCY_STOP
} control_startup_state_t;

typedef struct
{
    control_status_t status;
    uint32 fault_flags;
    uint32 scheduler_tick_ms;
    uint32 wheel_feedback_age_ms;
    uint32 imu_error_count;
    uint32 leg_error_count;
    uint32 jump_error_count;
    uint32 navigation_error_count;
    uint32 bridge_error_count;
    uint32 bumpy_error_count;
    uint32 rotation_error_count;
    uint32 emergency_stop_count;
    uint32 recovery_attempt_count;
    uint32 recovery_failure_count;
    uint32 last_imu_status;
    uint32 last_leg_status;
    uint32 last_jump_result;
    uint32 last_navigation_status;
    uint32 last_bridge_status;
    uint32 last_bumpy_status;
    uint32 last_rotation_status;
    uint32 last_rotation_result;
    uint8 balance_config_valid;
    uint8 wheel_config_valid;
    uint8 wheel_feedback_ready;
    uint8 jump_active;
    uint8 bridge_active;
    uint8 bumpy_active;
    uint8 rotation_active;
    uint8 stand_request_pending;
    uint8 balance_enabled;
    control_startup_state_t startup_state;
} control_system_state_t;

control_status_t control_system_init(void);

// Called from the 1 ms PIT callback. Work is executed only at configured rates.
void control_system_tick_1ms(void);

void control_system_set_command(const balance_command_t *command);
// Queues a zero-speed stand request. Success means the request was accepted;
// startup_state reports whether balance is active or still waiting on a gate.
uint8 control_system_request_stand(void);
uint8 control_system_set_enabled(uint8 enabled);
// Latches wheel stop before cancelling active actions. It never auto-recovers.
void control_system_emergency_stop(void);
// Called from main context. Keeps outputs locked unless every recovery gate and
// leg safe-pose command succeeds. A successful call leaves balance disabled;
// request_stand() rearms it.
uint8 control_system_recover_faults(void);
uint8 control_system_start_jump(void);
uint8 control_system_abort_jump(void);
uint8 control_system_start_rotation(rotation_dir_t direction, float turns);
uint8 control_system_abort_rotation(void);
uint8 control_system_release_rotation(void);
const rotation_ctrl_state_t *control_system_get_rotation_state(void);
uint8 control_system_set_bridge_enabled(uint8 enabled);
uint8 control_system_start_bridge(int8 roll_sign);
uint8 control_system_abort_bridge(void);
uint8 control_system_set_bumpy_enabled(uint8 enabled);
uint8 control_system_start_bumpy(void);
uint8 control_system_abort_bumpy(void);
const control_system_state_t *control_system_get_state(void);

#endif
