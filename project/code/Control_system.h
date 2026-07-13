#ifndef PROJECT_CONTROL_SYSTEM_H
#define PROJECT_CONTROL_SYSTEM_H

#include "Balance_ctrl.h"
#include "Leg_ctrl.h"
#include "zf_common_typedef.h"

typedef enum
{
    CONTROL_STATUS_OK = 0,
    CONTROL_STATUS_IMU_ERROR,
    CONTROL_STATUS_NOT_READY
} control_status_t;

typedef struct
{
    control_status_t status;
    uint32 scheduler_tick_ms;
    uint32 wheel_feedback_age_ms;
    uint32 imu_error_count;
    uint32 leg_error_count;
    uint8 wheel_feedback_ready;
} control_system_state_t;

control_status_t control_system_init(void);

// Called from the 1 ms PIT callback. Work is executed only at configured rates.
void control_system_tick_1ms(void);

void control_system_set_command(const balance_command_t *command);
uint8 control_system_set_enabled(uint8 enabled);
void control_system_clear_faults(void);
const control_system_state_t *control_system_get_state(void);

#endif
