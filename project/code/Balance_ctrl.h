#ifndef PROJECT_BALANCE_CTRL_H
#define PROJECT_BALANCE_CTRL_H

#include "Imu.h"
#include "Wheel_driver.h"
#include "zf_common_typedef.h"

typedef enum
{
    BALANCE_FAULT_NONE = 0U,
    BALANCE_FAULT_PITCH_LIMIT = 1U << 0,
    BALANCE_FAULT_ROLL_LIMIT = 1U << 1,
    BALANCE_FAULT_IMU = 1U << 2,
    BALANCE_FAULT_WHEEL_FEEDBACK = 1U << 3
} balance_fault_t;

typedef struct
{
    float target_speed_m_s;
    float target_yaw_rate_rad_s;
    float target_leg_x_offset_m;
    float target_leg_z_offset_m;
    // When set, speed is controlled by the common leg x offset. The wheel
    // controller keeps its pitch reference at BALANCE_PITCH_ZERO_RAD.
    uint8 use_leg_speed_control;
} balance_command_t;

typedef struct
{
    uint8 enabled;
    uint32 fault_flags;
    float measured_position_m;
    float position_reference_m;
    float measured_speed_m_s;
    float pitch_reference_rad;
    float pitch_rate_reference_rad_s;
    float balance_command;
    float yaw_command;
    int16 left_wheel_command;
    int16 right_wheel_command;
    uint8 leg_speed_control_active;
} balance_state_t;

void balance_ctrl_init(void);
void balance_ctrl_set_command(const balance_command_t *command);
const balance_command_t *balance_ctrl_get_command(void);
uint8 balance_ctrl_set_enabled(uint8 enabled);
void balance_ctrl_clear_faults(void);
void balance_ctrl_force_fault(balance_fault_t fault);
void balance_ctrl_update(const imu_data_t *imu,
                         const wheel_feedback_t *wheel,
                         uint8 run_speed_loop);
const balance_state_t *balance_ctrl_get_state(void);

#endif
