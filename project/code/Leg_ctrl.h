#ifndef PROJECT_LEG_CTRL_H
#define PROJECT_LEG_CTRL_H

#include "Five_bar.h"
#include "zf_common_typedef.h"

typedef enum
{
    LEG_SIDE_LEFT = 0,
    LEG_SIDE_RIGHT,
    LEG_SIDE_COUNT
} leg_side_t;

typedef enum
{
    LEG_CTRL_STATUS_OK = 0,
    LEG_CTRL_STATUS_DISABLED,
    LEG_CTRL_STATUS_UNCALIBRATED,
    LEG_CTRL_STATUS_INVALID_ARGUMENT,
    LEG_CTRL_STATUS_KINEMATICS_ERROR,
    LEG_CTRL_STATUS_SERVO_LIMIT
} leg_ctrl_status_t;

typedef struct
{
    int32 center_pwm;
    int8 direction;
    float pwm_per_rad;
    float zero_angle_rad;
    int32 minimum_pwm;
    int32 maximum_pwm;
} leg_servo_calibration_t;

typedef struct
{
    leg_ctrl_status_t status;
    five_bar_status_t kinematics_status[LEG_SIDE_COUNT];
    float reference_x_m[LEG_SIDE_COUNT];
    float reference_z_m[LEG_SIDE_COUNT];
    float requested_x_offset_m;
    float requested_z_offset_m;
    float requested_z_differential_m;
    float commanded_x_m[LEG_SIDE_COUNT];
    float commanded_z_m[LEG_SIDE_COUNT];
    five_bar_solution_t joint[LEG_SIDE_COUNT];
    int32 servo_pwm[4];
} leg_ctrl_state_t;

leg_ctrl_status_t leg_ctrl_init(void);
leg_ctrl_status_t leg_ctrl_set_geometry(leg_side_t side,
                                        const five_bar_geometry_t *geometry);
leg_ctrl_status_t leg_ctrl_set_servo_calibration(
    uint8 servo_index,
    const leg_servo_calibration_t *calibration);
leg_ctrl_status_t leg_ctrl_set_target_offset(float x_offset_m,
                                             float z_offset_m);
// Positive differential extends the left leg (+z) and retracts the right leg.
leg_ctrl_status_t leg_ctrl_set_differential_z_offset(
    float differential_z_offset_m);
leg_ctrl_status_t leg_ctrl_update(float roll_rad,
                                  float roll_rate_rad_s);
// Applies a scripted target in one update, bypassing the normal slew limit.
// Intended for fixed-time actions such as jumping, not normal balance control.
leg_ctrl_status_t leg_ctrl_move_to_offset_immediate(float x_offset_m,
                                                    float z_offset_m);
void leg_ctrl_disable_output(void);
uint8 leg_ctrl_is_ready(void);
const leg_ctrl_state_t *leg_ctrl_get_state(void);

#endif
