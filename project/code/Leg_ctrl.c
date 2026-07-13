#include "Leg_ctrl.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "zf_driver_pwm.h"

typedef struct
{
    pwm_channel_enum channel;
    leg_servo_calibration_t calibration;
} leg_servo_t;

static const uint8 leg_joint_servo_map[LEG_SIDE_COUNT][2] =
{
    {LEG_LEFT_JOINT_A_SERVO, LEG_LEFT_JOINT_B_SERVO},
    {LEG_RIGHT_JOINT_A_SERVO, LEG_RIGHT_JOINT_B_SERVO}
};

static five_bar_geometry_t leg_geometry[LEG_SIDE_COUNT];
static leg_servo_t leg_servo[LEG_SERVO_COUNT];
static leg_ctrl_state_t leg_state;
static uint8 leg_ready;

static float leg_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float leg_slew(float current, float target)
{
    float delta;

    delta = target - current;
    delta = leg_clamp(delta,
                      -LEG_MAX_TARGET_STEP_M,
                      LEG_MAX_TARGET_STEP_M);
    return current + delta;
}

static uint8 leg_float_is_finite(float value)
{
    return (uint8)((value == value)
                   && (value <= FLT_MAX)
                   && (value >= -FLT_MAX));
}

static uint8 leg_servo_calibration_is_valid(
    const leg_servo_calibration_t *calibration)
{
    if (0 == calibration)
    {
        return 0U;
    }

    return (uint8)(((1 == calibration->direction)
                    || (-1 == calibration->direction))
                   && (calibration->pwm_per_rad > 0.0f)
                   && (calibration->minimum_pwm < calibration->maximum_pwm)
                   && (calibration->center_pwm >= calibration->minimum_pwm)
                   && (calibration->center_pwm <= calibration->maximum_pwm));
}

static uint8 leg_all_calibration_is_valid(void)
{
    uint8 index;

    if (!five_bar_geometry_is_valid(&leg_geometry[LEG_SIDE_LEFT])
        || !five_bar_geometry_is_valid(&leg_geometry[LEG_SIDE_RIGHT]))
    {
        return 0U;
    }

    for (index = 0U; index < LEG_SERVO_COUNT; index++)
    {
        if (!leg_servo_calibration_is_valid(&leg_servo[index].calibration))
        {
            return 0U;
        }
    }
    return 1U;
}

static leg_ctrl_status_t leg_angle_to_pwm(uint8 servo_index,
                                           float angle_rad,
                                           int32 *pwm)
{
    const leg_servo_calibration_t *calibration;
    float pwm_float;
    int32 pwm_value;

    if ((servo_index >= LEG_SERVO_COUNT) || (0 == pwm))
    {
        return LEG_CTRL_STATUS_INVALID_ARGUMENT;
    }

    calibration = &leg_servo[servo_index].calibration;
    if (!leg_servo_calibration_is_valid(calibration))
    {
        return LEG_CTRL_STATUS_UNCALIBRATED;
    }

    pwm_float = (float)calibration->center_pwm
                + (float)calibration->direction
                  * calibration->pwm_per_rad
                  * (angle_rad - calibration->zero_angle_rad);
    pwm_value = (int32)(pwm_float + ((pwm_float >= 0.0f) ? 0.5f : -0.5f));
    if ((pwm_value < calibration->minimum_pwm)
        || (pwm_value > calibration->maximum_pwm))
    {
        return LEG_CTRL_STATUS_SERVO_LIMIT;
    }

    *pwm = pwm_value;
    return LEG_CTRL_STATUS_OK;
}

static void leg_load_default_configuration(void)
{
    five_bar_geometry_t geometry;
    const pwm_channel_enum channels[LEG_SERVO_COUNT] =
    {
        LEG_SERVO_1_PWM,
        LEG_SERVO_2_PWM,
        LEG_SERVO_3_PWM,
        LEG_SERVO_4_PWM
    };
    const int32 centers[LEG_SERVO_COUNT] =
    {
        LEG_SERVO_1_CENTER,
        LEG_SERVO_2_CENTER,
        LEG_SERVO_3_CENTER,
        LEG_SERVO_4_CENTER
    };
    const int8 directions[LEG_SERVO_COUNT] =
    {
        LEG_SERVO_1_DIRECTION,
        LEG_SERVO_2_DIRECTION,
        LEG_SERVO_3_DIRECTION,
        LEG_SERVO_4_DIRECTION
    };
    const float pwm_per_rad[LEG_SERVO_COUNT] =
    {
        LEG_SERVO_1_PWM_PER_RAD,
        LEG_SERVO_2_PWM_PER_RAD,
        LEG_SERVO_3_PWM_PER_RAD,
        LEG_SERVO_4_PWM_PER_RAD
    };
    const float zero_rad[LEG_SERVO_COUNT] =
    {
        LEG_SERVO_1_ZERO_RAD,
        LEG_SERVO_2_ZERO_RAD,
        LEG_SERVO_3_ZERO_RAD,
        LEG_SERVO_4_ZERO_RAD
    };
    const int32 minimum_pwm[LEG_SERVO_COUNT] =
    {
        LEG_SERVO_1_PWM_MIN,
        LEG_SERVO_2_PWM_MIN,
        LEG_SERVO_3_PWM_MIN,
        LEG_SERVO_4_PWM_MIN
    };
    const int32 maximum_pwm[LEG_SERVO_COUNT] =
    {
        LEG_SERVO_1_PWM_MAX,
        LEG_SERVO_2_PWM_MAX,
        LEG_SERVO_3_PWM_MAX,
        LEG_SERVO_4_PWM_MAX
    };
    uint8 index;

    geometry.base_spacing_m = LEG_BASE_SPACING_M;
    geometry.link_a_proximal_m = LEG_LINK_A_PROXIMAL_M;
    geometry.link_a_distal_m = LEG_LINK_A_DISTAL_M;
    geometry.link_b_proximal_m = LEG_LINK_B_PROXIMAL_M;
    geometry.link_b_distal_m = LEG_LINK_B_DISTAL_M;
    geometry.branch_a = LEG_BRANCH_A;
    geometry.branch_b = LEG_BRANCH_B;
    geometry.joint_a_min_rad = LEG_JOINT_A_MIN_RAD;
    geometry.joint_a_max_rad = LEG_JOINT_A_MAX_RAD;
    geometry.joint_b_min_rad = LEG_JOINT_B_MIN_RAD;
    geometry.joint_b_max_rad = LEG_JOINT_B_MAX_RAD;
    leg_geometry[LEG_SIDE_LEFT] = geometry;
    leg_geometry[LEG_SIDE_RIGHT] = geometry;

    for (index = 0U; index < LEG_SERVO_COUNT; index++)
    {
        leg_servo[index].channel = channels[index];
        leg_servo[index].calibration.center_pwm = centers[index];
        leg_servo[index].calibration.direction = directions[index];
        leg_servo[index].calibration.pwm_per_rad = pwm_per_rad[index];
        leg_servo[index].calibration.zero_angle_rad = zero_rad[index];
        leg_servo[index].calibration.minimum_pwm = minimum_pwm[index];
        leg_servo[index].calibration.maximum_pwm = maximum_pwm[index];
    }
}

leg_ctrl_status_t leg_ctrl_init(void)
{
    five_bar_point_t reference_point;
    five_bar_status_t reference_status;
    uint8 index;
    uint8 side;

    memset(&leg_state, 0, sizeof(leg_state));
    memset(leg_geometry, 0, sizeof(leg_geometry));
    memset(leg_servo, 0, sizeof(leg_servo));
    leg_ready = 0U;
    leg_load_default_configuration();

    for (side = 0U; side < LEG_SIDE_COUNT; side++)
    {
        reference_status = five_bar_forward(&leg_geometry[side],
                                            LEG_REFERENCE_JOINT_A_RAD,
                                            LEG_REFERENCE_JOINT_B_RAD,
                                            &reference_point);
        if (FIVE_BAR_STATUS_OK != reference_status)
        {
            leg_state.status = LEG_CTRL_STATUS_UNCALIBRATED;
            leg_state.kinematics_status[side] = reference_status;
            return leg_state.status;
        }
        leg_state.reference_x_m[side] = reference_point.x_m;
        leg_state.reference_z_m[side] = reference_point.z_m;
        leg_state.commanded_x_m[side] = reference_point.x_m
                                        + LEG_DEFAULT_X_OFFSET_M;
        leg_state.commanded_z_m[side] = reference_point.z_m
                                        + LEG_DEFAULT_Z_OFFSET_M;
    }
    leg_state.requested_x_offset_m = LEG_DEFAULT_X_OFFSET_M;
    leg_state.requested_z_offset_m = LEG_DEFAULT_Z_OFFSET_M;
    leg_state.requested_z_differential_m = 0.0f;

    if (!LEG_CONTROL_ENABLE)
    {
        leg_state.status = LEG_CTRL_STATUS_DISABLED;
        return leg_state.status;
    }
    if (!leg_all_calibration_is_valid())
    {
        leg_state.status = LEG_CTRL_STATUS_UNCALIBRATED;
        return leg_state.status;
    }

    for (index = 0U; index < LEG_SERVO_COUNT; index++)
    {
        pwm_init(leg_servo[index].channel,
                 LEG_SERVO_FREQUENCY_HZ,
                 (uint32)leg_servo[index].calibration.center_pwm);
        leg_state.servo_pwm[index] =
            leg_servo[index].calibration.center_pwm;
    }
    leg_ready = 1U;
    leg_state.status = LEG_CTRL_STATUS_OK;
    return leg_state.status;
}

leg_ctrl_status_t leg_ctrl_set_geometry(leg_side_t side,
                                        const five_bar_geometry_t *geometry)
{
    five_bar_point_t reference_point;

    if (leg_ready
        || (side >= LEG_SIDE_COUNT)
        || !five_bar_geometry_is_valid(geometry)
        || (FIVE_BAR_STATUS_OK
            != five_bar_forward(geometry,
                                LEG_REFERENCE_JOINT_A_RAD,
                                LEG_REFERENCE_JOINT_B_RAD,
                                &reference_point)))
    {
        return LEG_CTRL_STATUS_INVALID_ARGUMENT;
    }
    leg_geometry[side] = *geometry;
    leg_state.reference_x_m[side] = reference_point.x_m;
    leg_state.reference_z_m[side] = reference_point.z_m;
    leg_state.commanded_x_m[side] = reference_point.x_m
                                    + leg_state.requested_x_offset_m;
    leg_state.commanded_z_m[side] = reference_point.z_m
                                    + leg_state.requested_z_offset_m;
    return LEG_CTRL_STATUS_OK;
}

leg_ctrl_status_t leg_ctrl_set_servo_calibration(
    uint8 servo_index,
    const leg_servo_calibration_t *calibration)
{
    if (leg_ready
        || (servo_index >= LEG_SERVO_COUNT)
        || !leg_servo_calibration_is_valid(calibration))
    {
        return LEG_CTRL_STATUS_INVALID_ARGUMENT;
    }
    leg_servo[servo_index].calibration = *calibration;
    return LEG_CTRL_STATUS_OK;
}

leg_ctrl_status_t leg_ctrl_set_target_offset(float x_offset_m,
                                             float z_offset_m)
{
    if (!leg_float_is_finite(x_offset_m)
        || !leg_float_is_finite(z_offset_m)
        || (leg_state.reference_z_m[LEG_SIDE_LEFT] + z_offset_m
            + leg_state.requested_z_differential_m
            > LEG_MAX_ABSOLUTE_Z_M)
        || (leg_state.reference_z_m[LEG_SIDE_RIGHT] + z_offset_m
            - leg_state.requested_z_differential_m
            > LEG_MAX_ABSOLUTE_Z_M))
    {
        return LEG_CTRL_STATUS_INVALID_ARGUMENT;
    }
    leg_state.requested_x_offset_m = x_offset_m;
    leg_state.requested_z_offset_m = z_offset_m;
    return LEG_CTRL_STATUS_OK;
}

leg_ctrl_status_t leg_ctrl_set_differential_z_offset(
    float differential_z_offset_m)
{
    if (!leg_float_is_finite(differential_z_offset_m)
        || (fabsf(differential_z_offset_m)
            > LEG_MAX_DIFFERENTIAL_Z_OFFSET_M)
        || (leg_state.reference_z_m[LEG_SIDE_LEFT]
            + leg_state.requested_z_offset_m
            + differential_z_offset_m > LEG_MAX_ABSOLUTE_Z_M)
        || (leg_state.reference_z_m[LEG_SIDE_RIGHT]
            + leg_state.requested_z_offset_m
            - differential_z_offset_m > LEG_MAX_ABSOLUTE_Z_M))
    {
        return LEG_CTRL_STATUS_INVALID_ARGUMENT;
    }
    leg_state.requested_z_differential_m = differential_z_offset_m;
    return LEG_CTRL_STATUS_OK;
}

static leg_ctrl_status_t leg_ctrl_update_internal(float roll_rad,
                                                  float roll_rate_rad_s,
                                                  uint8 immediate)
{
    float roll_offset;
    float target_z[LEG_SIDE_COUNT];
    five_bar_solution_t solution[LEG_SIDE_COUNT];
    int32 servo_pwm[LEG_SERVO_COUNT];
    leg_ctrl_status_t status;
    uint8 side;
    uint8 joint;
    uint8 servo_index;

    if (!leg_ready)
    {
        return leg_state.status;
    }

    roll_offset = LEG_ROLL_DIRECTION
                  * (LEG_ROLL_KP_M_PER_RAD * roll_rad
                     + LEG_ROLL_KD_M_PER_RAD_S * roll_rate_rad_s);
    roll_offset = leg_clamp(roll_offset,
                            -LEG_MAX_ROLL_OFFSET_M,
                            LEG_MAX_ROLL_OFFSET_M);
    target_z[LEG_SIDE_LEFT] = leg_state.reference_z_m[LEG_SIDE_LEFT]
                              + leg_state.requested_z_offset_m
                              + leg_state.requested_z_differential_m
                              + roll_offset;
    target_z[LEG_SIDE_RIGHT] = leg_state.reference_z_m[LEG_SIDE_RIGHT]
                               + leg_state.requested_z_offset_m
                               - leg_state.requested_z_differential_m
                               - roll_offset;
    target_z[LEG_SIDE_LEFT] = leg_clamp(target_z[LEG_SIDE_LEFT],
                                        -FLT_MAX,
                                        LEG_MAX_ABSOLUTE_Z_M);
    target_z[LEG_SIDE_RIGHT] = leg_clamp(target_z[LEG_SIDE_RIGHT],
                                         -FLT_MAX,
                                         LEG_MAX_ABSOLUTE_Z_M);

    for (side = 0U; side < LEG_SIDE_COUNT; side++)
    {
        if (immediate)
        {
            leg_state.commanded_x_m[side] = leg_state.reference_x_m[side]
                + leg_state.requested_x_offset_m;
            leg_state.commanded_z_m[side] = target_z[side];
        }
        else
        {
            leg_state.commanded_x_m[side] = leg_slew(
                leg_state.commanded_x_m[side],
                leg_state.reference_x_m[side]
                + leg_state.requested_x_offset_m);
            leg_state.commanded_z_m[side] = leg_slew(
                leg_state.commanded_z_m[side],
                target_z[side]);
        }
        leg_state.kinematics_status[side] = five_bar_inverse(
            &leg_geometry[side],
            leg_state.commanded_x_m[side],
            leg_state.commanded_z_m[side],
            &solution[side]);
        if (FIVE_BAR_STATUS_OK != leg_state.kinematics_status[side])
        {
            leg_state.status = LEG_CTRL_STATUS_KINEMATICS_ERROR;
            return leg_state.status;
        }
    }

    for (side = 0U; side < LEG_SIDE_COUNT; side++)
    {
        const float joint_angle[2] =
        {
            solution[side].joint_a_rad,
            solution[side].joint_b_rad
        };
        for (joint = 0U; joint < 2U; joint++)
        {
            servo_index = leg_joint_servo_map[side][joint];
            status = leg_angle_to_pwm(servo_index,
                                      joint_angle[joint],
                                      &servo_pwm[servo_index]);
            if (LEG_CTRL_STATUS_OK != status)
            {
                leg_state.status = status;
                return leg_state.status;
            }
        }
    }

    for (servo_index = 0U; servo_index < LEG_SERVO_COUNT; servo_index++)
    {
        pwm_set_duty(leg_servo[servo_index].channel,
                     (uint32)servo_pwm[servo_index]);
        leg_state.servo_pwm[servo_index] = servo_pwm[servo_index];
    }
    leg_state.joint[LEG_SIDE_LEFT] = solution[LEG_SIDE_LEFT];
    leg_state.joint[LEG_SIDE_RIGHT] = solution[LEG_SIDE_RIGHT];
    leg_state.status = LEG_CTRL_STATUS_OK;
    return leg_state.status;
}

leg_ctrl_status_t leg_ctrl_update(float roll_rad,
                                  float roll_rate_rad_s)
{
    return leg_ctrl_update_internal(roll_rad, roll_rate_rad_s, 0U);
}

leg_ctrl_status_t leg_ctrl_move_to_offset_immediate(float x_offset_m,
                                                    float z_offset_m)
{
    leg_ctrl_status_t status;

    status = leg_ctrl_set_differential_z_offset(0.0f);
    if (LEG_CTRL_STATUS_OK != status)
    {
        return status;
    }
    status = leg_ctrl_set_target_offset(x_offset_m, z_offset_m);
    if (LEG_CTRL_STATUS_OK != status)
    {
        return status;
    }
    return leg_ctrl_update_internal(0.0f, 0.0f, 1U);
}

void leg_ctrl_disable_output(void)
{
    uint8 index;

    if (!leg_ready)
    {
        return;
    }
    for (index = 0U; index < LEG_SERVO_COUNT; index++)
    {
        pwm_set_duty(leg_servo[index].channel, 0U);
        leg_state.servo_pwm[index] = 0;
    }
    leg_ready = 0U;
    leg_state.status = LEG_CTRL_STATUS_DISABLED;
}

uint8 leg_ctrl_is_ready(void)
{
    return leg_ready;
}

const leg_ctrl_state_t *leg_ctrl_get_state(void)
{
    return &leg_state;
}
