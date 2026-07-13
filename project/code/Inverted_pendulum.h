#ifndef PROJECT_INVERTED_PENDULUM_H
#define PROJECT_INVERTED_PENDULUM_H

#include "zf_common_typedef.h"

typedef enum
{
    PENDULUM_STATUS_OK = 0,
    PENDULUM_STATUS_INVALID_ARGUMENT,
    PENDULUM_STATUS_INVALID_PARAMETERS
} pendulum_status_t;

typedef struct
{
    // cart_equivalent_mass includes both wheels and reflected wheel inertia.
    float cart_equivalent_mass_kg;
    float body_mass_kg;
    float body_com_height_m;
    float gravity_m_s2;
} pendulum_parameters_t;

typedef struct
{
    // State order is [position, speed, pitch, pitch_rate].
    float a[4][4];
    float b[4];
} pendulum_linear_model_t;

typedef struct
{
    float position_m;
    float speed_m_s;
    float pitch_rad;
    float pitch_rate_rad_s;
} pendulum_state_t;

typedef struct
{
    float position;
    float speed;
    float pitch;
    float pitch_rate;
} pendulum_feedback_gain_t;

// Generates the continuous-time upright cart-pole linear model. It is intended
// for offline LQR/pole-placement calculation after measured parameters arrive.
pendulum_status_t inverted_pendulum_linearize(
    const pendulum_parameters_t *parameters,
    pendulum_linear_model_t *model);

// Returns u=-Kx. K must already include any force/torque-to-driver conversion.
float inverted_pendulum_state_feedback(
    const pendulum_state_t *error_state,
    const pendulum_feedback_gain_t *gain);

#endif
