#include "Inverted_pendulum.h"

#include <string.h>

pendulum_status_t inverted_pendulum_linearize(
    const pendulum_parameters_t *parameters,
    pendulum_linear_model_t *model)
{
    float cart_mass;
    float body_mass;
    float height;
    float gravity;

    if ((0 == parameters) || (0 == model))
    {
        return PENDULUM_STATUS_INVALID_ARGUMENT;
    }
    if ((parameters->cart_equivalent_mass_kg <= 0.0f)
        || (parameters->body_mass_kg <= 0.0f)
        || (parameters->body_com_height_m <= 0.0f)
        || (parameters->gravity_m_s2 <= 0.0f))
    {
        return PENDULUM_STATUS_INVALID_PARAMETERS;
    }

    cart_mass = parameters->cart_equivalent_mass_kg;
    body_mass = parameters->body_mass_kg;
    height = parameters->body_com_height_m;
    gravity = parameters->gravity_m_s2;
    memset(model, 0, sizeof(*model));

    model->a[0][1] = 1.0f;
    model->a[1][2] = -body_mass * gravity / cart_mass;
    model->a[2][3] = 1.0f;
    model->a[3][2] = (cart_mass + body_mass) * gravity
                     / (cart_mass * height);
    model->b[1] = 1.0f / cart_mass;
    model->b[3] = -1.0f / (cart_mass * height);
    return PENDULUM_STATUS_OK;
}

float inverted_pendulum_state_feedback(
    const pendulum_state_t *error_state,
    const pendulum_feedback_gain_t *gain)
{
    if ((0 == error_state) || (0 == gain))
    {
        return 0.0f;
    }

    return -(gain->position * error_state->position_m
             + gain->speed * error_state->speed_m_s
             + gain->pitch * error_state->pitch_rad
             + gain->pitch_rate * error_state->pitch_rate_rad_s);
}
