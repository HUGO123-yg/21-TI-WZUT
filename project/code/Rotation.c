#include "Rotation.h"

#include "config.h"

volatile rotation_control_struct rotation =
{
    .state = ROT_IDLE,
    .dir = ROT_CW,
    .turn_duty = 0,
    .elapsed_ms = 0U,
};

static int16 rotation_apply_direction(int16 duty)
{
    int16 signed_duty = (int16)(duty * ROTATION_CW_OUTPUT_SIGN);

    return (ROT_CCW == rotation.dir) ? (int16)-signed_duty : signed_duty;
}

static int16 rotation_calculate_duty(void)
{
    uint32 remaining_ms = ROTATION_DURATION_MS - rotation.elapsed_ms;
    uint32 ramp_progress_ms = ROTATION_RAMP_MS;
    int32 duty;

    if (rotation.elapsed_ms < ROTATION_RAMP_MS)
    {
        ramp_progress_ms = rotation.elapsed_ms;
    }
    else if (remaining_ms < ROTATION_RAMP_MS)
    {
        ramp_progress_ms = remaining_ms;
    }

    duty = (int32)ROTATION_TURN_DUTY * (int32)ramp_progress_ms
         / (int32)ROTATION_RAMP_MS;
    if (duty < 0)
    {
        duty = 0;
    }
    else if (duty > ROTATION_TURN_DUTY)
    {
        duty = ROTATION_TURN_DUTY;
    }
    return rotation_apply_direction((int16)duty);
}

uint8 rotation_start(rotation_dir_enum dir)
{
    if ((ROT_IDLE != rotation.state)
        || ((ROT_CW != dir) && (ROT_CCW != dir)))
    {
        return 0U;
    }

    rotation.dir = dir;
    rotation.turn_duty = 0;
    rotation.elapsed_ms = 0U;
    rotation.state = ROT_RUNNING;
    return 1U;
}

void rotation_run(void)
{
    if (ROT_RUNNING != rotation.state)
    {
        return;
    }

    rotation.elapsed_ms++;
    if (rotation.elapsed_ms >= ROTATION_DURATION_MS)
    {
        rotation.turn_duty = 0;
        rotation.state = ROT_DONE;
        return;
    }

    rotation.turn_duty = rotation_calculate_duty();
}

void rotation_stop(void)
{
    rotation.state = ROT_IDLE;
    rotation.turn_duty = 0;
    rotation.elapsed_ms = 0U;
}

uint8 rotation_is_active(void)
{
    return (ROT_RUNNING == rotation.state) ? 1U : 0U;
}

uint8 rotation_is_done(void)
{
    return (ROT_DONE == rotation.state) ? 1U : 0U;
}

uint8 rotation_owns_output(void)
{
    return (ROT_IDLE != rotation.state) ? 1U : 0U;
}

const char *rotation_state_name(rotation_state_enum state)
{
    switch (state)
    {
        case ROT_IDLE:    return "Idle";
        case ROT_RUNNING: return "Running";
        case ROT_DONE:    return "Done";
        default:          return "Unknown";
    }
}
