#include "Jump.h"

#include <string.h>

#include "config.h"

static jump_state_t jump_state;

static uint8 jump_apply_leg_target(float z_offset_m)
{
    jump_state.last_leg_status = leg_ctrl_move_to_offset_immediate(
        JUMP_LEG_X_OFFSET_M,
        z_offset_m);
    if (LEG_CTRL_STATUS_OK != jump_state.last_leg_status)
    {
        jump_state.active = 0U;
        jump_state.phase = JUMP_PHASE_IDLE;
        jump_state.result = JUMP_RESULT_LEG_ERROR;
        return 0U;
    }
    return 1U;
}

static uint8 jump_enter_phase(jump_phase_t phase, float z_offset_m)
{
    if (!jump_apply_leg_target(z_offset_m))
    {
        return 0U;
    }
    jump_state.phase = phase;
    jump_state.phase_elapsed_ms = 0U;
    return 1U;
}

void jump_ctrl_init(void)
{
    memset(&jump_state, 0, sizeof(jump_state));
    jump_state.phase = JUMP_PHASE_IDLE;
    jump_state.result = JUMP_RESULT_IDLE;
    jump_state.last_leg_status = LEG_CTRL_STATUS_OK;
}

uint8 jump_ctrl_start(void)
{
    uint32 completed_count;

    if (jump_state.active)
    {
        return 0U;
    }
    if (!leg_ctrl_is_ready())
    {
        jump_state.result = JUMP_RESULT_NOT_READY;
        return 0U;
    }

    completed_count = jump_state.completed_count;
    memset(&jump_state, 0, sizeof(jump_state));
    jump_state.completed_count = completed_count;
    jump_state.active = 1U;
    jump_state.result = JUMP_RESULT_RUNNING;
    jump_state.last_leg_status = LEG_CTRL_STATUS_OK;
    return jump_enter_phase(JUMP_PHASE_EXTEND, JUMP_EXTEND_Z_OFFSET_M);
}

void jump_ctrl_tick_1ms(void)
{
    if (!jump_state.active)
    {
        return;
    }

    jump_state.phase_elapsed_ms++;
    jump_state.total_elapsed_ms++;
    switch (jump_state.phase)
    {
        case JUMP_PHASE_EXTEND:
            if (jump_state.phase_elapsed_ms >= JUMP_EXTEND_TIME_MS)
            {
                (void)jump_enter_phase(JUMP_PHASE_RETRACT,
                                       JUMP_RETRACT_Z_OFFSET_M);
            }
            break;

        case JUMP_PHASE_RETRACT:
            if (jump_state.phase_elapsed_ms >= JUMP_RETRACT_TIME_MS)
            {
                (void)jump_enter_phase(JUMP_PHASE_BUFFER,
                                       JUMP_BUFFER_Z_OFFSET_M);
            }
            break;

        case JUMP_PHASE_BUFFER:
            if (jump_state.phase_elapsed_ms >= JUMP_BUFFER_TIME_MS)
            {
                if (jump_apply_leg_target(JUMP_RETRACT_Z_OFFSET_M))
                {
                    jump_state.active = 0U;
                    jump_state.phase = JUMP_PHASE_IDLE;
                    jump_state.result = JUMP_RESULT_COMPLETED;
                    jump_state.completed_count++;
                }
            }
            break;

        case JUMP_PHASE_IDLE:
        default:
            jump_state.active = 0U;
            jump_state.result = JUMP_RESULT_LEG_ERROR;
            break;
    }
}

uint8 jump_ctrl_abort(void)
{
    jump_state.last_leg_status = leg_ctrl_recover();
    jump_state.active = 0U;
    jump_state.phase = JUMP_PHASE_IDLE;
    if (LEG_CTRL_STATUS_OK != jump_state.last_leg_status)
    {
        jump_state.result = JUMP_RESULT_LEG_ERROR;
        return 0U;
    }
    jump_state.result = JUMP_RESULT_ABORTED;
    return 1U;
}

void jump_ctrl_emergency_stop(void)
{
    if (!jump_state.active)
    {
        return;
    }

    jump_state.active = 0U;
    jump_state.phase = JUMP_PHASE_IDLE;
    jump_state.result = JUMP_RESULT_EMERGENCY_STOP;
}

uint8 jump_ctrl_recover(void)
{
    jump_state.active = 0U;
    jump_state.phase = JUMP_PHASE_IDLE;
    jump_state.last_leg_status = leg_ctrl_recover();
    if (LEG_CTRL_STATUS_OK != jump_state.last_leg_status)
    {
        jump_state.result = JUMP_RESULT_LEG_ERROR;
        return 0U;
    }

    jump_state.phase_elapsed_ms = 0U;
    jump_state.total_elapsed_ms = 0U;
    jump_state.result = JUMP_RESULT_RECOVERED;
    return 1U;
}

uint8 jump_ctrl_is_active(void)
{
    return jump_state.active;
}

const jump_state_t *jump_ctrl_get_state(void)
{
    return &jump_state;
}
