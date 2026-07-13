#ifndef PROJECT_JUMP_H
#define PROJECT_JUMP_H

#include "Leg_ctrl.h"
#include "zf_common_typedef.h"

typedef enum
{
    JUMP_PHASE_IDLE = 0,
    JUMP_PHASE_EXTEND,
    JUMP_PHASE_RETRACT,
    JUMP_PHASE_BUFFER
} jump_phase_t;

typedef enum
{
    JUMP_RESULT_IDLE = 0,
    JUMP_RESULT_RUNNING,
    JUMP_RESULT_COMPLETED,
    JUMP_RESULT_ABORTED,
    JUMP_RESULT_NOT_READY,
    JUMP_RESULT_LEG_ERROR
} jump_result_t;

typedef struct
{
    uint8 active;
    jump_phase_t phase;
    jump_result_t result;
    uint32 phase_elapsed_ms;
    uint32 total_elapsed_ms;
    uint32 completed_count;
    leg_ctrl_status_t last_leg_status;
} jump_state_t;

void jump_ctrl_init(void);
uint8 jump_ctrl_start(void);
void jump_ctrl_tick_1ms(void);
uint8 jump_ctrl_abort(void);
uint8 jump_ctrl_is_active(void);
const jump_state_t *jump_ctrl_get_state(void);

#endif
