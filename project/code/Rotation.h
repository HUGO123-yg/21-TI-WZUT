#ifndef PROJECT_CODE_ROTATION_H_
#define PROJECT_CODE_ROTATION_H_

#include "zf_common_typedef.h"

typedef enum
{
    ROT_CW = 0,
    ROT_CCW
} rotation_dir_enum;

typedef enum
{
    ROT_IDLE = 0,
    ROT_RUNNING,
    ROT_DONE
} rotation_state_enum;

typedef struct
{
    rotation_state_enum state;
    rotation_dir_enum dir;
    int16 turn_duty;
    uint32 elapsed_ms;
} rotation_control_struct;

extern volatile rotation_control_struct rotation;

// Starts one fixed-duration rotation action. Duration and duty are configured
// in config.h; returns 0 while a previous action still owns motor output.
uint8 rotation_start(rotation_dir_enum dir);

// Called once per millisecond after the latest attitude/safety update.
void rotation_run(void);

// Releases motor-differential ownership and returns to normal navigation.
void rotation_stop(void);

uint8 rotation_is_active(void);
uint8 rotation_is_done(void);

// DONE remains latched and keeps navigation differential suppressed until stop.
uint8 rotation_owns_output(void);

const char *rotation_state_name(rotation_state_enum state);

#endif
