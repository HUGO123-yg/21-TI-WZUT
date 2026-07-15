#ifndef PROJECT_WHEEL_DRIVER_H
#define PROJECT_WHEEL_DRIVER_H

#include "zf_common_typedef.h"

typedef struct
{
    int16 left_rpm;
    int16 right_rpm;
    uint32 valid_frame_count;
    uint32 invalid_frame_count;
    uint32 transmitted_frame_count;
    uint32 dropped_tx_frame_count;
} wheel_feedback_t;

void wheel_driver_init(void);
// Verifies that all logical-to-hardware direction values are exactly +/-1 and
// that the basic driver limits are usable. Invalid configuration locks output.
uint8 wheel_driver_config_is_valid(void);
void wheel_driver_uart_isr(void);
void wheel_driver_set_command(int16 left_command, int16 right_command);
void wheel_driver_stop(void);
// A locked driver rejects all non-zero duty commands until explicitly released.
void wheel_driver_set_stop_lock(uint8 locked);
uint8 wheel_driver_is_stop_locked(void);
void wheel_driver_request_speed(void);
void wheel_driver_get_feedback(wheel_feedback_t *feedback);

#endif
