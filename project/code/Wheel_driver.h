#ifndef PROJECT_WHEEL_DRIVER_H
#define PROJECT_WHEEL_DRIVER_H

#include "zf_common_typedef.h"

typedef struct
{
    int16 left_rpm;
    int16 right_rpm;
    uint32 valid_frame_count;
    uint32 invalid_frame_count;
} wheel_feedback_t;

void wheel_driver_init(void);
void wheel_driver_uart_isr(void);
void wheel_driver_set_command(int16 left_command, int16 right_command);
void wheel_driver_stop(void);
void wheel_driver_request_speed(void);
void wheel_driver_get_feedback(wheel_feedback_t *feedback);

#endif
