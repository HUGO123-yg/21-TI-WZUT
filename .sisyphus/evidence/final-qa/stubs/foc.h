// QA 5 stub: foc.h for standalone compilation testing
// Mirrors the real foc.h but uses stub includes for standalone cc compilation

#ifndef _foc_h_
#define _foc_h_

#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"

extern small_device_value_struct motor_value;

void foc_set_duty(int16 left_duty, int16 right_duty);

#endif
