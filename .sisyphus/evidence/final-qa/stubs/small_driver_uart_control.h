#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"

typedef int uart_index_enum;

typedef struct
{
    uart_index_enum driver_uart;
    unsigned char send_data_buffer[7];
    unsigned char receive_data_buffer[7];
    unsigned char receive_data_count;
    unsigned char sum_check_data;
    short int left_motor_dir;
    short int right_motor_dir;
    short int receive_left_speed_data;
    short int receive_right_speed_data;
    float receive_left_angle_data;
    float receive_right_angle_data;
    float receive_left_location_data;
    float receive_right_location_data;
} small_device_value_struct;

extern small_device_value_struct small_driver_value;
void small_driver_set_duty(small_device_value_struct *driver_value, int left_duty, int right_duty);

#endif
