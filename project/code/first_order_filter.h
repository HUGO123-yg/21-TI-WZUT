#ifndef __FIRST_ORDER_FILTER_H__
#define __FIRST_ORDER_FILTER_H__

#include "zf_common_typedef.h"

// 注意：此类型名 casade_common_value_struct（少一个 'c'）为 first_order_filter 模块专用，
// 与 Imu.h 中的 cascade_common_value_struct（拼写不同）是完全不同的类型，请勿混用。
typedef struct
{
    int16 *gyro_raw_data;
    int16 *acc_raw_data;
    float gyro_ration;
    float angle_temp;
    float acc_ration;
    float call_cycle;
    float filtering_angle;
    float mechanical_zero;
} casade_common_value_struct;

void first_order_filter_init(casade_common_value_struct *filter_value, int16 *gyro_raw_data, int16 *acc_raw_data, float gyro_ration, float acc_ration, float call_cycle, float mechanical_zero);
void first_order_filter_refresh(casade_common_value_struct *filter_value, int16 gyro_raw_data, int16 acc_raw_data);

#endif
