#include "first_order_filter.h"

void first_order_filter_init(casade_common_value_struct *filter_value, int16 *gyro_raw_data, int16 *acc_raw_data, float gyro_ration, float acc_ration, float call_cycle, float mechanical_zero)
{
    filter_value->gyro_raw_data = gyro_raw_data;
    filter_value->acc_raw_data = acc_raw_data;
    filter_value->gyro_ration = gyro_ration;
    filter_value->acc_ration = acc_ration;
    filter_value->call_cycle = call_cycle;
    filter_value->mechanical_zero = mechanical_zero;

    filter_value->filtering_angle = -mechanical_zero;
    filter_value->angle_temp = -mechanical_zero;
}

void first_order_filter_refresh(casade_common_value_struct *filter_value, int16 gyro_raw_data, int16 acc_raw_data)
{
    float gyro_temp;
    float acc_temp;

    gyro_temp = -gyro_raw_data * filter_value->gyro_ration;
    acc_temp = (acc_raw_data - filter_value->angle_temp) * filter_value->acc_ration;

    filter_value->angle_temp += ((gyro_temp + acc_temp) * filter_value->call_cycle);
    filter_value->filtering_angle = filter_value->angle_temp + filter_value->mechanical_zero;
}
