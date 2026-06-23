#ifndef _yaw_fusion_h_
#define _yaw_fusion_h_

#include "zf_common_headfile.h"

#define YAW_FUSION_KP_ANTENNA      0.08f
#define YAW_FUSION_KP_ANTENNA_HIGH 0.15f
#define YAW_FUSION_KP_MOTION       0.03f
#define YAW_FUSION_MIN_SPEED       1.0f
#define YAW_FUSION_UPDATE_MS       100
#define YAW_FUSION_STATIC_CALIB_MS 1500
#define YAW_FUSION_STATIC_SPEED_MAX 5.0f
#define YAW_FUSION_STATIC_GYRO_MAX 90
#define YAW_FUSION_STATIC_ACC_MIN  0.92f
#define YAW_FUSION_STATIC_ACC_MAX  1.08f
#define YAW_FUSION_GYRO_BIAS_MAX   300.0f

extern float yaw_fusion_gyro_bias;

void yaw_fusion_init(void);
void yaw_fusion_calibrate_gyro_bias(int16 gyro_z_raw);
void yaw_fusion_update(void);
bool yaw_fusion_is_calibrated(void);
bool yaw_fusion_is_gyro_bias_ready(void);

#endif
