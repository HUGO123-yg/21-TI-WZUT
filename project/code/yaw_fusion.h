#ifndef _yaw_fusion_h_
#define _yaw_fusion_h_

#include "zf_common_headfile.h"

#define YAW_FUSION_KP_ANTENNA      0.08f
#define YAW_FUSION_KP_MOTION       0.03f
#define YAW_FUSION_MIN_SPEED       1.0f
#define YAW_FUSION_UPDATE_MS       100

void yaw_fusion_init(void);
void yaw_fusion_update(void);
bool yaw_fusion_is_calibrated(void);

#endif
