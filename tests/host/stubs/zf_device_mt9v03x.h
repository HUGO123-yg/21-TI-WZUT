#ifndef TEST_ZF_DEVICE_MT9V03X_H
#define TEST_ZF_DEVICE_MT9V03X_H

#include "zf_common_typedef.h"

#define MT9V03X_W              (188U)
#define MT9V03X_H              (120U)
#define MT9V03X_AUTO_EXP_DEF   (0U)
#define MT9V03X_EXP_TIME_DEF   (400U)

extern volatile uint8 mt9v03x_finish_flag;
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];

uint8 mt9v03x_set_exposure_time(uint16 light);
uint8 mt9v03x_init(void);

#endif
