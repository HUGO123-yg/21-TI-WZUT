#ifndef _vision_h_
#define _vision_h_

//=====包含头文件=====
#include "zf_common_headfile.h"
#include "ipc_protocol.h"

//=====二值化阈值宏定义=====
#define CCD_THRESHOLD_HIGH      (180)                           // 二值化高阈值
#define CCD_THRESHOLD_LOW       (60)                            // 二值化低阈值

//=====二值化取值宏定义=====
#define CCD_VALUE_BLACK         (0)                             // 黑色值
#define CCD_VALUE_EDGE          (1)                             // 边缘值
#define CCD_VALUE_WHITE         (2)                             // 白色值

//=====全局变量声明=====
extern volatile uint8 tsl1401_finish_flag;                      // CCD 采集完成标志
extern uint8 ccd_ternary[128];                                  // 二值化结果数组
extern float image_error_cm7_1;                                 // 图像偏差

//=====函数声明=====
void vision_init                (void);                         // 视觉模块初始化
void vision_ternarize           (void);                         // 图像二值化
void vision_calc_line_error     (void);                         // 计算赛道偏差
void vision_send_to_cm7_0       (void);                         // 发送数据到 CM7_0

#endif
