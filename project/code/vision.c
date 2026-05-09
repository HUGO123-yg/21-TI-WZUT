#include "vision.h"
#include "zf_driver_ipc.h"

//=====全局变量定义=====
uint8 ccd_ternary[128] = {0};                                   // 二值化结果数组
float image_error_cm7_1 = 0.0f;                                 // 图像偏差

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       vision_init
// 功能说明       视觉模块初始化
// 输入参数       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void vision_init(void)
{
    tsl1401_init();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       vision_ternarize
// 功能说明       图像二值化处理：双阈值分类（黑/边缘/白）
//               像素值 > CCD_THRESHOLD_HIGH → 白色(2)
//               像素值 < CCD_THRESHOLD_LOW  → 黑色(0)
//               否则                       → 边缘(1)
// 输入参数       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void vision_ternarize(void)
{
    uint8 i;
    uint8 pixel_value;

    for(i = 0; i < 128; i++)
    {
        pixel_value = (uint8)tsl1401_data[0][i];

        if(pixel_value > CCD_THRESHOLD_HIGH)
        {
            ccd_ternary[i] = CCD_VALUE_WHITE;
        }
        else if(pixel_value < CCD_THRESHOLD_LOW)
        {
            ccd_ternary[i] = CCD_VALUE_BLACK;
        }
        else
        {
            ccd_ternary[i] = CCD_VALUE_EDGE;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       vision_calc_line_error
// 功能说明       计算赛道偏差：加权质心法，边缘/白色像素加权计算中心位置
//               center = sum(i * ccd_ternary[i]) / sum(ccd_ternary[i])
//               image_error_cm7_1 = center - 64.0f（正值表示赛道偏右）
// 输入参数       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void vision_calc_line_error(void)
{
    uint8 i;
    float weighted_sum = 0.0f;
    float weight_sum = 0.0f;

    for(i = 0; i < 128; i++)
    {
        if(ccd_ternary[i] >= CCD_VALUE_EDGE)
        {
            weighted_sum += (float)(i * ccd_ternary[i]);
            weight_sum += (float)ccd_ternary[i];
        }
    }

    if(weight_sum > 0.0f)
    {
        image_error_cm7_1 = (weighted_sum / weight_sum) - 64.0f;
    }
    else
    {
        image_error_cm7_1 = 0.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       vision_send_to_cm7_0
// 功能说明       发送数据到 CM7_0
// 输入参数       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void vision_send_to_cm7_0(void)
{
    ipc_send_data(IPC_VISION_PACK(image_error_cm7_1, 255));
}
