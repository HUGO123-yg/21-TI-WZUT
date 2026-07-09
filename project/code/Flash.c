#include "zf_common_headfile.h"


// 用户设置的目标速度，仅由按键四调整
float user_set_speed = 200; // 初始目标速度为200

float Nav_read[Read_MaxSize]; // 按5cm算的话,1000可以跑50m
Nag N;


// ============== 多路径选择变量 ==============
uint8 Nag_PathSelect = 1;  // 默认选择路径1
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据路径编号获取起始页
//-------------------------------------------------------------------------------------------------------------------
static uint8 get_path_start_page(uint8 path_id)
{
    switch(path_id)
    {
        case 1:  return NAG_PATH1_START;
        case 2:  return NAG_PATH2_START;
        case 3:  return NAG_PATH3_START;
        default: return NAG_PATH1_START;
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据路径编号获取结束页(元数据页)
//-------------------------------------------------------------------------------------------------------------------
static uint8 get_path_end_page(uint8 path_id)
{
    switch(path_id)
    {
        case 1:  return NAG_PATH1_END;
        case 2:  return NAG_PATH2_END;
        case 3:  return NAG_PATH3_END;
        default: return NAG_PATH1_END;
    }
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按路径初始化惯导 (设置Flash_page_index为对应路径的起始页)
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag_Path(uint8 path_id)
{
    Nag_PathSelect = path_id;
    memset(&N, 0, sizeof(N));
    N.Flash_page_index = get_path_start_page(path_id);
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入元数据页 (将3条路径的Save_index都写入page 1)
// 备注信息     在元数据页中:
//               buffer[MaxSize+0] = 路径1的Save_index
//               buffer[MaxSize+1] = 路径2的Save_index
//               buffer[MaxSize+2] = 路径3的Save_index
//-------------------------------------------------------------------------------------------------------------------
void flash_Nag_Write_Meta(void)
{
    uint16 save_idx_1 = Get_Path_SaveIndex(1);
    uint16 save_idx_2 = Get_Path_SaveIndex(2);
    uint16 save_idx_3 = Get_Path_SaveIndex(3);
    
    flash_buffer_clear();
    flash_union_buffer[MaxSize + 0].uint32_type = save_idx_1;
    flash_union_buffer[MaxSize + 1].uint32_type = save_idx_2;
    flash_union_buffer[MaxSize + 2].uint32_type = save_idx_3;

    // 简化处理：当前路径的Save_index直接写入
    flash_union_buffer[MaxSize + (Nag_PathSelect - 1)].uint32_type = N.Save_index;

    if (flash_check(0, NAG_META_PAGE))
        flash_erase_page(0, NAG_META_PAGE);
    flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取元数据页，获取指定路径的Save_index
//-------------------------------------------------------------------------------------------------------------------
uint16 Get_Path_SaveIndex(uint8 path_id)
{
    if (path_id < 1 || path_id > 3) return 0;

    flash_buffer_clear();
    flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
    uint32 save_idx = flash_union_buffer[MaxSize + (path_id - 1)].uint32_type;
    flash_buffer_clear();
    if (save_idx == 0xFFFFFFFF || save_idx > Read_MaxSize) return 0;
    return (uint16)save_idx;
}




//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取偏航角的线程函数
// 参数说明     读取偏航角的线程函数，通过切换N.End_f来切换线程
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read()
{
    switch (N.End_f)
    {
    case 0:
        Run_Nag_Save(); // 默认执行函数
        break;
    case 1:
        flash_Nag_Write(); // 写入最后一页，保证falsh存储满
        N.End_f++;
        break;
    case 2:        
//      gpio_set_level(BUZZER_PIN,1);
        N.End_f++; // 结束线程
        break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用于生成偏差计算
// 参数说明     N.Final_Out为最终生成的偏差大小
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{
    Run_Nag_GPS();    // 偏航角读取复现
    if (N.Nag_Stop_f) // 防止旋转
    {
        N.Final_Out = 0;
        target_speed = 0;
        fuxian = 0;
        STOP_FALG=0;
        return;
    }
    N.Final_Out = angle_plan(Nag_Yaw - N.Angle_Run);
//      N.Final_Out = (Nag_Yaw - N.Angle_Run);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角存入
// 参数说明     将读取的YAW存储到flash中存储
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

//不必记录距离，只需要记录偏航角，距离以点位的形式记录
void Run_Nag_Save(void)
{
    N.Mileage_All += (R_Mileage + L_Mileage) * 0.5f; // 历程计读取，左右编码器，使用浮点数的话误差能保留下来
  
//    N.Mileage_All =Car.mileage;//里程计读取
    // printf("Mileage_All=%f\r\n", N.Mileage_All);
    
    if (N.size >= MaxSize) // 当前页存满后写入，保留页尾空间给元数据页使用
    {
        flash_Nag_Write();
        N.size = 0;                                   // 索引重置为0从下一个缓冲区开始读取
        zf_assert(N.Flash_page_index > get_path_end_page(Nag_PathSelect)); // 防止写穿当前路径分区
        N.Flash_page_index--;                         // flash页面索引减小
    }

    if (N.Mileage_All >= Nag_Set_mileage) // 每隔Nag_Set_mileage记一次
    {
        int32 Save = (int32)(Nag_Yaw * 100);            // 读取的偏航角放大100倍，避免使用Float类型来存储
        flash_union_buffer[N.size++].int32_type = Save; // 将偏航角写入缓冲区
        N.Save_index++;
        // printf("Save=%f\r\n", (float)Save / 100.0f);
        
        
        if (N.Mileage_All > 0)  //5CM为一个周期，但是一个周期确不一定只跑了5CM,所以有余数处理
            N.Mileage_All -= Nag_Set_mileage; // 重置历程计数字//保存到flash
        else
            N.Mileage_All += Nag_Set_mileage; // 倒车
    }
}
// 偏航角复现
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角复现
// 参数说明     读取flash中存储的YAW
// 返回参数     void
// 使用示例     用户无需调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS(void)
{
    N.Mileage_All += (R_Mileage + L_Mileage) * 0.5f; // 历程计读取，左右编码器，使用浮点数的话误差能保留下来
    uint16 prospect = 0;
    if (N.Mileage_All >= Nag_Set_mileage)
    {
        if (N.Run_index > N.Save_index - 2)
        {
            N.Nag_Stop_f++;
            return;
        }
        N.Run_index++; // 如果需要跑两圈可以直接把这个赋值为0.

        prospect = N.Run_index; // 前瞻
        if (prospect > N.Save_index - 2)
            prospect = N.Save_index - 2;             // 越界保护
        N.Angle_Run = (Nav_read[prospect] / 100.0f); // 读取的偏航角复现，除以100还原
        // printf("N.Angle_Run=%f,N.Save_index=%d, N.Flash_page_index=%d,N.Nag_Stop_f=%d,N.Run_index=%d\r\n", N.Angle_Run, N.Save_index, N.Flash_page_index, N.Nag_Stop_f, N.Run_index);
        if (N.Mileage_All > 0)
            N.Mileage_All -= Nag_Set_mileage; // 重置历程计数字//保存到flash
        else
            N.Mileage_All += Nag_Set_mileage; // 倒车
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯导参数初始化
// 返回参数     void
// 使用示例     放入程序执行开始
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag(void)
{
    memset(&N, 0, sizeof(N));
    N.Flash_page_index = Nag_Start_Page;
    flash_buffer_clear();
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航执行函数
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     放入中断中
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_System(void)
{
    // 卫保护
    if (!N.Nag_SystemRun_Index || N.Nag_Stop_f)
        return;

    switch (N.Nag_SystemRun_Index)
    {
    case 1:
        Nag_Read(); // 1是读取
        break;
    case 2:
        fuxian = 1;
        target_speed = user_set_speed; // 复现时使用用户设置的目标速度
        NagFlashRead();
        break;
    case 3:
        Nag_Run();
        break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一次性读取程序，只读取一次！
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     放入主函数直接调用，demo中有示例。
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void NagFlashRead(void)
{
    if (N.Save_state)
        return;
    flash_Nag_Read();
    uint8 page_trun = 0;

    for (int index = 0; index <= N.Save_index; index++)
    {
        if (index >= N.Save_index)
        {
            N.Save_state = 1;
            break;
        }
        int temp_index = index - (MaxSize * page_trun);
        if (temp_index >= MaxSize) // 当前页读取完毕，切换到下一页
        {
            N.Flash_page_index--; // 页面减少
            page_trun++;
            flash_Nag_Read(); // 重新读取
            temp_index = index - (MaxSize * page_trun);
        }
        Nav_read[index] = flash_union_buffer[temp_index].int32_type;
         printf("Nav_read=%f\r\n", Nav_read[index]);
    }
    N.Nag_SystemRun_Index++;
}

/**
 * @brief 按键一启动惯导录制，按键三中止录制，按键二启动惯导复现
 *N.Save_index = 0; // 索引重置，防止越界
 */
uint8 fuxian = 0;
void control_navigation(void)
{
    if (key1_flag == 1) // 按键1控制惯导启动与停止
    {
        N.Nag_SystemRun_Index = 1; // 启动惯导读取与运行
        key1_flag = 0;
    }
    if (key3_flag == 1 && N.Nag_SystemRun_Index == 1) // 按键3控制惯导读取与运行
    {
        N.End_f = 1; // 中止惯导运行，停止采集
        key3_flag = 0;
    }
    if (key2_flag == 1) // 按键2控制惯导参数初始化
    {
        N.Nag_SystemRun_Index = 2;     // 复现惯导
        fuxian = 1;                    // 轨迹环开启
        target_speed = user_set_speed; // 复现时使用用户设置的目标速度
        key2_flag = 0;
    }
    // 按键四控制目标速度调整，按一次增加50
    if (key4_flag == 1)
    {
        user_set_speed += 50;
        if (user_set_speed > 700)
            user_set_speed = 50; // 超过700回到50
        key4_flag = 0;
    }

    // if (N.Nag_SystemRun_Index == 2)
    // {
    //     NagFlashRead();
    // }
}


/**************************惯导存取Flash********************************/
void flash_Nag_Write(void)
{
  


    if (flash_check(0, N.Flash_page_index))
        flash_erase_page(0, N.Flash_page_index);

    flash_write_page_from_buffer(0, N.Flash_page_index, FLASH_PAGE_LENGTH);
     // 调试：打印缓冲区前5条数据
    for (int i = 0; i < 5 && i < N.size; i++) {
        printf("Before write: buffer[%d] = %d (angle=%.2f)\n", 
               i, flash_union_buffer[i].int32_type, 
               flash_union_buffer[i].int32_type / 100.0f);
    }
    printf("N.size=%d, N.Save_index=%d\n", N.size, N.Save_index);
    if (N.End_f == 1)
    {
        flash_Nag_Write_Meta();
    }
    
    flash_buffer_clear();
    gpio_set_level(BUZZER_PIN,1);
}

void flash_Nag_Read(void)
{
    flash_buffer_clear();
    N.Save_index = Get_Path_SaveIndex(Nag_PathSelect);
    if (flash_check(0, N.Flash_page_index))
    {
        flash_read_page_to_buffer(0, N.Flash_page_index, FLASH_PAGE_LENGTH);
    }
}


/**
 * @brief 角度处理到-180~180度范围内
 *
 * @param angle 输入角度
 * @return double 处理后角度
 */
double angle_plan(double angle)
{
    while (angle > 180.0)
        angle -= 360.0;

    while (angle <= -180.0)
        angle += 360.0;

    return angle;
}
