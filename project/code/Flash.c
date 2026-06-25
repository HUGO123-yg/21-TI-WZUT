#include "zf_common_headfile.h"


// 用户设置的目标速度，回放时按此速度行驶
float user_set_speed = 200; // 初始目标速度为200

float Nav_read[Read_MaxSize]; // 以5cm为单位的回放缓冲，1000个点对应50m
Nag N;

static volatile uint8 nag_flash_write_pending = 0;
static volatile uint8 nag_flash_read_pending  = 0;
static volatile uint8 nag_flash_write_final   = 0;
static volatile uint8 nag_flash_read_active   = 0;
static uint8 nag_read_page                    = 0;
static uint8 nag_read_end_page                = 0;
static uint16 nag_read_count                  = 0;
static uint16 nag_read_copied                 = 0;


// ============== 路径选择相关 ==============
uint8 Nag_PathSelect = 1;  // 默认选择路径1
//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     按路径编号获取起始页
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
// 函数功能：     按路径编号获取结束页
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

static uint16 get_path_capacity(uint8 path_id)
{
    uint8 start_page = get_path_start_page(path_id);
    uint8 end_page   = get_path_end_page(path_id);
    uint16 capacity;

    if (start_page < end_page)
        return 0;

    capacity = (uint16)(start_page - end_page + 1) * NAG_POINTS_PER_PAGE;
    if (capacity > Read_MaxSize)
        capacity = Read_MaxSize;

    return capacity;
}

static uint16 nag_clamp_save_index(uint8 path_id, uint32 save_index)
{
    uint16 path_capacity = get_path_capacity(path_id);

    if (save_index == 0xFFFFFFFFu)
        return 0;

    if (save_index > path_capacity)
        return path_capacity;

    return (uint16)save_index;
}

static uint32 nag_meta_checksum(uint16 save_idx_1, uint16 save_idx_2, uint16 save_idx_3)
{
    uint32 checksum = NAG_META_MAGIC ^ NAG_META_VERSION ^ (uint32)Nag_Set_mileage;

    checksum ^= ((uint32)save_idx_1 << 16) | save_idx_2;
    checksum ^= ((uint32)save_idx_3 << 8);
    checksum ^= 0x5A5AA5A5u;

    return checksum;
}

static uint8 nag_meta_is_valid(void)
{
    uint32 save_idx_1 = flash_union_buffer[NAG_META_SAVE_INDEX_OFFSET + NAG_PATH1_META_SLOT].uint32_type;
    uint32 save_idx_2 = flash_union_buffer[NAG_META_SAVE_INDEX_OFFSET + NAG_PATH2_META_SLOT].uint32_type;
    uint32 save_idx_3 = flash_union_buffer[NAG_META_SAVE_INDEX_OFFSET + NAG_PATH3_META_SLOT].uint32_type;
    uint32 checksum   = flash_union_buffer[NAG_META_CHECKSUM_OFFSET].uint32_type;

    if (flash_union_buffer[NAG_META_MAGIC_OFFSET].uint32_type != NAG_META_MAGIC)
        return 0;

    if (flash_union_buffer[NAG_META_VERSION_OFFSET].uint32_type != NAG_META_VERSION)
        return 0;

    if (flash_union_buffer[NAG_META_SAMPLE_CM_OFFSET].uint32_type != Nag_Set_mileage)
        return 0;

    if (save_idx_1 > get_path_capacity(1) ||
        save_idx_2 > get_path_capacity(2) ||
        save_idx_3 > get_path_capacity(3))
    {
        return 0;
    }

    return checksum == nag_meta_checksum((uint16)save_idx_1, (uint16)save_idx_2, (uint16)save_idx_3);
}

static void nag_read_meta_values(uint16 save_index[NAG_PATH_COUNT])
{
    uint8 index;
    uint8 has_new_meta;
    uint8 meta_valid;

    for (index = 0; index < NAG_PATH_COUNT; index++)
    {
        save_index[index] = 0;
    }

    flash_buffer_clear();
    flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);

    has_new_meta = (flash_union_buffer[NAG_META_MAGIC_OFFSET].uint32_type != 0xFFFFFFFFu);
    meta_valid   = nag_meta_is_valid();
    if (has_new_meta && !meta_valid)
    {
        flash_buffer_clear();
        return;
    }

    for (index = 0; index < NAG_PATH_COUNT; index++)
    {
        save_index[index] = nag_clamp_save_index((uint8)(index + 1),
            flash_union_buffer[NAG_META_SAVE_INDEX_OFFSET + index].uint32_type);
    }

    flash_buffer_clear();
}

static void nag_write_meta_values(const uint16 save_index[NAG_PATH_COUNT])
{
    flash_buffer_clear();
    flash_union_buffer[NAG_META_SAVE_INDEX_OFFSET + NAG_PATH1_META_SLOT].uint32_type = save_index[NAG_PATH1_META_SLOT];
    flash_union_buffer[NAG_META_SAVE_INDEX_OFFSET + NAG_PATH2_META_SLOT].uint32_type = save_index[NAG_PATH2_META_SLOT];
    flash_union_buffer[NAG_META_SAVE_INDEX_OFFSET + NAG_PATH3_META_SLOT].uint32_type = save_index[NAG_PATH3_META_SLOT];
    flash_union_buffer[NAG_META_MAGIC_OFFSET].uint32_type     = NAG_META_MAGIC;
    flash_union_buffer[NAG_META_VERSION_OFFSET].uint32_type   = NAG_META_VERSION;
    flash_union_buffer[NAG_META_SAMPLE_CM_OFFSET].uint32_type = Nag_Set_mileage;
    flash_union_buffer[NAG_META_CHECKSUM_OFFSET].uint32_type  = nag_meta_checksum(save_index[NAG_PATH1_META_SLOT],
        save_index[NAG_PATH2_META_SLOT],
        save_index[NAG_PATH3_META_SLOT]);

    if (flash_check(0, NAG_META_PAGE))
        flash_erase_page(0, NAG_META_PAGE);
    flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

static void nag_request_flash_write(uint8 is_final)
{
    if (nag_flash_write_pending)
        return;

    nag_flash_write_final    = is_final;
    nag_flash_write_pending  = 1;
}

static void nag_request_flash_read(void)
{
    if (nag_flash_read_pending || nag_flash_read_active || N.Save_state)
        return;

    nag_flash_read_pending = 1;
}

static void nag_stop_replay(void)
{
    N.Nag_Stop_f = true;
    N.Final_Out  = 0.0f;
    target_speed = 0.0f;
    fuxian       = 0;
    STOP_FLAG    = 0;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     按路径编号初始化惯导 (设置Flash_page_index为对应路径的起始页)
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag_Path(uint8 path_id)
{
    Nag_PathSelect = path_id;
    memset(&N, 0, sizeof(N));
    N.Flash_page_index = get_path_start_page(path_id);
    nag_flash_write_pending = 0;
    nag_flash_read_pending  = 0;
    nag_flash_write_final   = 0;
    nag_flash_read_active   = 0;
    nag_read_page           = 0;
    nag_read_end_page       = 0;
    nag_read_count          = 0;
    nag_read_copied         = 0;
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     写入元数据页 (把3条路径的Save_index写入page 1)
// 备注信息：     元数据页布局:
    //               buffer[NAG_META_SAVE_INDEX_OFFSET+0] = 路径1的Save_index
    //               buffer[NAG_META_SAVE_INDEX_OFFSET+1] = 路径2的Save_index
    //               buffer[NAG_META_SAVE_INDEX_OFFSET+2] = 路径3的Save_index
//-------------------------------------------------------------------------------------------------------------------
void flash_Nag_Write_Meta(void)
{
    uint16 save_index[NAG_PATH_COUNT];

    nag_read_meta_values(save_index);

    if (Nag_PathSelect >= 1 && Nag_PathSelect <= NAG_PATH_COUNT)
    {
        save_index[Nag_PathSelect - 1] = nag_clamp_save_index(Nag_PathSelect, N.Save_index);
    }

    nag_write_meta_values(save_index);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     读取元数据页，读取指定路径的Save_index
//-------------------------------------------------------------------------------------------------------------------
uint16 Get_Path_SaveIndex(uint8 path_id)
{
    uint16 save_index[NAG_PATH_COUNT];

    if (path_id < 1 || path_id > NAG_PATH_COUNT)
        return 0;

    nag_read_meta_values(save_index);

    return save_index[path_id - 1];
}

void Nag_Clear_Path_Meta(uint8 path_id)
{
    uint16 save_index[NAG_PATH_COUNT];

    if (path_id < 1 || path_id > NAG_PATH_COUNT)
        return;

    nag_read_meta_values(save_index);
    save_index[path_id - 1] = 0;
    nag_write_meta_values(save_index);
}




//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     读取偏航角的线程函数
// 函数说明：     读取偏航角的线程函数，通过切换N.End_f进行切换线程
// 返回参数：     void
// 使用示例：     用户不要调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read()
{
    switch (N.End_f)
    {
    case 0:
        Run_Nag_Save(); // 默认执行记录函数
        break;
    case 1:
        nag_request_flash_write(1); // 写入最后一页，保证flash存储完整
        break;
    case 2:
//      gpio_set_level(BUZZER_PIN,1);
        N.End_f++; // 退出线程
        break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     执行回放偏差计算
// 函数说明：     N.Final_Out为回放生成的偏差角度
// 返回参数：     void
// 使用示例：     用户不要调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{
    Run_Nag_GPS();    // 偏航角度读取函数
    if (N.Nag_Stop_f) // 终止回放
    {
        N.Final_Out = 0;
        target_speed = 0;
        fuxian = 0;
        STOP_FLAG=0;
        return;
    }
    N.Final_Out = angle_plan(Nag_Yaw - N.Angle_Run);
//      N.Final_Out = (Nag_Yaw - N.Angle_Run);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     偏航角存储
// 函数说明：     将获取的YAW存储到flash中存储
// 返回参数：     void
// 使用示例：     用户不要调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

//关键记录里程，只需要记录偏航角，以离散单位形式记录
void Run_Nag_Save(void)
{
    uint16 path_capacity = get_path_capacity(Nag_PathSelect);

    N.Mileage_All += (R_Mileage + L_Mileage) * 0.5f; // 里程计取值，采用左右编码器平均值，该里程计能被编码器更新

    if (nag_flash_write_pending)
        return;

//    N.Mileage_All =Car.mileage;//里程计取值
    // printf("Mileage_All=%f\r\n", N.Mileage_All);

    if (N.Save_index >= path_capacity)
    {
        nag_request_flash_write(1);
        return;
    }

    if (N.size >= NAG_POINTS_PER_PAGE) // 缓存已满页中的flash大小时才写入一次，防止重复写入
    {
        nag_request_flash_write(0);
        return;
    }

    if (N.Mileage_All >= Nag_Set_mileage) // 每隔Nag_Set_mileage记录一次
    {
        int32 Save = (int32)(Nag_Yaw * 100);            // 获取的偏航角放大100倍，此处使用Float类型转换存储
        flash_union_buffer[N.size++].int32_type = Save; // 将偏航角写入缓冲区
        N.Save_index++;
        // printf("Save=%f\r\n", (float)Save / 100.0f);


        if (N.Mileage_All > 0)  //5CM为一个记录区间，接着下一个区间确保下一个只记录5CM,执行数值修正
            N.Mileage_All -= Nag_Set_mileage; // 减去走过里程计后//保存到flash
        else
            N.Mileage_All += Nag_Set_mileage; // 修正

        if (N.Save_index >= path_capacity)
        {
            nag_request_flash_write(1);
        }
        else if (N.size >= NAG_POINTS_PER_PAGE)
        {
            nag_request_flash_write(0);
        }
    }
}
// 偏航角跟随
//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     偏航角跟随
// 函数说明：     读取flash中存储的YAW
// 返回参数：     void
// 使用示例：     用户不要调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS(void)
{
    N.Mileage_All += (R_Mileage + L_Mileage) * 0.5f; // 里程计取值，采用左右编码器平均值，该里程计能被编码器更新
    uint16 prospect = 0;

    if (N.Save_index < 2)
    {
        nag_stop_replay();
        return;
    }

    if (N.Mileage_All >= Nag_Set_mileage)
    {
        if (N.Run_index > N.Save_index - 2)
        {
            nag_stop_replay();
            return;
        }
        N.Run_index++; // 无需需要精确圈数，直接把运行赋值为0.

        prospect = N.Run_index; // 前瞻
        if (prospect > N.Save_index - 2)
            prospect = N.Save_index - 2;             // 越界保护
        N.Angle_Run = (Nav_read[prospect] / 100.0f); // 读取的偏航角跟随，除以100还原
        // printf("N.Angle_Run=%f,N.Save_index=%d, N.Flash_page_index=%d,N.Nag_Stop_f=%d,N.Run_index=%d\r\n", N.Angle_Run, N.Save_index, N.Flash_page_index, N.Nag_Stop_f, N.Run_index);
        if (N.Mileage_All > 0)
            N.Mileage_All -= Nag_Set_mileage; // 减去走过里程计后//保存到flash
        else
            N.Mileage_All += Nag_Set_mileage; // 修正
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     惯导系统初始化
// 返回参数：     void
// 使用示例：     主函数中执行开始
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag(void)
{
    memset(&N, 0, sizeof(N));
    N.Flash_page_index = Nag_Start_Page;
    flash_buffer_clear();
}
//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     惯导控制执行函数
// 函数说明：     index           索引
// 函数说明：     type            设置值
// 返回参数：     void
// 使用示例：     中断中调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_System(void)
{
    // 保护判断
    if (!N.Nag_SystemRun_Index || N.Nag_Stop_f)
        return;

    switch (N.Nag_SystemRun_Index)
    {
    case NAG_RUN_RECORD:
        Nag_Read(); // 1是读取
        break;
    case NAG_RUN_PRELOAD:
        nag_request_flash_read();
        break;
    case NAG_RUN_REPLAY:
        Nag_Run();
        break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：     一次性读取（只读取一次）
// 函数说明：     index           索引
// 函数说明：     type            设置值
// 返回参数：     void
// 使用示例：     惯导控制中直接调用，demo中示例函数
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void NagFlashRead(void)
{
    uint16 path_capacity;

    if (N.Save_state)
        return;

    N.Save_index = Get_Path_SaveIndex(Nag_PathSelect);
    if (N.Save_index < 2)
    {
        N.Save_state = 1;
        N.Nag_SystemRun_Index = NAG_RUN_IDLE;
        nag_stop_replay();
        return;
    }

    path_capacity = get_path_capacity(Nag_PathSelect);
    if (N.Save_index > path_capacity)
    {
        N.Save_index = path_capacity;
    }

    nag_read_count    = (N.Save_index > Read_MaxSize) ? Read_MaxSize : N.Save_index;
    nag_read_copied   = 0;
    nag_read_page     = get_path_start_page(Nag_PathSelect);
    nag_read_end_page = get_path_end_page(Nag_PathSelect);
    nag_flash_read_active = 1;
}

static void nag_finish_flash_read(void)
{
    nag_flash_read_active = 0;
    N.Save_index = nag_read_copied;
    N.Save_state = 1;

    if (N.Save_index >= 2)
    {
        N.Run_index           = 0;
        N.Mileage_All         = 0.0f;
        N.Final_Out           = 0.0f;
        N.Nag_Stop_f          = false;
        N.Nag_SystemRun_Index = NAG_RUN_REPLAY;
        fuxian                = 1;
        target_speed          = user_set_speed;
    }
    else
    {
        N.Nag_SystemRun_Index = NAG_RUN_IDLE;
        nag_stop_replay();
    }
}

static void nag_flash_read_step(void)
{
    uint16 page_index = 0;

    if (!nag_flash_read_active)
        return;

    if ((nag_read_copied >= nag_read_count) || (nag_read_page < nag_read_end_page))
    {
        nag_finish_flash_read();
        return;
    }

    flash_buffer_clear();
    if (!flash_check(0, nag_read_page))
    {
        nag_finish_flash_read();
        return;
    }

    flash_read_page_to_buffer(0, nag_read_page, FLASH_PAGE_LENGTH);

    while ((page_index < NAG_POINTS_PER_PAGE) && (nag_read_copied < nag_read_count))
    {
        Nav_read[nag_read_copied++] = flash_union_buffer[page_index++].int32_type;
    }

    if (nag_read_copied >= nag_read_count)
    {
        nag_finish_flash_read();
        return;
    }

    if (nag_read_page > nag_read_end_page)
    {
        nag_read_page--;
    }
}

/**
 * @brief 开启一次惯导记录，完成后终止记录，完成后启动惯导回放
 *N.Save_index = 0; // 调试用，防止越界
 */
uint8 fuxian = 0;
void control_navigation(void)
{
    if (key1_flag == 1) // 按键1控制惯导开始或停止
    {
        N.Nag_SystemRun_Index = NAG_RUN_RECORD; // 开启惯导读取记录
        key1_flag = 0;
    }
    if (key3_flag == 1 && N.Nag_SystemRun_Index == NAG_RUN_RECORD) // 按键3控制惯导读取记录
    {
        N.End_f = 1; // 终止惯导进行，停止采集
        key3_flag = 0;
    }
    if (key2_flag == 1) // 按键2控制惯导回放开始
    {
        Init_Nag_Path(Nag_PathSelect);
        N.Nag_SystemRun_Index = NAG_RUN_PRELOAD; // 开启惯导
        fuxian = 0;                    // 轨迹清零
        target_speed = 0;              // 路径读取完成之后再赋值
        key2_flag = 0;
    }
    // 按键4控制目标目标速度的递增，每按键一次递增50
    if (key4_flag == 1)
    {
        user_set_speed += 50;
        if (user_set_speed > 700)
            user_set_speed = 50; // 超过700回到50
        key4_flag = 0;
    }

    // if (N.Nag_SystemRun_Index == NAG_RUN_PRELOAD)
    // {
    //     NagFlashRead();
    // }
}

void Nag_Service(void)
{
    if (nag_flash_write_pending)
    {
        uint8 is_final = nag_flash_write_final;
        uint8 end_page = get_path_end_page(Nag_PathSelect);
        uint8 wrote_page = 0;

        if (N.size > 0)
        {
            flash_Nag_Write();
            wrote_page = 1;
        }

        if (is_final && (!wrote_page || N.End_f != 1))
        {
            flash_Nag_Write_Meta();
        }

        if (is_final)
        {
            N.size                = 0;
            N.End_f               = 2;
            N.Nag_SystemRun_Index = NAG_RUN_IDLE;
            flash_buffer_clear();
        }
        else if (N.Flash_page_index <= end_page)
        {
            N.size                = 0;
            N.End_f               = 2;
            N.Nag_SystemRun_Index = NAG_RUN_IDLE;
            N.Nag_Stop_f          = true;
            flash_Nag_Write_Meta();
            flash_buffer_clear();
        }
        else
        {
            N.size = 0;
            N.Flash_page_index--;
            flash_buffer_clear();
        }

        nag_flash_write_final   = 0;
        nag_flash_write_pending = 0;
    }

    if (nag_flash_read_pending)
    {
        NagFlashRead();
        nag_flash_read_pending = 0;
    }

    if (nag_flash_read_active)
    {
        nag_flash_read_step();
    }
}


/**************************惯导读取Flash********************************/
void flash_Nag_Write(void)
{
    if (N.size == 0)
        return;

    if (flash_check(0, N.Flash_page_index))
        flash_erase_page(0, N.Flash_page_index);

    flash_write_page_from_buffer(0, N.Flash_page_index, FLASH_PAGE_LENGTH);
    if (N.End_f == 1)
    {
        flash_Nag_Write_Meta();
    }

    flash_buffer_clear();
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
 * @brief 角度处理至-180~180度范围内
 *
 * @param angle 输入角度
 * @return double 返回角度
 */
float angle_plan(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle <= -180.0f)
        angle += 360.0f;

    return angle;
}
