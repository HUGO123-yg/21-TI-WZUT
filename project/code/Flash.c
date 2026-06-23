#include "zf_common_headfile.h"


// �û����õ�Ŀ���ٶȣ����ɰ����ĵ���
float user_set_speed = 200; // ��ʼĿ���ٶ�Ϊ200

float Nav_read[Read_MaxSize]; // ��5cm��Ļ�,1000������50m
Nag N;

static volatile uint8 nag_flash_write_pending = 0;
static volatile uint8 nag_flash_read_pending  = 0;
static volatile uint8 nag_flash_write_final   = 0;
static volatile uint8 nag_flash_read_active   = 0;
static uint8 nag_read_page                    = 0;
static uint8 nag_read_end_page                = 0;
static uint16 nag_read_count                  = 0;
static uint16 nag_read_copied                 = 0;


// ============== ��·��ѡ����� ==============
uint8 Nag_PathSelect = 1;  // Ĭ��ѡ��·��1
//-------------------------------------------------------------------------------------------------------------------
// �������     ����·����Ż�ȡ��ʼҳ
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
// �������     ����·����Ż�ȡ����ҳ
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
// �������     ��·����ʼ���ߵ� (����Flash_page_indexΪ��Ӧ·������ʼҳ)
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
// �������     д��Ԫ����ҳ (��3��·����Save_index��д��page 1)
// ��ע��Ϣ     ��Ԫ����ҳ��:
    //               buffer[NAG_META_SAVE_INDEX_OFFSET+0] = ·��1��Save_index
    //               buffer[NAG_META_SAVE_INDEX_OFFSET+1] = ·��2��Save_index
    //               buffer[NAG_META_SAVE_INDEX_OFFSET+2] = ·��3��Save_index
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
// �������     ��ȡԪ����ҳ����ȡָ��·����Save_index
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
// �������     ��ȡƫ���ǵ��̺߳���
// ����˵��     ��ȡƫ���ǵ��̺߳�����ͨ���л�N.End_f���л��߳�
// ���ز���     void
// ʹ��ʾ��     �û��������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read()
{
    switch (N.End_f)
    {
    case 0:
        Run_Nag_Save(); // Ĭ��ִ�к���
        break;
    case 1:
        nag_request_flash_write(1); // д�����һҳ����֤falsh�洢��
        break;
    case 2:
//      gpio_set_level(BUZZER_PIN,1);
        N.End_f++; // �����߳�
        break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ƫ�����
// ����˵��     N.Final_OutΪ�������ɵ�ƫ���С
// ���ز���     void
// ʹ��ʾ��     �û��������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{
    Run_Nag_GPS();    // ƫ���Ƕ�ȡ����
    if (N.Nag_Stop_f) // ��ֹ��ת
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
// �������     ƫ���Ǵ���
// ����˵��     ����ȡ��YAW�洢��flash�д洢
// ���ز���     void
// ʹ��ʾ��     �û��������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------

//���ؼ�¼���룬ֻ��Ҫ��¼ƫ���ǣ������Ե�λ����ʽ��¼
void Run_Nag_Save(void)
{
    uint16 path_capacity = get_path_capacity(Nag_PathSelect);

    N.Mileage_All += (R_Mileage + L_Mileage) * 0.5f; // ���̼ƶ�ȡ�����ұ�������ʹ�ø������Ļ�����ܱ�������

    if (nag_flash_write_pending)
        return;

//    N.Mileage_All =Car.mileage;//��̼ƶ�ȡ
    // printf("Mileage_All=%f\r\n", N.Mileage_All);

    if (N.Save_index >= path_capacity)
    {
        nag_request_flash_write(1);
        return;
    }

    if (N.size >= NAG_POINTS_PER_PAGE) // ��������ҳ�е�flash��С��ʱ��д��һ�Σ���ֹ�ظ�д��
    {
        nag_request_flash_write(0);
        return;
    }

    if (N.Mileage_All >= Nag_Set_mileage) // ÿ��Nag_Set_mileage��һ��
    {
        int32 Save = (int32)(Nag_Yaw * 100);            // ��ȡ��ƫ���ǷŴ�100��������ʹ��Float�������洢
        flash_union_buffer[N.size++].int32_type = Save; // ��ƫ����д�뻺����
        N.Save_index++;
        // printf("Save=%f\r\n", (float)Save / 100.0f);


        if (N.Mileage_All > 0)  //5CMΪһ�����ڣ�����һ������ȷ��һ��ֻ����5CM,��������������
            N.Mileage_All -= Nag_Set_mileage; // �������̼�����//���浽flash
        else
            N.Mileage_All += Nag_Set_mileage; // ����

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
// ƫ���Ǹ���
//-------------------------------------------------------------------------------------------------------------------
// �������     ƫ���Ǹ���
// ����˵��     ��ȡflash�д洢��YAW
// ���ز���     void
// ʹ��ʾ��     �û��������
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS(void)
{
    N.Mileage_All += (R_Mileage + L_Mileage) * 0.5f; // ���̼ƶ�ȡ�����ұ�������ʹ�ø������Ļ�����ܱ�������
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
        N.Run_index++; // �����Ҫ����Ȧ����ֱ�Ӱ������ֵΪ0.

        prospect = N.Run_index; // ǰհ
        if (prospect > N.Save_index - 2)
            prospect = N.Save_index - 2;             // Խ�籣��
        N.Angle_Run = (Nav_read[prospect] / 100.0f); // ��ȡ��ƫ���Ǹ��֣�����100��ԭ
        // printf("N.Angle_Run=%f,N.Save_index=%d, N.Flash_page_index=%d,N.Nag_Stop_f=%d,N.Run_index=%d\r\n", N.Angle_Run, N.Save_index, N.Flash_page_index, N.Nag_Stop_f, N.Run_index);
        if (N.Mileage_All > 0)
            N.Mileage_All -= Nag_Set_mileage; // �������̼�����//���浽flash
        else
            N.Mileage_All += Nag_Set_mileage; // ����
    }
}
//-------------------------------------------------------------------------------------------------------------------
// �������     �ߵ�������ʼ��
// ���ز���     void
// ʹ��ʾ��     �������ִ�п�ʼ
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag(void)
{
    memset(&N, 0, sizeof(N));
    N.Flash_page_index = Nag_Start_Page;
    flash_buffer_clear();
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ���Ե���ִ�к���
// ����˵��     index           ����
// ����˵��     type            ����ֵ
// ���ز���     void
// ʹ��ʾ��     �����ж���
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void Nag_System(void)
{
    // ������
    if (!N.Nag_SystemRun_Index || N.Nag_Stop_f)
        return;

    switch (N.Nag_SystemRun_Index)
    {
    case NAG_RUN_RECORD:
        Nag_Read(); // 1�Ƕ�ȡ
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
// �������     һ���Զ�ȡ����ֻ��ȡһ�Σ�
// ����˵��     index           ����
// ����˵��     type            ����ֵ
// ���ز���     void
// ʹ��ʾ��     ����������ֱ�ӵ��ã�demo����ʾ����
// ��ע��Ϣ
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
 * @brief ����һ�����ߵ�¼�ƣ���������ֹ¼�ƣ������������ߵ�����
 *N.Save_index = 0; // �������ã���ֹԽ��
 */
uint8 fuxian = 0;
void control_navigation(void)
{
    if (key1_flag == 1) // ����1���ƹߵ�������ֹͣ
    {
        N.Nag_SystemRun_Index = NAG_RUN_RECORD; // �����ߵ���ȡ������
        key1_flag = 0;
    }
    if (key3_flag == 1 && N.Nag_SystemRun_Index == NAG_RUN_RECORD) // ����3���ƹߵ���ȡ������
    {
        N.End_f = 1; // ��ֹ�ߵ����У�ֹͣ�ɼ�
        key3_flag = 0;
    }
    if (key2_flag == 1) // ����2���ƹߵ�������ʼ��
    {
        Init_Nag_Path(Nag_PathSelect);
        N.Nag_SystemRun_Index = NAG_RUN_PRELOAD; // ���ֹߵ�
        fuxian = 0;                    // �켣������
        target_speed = 0;              // ·����ȡ���֮������
        key2_flag = 0;
    }
    // �����Ŀ���Ŀ���ٶȵ�������һ������50
    if (key4_flag == 1)
    {
        user_set_speed += 50;
        if (user_set_speed > 700)
            user_set_speed = 50; // ����700�ص�50
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


/**************************�ߵ���ȡFlash********************************/
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
 * @brief �Ƕȴ�����-180~180�ȷ�Χ��
 *
 * @param angle ����Ƕ�
 * @return double ������Ƕ�
 */
float angle_plan(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle <= -180.0f)
        angle += 360.0f;

    return angle;
}
