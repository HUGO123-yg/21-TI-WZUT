#include "zf_common_headfile.h"


// �û����õ�Ŀ���ٶȣ����ɰ����ĵ���
float user_set_speed = 200; // ��ʼĿ���ٶ�Ϊ200

float Nav_read[Read_MaxSize]; // ��5cm��Ļ�,1000������50m
Nag N;


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
// �������     ��·����ʼ���ߵ� (����Flash_page_indexΪ��Ӧ·������ʼҳ)
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag_Path(uint8 path_id)
{
    Nag_PathSelect = path_id;
    memset(&N, 0, sizeof(N));
    N.Flash_page_index = get_path_start_page(path_id);
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// �������     д��Ԫ����ҳ (��3��·����Save_index��д��page 1)
// ��ע��Ϣ     ��Ԫ����ҳ��:
//               buffer[MaxSize+0] = ·��1��Save_index
//               buffer[MaxSize+1] = ·��2��Save_index
//               buffer[MaxSize+2] = ·��3��Save_index
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

    // �򻯴�������ǰ·����Save_indexֱ��д��
    flash_union_buffer[MaxSize + (Nag_PathSelect - 1)].uint32_type = N.Save_index;

    if (flash_check(0, NAG_META_PAGE))
        flash_erase_page(0, NAG_META_PAGE);
    flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡԪ����ҳ����ȡָ��·����Save_index
//-------------------------------------------------------------------------------------------------------------------
uint16 Get_Path_SaveIndex(uint8 path_id)
{
    uint32 save_idx_raw;

    if (path_id < 1 || path_id > 3) return 0;

    flash_buffer_clear();
    flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
    save_idx_raw = flash_union_buffer[MaxSize + (path_id - 1)].uint32_type;
    flash_buffer_clear();

    if (save_idx_raw == 0xFFFFFFFF)
    {
        return 0;
    }

    return (uint16)save_idx_raw;
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
        flash_Nag_Write(); // д�����һҳ����֤falsh�洢��
        N.End_f++;
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
    N.Mileage_All += (R_Mileage + L_Mileage) * 0.5f; // ���̼ƶ�ȡ�����ұ�������ʹ�ø������Ļ�����ܱ�������
  
//    N.Mileage_All =Car.mileage;//��̼ƶ�ȡ
    // printf("Mileage_All=%f\r\n", N.Mileage_All);
    
    if (N.size > MaxSize) // ��������ҳ�е�flash��С��ʱ��д��һ�Σ���ֹ�ظ�д��
    {
        flash_Nag_Write();
        N.size = 0;                                   // ��������Ϊ0����һ����������ʼ��ȡ
        N.Flash_page_index--;                         // flashҳ��������С
        zf_assert(N.Flash_page_index > Nag_End_Page); // ��ֹԽ�籨��
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
    if (N.Mileage_All >= Nag_Set_mileage)
    {
        if (N.Run_index > N.Save_index - 2)
        {
            N.Nag_Stop_f = true;
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
    case 1:
        Nag_Read(); // 1�Ƕ�ȡ
        break;
    case 2:
        fuxian = 1;
        target_speed = user_set_speed; // ����ʱʹ���û����õ�Ŀ���ٶ�
        NagFlashRead();
        break;
    case 3:
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
        int temp_index = index - (500 * page_trun);
        if (temp_index > MaxSize) // �������趨��flsh��С��ʱ��
        {
            N.Flash_page_index--; // ҳ�����
            page_trun++;
            flash_Nag_Read(); // ���¶�ȡ
        }
        Nav_read[index] = flash_union_buffer[index - (500 * page_trun)].int32_type;
         printf("Nav_read=%f\r\n", Nav_read[index]);
    }
    N.Nag_SystemRun_Index++;
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
        N.Nag_SystemRun_Index = 1; // �����ߵ���ȡ������
        key1_flag = 0;
    }
    if (key3_flag == 1 && N.Nag_SystemRun_Index == 1) // ����3���ƹߵ���ȡ������
    {
        N.End_f = 1; // ��ֹ�ߵ����У�ֹͣ�ɼ�
        key3_flag = 0;
    }
    if (key2_flag == 1) // ����2���ƹߵ�������ʼ��
    {
        N.Nag_SystemRun_Index = 2;     // ���ֹߵ�
        fuxian = 1;                    // �켣������
        target_speed = user_set_speed; // ����ʱʹ���û����õ�Ŀ���ٶ�
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

    // if (N.Nag_SystemRun_Index == 2)
    // {
    //     NagFlashRead();
    // }
}


/**************************�ߵ���ȡFlash********************************/
void flash_Nag_Write(void)
{
  


    if (flash_check(0, N.Flash_page_index))
        flash_erase_page(0, N.Flash_page_index);

    flash_write_page_from_buffer(0, N.Flash_page_index, FLASH_PAGE_LENGTH);
    if (N.End_f == 1)
    {
        flash_Nag_Write_Meta();
    }
     // ���ԣ���ӡ������ǰ5������
    for (int i = 0; i < 5 && i < N.size; i++) {
        printf("Before write: buffer[%d] = %d (angle=%.2f)\n", 
               i, flash_union_buffer[i].int32_type, 
               flash_union_buffer[i].int32_type / 100.0f);
    }
    printf("N.size=%d, N.Save_index=%d\n", N.size, N.Save_index);
    
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
 * @brief �Ƕȴ�����-180~180�ȷ�Χ��
 *
 * @param angle ����Ƕ�
 * @return double ������Ƕ�
 */
double angle_plan(double angle)
{
    while (angle > 180.0)
        angle -= 360.0;

    while (angle <= -180.0)
        angle += 360.0;

    return angle;
}
