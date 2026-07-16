#include "zf_common_headfile.h"

float user_set_speed = 200.0f;
float Nav_read[Read_MaxSize];
Nag N;
uint8 Nag_PathSelect = 1U;
uint8 fuxian = 0U;

static uint8 nag_path_is_valid(uint8 path_id)
{
    return (uint8)(path_id >= 1U && path_id <= NAG_PATH_COUNT);
}

static uint8 get_path_start_page(uint8 path_id)
{
    switch (path_id)
    {
        case 1U: return NAG_PATH1_START;
        case 2U: return NAG_PATH2_START;
        case 3U: return NAG_PATH3_START;
        default: return NAG_PATH1_START;
    }
}

static uint8 get_path_end_page(uint8 path_id)
{
    switch (path_id)
    {
        case 1U: return NAG_PATH1_END;
        case 2U: return NAG_PATH2_END;
        case 3U: return NAG_PATH3_END;
        default: return NAG_PATH1_END;
    }
}

static uint8 nag_page_belongs_to_path(uint8 path_id, uint8 page)
{
    return (uint8)(page >= get_path_end_page(path_id)
        && page <= get_path_start_page(path_id));
}

static uint16 get_path_sample_capacity(uint8 path_id)
{
    uint32 page_count;
    uint32 capacity;

    if (!nag_path_is_valid(path_id))
    {
        return 0U;
    }

    page_count = (uint32)get_path_start_page(path_id)
        - (uint32)get_path_end_page(path_id) + 1U;
    capacity = page_count * NAG_SAMPLES_PER_PAGE;
    if (capacity > Read_MaxSize)
    {
        capacity = Read_MaxSize;
    }
    return (uint16)capacity;
}

static uint16 sanitize_save_index(uint8 path_id, uint32 raw_save_index)
{
    uint16 capacity = get_path_sample_capacity(path_id);

    if (raw_save_index == 0xFFFFFFFFU || raw_save_index > capacity)
    {
        return 0U;
    }
    return (uint16)raw_save_index;
}

static void read_all_path_metadata(uint16 save_indexes[NAG_PATH_COUNT])
{
    uint8 path_index;

    flash_buffer_clear();
    if (flash_check(0U, NAG_META_PAGE))
    {
        flash_read_page_to_buffer(0U, NAG_META_PAGE, FLASH_PAGE_LENGTH);
    }

    for (path_index = 0U; path_index < NAG_PATH_COUNT; path_index++)
    {
        save_indexes[path_index] = sanitize_save_index(
            (uint8)(path_index + 1U),
            flash_union_buffer[NAG_SAMPLES_PER_PAGE + path_index].uint32_type);
    }
    flash_buffer_clear();
}

static void write_all_path_metadata(const uint16 save_indexes[NAG_PATH_COUNT])
{
    uint8 path_index;

    flash_buffer_clear();
    for (path_index = 0U; path_index < NAG_PATH_COUNT; path_index++)
    {
        flash_union_buffer[NAG_SAMPLES_PER_PAGE + path_index].uint32_type =
            save_indexes[path_index];
    }

    if (flash_check(0U, NAG_META_PAGE))
    {
        flash_erase_page(0U, NAG_META_PAGE);
    }
    flash_write_page_from_buffer(0U, NAG_META_PAGE, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

static void nag_finish_flash_load(void)
{
    N.Save_state = 1U;
    N.Flash_read_f = 0U;
    N.Nag_SystemRun_Index = 3U;
    flash_buffer_clear();
}

void Init_Nag_Path(uint8 path_id)
{
    if (!nag_path_is_valid(path_id))
    {
        path_id = 1U;
    }

    Nag_PathSelect = path_id;
    memset(&N, 0, sizeof(N));
    N.Flash_page_index = get_path_start_page(path_id);
    flash_buffer_clear();
}

void Init_Nag(void)
{
    Init_Nag_Path(1U);
}

void flash_Nag_Write_Meta(void)
{
    uint16 save_indexes[NAG_PATH_COUNT];
    uint16 capacity;

    if (!nag_path_is_valid(Nag_PathSelect))
    {
        return;
    }

    read_all_path_metadata(save_indexes);
    capacity = get_path_sample_capacity(Nag_PathSelect);
    save_indexes[Nag_PathSelect - 1U] =
        (N.Save_index <= capacity) ? N.Save_index : capacity;
    write_all_path_metadata(save_indexes);
}

void flash_Nag_Read_Meta(void)
{
    N.Save_index = Get_Path_SaveIndex(Nag_PathSelect);
}

uint16 Get_Path_SaveIndex(uint8 path_id)
{
    uint16 save_indexes[NAG_PATH_COUNT];

    if (!nag_path_is_valid(path_id))
    {
        return 0U;
    }

    read_all_path_metadata(save_indexes);
    return save_indexes[path_id - 1U];
}

uint8 flash_Nag_Clear_Path(uint8 path_id)
{
    uint8 page;
    uint16 save_indexes[NAG_PATH_COUNT];

    if (!nag_path_is_valid(path_id))
    {
        return 1U;
    }

    page = get_path_end_page(path_id);
    while (page <= get_path_start_page(path_id))
    {
        if (flash_check(0U, page))
        {
            flash_erase_page(0U, page);
        }
        page++;
    }

    read_all_path_metadata(save_indexes);
    save_indexes[path_id - 1U] = 0U;
    write_all_path_metadata(save_indexes);

    if (Nag_PathSelect == path_id)
    {
        Init_Nag_Path(path_id);
    }
    return 0U;
}

void Nag_Read(void)
{
    switch (N.End_f)
    {
        case 0U:
            Run_Nag_Save();
            break;

        case 1U:
            flash_Nag_Write();
            N.End_f = 2U;
            break;

        case 2U:
            N.End_f = 3U;
            break;

        default:
            break;
    }
}

void Nag_Run(void)
{
    Run_Nag_GPS();
    if (N.Nag_Stop_f)
    {
        N.Final_Out = 0.0f;
        target_speed = 0.0f;
        fuxian = 0U;
        STOP_FALG = 0;
        return;
    }

    N.Final_Out = (float)angle_plan(Nag_Yaw - N.Angle_Run);
}

void Run_Nag_Save(void)
{
    uint16 capacity = get_path_sample_capacity(Nag_PathSelect);
    uint8 end_page = get_path_end_page(Nag_PathSelect);

    N.Mileage_All += (R_Mileage + L_Mileage) * 0.5f;

    if (N.size >= NAG_SAMPLES_PER_PAGE)
    {
        flash_Nag_Write();
        N.size = 0U;

        if (N.Flash_page_index <= end_page || N.Save_index >= capacity)
        {
            flash_Nag_Write_Meta();
            N.End_f = 2U;
            return;
        }
        N.Flash_page_index--;
    }

    if (N.Mileage_All >= Nag_Set_mileage)
    {
        int32 save_value;

        if (N.Save_index >= capacity)
        {
            N.End_f = 1U;
            return;
        }

        save_value = (int32)(Nag_Yaw * 100.0f);
        flash_union_buffer[N.size].int32_type = save_value;
        N.size++;
        N.Save_index++;
        N.Mileage_All -= Nag_Set_mileage;
    }
}

void Run_Nag_GPS(void)
{
    uint16 prospect;

    N.Mileage_All += (R_Mileage + L_Mileage) * 0.5f;
    if (N.Mileage_All < Nag_Set_mileage)
    {
        return;
    }

    if (N.Save_index < 2U || N.Run_index >= (uint16)(N.Save_index - 1U))
    {
        N.Nag_Stop_f = true;
        return;
    }

    N.Run_index++;
    prospect = N.Run_index;
    if (prospect >= N.Save_index)
    {
        prospect = (uint16)(N.Save_index - 1U);
    }
    N.Angle_Run = Nav_read[prospect] / 100.0f;
    N.Mileage_All -= Nag_Set_mileage;
}

void Nag_System(void)
{
    if (!N.Nag_SystemRun_Index || N.Nag_Stop_f)
    {
        return;
    }

    switch (N.Nag_SystemRun_Index)
    {
        case 1U:
            Nag_Read();
            break;

        case 2U:
            fuxian = 1U;
            target_speed = user_set_speed;
            NagFlashRead();
            break;

        case 3U:
            Nag_Run();
            break;

        default:
            break;
    }
}

void NagFlashRead(void)
{
    uint16 page_sample_count;
    uint16 page_index;

    if (N.Save_state)
    {
        return;
    }

    if (!N.Flash_read_f)
    {
        flash_Nag_Read_Meta();
        N.Save_count = 0U;
        N.Flash_page_index = get_path_start_page(Nag_PathSelect);
        N.Flash_read_f = 1U;

        if (N.Save_index == 0U)
        {
            nag_finish_flash_load();
            return;
        }
    }

    if (!nag_page_belongs_to_path(Nag_PathSelect, N.Flash_page_index)
        || !flash_check(0U, N.Flash_page_index))
    {
        N.Save_index = 0U;
        nag_finish_flash_load();
        return;
    }

    flash_Nag_Read();
    page_sample_count = (uint16)(N.Save_index - N.Save_count);
    if (page_sample_count > NAG_SAMPLES_PER_PAGE)
    {
        page_sample_count = NAG_SAMPLES_PER_PAGE;
    }

    for (page_index = 0U; page_index < page_sample_count; page_index++)
    {
        Nav_read[N.Save_count + page_index] =
            (float)flash_union_buffer[page_index].int32_type;
    }
    N.Save_count = (uint16)(N.Save_count + page_sample_count);

    if (N.Save_count >= N.Save_index)
    {
        nag_finish_flash_load();
        return;
    }

    if (N.Flash_page_index <= get_path_end_page(Nag_PathSelect))
    {
        N.Save_index = 0U;
        nag_finish_flash_load();
        return;
    }
    N.Flash_page_index--;
    flash_buffer_clear();
}

void control_navigation(void)
{
    if (key1_flag == 1)
    {
        N.Nag_SystemRun_Index = 1U;
        key1_flag = 0;
    }

    if (key3_flag == 1 && N.Nag_SystemRun_Index == 1U)
    {
        N.End_f = 1U;
        key3_flag = 0;
    }

    if (key2_flag == 1)
    {
        N.Nag_SystemRun_Index = 2U;
        fuxian = 1U;
        target_speed = user_set_speed;
        key2_flag = 0;
    }

    if (key4_flag == 1)
    {
        user_set_speed += 50.0f;
        if (user_set_speed > 700.0f)
        {
            user_set_speed = 50.0f;
        }
        key4_flag = 0;
    }
}

void flash_Nag_Write(void)
{
    if (N.size > 0U)
    {
        if (!nag_page_belongs_to_path(Nag_PathSelect, N.Flash_page_index))
        {
            N.Save_index = 0U;
            N.End_f = 1U;
        }
        else
        {
            if (flash_check(0U, N.Flash_page_index))
            {
                flash_erase_page(0U, N.Flash_page_index);
            }
            flash_write_page_from_buffer(
                0U,
                N.Flash_page_index,
                FLASH_PAGE_LENGTH);
            gpio_set_level(BUZZER_PIN, 1);
        }
    }

    if (N.End_f == 1U)
    {
        flash_Nag_Write_Meta();
    }
    flash_buffer_clear();
}

void flash_Nag_Read(void)
{
    flash_buffer_clear();
    if (nag_page_belongs_to_path(Nag_PathSelect, N.Flash_page_index)
        && flash_check(0U, N.Flash_page_index))
    {
        flash_read_page_to_buffer(
            0U,
            N.Flash_page_index,
            FLASH_PAGE_LENGTH);
    }
}

double angle_plan(double angle)
{
    while (angle > 180.0)
    {
        angle -= 360.0;
    }

    while (angle <= -180.0)
    {
        angle += 360.0;
    }
    return angle;
}
