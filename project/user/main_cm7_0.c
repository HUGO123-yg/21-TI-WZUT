/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_0
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "config.h"
#include "Control_system.h"
#include "Imu.h"
#include "Menu.h"
#include "Pit_scheduler.h"
#include "Terrain_vision.h"
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************

#define IMU_DEBUG_PRINT_INTERVAL_SAMPLES    (20U)

int main(void)
{
    const imu_data_t *imu;
    uint32 imu_last_print_sample_count = 0U;
    control_status_t control_status;

    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                       // ���Դ�����Ϣ��ʼ��
    printf("\r\nDebug UART ready.\r\n");
    // �˴���д�û����� ���������ʼ�������

    // �ȳ�ʼ����� Work Flash �������ٶ�ȡ��Ŀ���˫����·��Ŀ¼��
    // nav_flash_init() �״��ϵ�ֻ��ȡ�������޹ʲ�д�հ� Flash��
    flash_init();
    if (NAV_FLASH_STATUS_OK != nav_flash_init())
    {
        zf_log(0, "navigation flash init error.");
    }

    control_status = control_system_init();
    if (CONTROL_STATUS_OK != control_status)
    {
        printf("control init failed: %d\r\n", (int)control_status);
        zf_log(0, "control system init error.");
    }
    else
    {
        printf("IMU init ok.\r\n");
    }

    if (TERRAIN_VISION_STATUS_OK != terrain_vision_init())
    {
        zf_log(0, "terrain vision init error.");
    }

    menu_init();

    pit_scheduler_init();
    pit_ms_init(PIT_CH0,1);

    // �˴���д�û����� ���������ʼ�������
    while(true)
    {
        menu_task();
        terrain_vision_task();

        if (imu_is_initialized())
        {
            imu = imu_get_data();
            if ((imu->sample_count - imu_last_print_sample_count)
                >= IMU_DEBUG_PRINT_INTERVAL_SAMPLES)
            {
                imu_last_print_sample_count = imu->sample_count;
                printf("roll=%.2f\r\n", imu->roll_deg);
                printf("pitch=%.2f\r\n", imu->pitch_deg);
                printf("yaw=%.2f\r\n", imu->yaw_deg);
            }
        }

        // �˴���д��Ҫѭ��ִ�еĴ���
//          CYT2_D_motor_ctrl(1000,1000);
//      printf("%d,%d,%d\n",imu660rb_gyro_x, imu660rb_gyro_y, imu660rb_gyro_z);
//        printf("%f,%f,%f\r\n",roll_balance_cascade.posture_value.pit, roll_balance_cascade.posture_value.rol, -roll_balance_cascade.posture_value.yaw);
//        printf("%d,%d\r\n",motor_value.receive_left_speed_data , motor_value.receive_right_speed_data );
//        printf("%d,%d\r\n",left_motor_duty ,right_motor_duty);
//        printf("%f\r\n",Car.mileage);

//      system_delay_ms(10);

        // Flash ҳд��������������ֻ��������ѭ��ִ�У��жϲ�¼�ƽӿ�
        // ���Ѳ���д�� RAM ˫���壬��֪ͨ���ﴦ����дҳ�档
        (void)nav_flash_service();

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

void pit0_ch0_isr()                     // ��ʱ��ͨ�� 0 �����жϷ�����
{
    uint32 start_cycles;

    start_cycles = pit_scheduler_measure_begin();
    pit_isr_flag_clear(PIT_CH0);
    pit_scheduler_schedule_tick_from_isr();
    menu_tick_1ms();
    pit_scheduler_record_pit_isr(start_cycles);
}

void PendSV_Handler(void)
{
    pit_scheduler_run_deferred();
}

void pit0_ch1_isr()                     // ��ʱ��ͨ�� 1 �����жϷ�����
{
    pit_isr_flag_clear(PIT_CH1);

}



// **************************** �������� ****************************
