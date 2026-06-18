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
#include "gnss_cache.h"
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                       // ���Դ�����Ϣ��ʼ��
    // �˴���д�û����� ���������ʼ�������
    
    BUZZER_init();
//    imu963ra_init();
//    imu660ra_init();
      imu660rb_init();

    
    flash_init();
    Init_Nag();

    gnss_init(TAU1201);
    gnss_auto_start();

    small_driver_uart_init();
    balance_cascade_init();
    steer_control_init();
    ips_init(IPS200_TYPE_SPI);
    Key_init();
    system_delay_ms(1000);
    
    
    
    pit_ms_init(PIT_CH0,1);
    pit_ms_init(PIT_CH1,5);

    BUZZER_check(50);                       //�Լ�

    
    
    // �˴���д�û����� ���������ʼ�������
    while(true)
    {
        // GPS 数据解析与缓存保存
        if (gnss_flag)
        {
            gnss_flag = 0;
            if (gnss_data_parse() == 0 && gnss.state == 1)
            {
                gnss_cache_struct cache;
                cache.magic           = GNSS_CACHE_MAGIC;
                cache.latitude_e7     = (int32)(gnss.latitude * 1e7);
                cache.longitude_e7    = (int32)(gnss.longitude * 1e7);
                cache.altitude_cm     = (int32)(gnss.height * 100.0f);
                cache.ground_speed_cm = (int32)(gnss.speed * 100.0f);
                cache.heading_deg     = (int16)(gnss.direction * 100.0f);
                cache.year            = gnss.time.year;
                cache.month           = gnss.time.month;
                cache.day             = gnss.time.day;
                cache.hour            = gnss.time.hour;
                cache.minute          = gnss.time.minute;
                cache.second          = gnss.time.second;
                cache.satellite_used  = gnss.satellite_used;
                cache.save_count      = 1;
                memset(cache.reserved, 0, sizeof(cache.reserved));
                gnss_cache_save(&cache);
            }
        }

        Menu();

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

void pit0_ch0_isr()                     // ��ʱ��ͨ�� 0 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH0);
    
    pit_call_back();
    
//    imu660rb_get_gyro();                             // ��ȡ IMU660RA ����������
//    imu660rb_get_acc();                              // ��ȡ IMU660RA ���ٶȼ�����
//    quaternion_module_calculate(&roll_balance_cascade); // ������Ԫ����������̬����
    
}

void pit0_ch1_isr()                     // ��ʱ��ͨ�� 1 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH1);
    
    key_scan();
}



// **************************** �������� ****************************
