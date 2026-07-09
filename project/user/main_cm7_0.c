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
#include "bridge_ctrl.h"
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
    while(imu660rb_init())
    {
        printf("\r\n IMU660RB init error.");
        BUZZER_check(50);
        system_delay_ms(100);
    }

    
    flash_init();
    Init_Nag();
    
    small_driver_uart_init();
    balance_cascade_init();
    balance_steering_init();
    terrain_init();
#ifdef USE_TERRAIN_DEBUG
    seekfree_assistant_interface_init(TERRAIN_DEBUG_ASSISTANT_DEVICE);
#endif
    bridge_init();
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
        // �˴���д��Ҫѭ��ִ�еĴ���
//          CYT2_D_motor_ctrl(1000,1000);
//      printf("%d,%d,%d\n",imu660rb_gyro_x, imu660rb_gyro_y, imu660rb_gyro_z);
//        printf("%f,%f,%f\r\n",roll_balance_cascade.posture_value.pit, roll_balance_cascade.posture_value.rol, -roll_balance_cascade.posture_value.yaw);
//        printf("%d,%d\r\n",motor_value.receive_left_speed_data , motor_value.receive_right_speed_data );
//        printf("%d,%d\r\n",left_motor_duty ,right_motor_duty);
//        printf("%f\r\n",Car.mileage);

//      system_delay_ms(10);
      
#ifdef AUTO_RUN_STRAIGHT
        static uint8 auto_run_armed = 0;
        if (!auto_run_armed && sys_times > AUTO_RUN_ARM_DELAY_MS)
        {
            target_speed = AUTO_RUN_SPEED;
            system_armed = 1;
            auto_run_armed = 1;
            BUZZER_check(100);
        }

        static uint32 last_disp = 0;
        if (sys_times - last_disp > 100)
        {
            last_disp = sys_times;
            ips200_show_string(0, 0, "AUTO RUN");
            ips200_show_string(0, 16, "ARMED:");
            ips200_show_int(8 * 7, 16, system_armed, 1);
            ips200_show_string(0, 32, "ROL:");
            ips200_show_float(8 * 5, 32, roll_balance_cascade.posture_value.rol, 5, 1);
            ips200_show_string(0, 48, "PIT:");
            ips200_show_float(8 * 5, 48, roll_balance_cascade.posture_value.pit, 5, 1);
            ips200_show_string(0, 64, "STBAL:");
            ips200_show_float(8 * 7, 64, pitch_balance_cascade.angle_cycle.out, 5, 1);
#ifdef USE_BRIDGE_CONTROL
            ips200_show_string(0, 80, "BRG:");
            ips200_show_int(8 * 5, 80, bridge_get_state(), 1);
#endif
        }
#else
        Menu();
#endif

        terrain_debug_service();

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
