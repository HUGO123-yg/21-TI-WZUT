#include "small_driver_uart_control.h"

small_device_value_struct small_driver_value;      // 
static uint32 motor_last_reply_tick = 0;       // 电机最后一次回复的时间戳

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ���ڽ��ջص�����
// ����˵��     driver_value   ������Ӧ�����ݽṹ��
// ���ز���     void
// ʹ��ʾ��     uart_control_callback(&small_driver_value);
// ��ע��Ϣ     ���ڽ������յ����ٶ�����  �ú�����Ҫ�ڶ�Ӧ�Ĵ��ڽ����ж��е���
//-------------------------------------------------------------------------------------------------------------------
void small_driver_control_callback(small_device_value_struct *driver_value)
{
    uint8 receive_data[7];
    uint8 receive_len;
    
    receive_len = uart_query_buffer(driver_value->driver_uart, receive_data);
        
    if(receive_len)
    {
        if(driver_value->receive_data_buffer[0] != 0xA5)
        {
            driver_value->receive_data_count = 0;
        }
        
        for(int i = 0; i < receive_len; i ++)
        {    
            driver_value->receive_data_buffer[driver_value->receive_data_count ++] = receive_data[i];
        }
        
        if(driver_value->receive_data_count >= 7)
        {
            if(driver_value->receive_data_buffer[0] == 0xA5)
            {
                driver_value->sum_check_data =  driver_value->receive_data_buffer[0] +
                                                driver_value->receive_data_buffer[1] +
                                                driver_value->receive_data_buffer[2] +
                                                driver_value->receive_data_buffer[3] +
                                                driver_value->receive_data_buffer[4] +
                                                driver_value->receive_data_buffer[5];
                if(driver_value->sum_check_data == driver_value->receive_data_buffer[6])
                {
                    if(driver_value->receive_data_buffer[1] == 0x02)
                    {
                        driver_value->receive_left_speed_data = (((int)driver_value->receive_data_buffer[2] << 8) | (int)driver_value->receive_data_buffer[3]);
                        driver_value->receive_right_speed_data = (((int)driver_value->receive_data_buffer[4] << 8) | (int)driver_value->receive_data_buffer[5]);
                    }
                    else if(driver_value->receive_data_buffer[1] == 0x04)
                    {
                        short int left_angle_temp  = (((int)driver_value->receive_data_buffer[2] << 8) | (int)driver_value->receive_data_buffer[3]);
                        short int right_angle_temp = (((int)driver_value->receive_data_buffer[4] << 8) | (int)driver_value->receive_data_buffer[5]);
                        
                        driver_value->receive_left_angle_data  = (float)left_angle_temp  / 100.0f;
                        driver_value->receive_right_angle_data = (float)right_angle_temp / 100.0f;
                    }
                    else if(driver_value->receive_data_buffer[1] == 0x05)
                    {
                        short int left_location_temp  = (((int)driver_value->receive_data_buffer[2] << 8) | (int)driver_value->receive_data_buffer[3]);
                        short int right_location_temp = (((int)driver_value->receive_data_buffer[4] << 8) | (int)driver_value->receive_data_buffer[5]);
                        
                        driver_value->receive_left_location_data  = (float)left_location_temp  / 100.0f * driver_value->left_motor_dir;
                        driver_value->receive_right_location_data = (float)right_location_temp / 100.0f * driver_value->right_motor_dir;
                    }
                    driver_value->receive_data_count = 0;
                    motor_last_reply_tick = timer_get(TC_TIME2_CH0);            // 更新最后回复时间戳
                    memset(driver_value->receive_data_buffer, 0, sizeof(driver_value->receive_data_buffer));
                }
                else
                {
                    driver_value->receive_data_count = 0;
                    memset(driver_value->receive_data_buffer, 0, sizeof(driver_value->receive_data_buffer));
                }
            }
            else
            {
                    driver_value->receive_data_count = 0;
                    memset(driver_value->receive_data_buffer, 0, sizeof(driver_value->receive_data_buffer));
            }
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ���� ���ռ�ձ�
// ����˵��     driver_value   ������Ӧ�����ݽṹ��
// ����˵��     left_duty      �����ռ�ձ� ��ֵ-10000~10000 ��Ӧ���ռ�ձ� 0% ~ 100%���������������ת��
// ����˵��     right_duty     �Ҳ���ռ�ձ� ��ֵ-10000~10000 ��Ӧ���ռ�ձ� 0% ~ 100%���������������ת��
// ���ز���     void
// ʹ��ʾ��     small_driver_set_duty(&small_driver_value, -1000, 1000);
// ��ע��Ϣ     ���ڿ��Ƶ���������
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(small_device_value_struct *driver_value, int left_duty, int right_duty)
{
    left_duty  = func_limit_ab(left_duty, -10000, 10000);
    right_duty = func_limit_ab(right_duty, -10000, 10000);
    
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0x01;
    driver_value->send_data_buffer[2] = (unsigned char)((left_duty & 0xff00) >> 8);
    driver_value->send_data_buffer[3] = (unsigned char)(left_duty & 0x00ff);
    driver_value->send_data_buffer[4] = (unsigned char)((right_duty & 0xff00) >> 8);
    driver_value->send_data_buffer[5] = (unsigned char)(right_duty & 0x00ff);
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];

    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ���� ��λ��
// ����˵��     driver_value   ������Ӧ�����ݽṹ��
// ���ز���     void
// ʹ��ʾ��     small_driver_set_location_zero(&small_driver_value);
// ��ע��Ϣ     ���ڽ����ٺ�����ĵ�ǰλ������Ϊ0�Ƕ� �������ڴ˽Ƕ���ת
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_location_zero(small_device_value_struct *driver_value)
{
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0X06;
    driver_value->send_data_buffer[2] = 0x00;
    driver_value->send_data_buffer[3] = 0x00;
    driver_value->send_data_buffer[4] = 0x00;
    driver_value->send_data_buffer[5] = 0x00;
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];
    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ��ȡ ת������
// ����˵��     driver_value   ������Ӧ�����ݽṹ��
// ���ز���     void
// ʹ��ʾ��     small_driver_get_speed(&small_driver_value);
// ��ע��Ϣ     ��ȡ�����ǰת������ ��λ RPM
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(small_device_value_struct *driver_value)
{
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0x02;
    driver_value->send_data_buffer[2] = 0x00;
    driver_value->send_data_buffer[3] = 0x00;
    driver_value->send_data_buffer[4] = 0x00;
    driver_value->send_data_buffer[5] = 0x00;
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];
    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ��ȡ �����ǰת�ӻ�е�Ƕ�
// ����˵��     driver_value   ������Ӧ�����ݽṹ��
// ���ز���     void
// ʹ��ʾ��     small_driver_get_angle(&small_driver_value);
// ��ע��Ϣ     ��ȡ�����ǰת�ӻ�е�Ƕ� ��λ ��
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_angle(small_device_value_struct *driver_value)
{
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0X04;
    driver_value->send_data_buffer[2] = 0x00;
    driver_value->send_data_buffer[3] = 0x00;
    driver_value->send_data_buffer[4] = 0x00;
    driver_value->send_data_buffer[5] = 0x00;
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];
    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ��ȡ �����ǰͨ�����ٽṹ�������Ƕ� 
// ����˵��     driver_value   ������Ӧ�����ݽṹ��
// ���ز���     void
// ʹ��ʾ��     small_driver_get_location(&small_driver_value);
// ��ע��Ϣ     ��ȡ�����ǰͨ�����ٽṹ�������Ƕ�  ��λ ��
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_location(small_device_value_struct *driver_value)
{
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0X05;
    driver_value->send_data_buffer[2] = 0x00;
    driver_value->send_data_buffer[3] = 0x00;
    driver_value->send_data_buffer[4] = 0x00;
    driver_value->send_data_buffer[5] = 0x00;
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];
    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ����ͨѶ��ʼ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     small_driver_uart_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    memset(&small_driver_value, 0, sizeof(small_driver_value));                                 // ����ṹ������
    
    small_driver_value.driver_uart = SMALL_DRIVER_UART;                                         // ��¼��ǰ����ʹ�õĴ��ں�

    small_driver_value.left_motor_dir  = LEFT_MOTOR_DIR;                                        // ��¼��ǰ�������������ת����
    
    small_driver_value.right_motor_dir = RIGHT_MOTOR_DIR;                                       // ��¼��ǰ�����Ҳ�������ת����
    
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // ���ڳ�ʼ��
    
    uart_rx_trigger_interrupt(SMALL_DRIVER_UART, 6);                                            // ����Ϊ7�ֽ��������
    
    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // ʹ�ܴ��ڽ����ж�

    small_driver_set_duty(&small_driver_value, 0, 0);                                           // ����0ռ�ձ�

        small_driver_get_speed(&small_driver_value);                                                // 

}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       motor_timeout_check
// 功能说明       检查电机驱动回复是否超时
// 输入参数       timeout_ms  超时阈值(毫秒)
// 返回参数       uint8       0=正常, 1=超时
//-------------------------------------------------------------------------------------------------------------------
uint8 motor_timeout_check(uint32 timeout_ms)
{
    uint32 now = timer_get(TC_TIME2_CH0);
    uint32 delta = now - motor_last_reply_tick;
    
    if (delta > timeout_ms * 1000)
    {
        return 1;
    }
    return 0;
}














