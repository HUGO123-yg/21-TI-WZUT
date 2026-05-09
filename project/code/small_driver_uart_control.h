#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_1     )

#define SMALL_DRIVER_BAUDRATE                   (460800        )

#define SMALL_DRIVER_RX                         (UART1_TX_P14_1)

#define SMALL_DRIVER_TX                         (UART1_RX_P14_0)

#define LEFT_MOTOR_DIR                          (1)

#define RIGHT_MOTOR_DIR                         (-1)

typedef struct
{
    uart_index_enum driver_uart;                        
    
    unsigned char send_data_buffer[7];
    unsigned char receive_data_buffer[7];
    unsigned char receive_data_count;
    unsigned char sum_check_data;

    short int left_motor_dir;                           
    short int right_motor_dir;
    
    short int receive_left_speed_data;                  // ���յ��� ����� ת������
    short int receive_right_speed_data;                 // ���յ��� �Ҳ��� ת������
    
    float receive_left_angle_data;                      // ���յ��� ����� �Ƕ�����
    float receive_right_angle_data;                     // ���յ��� �Ҳ��� �Ƕ�����
    
    float receive_left_location_data;                   // ���յ��� ����� λ������
    float receive_right_location_data;                  // ���յ��� �Ҳ��� λ������
}small_device_value_struct;

extern small_device_value_struct small_driver_value;


void small_driver_control_callback(small_device_value_struct *driver_value);                            // ��ˢ���� ���ڽ��ջص�����

void small_driver_set_duty(small_device_value_struct *driver_value, int left_duty, int right_duty);     // ��ˢ���� ���� ���ռ�ձ�
    
void small_driver_set_location_zero(small_device_value_struct *driver_value);                           // ��ˢ���� ���� ��λ��

void small_driver_get_speed(small_device_value_struct *driver_value);                                   // ��ˢ���� ��ȡ ת������

void small_driver_get_angle(small_device_value_struct *driver_value);                                   // ��ˢ���� ��ȡ �����ǰת�ӻ�е�Ƕ�

void small_driver_get_location(small_device_value_struct *driver_value);                                // ��ˢ���� ��ȡ �����ǰͨ�����ٽṹ�������Ƕ� 

void small_driver_uart_init(void);                                                                      // ��ˢ���� ����ͨѶ��ʼ��

#endif
