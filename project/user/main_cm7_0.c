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
#include "small_driver_uart_control.h"
#include "euler.h"
#include "state_estimator.h"
#include "ipc_protocol.h"
#include "servo.h"
#include "control.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************

#define LED1                    (P19_0)    

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       ipc_cm7_0_callback
// 功能说明       CM7_0 IPC 接收回调：从 CM7_1 接收视觉解包数据，更新 image_error
// 输入参数       ipc_data      CM7_1 发送的 uint32 打包数据
// 返回参数       void
// 使用示例       ipc_communicate_init(IPC_PORT_1, ipc_cm7_0_callback);
// 备注信息       此回调在 IPC 中断上下文中执行，必须快速返回（不阻塞、不自旋等待）
//-------------------------------------------------------------------------------------------------------------------
// 全局心跳时间戳
static uint32 cm7_1_last_heartbeat = 0;

void ipc_cm7_0_callback(uint32 ipc_data)
{
    if (ipc_data == IPC_MSG_TYPE_HEARTBEAT)
    {
        cm7_1_last_heartbeat = timer_get(TC_TIME2_CH0);
        return;
    }

    uint8 quality = IPC_VISION_UNPACK_QUALITY(ipc_data);
    if (quality > 0)
    {
        image_error = IPC_VISION_UNPACK_ERROR(ipc_data);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       cm7_1_is_alive
// 功能说明       CM7_1 存活检测 — 检查最后心跳时间
// 输入参数       timeout_ms  超时阈值(毫秒)
// 返回参数       uint8       0=存活, 1=无响应
//-------------------------------------------------------------------------------------------------------------------
uint8 cm7_1_is_alive(uint32 timeout_ms)
{
    uint32 now = timer_get(TC_TIME2_CH0);
    uint32 delta = now - cm7_1_last_heartbeat;

    if (cm7_1_last_heartbeat == 0) return 0;  // 尚未收到首次心跳
    if (delta > timeout_ms * 1000) return 1;
    return 0;
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    dwt_init();                         // DWT 周期计数器使能（用于 PID dt 测量，替代 TCPWM 定时器）
    euler_init();                       // IMU 初始化 + 陀螺仪零偏校准（100次静止采样）
    state_estimator_init();             // 状态估计器初始化：清零速度和位置估计值
    debug_init();                       // ���Դ�����Ϣ��ʼ��
    // �˴���д�û����� ���������ʼ�������
    small_driver_uart_init();           // 电机 UART 初始化 (UART_1, 460800 baud)
    steering_Init();                    // 舵机 PWM 初始化
    pid_ctrl_Init();                    // PID 参数初始化

    ipc_communicate_init(IPC_PORT_1, ipc_cm7_0_callback);   // CM7_0 IPC init: receive vision data from CM7_1
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);   

    pit_init(PIT_CH0, 1000);                // 1kHz motion control ISR (1000us period)

    // 看门狗初始化 (1s 超时，需在 ISR 中定期喂狗)
    Cy_WDT_Init();                          // 匹配值 32000 ≈ 1s @ 32KHz ILO, 超时动作=RESET
    Cy_WDT_SetDebugRun(CY_WDT_ENABLE);      // 调试时暂停看门狗，避免断点触发复位
    Cy_WDT_Enable();

    // ˴дû  ѭִеĴ
    while(true)
    {
      
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}



// **************************** �������� ****************************
