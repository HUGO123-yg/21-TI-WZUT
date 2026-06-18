/*
 * Menu.c
 *
 *  Created on: 2026��3��24��
 *      Author: 24244
 */

#include "zf_common_headfile.h"

int  func_index = 0; //��ʼ��ʾ��ӭ����
int  last_index = 127; //last��ʼΪ��Чֵ


void (*current_operation_index)(void);       //��ʾ��������ָ��(��ǰ��������)

void fun_0(void);

void fun_a1(void);
void fun_b1(void);
void fun_c1(void);
void fun_d1(void);
void fun_e1(void);
void fun_f1(void);

void fun_a21(void);
void fun_a22(void);
void fun_a23(void);
void fun_a24(void);
void fun_a25(void);
void fun_a26(void);

void fun_b21(void);
void fun_b22(void);
void fun_b23(void);
void fun_b24(void);
void fun_b25(void);
void fun_b26(void);

void fun_c21(void);
void fun_c22(void);
void fun_c23(void);
void fun_c24(void);
void fun_c25(void);
void fun_c26(void);

void fun_d21(void);
void fun_d22(void);
void fun_d23(void);
void fun_d24(void);
void fun_d25(void);
void fun_d26(void);

void fun_e21(void);
void fun_e22(void);
void fun_e23(void);
void fun_e24(void);
void fun_e25(void);
void fun_e26(void);

void fun_a31(void);
void fun_a32(void);
void fun_a33(void);
void fun_a34(void);
void fun_a35(void);

void fun_b31(void);
void fun_b32(void);
void fun_b33(void);
void fun_b34(void);
void fun_b35(void);

void fun_c31(void);
void fun_c32(void);
void fun_c33(void);
void fun_c34(void);
void fun_c35(void);

void fun_d31(void);
void fun_d32(void);
void fun_d33(void);
void fun_d34(void);
void fun_d35(void);

void fun_e31(void);
void fun_e32(void);
void fun_e33(void);
void fun_e34(void);
void fun_e35(void);

key_table table_display[100]=                 //�ṹ������
{
//{���������ϣ����£�ȷ�ϣ���ʾ����}
    //��0��
    {0,0,0,1,(*fun_0)},                     //AIIT_meun

    //��1��
    {1,6,2, 7,(*fun_a1)},
    {2,1,3,13,(*fun_b1)},
    {3,2,4,19,(*fun_c1)},
    {4,3,5,25,(*fun_d1)},
    {5,4,6,31,(*fun_e1)},
    {6,5,1, 0,(*fun_f1)},

    //��2��
    {7,12, 8, 37, (*fun_a21)},
    {8, 7, 9, 38, (*fun_a22)},
    {9, 8, 10,39, (*fun_a23)},
    {10,9, 11,40, (*fun_a24)},
    {11,10,12,41, (*fun_a25)},
    {12,11,7,  1, (*fun_a26)},            //ESC

    {13,18,14,42, (*fun_b21)},
    {14,13,15,43, (*fun_b22)},
    {15,14,16,44, (*fun_b23)},
    {16,15,17,45, (*fun_b24)},
    {17,16,18,46, (*fun_b25)},
    {18,17,13, 2, (*fun_b26)},           //ESC

    {19,24,20,47, (*fun_c21)},
    {20,19,21,48, (*fun_c22)},
    {21,20,22,49, (*fun_c23)},
    {22,21,23,50, (*fun_c24)},
    {23,22,24,51, (*fun_c25)},
    {24,23,19,3,  (*fun_c26)},           //ESC

    {25,30,26,52, (*fun_d21)},
    {26,25,27,53, (*fun_d22)},
    {27,26,28,54, (*fun_d23)},
    {28,27,29,55, (*fun_d24)},
    {29,28,30,56, (*fun_d25)},
    {30,29,25,4,  (*fun_d26)},           //ESC

    {31,36,32,57, (*fun_e21)},
    {32,31,33,58, (*fun_e22)},
    {33,32,34,59, (*fun_e23)},
    {34,33,35,60, (*fun_e24)},
    {35,34,36,61, (*fun_e25)},
    {36,35,31,5,  (*fun_e26)},           //ESC

    //��3��
    {37,37,37,7, (*fun_a31)},
    {38,38,38,8, (*fun_a32)},
    {39,39,39,9, (*fun_a33)},
    {40,40,40,10,(*fun_a34)},
    {41,41,41,11,(*fun_a35)},

    {42,42,42,13,(*fun_b31)},
    {43,43,43,14,(*fun_b32)},
    {44,44,44,15,(*fun_b33)},
    {45,45,45,16,(*fun_b34)},
    {46,46,46,17,(*fun_b35)},

    {47,47,47,19,(*fun_c31)},
    {48,48,48,20,(*fun_c32)},
    {49,49,49,21,(*fun_c33)},
    {50,50,50,22,(*fun_c34)},
    {51,51,51,23,(*fun_c35)},

    {52,52,52,25,(*fun_d31)},
    {53,53,53,26,(*fun_d32)},
    {54,54,54,27,(*fun_d33)},
    {55,55,55,28,(*fun_d34)},
    {56,56,56,29,(*fun_d35)},

    {57,57,57,31,(*fun_e31)},
    {58,58,58,32,(*fun_e32)},
    {59,59,59,33,(*fun_e33)},
    {60,60,60,34,(*fun_e34)},
    {61,61,61,35,(*fun_e35)},
};


void Menu(void)//�˵�����
{
                uint8 key1_event;
                uint8 key2_event;
                uint8 key3_event;
                uint32 interrupt_state;

                interrupt_state = interrupt_global_disable();
                key1_event = key1_flag;
                key2_event = key2_flag;
                key3_event = key3_flag;
                key1_flag = 0;
                key2_flag = 0;
                key3_flag = 0;
                interrupt_global_enable(interrupt_state);

                if(key1_event)
                {

                    func_index = table_display[func_index].up;    //���Ϸ�
                    BUZZER_check(50);
                }
                if(key2_event)
                {

                    func_index = table_display[func_index].down;    //���·�
                     BUZZER_check(50);

                }
                if(key3_event)
                {

                    func_index = table_display[func_index].enter;    //ȷ��
                    BUZZER_check(50);

                }


            if (func_index != last_index)
            {
                current_operation_index = table_display[func_index].current_operation;

                ips200_clear();
                (*current_operation_index)();//ִ�е�ǰ��������
                last_index = func_index;

            }
            else
            {
                (*current_operation_index)();//ִ�е�ǰ��������
            }
  }


///*********��0��***********/
void fun_0(void)
{


//    show_rgb565_image(0,16*5, (const uint16 *)gImage_ORRN, 240, 135, 240, 135, 0);
    ips200_show_string(100,300,"Designed_by_WMCA");


}

////////////////////////////////////////////////////////////////////////////////////////////////////////��һ��///////////////////////////////////////////////////////////////////////////////////////////////////////////
void fun_a1(void)
{

    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");

}

void fun_b1(void)
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");



}

void fun_c1(void)
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");



}

void fun_d1(void)
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "GO");                ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "JUMP");
    ips200_show_string(20, 16*6, "ESC");

}

void fun_e1(void)
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "JUMP");
    ips200_show_string(20, 16*6, "ESC");

}

void fun_f1(void)
{
    ips200_show_string(0,  16*6, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");

}

////////////////////////////////////////////////////////////////////////////////////////////////////////�ڶ���///////////////////////////////////////////////////////////////////////////////////////////////////////////
void fun_a21(void)//
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_a22(void)
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_a23(void)
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_a24(void)
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_a25(void)
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}
void fun_a26(void)
{
    ips200_show_string(0,  16*6, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void fun_b21(void)
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "Clear");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b22(void)
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "B_4");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b23(void)
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1,"Record");               ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "B_4");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b24(void)
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "Record");               ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "B_4");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b25(void)
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1,"Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "B_4");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b26(void)
{
    ips200_show_string(0,  16*6, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "B_4");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void fun_c21(void)
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2,  "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c22(void)
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2,  "SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c23(void)
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1,"Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c24(void)
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c25(void)
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2,"SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c26(void)
{
    ips200_show_string(0,  16*6, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2,"SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void fun_d21(void)
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "D_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d22(void)
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "D_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d23(void)
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1, "D_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d24(void)
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "D_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d25(void)
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "D_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d26(void)
{
    ips200_show_string(0,  16*6, "->");
    ips200_show_string(20, 16*1, "D_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void fun_e21(void)
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "Trigger");                ips200_show_string(8*23, 16*19, "Jump");
    ips200_show_string(20, 16*2, "Config");
    ips200_show_string(20, 16*3, "Abort");
    ips200_show_string(20, 16*4, "Default");
    ips200_show_string(20, 16*5, "Status");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e22(void)
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "Trigger");                ips200_show_string(8*23, 16*19, "Jump");
    ips200_show_string(20, 16*2, "Config");
    ips200_show_string(20, 16*3, "Abort");
    ips200_show_string(20, 16*4, "Default");
    ips200_show_string(20, 16*5, "Status");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e23(void)
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1, "Trigger");                ips200_show_string(8*23, 16*19, "Jump");
    ips200_show_string(20, 16*2, "Config");
    ips200_show_string(20, 16*3, "Abort");
    ips200_show_string(20, 16*4, "Default");
    ips200_show_string(20, 16*5, "Status");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e24(void)
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "Trigger");                ips200_show_string(8*23, 16*19, "Jump");
    ips200_show_string(20, 16*2, "Config");
    ips200_show_string(20, 16*3, "Abort");
    ips200_show_string(20, 16*4, "Default");
    ips200_show_string(20, 16*5, "Status");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e25(void)
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "Trigger");                ips200_show_string(8*23, 16*19, "Jump");
    ips200_show_string(20, 16*2, "Config");
    ips200_show_string(20, 16*3, "Abort");
    ips200_show_string(20, 16*4, "Default");
    ips200_show_string(20, 16*5, "Status");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e26(void)
{
    ips200_show_string(0,  16*6, "->");
    ips200_show_string(20, 16*1, "Trigger");                ips200_show_string(8*23, 16*19, "Jump");
    ips200_show_string(20, 16*2, "Config");
    ips200_show_string(20, 16*3, "Abort");
    ips200_show_string(20, 16*4, "Default");
    ips200_show_string(20, 16*5, "Status");
    ips200_show_string(20, 16*6, "ESC");
}



////////////////////////////////////////////////////////////////////////////////////////////////////////������///////////////////////////////////////////////////////////////////////////////////////////////////////////

void fun_a31(void)//��Ŀһ���
{
    ips_show_string(8*0, 16*0, "P1 recording....");

    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        Init_Nag_Path(1);             // ��ʼ��·��1
        N.Nag_SystemRun_Index = 1;     // �����ߵ�¼��
        once_flag = 1;
    }

    ips_show_string(8*0, 16*1, "Distance"); ips_show_float(8*10, 16*1, N.Mileage_All, 5, 3);
    ips_show_string(8*0, 16*2, "Angular");  ips_show_float(8*10, 16*2, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*3, "SaveIdx");   ips_show_int(8*10, 16*3, N.Save_index, 5);
}

void fun_a32(void)//��Ŀ�����
{
 ips_show_string(8*0, 16*0, "P1 SAVE....");

    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        if (N.Nag_SystemRun_Index == 1)
        {
            N.End_f = 1;  // ��ֹ�ߵ����У�ֹͣ�ɼ�
        }
        once_flag = 1;
    }

}

void fun_a33(void)//��Ŀ��
{
 
    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
       Init_Nag_Path(1);             // ѡ��·��1
        N.Nag_SystemRun_Index = 2;     // ����
        fuxian = 1;                    // �켣������
        target_speed = user_set_speed;
        once_flag = 1;
    }

    ips_show_string(8*0, 16*0, "P1 Replay");
    ips_show_string(8*0, 16*1, "BASE");     ips_show_float(8*10, 16*1, -roll_balance_cascade.angular_speed_cycle.out, 5, 3);
    ips_show_string(8*0, 16*2, "TRACK");    ips_show_float(8*10, 16*2, track_cascade.track_cycle.out, 5, 3);
    ips_show_string(8*0, 16*3, "GD_SC");    ips_show_float(8*10, 16*3, N.Final_Out, 5, 3);
    ips_show_string(8*0, 16*4, "Nag_Yaw");  ips_show_float(8*10, 16*4, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*5, "Angle_Run");ips_show_float(8*10, 16*5, N.Angle_Run, 5, 3);
    ips_show_string(0, 16*6, "SaveIdx:");  ips_show_int(8*10, 16*6, N.Save_index, 5);
    ips_show_string(0, 16*7, "Nav0:");     ips_show_float(8*10, 16*7, Nav_read[0] / 100.0f, 5, 2);
}

void fun_a34(void)//��Ŀ��
{
    ips_show_string(8*0, 16*0, "P1 Clear...");

    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        // ����·��1������Flashҳ
        for (uint8 page = NAG_PATH1_END; page <= NAG_PATH1_START; page++)
        {
            if (flash_check(0, page))
                flash_erase_page(0, page);
        }
        // ���Ԫ����ҳ��·��1��Save_index
        flash_buffer_clear();
        flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_union_buffer[MaxSize + 0].uint32_type = 0;  // ·��1 Save_index����
        if (flash_check(0, NAG_META_PAGE))
            flash_erase_page(0, NAG_META_PAGE);
        flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_buffer_clear();

        Init_Nag_Path(1);  // ���³�ʼ��
        once_flag = 1;
    }

    ips_show_string(8*0, 16*2, "P1 Data Cleared!");
}


void fun_a35(void)//ǿ���������
{

}

void fun_b31(void)
{
    ips_show_string(8*0, 16*0, "P2 recording....");

    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        Init_Nag_Path(2);             // ��ʼ��·��2
        N.Nag_SystemRun_Index = 1;     // �����ߵ�¼��
        once_flag = 1;
    }

    ips_show_string(8*0, 16*1, "Distance"); ips_show_float(8*10, 16*1, N.Mileage_All, 5, 3);
    ips_show_string(8*0, 16*2, "Angular");  ips_show_float(8*10, 16*2, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*3, "SaveIdx");   ips_show_int(8*10, 16*3, N.Save_index, 5);


}

void fun_b32(void)
{
ips_show_string(8*0, 16*0, "P2 SAVE....");

    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        if (N.Nag_SystemRun_Index == 1)
        {
            N.End_f = 1;
        }
        once_flag = 1;
    }


}

void fun_b33(void)
{
static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        Init_Nag_Path(2);               // ѡ��·��2
        N.Nag_SystemRun_Index = 2;     // ����
        fuxian = 1;
        target_speed = user_set_speed;
        once_flag = 1;
    }

    ips_show_string(8*0, 16*0, "P2 Replay");
    ips_show_string(8*0, 16*1, "BASE");     ips_show_float(8*10, 16*1, -roll_balance_cascade.angular_speed_cycle.out, 5, 3);
    ips_show_string(8*0, 16*2, "TRACK");    ips_show_float(8*10, 16*2, track_cascade.track_cycle.out, 5, 3);
    ips_show_string(8*0, 16*3, "GD_SC");    ips_show_float(8*10, 16*3, N.Final_Out, 5, 3);
    ips_show_string(8*0, 16*4, "Nag_Yaw");  ips_show_float(8*10, 16*4, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*5, "Angle_Run");ips_show_float(8*10, 16*5, N.Angle_Run, 5, 3);
    ips_show_string(0, 16*6, "SaveIdx:");  ips_show_int(8*10, 16*6, N.Save_index, 5);
    ips_show_string(0, 16*7, "Nav0:");     ips_show_float(8*10, 16*7, Nav_read[0] / 100.0f, 5, 2);
}




void fun_b34(void)
{
   ips_show_string(8*0, 16*0, "P2 Clear...");

    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        for (uint8 page = NAG_PATH2_END; page <= NAG_PATH2_START; page++)
        {
            if (flash_check(0, page))
                flash_erase_page(0, page);
        }
        flash_buffer_clear();
        flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_union_buffer[MaxSize + 1].uint32_type = 0;  // ·��2 Save_index����
        if (flash_check(0, NAG_META_PAGE))
            flash_erase_page(0, NAG_META_PAGE);
        flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_buffer_clear();

        Init_Nag_Path(2);
        once_flag = 1;
    }

    ips_show_string(8*0, 16*2, "P2 Data Cleared!");


}

void fun_b35(void)
{



}

void fun_c31(void)
{
  ips_show_string(8*0, 16*0, "P3 recording....");

    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        Init_Nag_Path(3);             // ��ʼ��·��3
        N.Nag_SystemRun_Index = 1;     // �����ߵ�¼��
        once_flag = 1;
    }

    ips_show_string(8*0, 16*1, "Distance"); ips_show_float(8*10, 16*1, N.Mileage_All, 5, 3);
    ips_show_string(8*0, 16*2, "Angular");  ips_show_float(8*10, 16*2, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*3, "SaveIdx");   ips_show_int(8*10, 16*3, N.Save_index, 5);

}

void fun_c32(void)
{
  ips_show_string(8*0, 16*0, "P3 SAVE....");

    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        if (N.Nag_SystemRun_Index == 1)
        {
            N.End_f = 1;
        }
        once_flag = 1;
    }

}

void fun_c33(void)
{
  static uint8 once_flag = 0;
    if (once_flag == 0)
    {
      Init_Nag_Path(3);             // ѡ��·��3
        N.Nag_SystemRun_Index = 2;     // ����
        fuxian = 1;
        target_speed = user_set_speed;
        once_flag = 1;
    }

    ips_show_string(8*0, 16*0, "P3 Replay");
    ips_show_string(8*0, 16*1, "BASE");     ips_show_float(8*10, 16*1, -roll_balance_cascade.angular_speed_cycle.out, 5, 3);
    ips_show_string(8*0, 16*2, "TRACK");    ips_show_float(8*10, 16*2, track_cascade.track_cycle.out, 5, 3);
    ips_show_string(8*0, 16*3, "GD_SC");    ips_show_float(8*10, 16*3, N.Final_Out, 5, 3);
    ips_show_string(8*0, 16*4, "Nag_Yaw");  ips_show_float(8*10, 16*4, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*5, "Angle_Run");ips_show_float(8*10, 16*5, N.Angle_Run, 5, 3);
    ips_show_string(0, 16*6, "SaveIdx:");  ips_show_int(8*10, 16*6, N.Save_index, 5);
    ips_show_string(0, 16*7, "Nav0:");     ips_show_float(8*10, 16*7, Nav_read[0] / 100.0f, 5, 2);
}



void fun_c34(void)
{
 ips_show_string(8*0, 16*0, "P3 Clear...");

    static uint8 once_flag = 0;
    if (once_flag == 0)
    {
        for (uint8 page = NAG_PATH3_END; page <= NAG_PATH3_START; page++)
        {
            if (flash_check(0, page))
                flash_erase_page(0, page);
        }
        flash_buffer_clear();
        flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_union_buffer[MaxSize + 2].uint32_type = 0;  // ·��3 Save_index����
        if (flash_check(0, NAG_META_PAGE))
            flash_erase_page(0, NAG_META_PAGE);
        flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_buffer_clear();

        Init_Nag_Path(3);
        once_flag = 1;
    }

    ips_show_string(8*0, 16*2, "P3 Data Cleared!");

}


void fun_c35(void)
{


}

void fun_d31(void)
{


}

void fun_d32(void)
{


}

void fun_d33(void)
{


}

void fun_d34(void)
{


}


void fun_d35(void)
{


}

void fun_e31(void)
{
    ips_show_string(8*0, 16*0, "Jump Trigger");
    jump_trigger();
    ips_show_string(8*0, 16*2, "Result:");
    if (jump_cfg.state != JUMP_IDLE)
    {
        ips_show_string(8*0, 16*3, "OK - Jump Started");
    }
    else
    {
        ips_show_string(8*0, 16*3, "FAIL - Check State");
    }
}

void fun_e32(void)
{
    ips_show_string(8*0, 16*0, "Jump Config");
    ips_show_string(8*0, 16*1, "ChargeT");  ips_show_int(8*10, 16*1, jump_cfg.charge_ticks, 5);
    ips_show_string(8*0, 16*2, "ChargeD");  ips_show_int(8*10, 16*2, jump_cfg.charge_duty, 5);
    ips_show_string(8*0, 16*3, "AirTmout"); ips_show_int(8*10, 16*3, jump_cfg.airborne_timeout, 5);
    ips_show_string(8*0, 16*4, "Boost");    ips_show_float(8*10, 16*4, jump_cfg.forward_motor_boost, 5, 1);
    ips_show_string(8*0, 16*5, "TiltAbort"); ips_show_float(8*10, 16*5, jump_cfg.max_tilt_abort, 5, 1);
    ips_show_string(8*0, 16*6, "AirAccTh"); ips_show_float(8*10, 16*6, jump_cfg.airborne_acc_threshold, 5, 2);
    ips_show_string(8*0, 16*7, "LandAccTh"); ips_show_float(8*10, 16*7, jump_cfg.landing_acc_threshold, 5, 2);
    ips_show_string(8*0, 16*8, "VisionEn"); ips_show_int(8*10, 16*8, jump_cfg.vision_jump_enable, 5);
}

void fun_e33(void)
{
    ips_show_string(8*0, 16*0, "Jump Abort");
    jump_abort();
    ips_show_string(8*0, 16*2, "Aborted - IDLE");
}

void fun_e34(void)
{
    ips_show_string(8*0, 16*0, "Jump Default");
    jump_config_default();
    ips_show_string(8*0, 16*2, "Reset to Defaults");
}

void fun_e35(void)
{
    ips_show_string(8*0, 16*0, "Jump Status");
    ips_show_string(8*0, 16*1, "State:");   ips_show_int(8*10, 16*1, jump_cfg.state, 3);
    ips_show_string(8*0, 16*2, "Elapsed:"); ips_show_int(8*10, 16*2, jump_cfg.elapsed, 5);
    ips_show_string(8*0, 16*3, "Count:");   ips_show_int(8*10, 16*3, jump_cfg.jump_count, 5);
    ips_show_string(8*0, 16*4, "PeakAcc:"); ips_show_float(8*10, 16*4, jump_cfg.peak_acc_magnitude, 5, 2);
    ips_show_string(8*0, 16*5, "CanTrig:"); ips_show_int(8*10, 16*5, jump_can_trigger(), 3);
}

