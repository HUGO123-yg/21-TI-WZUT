/*
 * Menu.c
 *
 *  Created on: 2026年3月24日
 *      Author: 24244
 */

#include "zf_common_headfile.h"

int  func_index = 0; //初始显示欢迎界面
int  last_index = 127; //last初始为无效值


void (*current_operation_index)(void);       //显示函数索引指针(当前操作索引)

key_table table_dispaly[100]=                 //结构体数组
{
//{索引，向上，向下，确认，显示函数}
    //第0层
    {0,0,0,1,(*fun_0)},                     //AIIT_meun

    //第1层
    {1,6,2, 7,(*fun_a1)},
    {2,1,3,13,(*fun_b1)},
    {3,2,4,19,(*fun_c1)},
    {4,3,5,25,(*fun_d1)},
    {5,4,6,31,(*fun_e1)},
    {6,5,1, 0,(*fun_f1)},

    //第2层
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

    //第3层
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


// 判断当前菜单索引是否为需要电机输出的运行功能
static uint8 menu_is_running_action(int index)
{
    return (index == 37 || index == 39 || index == 42 || index == 44 ||
            index == 47 || index == 49 || index == 57 || index == 58 ||
            index == 59 || index == 60 || index == 61 || index == 52) ? 1 : 0;
}

static uint8 menu_is_bridge_action(int index)
{
    return (index == 52) ? 1 : 0;
}

static uint8 menu_is_obstacle_action(int index)
{
    return (index == 39) ? 1 : 0;
}

static uint8 menu_is_stair_action(int index)
{
    return (index == 59 || index == 60 || index == 61) ? 1 : 0;
}

void Menu(void)//菜单函数
{



                if(key1_flag)
                {

                    func_index = table_dispaly[func_index].up;    //向上翻
                    key1_clear();
                }
                if(key2_flag)
                {

                    func_index = table_dispaly[func_index].down;    //向下翻
                     key2_clear();

                }
                if(key3_flag)
                {

                    func_index = table_dispaly[func_index].enter;    //确认
                    key3_clear();

                }

            if (menu_is_stair_action(last_index) && !menu_is_stair_action(func_index))
            {
                stair_abort();
            }

            if (menu_is_bridge_action(last_index) && !menu_is_bridge_action(func_index))
            {
                target_speed = 0.0f;
                bridge_test_active = 0;
                N.Final_Out = 0.0f;
                bridge_init();
            }

            if (menu_is_obstacle_action(last_index) && !menu_is_obstacle_action(func_index))
            {
                obstacle_abort();
            }

            // 仅在进入会动的运行功能页面时允许电机输出，离开或回到菜单时立即静止
            if (menu_is_running_action(func_index))
            {
                system_armed = 1;
            }
            else
            {
                system_armed = 0;
            }


            if (func_index != last_index)
            {
                current_operation_index = table_dispaly[func_index].current_operation;

                ips200_clear();
                (*current_operation_index)();//执行当前操作函数
                last_index = func_index;

            }
            else
            {
                (*current_operation_index)();//执行当前操作函数
            }
  }


///*********第0层***********/
void fun_0()
{


//    show_rgb565_image(0,16*5, (const uint16 *)gImage_ORRN, 240, 135, 240, 135, 0);
    ips200_show_string(100,300,"Designed_by_WMCA");


}

////////////////////////////////////////////////////////////////////////////////////////////////////////第一层///////////////////////////////////////////////////////////////////////////////////////////////////////////
void fun_a1()
{

    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");

}

void fun_b1()
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");



}

void fun_c1()
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");



}

void fun_d1()
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "GO");                ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");

}

void fun_e1()
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");

}

void fun_f1()
{
    ips200_show_string(0,  16*6, "->");
    ips200_show_string(20, 16*1, "GO");                 ips200_show_string(8*23, 16*19, "Page_1");
    ips200_show_string(20, 16*2, "B");
    ips200_show_string(20, 16*3, "C");
    ips200_show_string(20, 16*4, "D");
    ips200_show_string(20, 16*5, "E");
    ips200_show_string(20, 16*6, "ESC");

}

////////////////////////////////////////////////////////////////////////////////////////////////////////第二层///////////////////////////////////////////////////////////////////////////////////////////////////////////
void fun_a21()//
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_a22()
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_a23()
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_a24()
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_a25()
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "A_4");
    ips200_show_string(20, 16*5, "A_5");
    ips200_show_string(20, 16*6, "ESC");
}
void fun_a26()
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

void fun_b21()
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "Clear");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b22()
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "B_4");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b23()
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1,"Record");               ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "B_4");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b24()
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "Record");               ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "B_4");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b25()
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1,"Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "B_4");
    ips200_show_string(20, 16*5, "B_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_b26()
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
void fun_c21()
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2,  "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c22()
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2,  "SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c23()
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1,"Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c24()
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "SAVE");
    ips200_show_string(20, 16*3, "Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c25()
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "Record");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2,"SAVE");
    ips200_show_string(20, 16*3,"Reproduce");
    ips200_show_string(20, 16*4, "Mt9v03_text");
    ips200_show_string(20, 16*5, "C_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_c26()
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
void fun_d21()
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "Bridge_Test");        ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d22()
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "Bridge_Test");        ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d23()
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1, "Bridge_Test");        ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d24()
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "Bridge_Test");        ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d25()
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "Bridge_Test");        ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_d26()
{
    ips200_show_string(0,  16*6, "->");
    ips200_show_string(20, 16*1, "Bridge_Test");        ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "D_2");
    ips200_show_string(20, 16*3, "D_3");
    ips200_show_string(20, 16*4, "D_4");
    ips200_show_string(20, 16*5, "D_5");
    ips200_show_string(20, 16*6, "ESC");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void fun_e21()
{
    ips200_show_string(0,  16*1, "->");
    ips200_show_string(20, 16*1, "Rot_Subj");               ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "Start_2");
    ips200_show_string(20, 16*3, "Start_3");
    ips200_show_string(20, 16*4, "Start_4");
    ips200_show_string(20, 16*5, "E_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e22()
{
    ips200_show_string(0,  16*2, "->");
    ips200_show_string(20, 16*1, "Start_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "Start_2");
    ips200_show_string(20, 16*3, "Start_3");
    ips200_show_string(20, 16*4, "Start_4");
    ips200_show_string(20, 16*5, "E_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e23()
{
    ips200_show_string(0,  16*3, "->");
    ips200_show_string(20, 16*1, "Start_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "Start_2");
    ips200_show_string(20, 16*3, "Start_3");
    ips200_show_string(20, 16*4, "Start_4");
    ips200_show_string(20, 16*5, "E_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e24()
{
    ips200_show_string(0,  16*4, "->");
    ips200_show_string(20, 16*1, "Start_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "Start_2");
    ips200_show_string(20, 16*3, "Start_3");
    ips200_show_string(20, 16*4, "Start_4");
    ips200_show_string(20, 16*5, "E_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e25()
{
    ips200_show_string(0,  16*5, "->");
    ips200_show_string(20, 16*1, "Start_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "Start_2");
    ips200_show_string(20, 16*3, "Start_3");
    ips200_show_string(20, 16*4, "Start_4");
    ips200_show_string(20, 16*5, "E_5");
    ips200_show_string(20, 16*6, "ESC");
}

void fun_e26()
{
    ips200_show_string(0,  16*6, "->");
    ips200_show_string(20, 16*1, "Start_1");                ips200_show_string(8*23, 16*19, "Page_2");
    ips200_show_string(20, 16*2, "Start_2");
    ips200_show_string(20, 16*3, "Start_3");
    ips200_show_string(20, 16*4, "Start_4");
    ips200_show_string(20, 16*5, "E_5");
    ips200_show_string(20, 16*6, "ESC");
}



////////////////////////////////////////////////////////////////////////////////////////////////////////第三层///////////////////////////////////////////////////////////////////////////////////////////////////////////

void fun_a31()//科目一相关
{
    ips_show_string(8*0, 16*0, "P1 recording....");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        Init_Nag_Path(1);             // 初始化路径1
        N.Nag_SystemRun_Index = 1;     // 启动惯导录制
        once_flag = 1;
    }

    ips_show_string(8*0, 16*1, "Distance"); ips_show_float(8*10, 16*1, N.Mileage_All, 5, 3);
    ips_show_string(8*0, 16*2, "Angular");  ips_show_float(8*10, 16*2, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*3, "SaveIdx");   ips_show_int(8*10, 16*3, N.Save_index, 5);
}

void fun_a32()//科目二相关
{
 ips_show_string(8*0, 16*0, "P1 SAVE....");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        if (N.Nag_SystemRun_Index == 1)
        {
            N.End_f = 1;  // 中止惯导运行，停止采集
        }
        once_flag = 1;
    }

}

void fun_a33()//科目三
{
 
    if(last_index != func_index)
    {
       Init_Nag_Path(1);             // 选择路径1
        N.Nag_SystemRun_Index = 2;     // 复现
        fuxian = 1;                    // 轨迹环开启
        target_speed = user_set_speed;
        terrain_course_reset(Car.mileage);
#if defined(USE_OBSTACLE_CONTROL) && (OBSTACLE_AUTO_ARM_IN_SUBJECT3 != 0)
        obstacle_arm(Car.mileage);
#endif
    }

    if(key4_flag)
    {
#ifdef USE_OBSTACLE_CONTROL
        obstacle_arm(Car.mileage);
#endif
        key4_clear();
    }

    ips_show_string(8*0, 16*0, "P1 Replay");
    ips_show_string(8*0, 16*1, "BASE");     ips_show_float(8*10, 16*1, -roll_balance_cascade.angular_speed_cycle.out, 5, 3);
    ips_show_string(8*0, 16*2, "TRACK");    ips_show_float(8*10, 16*2, track_cascade.track_cycle.out, 5, 3);
    ips_show_string(8*0, 16*3, "GD_SC");    ips_show_float(8*10, 16*3, N.Final_Out, 5, 3);
    ips_show_string(8*0, 16*4, "Nag_Yaw");  ips_show_float(8*10, 16*4, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*5, "Angle_Run");ips_show_float(8*10, 16*5, N.Angle_Run, 5, 3);
    ips_show_string(0, 16*6, "SaveIdx:");  ips_show_int(8*10, 16*6, N.Save_index, 5);
    ips_show_string(0, 16*7, "Nav0:");     ips_show_float(8*10, 16*7, Nav_read[0] / 100.0f, 5, 2);
    ips_show_string(0, 16*8, "ObsSt:");     ips_show_int(8*10, 16*8, obstacle_get_state(), 2);
    ips_show_string(0, 16*9, "ObsD:");      ips_show_float(8*10, 16*9, obstacle_get_distance_cm(), 5, 1);
    ips_show_string(0, 16*10, "ObsTry:");   ips_show_int(8*10, 16*10, obstacle_get_retry_count(), 2);
    ips_show_string(0, 16*11, "Ter:");      ips_show_int(8*10, 16*11, terrain_get_state(), 2);
    ips_show_string(0, 16*12, "CM:");       ips_show_float(8*10, 16*12, terrain_get_course_mileage(), 5, 1);
    ips_show_string(0, 16*13, "BW/OW:");    ips_show_int(8*10, 16*13, terrain_bridge_mileage_window_active(), 1);
                                             ips_show_int(8*13, 16*13, terrain_obstacle_mileage_window_active(), 1);
}

void fun_a34()//科目四
{
    ips_show_string(8*0, 16*0, "P1 Clear...");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        // 擦除路径1的所有Flash页
        for (uint8 page = NAG_PATH1_END; page <= NAG_PATH1_START; page++)
        {
            if (flash_check(0, page))
                flash_erase_page(0, page);
        }
        // 清除元数据页中路径1的Save_index
        flash_buffer_clear();
        flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_union_buffer[MaxSize + 0].uint32_type = 0;  // 路径1 Save_index清零
        if (flash_check(0, NAG_META_PAGE))
            flash_erase_page(0, NAG_META_PAGE);
        flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_buffer_clear();

        Init_Nag_Path(1);  // 重新初始化
        once_flag = 1;
    }

    ips_show_string(8*0, 16*2, "P1 Data Cleared!");
}


void fun_a35()//强制清除数据
{

}

void fun_b31()
{
    ips_show_string(8*0, 16*0, "P2 recording....");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        Init_Nag_Path(2);             // 初始化路径2
        N.Nag_SystemRun_Index = 1;     // 启动惯导录制
        once_flag = 1;
    }

    ips_show_string(8*0, 16*1, "Distance"); ips_show_float(8*10, 16*1, N.Mileage_All, 5, 3);
    ips_show_string(8*0, 16*2, "Angular");  ips_show_float(8*10, 16*2, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*3, "SaveIdx");   ips_show_int(8*10, 16*3, N.Save_index, 5);


}

void fun_b32()
{
ips_show_string(8*0, 16*0, "P2 SAVE....");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        if (N.Nag_SystemRun_Index == 1)
        {
            N.End_f = 1;
        }
        once_flag = 1;
    }


}

void fun_b33()
{
static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        Init_Nag_Path(2);               // 选择路径2
        N.Nag_SystemRun_Index = 2;     // 复现
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




void fun_b34()
{
   ips_show_string(8*0, 16*0, "P2 Clear...");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        for (uint8 page = NAG_PATH2_END; page <= NAG_PATH2_START; page++)
        {
            if (flash_check(0, page))
                flash_erase_page(0, page);
        }
        flash_buffer_clear();
        flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_union_buffer[MaxSize + 1].uint32_type = 0;  // 路径2 Save_index清零
        if (flash_check(0, NAG_META_PAGE))
            flash_erase_page(0, NAG_META_PAGE);
        flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_buffer_clear();

        Init_Nag_Path(2);
        once_flag = 1;
    }

    ips_show_string(8*0, 16*2, "P2 Data Cleared!");


}

void fun_b35()
{



}

void fun_c31()
{
  ips_show_string(8*0, 16*0, "P3 recording....");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        Init_Nag_Path(3);             // 初始化路径3
        N.Nag_SystemRun_Index = 1;     // 启动惯导录制
        once_flag = 1;
    }

    ips_show_string(8*0, 16*1, "Distance"); ips_show_float(8*10, 16*1, N.Mileage_All, 5, 3);
    ips_show_string(8*0, 16*2, "Angular");  ips_show_float(8*10, 16*2, Nag_Yaw, 5, 3);
    ips_show_string(8*0, 16*3, "SaveIdx");   ips_show_int(8*10, 16*3, N.Save_index, 5);

}

void fun_c32()
{
  ips_show_string(8*0, 16*0, "P3 SAVE....");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        if (N.Nag_SystemRun_Index == 1)
        {
            N.End_f = 1;
        }
        once_flag = 1;
    }

}

void fun_c33()
{
  static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
      Init_Nag_Path(3);             // 选择路径3
        N.Nag_SystemRun_Index = 2;     // 复现
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



void fun_c34()
{
 ips_show_string(8*0, 16*0, "P3 Clear...");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        for (uint8 page = NAG_PATH3_END; page <= NAG_PATH3_START; page++)
        {
            if (flash_check(0, page))
                flash_erase_page(0, page);
        }
        flash_buffer_clear();
        flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_union_buffer[MaxSize + 2].uint32_type = 0;  // 路径3 Save_index清零
        if (flash_check(0, NAG_META_PAGE))
            flash_erase_page(0, NAG_META_PAGE);
        flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_buffer_clear();

        Init_Nag_Path(3);
        once_flag = 1;
    }

    ips_show_string(8*0, 16*2, "P3 Data Cleared!");

}


void fun_c35()
{


}

void fun_d31()
{
    if(last_index != func_index)
    {
        target_speed = BRIDGE_TEST_SPEED_RPM;
        bridge_test_active = 1;
        STOP_FALG = 1;
        fuxian = 0;
        N.Nag_SystemRun_Index = 0;
        N.Final_Out = 0.0f;
        bridge_init();
    }

    ips_show_string(8*0, 16*0, "Bridge Test");
    ips_show_string(8*0, 16*1, "Speed:"); ips_show_float(8*8, 16*1, target_speed, 5, 1);
    ips_show_string(8*0, 16*2, "State:"); ips_show_int(8*8, 16*2, bridge_get_state(), 1);
    ips_show_string(8*0, 16*3, "Roll:");  ips_show_float(8*8, 16*3, roll_balance_cascade.posture_value.rol, 5, 1);
    ips_show_string(8*0, 16*4, "Mile:");  ips_show_float(8*8, 16*4, Car.mileage, 5, 1);
}

void fun_d32()
{


}

void fun_d33()
{


}

void fun_d34()
{


}


void fun_d35()
{


}

void fun_e31()//科目一
{
    ips_show_string(8*0, 16*0, "Sub1 Rot90");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        rotation_start_turns(ROT_CW, 900, 0.25f);
        once_flag = 1;
    }

    ips_show_string(8*0, 16*2, "Angle:"); ips_show_float(8*8, 16*2, rotation.accumulated_angle, 5, 1);
}

void fun_e32()//科目二
{
    ips_show_string(8*0, 16*0, "Sub2 Rot180");

    static uint8_t once_flag = 0;
    if (once_flag == 0)
    {
        rotation_start_turns(ROT_CW, 900, 0.5f);
        once_flag = 1;
    }

    ips_show_string(8*0, 16*2, "Angle:"); ips_show_float(8*8, 16*2, rotation.accumulated_angle, 5, 1);
}

void fun_e33()//科目三
{
    if(last_index != func_index)
    {
        stair_jump_start();
    }

    if(key4_flag)
    {
        stair_jump_start();
        key4_clear();
    }

    stair_jump_run();
}

void fun_e34()
{
    if(last_index != func_index)
    {
        stair_single_start();
    }

    if(key4_flag)
    {
        stair_single_start();
        key4_clear();
    }

    stair_single_run();

}


void fun_e35()
{
    if(last_index != func_index)
    {
        stair_seq_start();
    }

    if(key4_flag)
    {
        stair_seq_start();
        key4_clear();
    }

    stair_seq_run();
}

