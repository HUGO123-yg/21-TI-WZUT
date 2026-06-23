//*********************用户设置区域****************************//

#include "zf_common_headfile.h"

#define MaxSize 500 // flash存储的最大页面

#define Read_MaxSize 10000 // 最大读取设置，1w个应该是够了
#define NAG_POINTS_PER_PAGE       MaxSize
#define NAG_META_SAVE_INDEX_OFFSET NAG_POINTS_PER_PAGE
#define NAG_PATH_COUNT            3
#define NAG_PATH1_META_SLOT       0
#define NAG_PATH2_META_SLOT       1
#define NAG_PATH3_META_SLOT       2
#define NAG_META_MAGIC_OFFSET     (NAG_META_SAVE_INDEX_OFFSET + NAG_PATH_COUNT)
#define NAG_META_VERSION_OFFSET   (NAG_META_MAGIC_OFFSET + 1)
#define NAG_META_SAMPLE_CM_OFFSET (NAG_META_VERSION_OFFSET + 1)
#define NAG_META_CHECKSUM_OFFSET  (NAG_META_SAMPLE_CM_OFFSET + 1)
#define NAG_META_MAGIC            0x4E414731u
#define NAG_META_VERSION          1u

#define NAG_RUN_IDLE              0
#define NAG_RUN_RECORD            1
#define NAG_RUN_PRELOAD           2
#define NAG_RUN_REPLAY            3

// 参数范围 <0 - 95>
#define Nag_End_Page 1    // flash中止页面
#define Nag_Start_Page 95 // flah起始页面


#define NAG_META_PAGE         1    // 元数据页 (存3条路径的Save_index)
#define NAG_PATH1_START       95   // 路径1起始页
#define NAG_PATH1_END         65   // 路径1结束页
#define NAG_PATH2_START       64   // 路径2起始页
#define NAG_PATH2_END         34   // 路径2结束页
#define NAG_PATH3_START       33   // 路径3起始页
#define NAG_PATH3_END         3    // 路径3结束页






#define Nag_Set_mileage 5
#define Nag_Prev 200            // 前瞻
#define Nag_Yaw  roll_balance_cascade.posture_value.yaw  // 陀螺仪读取出来的偏航角
#define R_Mileage Car.mileage_R // 右轮里程
#define L_Mileage Car.mileage_L // 左轮里程
//********************************************************//


typedef struct
{
       float Final_Out;    // 最终输出,最终偏差，
       float Mileage_All;  // 里程计数，即编码器计算出来跑过的距离,5cm为单位
       float Angle_Run;    // 读取的偏航角，推车得到的角度
       bool Nag_Stop_f;    // 惯导中止flag
       uint8 Flash_read_f; // 惯导读取flag
       uint16 size;        // 惯导数组索引通用计数--当前缓冲区已存入的数据条数（也是下一个要写入的位置索引）
       uint16 Run_index;
       uint16 Save_count;
       uint16 Save_index; // 保存的flag
       uint8 Save_state;
       uint8 End_f; // 中止flag
       // 与flash相关的
       uint8 Flash_page_index;      // flash页面索引
       uint8 Flash_Save_Page_Index; // flash保存页码索引
       uint8 Nag_SystemRun_Index;   // 惯导执行索引
       // 暂时未开发部分
       int Prev_mile[Nag_Prev]; // 前瞻
} Nag;

extern uint8 Nag_PathSelect;  // 当前选择的路径编号 (1/2/3)


extern uint8 fuxian;
extern Nag N;                        // 整个变量的结构体，方便开发和移植
extern float user_set_speed; // 用户设置的目标速度，仅由按键四调整
extern float Nav_read[Read_MaxSize];

void Run_Nag_Save(void);
void flash_Nag_Write(void);
void flash_Nag_Read(void);
void Run_Nag_GPS(void);
float angle_plan(float angle);
void NagFlashRead(void);
void Init_Nag(void);
void Nag_System(void);
void Nag_Service(void);

// ============== 多路径新增接口 ==============
void Init_Nag_Path(uint8 path_id);     // 按路径初始化惯导 (设置Flash_page_index)
void flash_Nag_Write_Meta(void);      // 写入元数据页 (3条路径的Save_index)
void flash_Nag_Read_Meta(void);
void Nag_Clear_Path_Meta(uint8 path_id);
// 读取元数据页
uint16 Get_Path_SaveIndex(uint8 path_id); // 获取指定路径的Save_index

