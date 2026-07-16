#ifndef PROJECT_FLASH_H_
#define PROJECT_FLASH_H_

#include "zf_common_headfile.h"

#define NAG_PATH_COUNT             (3U)
#define NAG_SAMPLES_PER_PAGE       (500U)
#define Read_MaxSize               (10000U)

#define NAG_META_PAGE              (1U)
#define NAG_PATH1_START            (95U)
#define NAG_PATH1_END              (65U)
#define NAG_PATH2_START            (64U)
#define NAG_PATH2_END              (34U)
#define NAG_PATH3_START            (33U)
#define NAG_PATH3_END              (3U)

#if FLASH_PAGE_LENGTH < (NAG_SAMPLES_PER_PAGE + NAG_PATH_COUNT)
#error "Flash page is too small for navigation data and metadata slots."
#endif

#define Nag_Set_mileage            (5)
#define Nag_Prev                   (200U)
#define Nag_Yaw                    roll_balance_cascade.posture_value.yaw
#define R_Mileage                  Car.mileage_R
#define L_Mileage                  Car.mileage_L

typedef struct
{
    float Final_Out;
    float Mileage_All;
    float Angle_Run;
    bool Nag_Stop_f;
    uint8 Flash_read_f;
    uint16 size;
    uint16 Run_index;
    uint16 Save_count;
    uint16 Save_index;
    uint8 Save_state;
    uint8 End_f;
    uint8 Flash_page_index;
    uint8 Nag_SystemRun_Index;
    int Prev_mile[Nag_Prev];
} Nag;

extern uint8 Nag_PathSelect;
extern uint8 fuxian;
extern Nag N;
extern float user_set_speed;
extern float Nav_read[Read_MaxSize];

void Init_Nag(void);
void Init_Nag_Path(uint8 path_id);
void Nag_Read(void);
void Nag_Run(void);
void Nag_System(void);
void NagFlashRead(void);
void Run_Nag_Save(void);
void Run_Nag_GPS(void);
void control_navigation(void);

void flash_Nag_Write(void);
void flash_Nag_Read(void);
void flash_Nag_Write_Meta(void);
void flash_Nag_Read_Meta(void);
uint16 Get_Path_SaveIndex(uint8 path_id);
uint8 flash_Nag_Clear_Path(uint8 path_id);

double angle_plan(double angle);

#endif
