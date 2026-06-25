/*
 * Menu.h
 *
 *  Created on: 2026年3月24日
 *      Author: 24244
 *  Optimized: 2026-06-19 — enum-driven nav, data-driven L1/L2 draw, init_flag lifecycle
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

// ============================================================
// 菜单节点枚举 — 编译期安全替代魔数索引
// ============================================================
typedef enum
{
    MENU_ROOT = 0,

    // 第1层 — 主菜单
    MENU_L1_A, MENU_L1_B, MENU_L1_C, MENU_L1_D, MENU_L1_E, MENU_L1_F,

    // 第2层 — A组 (Record/SAVE/Reproduce)
    MENU_L2_A1, MENU_L2_A2, MENU_L2_A3, MENU_L2_A4, MENU_L2_A5, MENU_L2_A6,
    // 第2层 — B组 (Record/SAVE/Reproduce/Clear)
    MENU_L2_B1, MENU_L2_B2, MENU_L2_B3, MENU_L2_B4, MENU_L2_B5, MENU_L2_B6,
    // 第2层 — C组 (Record/SAVE/Reproduce/Mt9v03)
    MENU_L2_C1, MENU_L2_C2, MENU_L2_C3, MENU_L2_C4, MENU_L2_C5, MENU_L2_C6,
    // 第2层 — D组 (测试项 + ESC)
    MENU_L2_D1, MENU_L2_D2, MENU_L2_D3, MENU_L2_D4, MENU_L2_D5, MENU_L2_D6, MENU_L2_D7,
    // 第2层 — E组 (Jump: Trigger/Config/Abort/Default/Status)
    MENU_L2_E1, MENU_L2_E2, MENU_L2_E3, MENU_L2_E4, MENU_L2_E5, MENU_L2_E6,

    // 第3层 — A组 (路径1: 录制/保存/回放/清除)
    MENU_L3_A1, MENU_L3_A2, MENU_L3_A3, MENU_L3_A4, MENU_L3_A5,
    // 第3层 — B组 (路径2: 录制/保存/回放/清除)
    MENU_L3_B1, MENU_L3_B2, MENU_L3_B3, MENU_L3_B4, MENU_L3_B5,
    // 第3层 — C组 (路径3: 录制/保存/回放/清除)
    MENU_L3_C1, MENU_L3_C2, MENU_L3_C3, MENU_L3_C4, MENU_L3_C5,
    // 第3层 — D组测试
    MENU_L3_D1, MENU_L3_D2, MENU_L3_D3, MENU_L3_D4, MENU_L3_D5, MENU_L3_D6,
    // 第3层 — E组 (Jump 功能)
    MENU_L3_E1, MENU_L3_E2, MENU_L3_E3, MENU_L3_E4, MENU_L3_E5,

    MENU_COUNT
} menu_node_t;

// ============================================================
// 键值表结构体 — 菜单状态机节点
// ============================================================
typedef struct
{
    menu_node_t up;                  // 向上跳转索引
    menu_node_t down;                // 向下跳转索引
    menu_node_t enter;               // 确定跳转索引
    void       (*draw)(void);        // 当前页面显示函数指针
    uint8       cursor;              // 光标位置，0=不使用光标
    uint8       is_static;           // 1=静态页面(不变时不重绘) 0=动态页面(持续刷新)
    uint8       init_flag;           // 一次性初始化标志，离开时自动清零
} key_table;

// ============================================================
// 菜单引擎
// ============================================================
void Menu(void);

#endif /* CODE_MENU_H_ */
