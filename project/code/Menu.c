/*
 * Menu.c — 3-Level Hierarchical Menu State Machine
 *
 *  Created on: 2026-6-19
 *      Author: HUGO
 *  Optimized: 2026-06-19 — data-driven draw, enum navigation, init_flag lifecycle
 *
 * Architecture:
 *   A key_table[] drives all navigation. Each entry is a node in a 3-level tree:
 *     Level 0:      Welcome page (MENU_ROOT)
 *     Level 1 (x6): Main menu groups A-F — select a subsystem
 *     Level 2 (x30): Sub-menus within each group — pick an operation
 *     Level 3 (x25): Leaf pages — execute the operation (record / save / replay / clear / jump)
 *
 *   Navigation: up/down/enter fields hold the NEXT node index. Pressing a key
 *   overwrites func_index, then Menu() dispatches the new node's draw callback.
 *   Level 3 has up==down==self (no vertical movement at leaf level);
 *   enter points back to the parent Level 2 entry for "go back".
 *
 *   init_flag lifecycle:
 *   - Each entry has an init_flag (in key_table). Level 3 recording/save/replay/clear
 *     functions use it for one-shot initialization.
 *   - Menu() resets init_flag to 0 when the user LEAVES a node, so re-entering
 *     triggers a fresh init (fixes the old static-once_flag bug).
 *
 *   Redraw optimization:
 *   - is_static=1 (Level 0/1/2): Skip redraw when func_index hasn't changed.
 *   - is_static=0 (Level 3 telemetry): Continuous redraw for live data display.
 */

#include "zf_common_headfile.h"

// ============================================================
// Menu Page Label Tables (const → .rodata, shared across nodes)
// ============================================================

// ---- Level 1: two variants — "E" vs "JUMP" on rows 4-5 ----------
static const char *page_l1_items[] = { "GO", "B", "C", "TEST", "E"    };
static const char *page_l1_jump[]  = { "GO", "B", "C", "TEST", "JUMP" };

// ---- Level 2 group A: Record / SAVE / Reproduce -----------------
static const char *page_l2_a[] = { "Record", "SAVE", "Reproduce", "A_4", "A_5" };

// ---- Level 2 group B: row 1 shows "Clear" instead of "B_4" ------
static const char *page_l2_b[]       = { "Record", "SAVE", "Reproduce", "B_4",  "B_5" };
static const char *page_l2_b_clear[] = { "Record", "SAVE", "Reproduce", "Clear","B_5" };

// ---- Level 2 group C: row 4 shows camera sensor name ------------
static const char *page_l2_c[] = { "Record", "SAVE", "Reproduce", "Mt9v03_text", "C_5" };

// ---- Level 2 group D: 测试模块（直行100m等） --------------------
static const char *page_l2_test[] = { "Straight", "Circle", "Square", "S-Curve", "Custom" };

// ---- Level 2 group E: Jump subsystem commands --------------------
static const char *page_l2_e[] = { "Trigger", "Config", "Abort", "Default", "Status" };

// ============================================================
// Global State
// ============================================================
// func_index — current active menu node (overwritten by key events)
// last_index — previous node; used to detect page transitions and
//              reset init_flag of the node being left.
//              Initialized to MENU_COUNT (invalid sentinel) so the
//              first frame always triggers a full draw.
static menu_node_t  func_index = MENU_ROOT;
static menu_node_t  last_index = MENU_COUNT;

// Cached function pointer — avoids re-reading table[].draw on
// every frame when is_static=1 skips the redraw.
static void       (*current_operation_index)(void);

// ============================================================
// Forward Declarations (required for state table initializers)
// ============================================================
static void fun_0(void);
static void draw_not_implemented(void);
static void draw_menu_items(const char *items[], uint8 count, uint8 cursor, const char *title);
static void draw_l1_menu(void);
static void draw_l2_a(void);
static void draw_l2_b(void);
static void draw_l2_c(void);
static void draw_l2_d(void);
static void draw_l2_e(void);
void fun_a31(void); void fun_a32(void); void fun_a33(void); void fun_a34(void);
void fun_b31(void); void fun_b32(void); void fun_b33(void); void fun_b34(void);
void fun_c31(void); void fun_c32(void); void fun_c33(void); void fun_c34(void);
void fun_e31(void); void fun_e32(void); void fun_e33(void); void fun_e34(void); void fun_e35(void);
void fun_d31(void); void fun_d32(void); void fun_d33(void); void fun_d34(void); void fun_d35(void);

// ============================================================
// Menu State Table — 62 entries, indexed by menu_node_t enum
//   Field layout: { up, down, enter, draw, cursor, is_static, init_flag }
//
//   up/down/enter : next node on UP / DOWN / ENTER key press.
//   draw          : callback invoked when this node is active.
//   cursor        : row 1-6 where the "->" indicator is drawn (0 = unused).
//   is_static     : 1 = static page (skip redraw when func_index unchanged).
//                   0 = dynamic page (telemetry, needs continuous refresh).
//   init_flag     : one-shot init guard; reset to 0 by Menu() when leaving node.
//
//   Level 3 entries: up==down==self (locked), enter=parent L2 node.
// ============================================================
static key_table table_dispaly[MENU_COUNT] =
{
    // ---- Level 0: Welcome ----------------------------------------
    [MENU_ROOT] = { MENU_ROOT, MENU_ROOT, MENU_L1_A, fun_0, 0, 1, 0 },

    // ---- Level 1: Main menu (all share draw_l1_menu) -------------
    [MENU_L1_A] = { MENU_L1_F, MENU_L1_B, MENU_L2_A1, draw_l1_menu, 1, 1, 0 },
    [MENU_L1_B] = { MENU_L1_A, MENU_L1_C, MENU_L2_B1, draw_l1_menu, 2, 1, 0 },
    [MENU_L1_C] = { MENU_L1_B, MENU_L1_D, MENU_L2_C1, draw_l1_menu, 3, 1, 0 },
    [MENU_L1_D] = { MENU_L1_C, MENU_L1_E, MENU_L2_D1, draw_l1_menu, 4, 1, 0 },
    [MENU_L1_E] = { MENU_L1_D, MENU_L1_F, MENU_L2_E1, draw_l1_menu, 5, 1, 0 },
    [MENU_L1_F] = { MENU_L1_E, MENU_L1_A, MENU_ROOT,  draw_l1_menu, 6, 1, 0 }, // ESC → root

    // ---- Level 2 group A: Navigation path 1 submenu --------------
    [MENU_L2_A1] = { MENU_L2_A6, MENU_L2_A2, MENU_L3_A1, draw_l2_a, 1, 1, 0 },
    [MENU_L2_A2] = { MENU_L2_A1, MENU_L2_A3, MENU_L3_A2, draw_l2_a, 2, 1, 0 },
    [MENU_L2_A3] = { MENU_L2_A2, MENU_L2_A4, MENU_L3_A3, draw_l2_a, 3, 1, 0 },
    [MENU_L2_A4] = { MENU_L2_A3, MENU_L2_A5, MENU_L3_A4, draw_l2_a, 4, 1, 0 },
    [MENU_L2_A5] = { MENU_L2_A4, MENU_L2_A6, MENU_L3_A5, draw_l2_a, 5, 1, 0 },
    [MENU_L2_A6] = { MENU_L2_A5, MENU_L2_A1, MENU_L1_A,  draw_l2_a, 6, 1, 0 }, // ESC

    // ---- Level 2 group B: Navigation path 2 submenu --------------
    [MENU_L2_B1] = { MENU_L2_B6, MENU_L2_B2, MENU_L3_B1, draw_l2_b, 1, 1, 0 },
    [MENU_L2_B2] = { MENU_L2_B1, MENU_L2_B3, MENU_L3_B2, draw_l2_b, 2, 1, 0 },
    [MENU_L2_B3] = { MENU_L2_B2, MENU_L2_B4, MENU_L3_B3, draw_l2_b, 3, 1, 0 },
    [MENU_L2_B4] = { MENU_L2_B3, MENU_L2_B5, MENU_L3_B4, draw_l2_b, 4, 1, 0 },
    [MENU_L2_B5] = { MENU_L2_B4, MENU_L2_B6, MENU_L3_B5, draw_l2_b, 5, 1, 0 },
    [MENU_L2_B6] = { MENU_L2_B5, MENU_L2_B1, MENU_L1_B,  draw_l2_b, 6, 1, 0 }, // ESC

    // ---- Level 2 group C: Navigation path 3 submenu --------------
    [MENU_L2_C1] = { MENU_L2_C6, MENU_L2_C2, MENU_L3_C1, draw_l2_c, 1, 1, 0 },
    [MENU_L2_C2] = { MENU_L2_C1, MENU_L2_C3, MENU_L3_C2, draw_l2_c, 2, 1, 0 },
    [MENU_L2_C3] = { MENU_L2_C2, MENU_L2_C4, MENU_L3_C3, draw_l2_c, 3, 1, 0 },
    [MENU_L2_C4] = { MENU_L2_C3, MENU_L2_C5, MENU_L3_C4, draw_l2_c, 4, 1, 0 },
    [MENU_L2_C5] = { MENU_L2_C4, MENU_L2_C6, MENU_L3_C5, draw_l2_c, 5, 1, 0 },
    [MENU_L2_C6] = { MENU_L2_C5, MENU_L2_C1, MENU_L1_C,  draw_l2_c, 6, 1, 0 }, // ESC

    // ---- Level 2 group D: Reserved / unimplemented ---------------
    [MENU_L2_D1] = { MENU_L2_D6, MENU_L2_D2, MENU_L3_D1, draw_l2_d, 1, 1, 0 },
    [MENU_L2_D2] = { MENU_L2_D1, MENU_L2_D3, MENU_L3_D2, draw_l2_d, 2, 1, 0 },
    [MENU_L2_D3] = { MENU_L2_D2, MENU_L2_D4, MENU_L3_D3, draw_l2_d, 3, 1, 0 },
    [MENU_L2_D4] = { MENU_L2_D3, MENU_L2_D5, MENU_L3_D4, draw_l2_d, 4, 1, 0 },
    [MENU_L2_D5] = { MENU_L2_D4, MENU_L2_D6, MENU_L3_D5, draw_l2_d, 5, 1, 0 },
    [MENU_L2_D6] = { MENU_L2_D5, MENU_L2_D1, MENU_L1_D,  draw_l2_d, 6, 1, 0 }, // ESC

    // ---- Level 2 group E: Jump control submenu -------------------
    [MENU_L2_E1] = { MENU_L2_E6, MENU_L2_E2, MENU_L3_E1, draw_l2_e, 1, 1, 0 },
    [MENU_L2_E2] = { MENU_L2_E1, MENU_L2_E3, MENU_L3_E2, draw_l2_e, 2, 1, 0 },
    [MENU_L2_E3] = { MENU_L2_E2, MENU_L2_E4, MENU_L3_E3, draw_l2_e, 3, 1, 0 },
    [MENU_L2_E4] = { MENU_L2_E3, MENU_L2_E5, MENU_L3_E4, draw_l2_e, 4, 1, 0 },
    [MENU_L2_E5] = { MENU_L2_E4, MENU_L2_E6, MENU_L3_E5, draw_l2_e, 5, 1, 0 },
    [MENU_L2_E6] = { MENU_L2_E5, MENU_L2_E1, MENU_L1_E,  draw_l2_e, 6, 1, 0 }, // ESC

    // ---- Level 3 leaf nodes: is_static=0 (live telemetry) --------
    // A group: Path 1 operations
    [MENU_L3_A1] = { MENU_L3_A1, MENU_L3_A1, MENU_L2_A1, fun_a31,              0, 0, 0 },
    [MENU_L3_A2] = { MENU_L3_A2, MENU_L3_A2, MENU_L2_A2, fun_a32,              0, 0, 0 },
    [MENU_L3_A3] = { MENU_L3_A3, MENU_L3_A3, MENU_L2_A3, fun_a33,              0, 0, 0 },
    [MENU_L3_A4] = { MENU_L3_A4, MENU_L3_A4, MENU_L2_A4, fun_a34,              0, 0, 0 },
    [MENU_L3_A5] = { MENU_L3_A5, MENU_L3_A5, MENU_L2_A5, draw_not_implemented, 0, 0, 0 },

    // B group: Path 2 operations
    [MENU_L3_B1] = { MENU_L3_B1, MENU_L3_B1, MENU_L2_B1, fun_b31,              0, 0, 0 },
    [MENU_L3_B2] = { MENU_L3_B2, MENU_L3_B2, MENU_L2_B2, fun_b32,              0, 0, 0 },
    [MENU_L3_B3] = { MENU_L3_B3, MENU_L3_B3, MENU_L2_B3, fun_b33,              0, 0, 0 },
    [MENU_L3_B4] = { MENU_L3_B4, MENU_L3_B4, MENU_L2_B4, fun_b34,              0, 0, 0 },
    [MENU_L3_B5] = { MENU_L3_B5, MENU_L3_B5, MENU_L2_B5, draw_not_implemented, 0, 0, 0 },

    // C group: Path 3 operations
    [MENU_L3_C1] = { MENU_L3_C1, MENU_L3_C1, MENU_L2_C1, fun_c31,              0, 0, 0 },
    [MENU_L3_C2] = { MENU_L3_C2, MENU_L3_C2, MENU_L2_C2, fun_c32,              0, 0, 0 },
    [MENU_L3_C3] = { MENU_L3_C3, MENU_L3_C3, MENU_L2_C3, fun_c33,              0, 0, 0 },
    [MENU_L3_C4] = { MENU_L3_C4, MENU_L3_C4, MENU_L2_C4, fun_c34,              0, 0, 0 },
    [MENU_L3_C5] = { MENU_L3_C5, MENU_L3_C5, MENU_L2_C5, draw_not_implemented, 0, 0, 0 },

    // D group: 测试模块 — L3 leaf pages
    [MENU_L3_D1] = { MENU_L3_D1, MENU_L3_D1, MENU_L2_D1, fun_d31, 0, 0, 0 },
    [MENU_L3_D2] = { MENU_L3_D2, MENU_L3_D2, MENU_L2_D2, fun_d32, 0, 0, 0 },
    [MENU_L3_D3] = { MENU_L3_D3, MENU_L3_D3, MENU_L2_D3, fun_d33, 0, 0, 0 },
    [MENU_L3_D4] = { MENU_L3_D4, MENU_L3_D4, MENU_L2_D4, fun_d34, 0, 0, 0 },
    [MENU_L3_D5] = { MENU_L3_D5, MENU_L3_D5, MENU_L2_D5, fun_d35, 0, 0, 0 },

    // E group: Jump control leaf pages
    [MENU_L3_E1] = { MENU_L3_E1, MENU_L3_E1, MENU_L2_E1, fun_e31, 0, 0, 0 },
    [MENU_L3_E2] = { MENU_L3_E2, MENU_L3_E2, MENU_L2_E2, fun_e32, 0, 0, 0 },
    [MENU_L3_E3] = { MENU_L3_E3, MENU_L3_E3, MENU_L2_E3, fun_e33, 0, 0, 0 },
    [MENU_L3_E4] = { MENU_L3_E4, MENU_L3_E4, MENU_L2_E4, fun_e34, 0, 0, 0 },
    [MENU_L3_E5] = { MENU_L3_E5, MENU_L3_E5, MENU_L2_E5, fun_e35, 0, 0, 0 },
};

// ============================================================
// Data-Driven Draw Functions
// ============================================================
// These 6 thin wrappers replace 36 copy-paste functions from the
// original code. Each per-group wrapper reads the cursor position
// from the current table entry and delegates to draw_menu_items().
// The label tables are compile-time constants in .rodata — no
// runtime string duplication.

// ---- Core: draws a 6-row menu page (rows 1-6) -------------------
//   items[0..count-1] = row labels (row 1..count)
//   row count+1..5 get "ESC", row 6 always gets "ESC"
//   The "->" cursor indicator is drawn on the row matching `cursor`.
static void draw_menu_items(const char *items[], uint8 count, uint8 cursor,
                            const char *title)
{
    uint8 i;
    ips_clear();
    for (i = 1; i <= 6; i++)
    {
        if (i == cursor)
            ips_show_string(0, 16 * i, "->");
        ips_show_string(20, 16 * i, (i <= count) ? items[i - 1] : "ESC");
    }
    ips_show_string(8 * 23, 16 * 19, title);
}

// ---- Level 1 wrapper: selects variant based on cursor position ----
//   Cursor 4 or 5 → "JUMP" variant; others → "E" variant.
static void draw_l1_menu(void)
{
    key_table *e = &table_dispaly[func_index];
    const char **items = (e->cursor == 4 || e->cursor == 5) ? page_l1_jump
                                                              : page_l1_items;
    draw_menu_items(items, 5, e->cursor, "Page_1");
}

// ---- Level 2 group A wrapper ------------------------------------
static void draw_l2_a(void)
{
    draw_menu_items(page_l2_a, 5, table_dispaly[func_index].cursor, "Page_2");
}

// ---- Level 2 group B wrapper: cursor 1 shows "Clear" ------------
static void draw_l2_b(void)
{
    const char **items = (table_dispaly[func_index].cursor == 1) ? page_l2_b_clear
                                                                  : page_l2_b;
    draw_menu_items(items, 5, table_dispaly[func_index].cursor, "Page_2");
}

// ---- Level 2 groups C / D / E wrappers --------------------------
static void draw_l2_c(void)
{
    draw_menu_items(page_l2_c, 5, table_dispaly[func_index].cursor, "Page_2");
}

static void draw_l2_d(void)
{
    draw_menu_items(page_l2_test, 5, table_dispaly[func_index].cursor, "Test");
}

static void draw_l2_e(void)
{
    draw_menu_items(page_l2_e, 5, table_dispaly[func_index].cursor, "Jump");
}

// ---- Level 0: Welcome / splash screen ---------------------------
static void fun_0(void)
{
    ips_show_string(100, 300, "Designed_by_WMCA");
}

// ---- Fallback for unimplemented leaf nodes -----------------------
static void draw_not_implemented(void)
{
    ips_clear();
    ips_show_string(8 * 6, 16 * 4, "Not Implemented");
}

// ============================================================
// Menu() — Core Dispatch Engine
// ============================================================
// Called every super-loop iteration from main_cm7_0.c.
// Key events are set asynchronously by the 5ms PIT ISR (key_scan).
//
// Three-phase logic per frame:
//   1. Read key flags → update func_index from state table.
//   2. If func_index changed:
//      a. Reset init_flag of the PREVIOUS node (enables re-entry).
//      b. Execute the new node's draw callback.
//   3. If func_index unchanged:
//      a. Static pages (is_static=1): skip redraw (save CPU/SPI).
//      b. Dynamic pages (is_static=0): continuous redraw (telemetry).
void Menu(void)
{
    // ---- Phase 1: key-to-index translation -----------------------
    if (key1_flag) { func_index = table_dispaly[func_index].up;    key1_clear(); }
    if (key2_flag) { func_index = table_dispaly[func_index].down;  key2_clear(); }
    if (key3_flag) { func_index = table_dispaly[func_index].enter; key3_clear(); }

    // ---- Phase 2: page transition --------------------------------
    if (func_index != last_index)
    {
        // Reset init_flag of the node being left so re-entry
        // triggers a fresh one-shot initialization.
        // Guard: last_index == MENU_COUNT on first frame (sentinel).
        if (last_index < MENU_COUNT)
        {
            table_dispaly[last_index].init_flag = 0;
        }
        current_operation_index = table_dispaly[func_index].draw;
        (*current_operation_index)();
        last_index = func_index;
    }
    // ---- Phase 3: same-page refresh ------------------------------
    else
    {
        // Static menu pages don't need redraw — only dynamic
        // telemetry pages (recording, replay, jump status) do.
        if (!table_dispaly[func_index].is_static)
        {
            (*current_operation_index)();
        }
    }
}

// ============================================================
// Level 3 — Group A: Navigation Path 1
// ============================================================
// Each function follows the same pattern:
//   - Check e->init_flag: if 0, run one-shot init and set to 1.
//   - Display live telemetry (every frame).
//   Menu() resets init_flag to 0 when the user navigates away,
//   so re-entering the page triggers fresh initialization.

// fun_a31: Path 1 — Start Recording
//   Sets Nag_SystemRun_Index=1 to begin logging trajectory data.
//   Displays real-time distance, yaw angle, and save index.
void fun_a31(void)
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        Init_Nag_Path(1);
        N.Nag_SystemRun_Index = 1;          // Enter recording mode
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 0, "P1 recording....");
    ips_show_string(8 * 0, 16 * 1, "Distance"); ips_show_float(8 * 10, 16 * 1, N.Mileage_All, 5, 3);
    ips_show_string(8 * 0, 16 * 2, "Angular");  ips_show_float(8 * 10, 16 * 2, Nag_Yaw, 5, 3);
    ips_show_string(8 * 0, 16 * 3, "SaveIdx");   ips_show_int(8 * 10, 16 * 3, N.Save_index, 5);
}

// fun_a32: Path 1 — Save (Stop Recording)
//   Sets N.End_f=1 to finalize the recorded trajectory.
void fun_a32(void)
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        if (N.Nag_SystemRun_Index == 1) { N.End_f = 1; }
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 0, "P1 SAVE....");
}

// fun_a33: Path 1 — Replay
//   Sets Nag_SystemRun_Index=2 to enter replay mode.
//   Displays PID output values, yaw, angle run, and navigation index.
void fun_a33(void)
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        Init_Nag_Path(1);
        N.Nag_SystemRun_Index = 2;          // Enter replay mode
        fuxian = 1;                          // Enable trajectory tracking
        target_speed = user_set_speed;
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 0, "P1 Replay");
    ips_show_string(8 * 0, 16 * 1, "BASE");     ips_show_float(8 * 10, 16 * 1, -roll_balance_cascade.angular_speed_cycle.out, 5, 3);
    ips_show_string(8 * 0, 16 * 2, "TRACK");    ips_show_float(8 * 10, 16 * 2, track_cascade.track_cycle.out, 5, 3);
    ips_show_string(8 * 0, 16 * 3, "GD_SC");    ips_show_float(8 * 10, 16 * 3, N.Final_Out, 5, 3);
    ips_show_string(8 * 0, 16 * 4, "Nag_Yaw");  ips_show_float(8 * 10, 16 * 4, Nag_Yaw, 5, 3);
    ips_show_string(8 * 0, 16 * 5, "Angle_Run");ips_show_float(8 * 10, 16 * 5, N.Angle_Run, 5, 3);
    ips_show_string(0, 16 * 6, "SaveIdx:");      ips_show_int(8 * 10, 16 * 6, N.Save_index, 5);
    ips_show_string(0, 16 * 7, "Nav0:");         ips_show_float(8 * 10, 16 * 7, Nav_read[0] / 100.0f, 5, 2);
}

// fun_a34: Path 1 — Clear
//   Erases Path 1 flash pages and zeroes the save index in the meta page.
//   Then re-initializes Path 1 to a fresh state.
void fun_a34(void)
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        uint8 page;
        // Erase all flash pages in Path 1 range ---------------
        for (page = NAG_PATH1_END; page <= NAG_PATH1_START; page++)
        {
            if (flash_check(0, page))
                flash_erase_page(0, page);
        }
        // Zero Path 1 save index in meta page ----------------
        flash_buffer_clear();
        flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_union_buffer[MaxSize + 0].uint32_type = 0;   // offset 0 = Path 1
        if (flash_check(0, NAG_META_PAGE))
            flash_erase_page(0, NAG_META_PAGE);
        flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_buffer_clear();

        Init_Nag_Path(1);                   // Re-init to clean state
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 2, "P1 Data Cleared!");
}

// ============================================================
// Level 3 — Group B: Navigation Path 2
// ============================================================
// Identical structure to Group A but operates on Path 2.
// flash_union_buffer offset is MaxSize+1 (Path 2's save index).

void fun_b31(void)  // Path 2 — Start Recording
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        Init_Nag_Path(2);
        N.Nag_SystemRun_Index = 1;
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 0, "P2 recording....");
    ips_show_string(8 * 0, 16 * 1, "Distance"); ips_show_float(8 * 10, 16 * 1, N.Mileage_All, 5, 3);
    ips_show_string(8 * 0, 16 * 2, "Angular");  ips_show_float(8 * 10, 16 * 2, Nag_Yaw, 5, 3);
    ips_show_string(8 * 0, 16 * 3, "SaveIdx");   ips_show_int(8 * 10, 16 * 3, N.Save_index, 5);
}

void fun_b32(void)  // Path 2 — Save
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        if (N.Nag_SystemRun_Index == 1) { N.End_f = 1; }
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 0, "P2 SAVE....");
}

void fun_b33(void)  // Path 2 — Replay
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        Init_Nag_Path(2);
        N.Nag_SystemRun_Index = 2;
        fuxian = 1;
        target_speed = user_set_speed;
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 0, "P2 Replay");
    ips_show_string(8 * 0, 16 * 1, "BASE");     ips_show_float(8 * 10, 16 * 1, -roll_balance_cascade.angular_speed_cycle.out, 5, 3);
    ips_show_string(8 * 0, 16 * 2, "TRACK");    ips_show_float(8 * 10, 16 * 2, track_cascade.track_cycle.out, 5, 3);
    ips_show_string(8 * 0, 16 * 3, "GD_SC");    ips_show_float(8 * 10, 16 * 3, N.Final_Out, 5, 3);
    ips_show_string(8 * 0, 16 * 4, "Nag_Yaw");  ips_show_float(8 * 10, 16 * 4, Nag_Yaw, 5, 3);
    ips_show_string(8 * 0, 16 * 5, "Angle_Run");ips_show_float(8 * 10, 16 * 5, N.Angle_Run, 5, 3);
    ips_show_string(0, 16 * 6, "SaveIdx:");      ips_show_int(8 * 10, 16 * 6, N.Save_index, 5);
    ips_show_string(0, 16 * 7, "Nav0:");         ips_show_float(8 * 10, 16 * 7, Nav_read[0] / 100.0f, 5, 2);
}

void fun_b34(void)  // Path 2 — Clear
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        uint8 page;
        for (page = NAG_PATH2_END; page <= NAG_PATH2_START; page++)
        {
            if (flash_check(0, page))
                flash_erase_page(0, page);
        }
        flash_buffer_clear();
        flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_union_buffer[MaxSize + 1].uint32_type = 0;   // offset 1 = Path 2
        if (flash_check(0, NAG_META_PAGE))
            flash_erase_page(0, NAG_META_PAGE);
        flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_buffer_clear();

        Init_Nag_Path(2);
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 2, "P2 Data Cleared!");
}

// ============================================================
// Level 3 — Group C: Navigation Path 3
// ============================================================
// Identical structure to Groups A/B but operates on Path 3.
// flash_union_buffer offset is MaxSize+2.

void fun_c31(void)  // Path 3 — Start Recording
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        Init_Nag_Path(3);
        N.Nag_SystemRun_Index = 1;
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 0, "P3 recording....");
    ips_show_string(8 * 0, 16 * 1, "Distance"); ips_show_float(8 * 10, 16 * 1, N.Mileage_All, 5, 3);
    ips_show_string(8 * 0, 16 * 2, "Angular");  ips_show_float(8 * 10, 16 * 2, Nag_Yaw, 5, 3);
    ips_show_string(8 * 0, 16 * 3, "SaveIdx");   ips_show_int(8 * 10, 16 * 3, N.Save_index, 5);
}

void fun_c32(void)  // Path 3 — Save
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        if (N.Nag_SystemRun_Index == 1) { N.End_f = 1; }
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 0, "P3 SAVE....");
}

void fun_c33(void)  // Path 3 — Replay
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        Init_Nag_Path(3);
        N.Nag_SystemRun_Index = 2;
        fuxian = 1;
        target_speed = user_set_speed;
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 0, "P3 Replay");
    ips_show_string(8 * 0, 16 * 1, "BASE");     ips_show_float(8 * 10, 16 * 1, -roll_balance_cascade.angular_speed_cycle.out, 5, 3);
    ips_show_string(8 * 0, 16 * 2, "TRACK");    ips_show_float(8 * 10, 16 * 2, track_cascade.track_cycle.out, 5, 3);
    ips_show_string(8 * 0, 16 * 3, "GD_SC");    ips_show_float(8 * 10, 16 * 3, N.Final_Out, 5, 3);
    ips_show_string(8 * 0, 16 * 4, "Nag_Yaw");  ips_show_float(8 * 10, 16 * 4, Nag_Yaw, 5, 3);
    ips_show_string(8 * 0, 16 * 5, "Angle_Run");ips_show_float(8 * 10, 16 * 5, N.Angle_Run, 5, 3);
    ips_show_string(0, 16 * 6, "SaveIdx:");      ips_show_int(8 * 10, 16 * 6, N.Save_index, 5);
    ips_show_string(0, 16 * 7, "Nav0:");         ips_show_float(8 * 10, 16 * 7, Nav_read[0] / 100.0f, 5, 2);
}

void fun_c34(void)  // Path 3 — Clear
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        uint8 page;
        for (page = NAG_PATH3_END; page <= NAG_PATH3_START; page++)
        {
            if (flash_check(0, page))
                flash_erase_page(0, page);
        }
        flash_buffer_clear();
        flash_read_page_to_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_union_buffer[MaxSize + 2].uint32_type = 0;   // offset 2 = Path 3
        if (flash_check(0, NAG_META_PAGE))
            flash_erase_page(0, NAG_META_PAGE);
        flash_write_page_from_buffer(0, NAG_META_PAGE, FLASH_PAGE_LENGTH);
        flash_buffer_clear();

        Init_Nag_Path(3);
        e->init_flag = 1;
    }
    ips_show_string(8 * 0, 16 * 2, "P3 Data Cleared!");
}

// ============================================================
// Level 3 — Group E: Jump Control
// ============================================================
// These functions interact with the 7-state jump FSM in Body_ctrl.c.
// Unlike the navigation path functions, they don't use init_flag
// because their operations are fire-and-forget (trigger, abort, reset).

void fun_e31(void)  // Jump — Trigger
{
    ips_show_string(8 * 0, 16 * 0, "Jump Trigger");
    jump_trigger();                          // Start the jump sequence
    ips_show_string(8 * 0, 16 * 2, "Result:");
    if (jump_cfg.state != JUMP_IDLE)
    {
        ips_show_string(8 * 0, 16 * 3, "OK - Jump Started");
    }
    else
    {
        ips_show_string(8 * 0, 16 * 3, "FAIL - Check State");
    }
}

void fun_e32(void)  // Jump — Config Display (read-only)
{
    ips_show_string(8 * 0, 16 * 0, "Jump Config");
    ips_show_string(8 * 0, 16 * 1, "ChargeT");   ips_show_int(8 * 10, 16 * 1, jump_cfg.charge_ticks, 5);
    ips_show_string(8 * 0, 16 * 2, "ChargeD");   ips_show_int(8 * 10, 16 * 2, jump_cfg.charge_duty, 5);
    ips_show_string(8 * 0, 16 * 3, "AirTmout");  ips_show_int(8 * 10, 16 * 3, jump_cfg.airborne_timeout, 5);
    ips_show_string(8 * 0, 16 * 4, "Boost");     ips_show_float(8 * 10, 16 * 4, jump_cfg.forward_motor_boost, 5, 1);
    ips_show_string(8 * 0, 16 * 5, "TiltAbort"); ips_show_float(8 * 10, 16 * 5, jump_cfg.max_tilt_abort, 5, 1);
    ips_show_string(8 * 0, 16 * 6, "AirAccTh");  ips_show_float(8 * 10, 16 * 6, jump_cfg.airborne_acc_threshold, 5, 2);
    ips_show_string(8 * 0, 16 * 7, "LandAccTh"); ips_show_float(8 * 10, 16 * 7, jump_cfg.landing_acc_threshold, 5, 2);
    ips_show_string(8 * 0, 16 * 8, "VisionEn");  ips_show_int(8 * 10, 16 * 8, jump_cfg.vision_jump_enable, 5);
}

void fun_e33(void)  // Jump — Abort (emergency stop)
{
    ips_show_string(8 * 0, 16 * 0, "Jump Abort");
    jump_abort();
    ips_show_string(8 * 0, 16 * 2, "Aborted - IDLE");
}

void fun_e34(void)  // Jump — Reset to Defaults
{
    ips_show_string(8 * 0, 16 * 0, "Jump Default");
    jump_config_default();
    ips_show_string(8 * 0, 16 * 2, "Reset to Defaults");
}

void fun_e35(void)  // Jump — Live Status
{
    ips_show_string(8 * 0, 16 * 0, "Jump Status");
    ips_show_string(8 * 0, 16 * 1, "State:");   ips_show_int(8 * 10, 16 * 1, jump_cfg.state, 3);
    ips_show_string(8 * 0, 16 * 2, "Elapsed:"); ips_show_int(8 * 10, 16 * 2, jump_cfg.elapsed, 5);
    ips_show_string(8 * 0, 16 * 3, "Count:");   ips_show_int(8 * 10, 16 * 3, jump_cfg.jump_count, 5);
    ips_show_string(8 * 0, 16 * 4, "PeakAcc:"); ips_show_float(8 * 10, 16 * 4, jump_cfg.peak_acc_magnitude, 5, 2);
    ips_show_string(8 * 0, 16 * 5, "CanTrig:"); ips_show_int(8 * 10, 16 * 5, jump_can_trigger(), 3);
}

// ============================================================
// Level 3 — Group D: 测试模块
// ============================================================

// fun_d31: 直行100m综合测试
//   进入时触发 straight_test_start()，init_flag=1 后持续刷新显示
//   测试中显示距离+偏差；完成后显示侧偏+航向漂移+评级
void fun_d31(void)
{
    key_table *e = &table_dispaly[func_index];
    if (e->init_flag == 0)
    {
        straight_test_start();
        e->init_flag = 1;
    }

    ips_show_string(8 * 0, 16 * 0, "Straight 100m");

    switch (straight_test.state)
    {
    case STRAIGHT_LOCKING:
        ips_show_string(8 * 0, 16 * 2, "Locking heading...");
        ips_show_string(8 * 0, 16 * 3, "GPS:");
        if (gnss.antenna_direction_state == 1)
            ips_show_float(8 * 10, 16 * 3, gnss.antenna_direction, 5, 1);
        else if (gnss.state == 1)
            ips_show_float(8 * 10, 16 * 3, gnss.direction, 5, 1);
        else
            ips_show_string(8 * 10, 16 * 3, "NO GPS");
        break;

    case STRAIGHT_RUNNING:
        ips_show_string(8 * 0, 16 * 2, "Dist:");     ips_show_float(8 * 10, 16 * 2, straight_test.distance * 0.01f, 5, 1);
        ips_show_string(8 * 0, 16 * 3, "Heading:");  ips_show_float(8 * 10, 16 * 3, straight_test.target_heading, 5, 1);
        ips_show_string(8 * 0, 16 * 4, "Yaw:");      ips_show_float(8 * 10, 16 * 4, (float)Nag_Yaw, 5, 1);
        ips_show_string(8 * 0, 16 * 5, "Corr:");     ips_show_float(8 * 10, 16 * 5, straight_test.steer_correction, 5, 1);
        ips_show_string(8 * 0, 16 * 6, "Sat:");      ips_show_int(8 * 10, 16 * 6, gnss.satellite_used, 3);
        break;

    case STRAIGHT_DONE:
        ips_show_string(8 * 0, 16 * 2, "=== RESULT ===");
        ips_show_string(8 * 0, 16 * 3, "Lateral:");   ips_show_float(8 * 10, 16 * 3, straight_test.lateral_deviation, 5, 2);
        ips_show_string(8 * 0, 16 * 4, "Drift:");     ips_show_float(8 * 10, 16 * 4, straight_test.yaw_drift, 5, 1);
        ips_show_string(8 * 0, 16 * 5, "Rating:");    ips_show_int(8 * 10, 16 * 5, straight_test.rating, 1);
        ips_show_string(8 * 12, 16 * 5, "/5");
        ips_show_string(8 * 0, 16 * 7, "Press <- to exit");
        break;

    default:
        break;
    }
}

void fun_d32(void) { draw_not_implemented(); }  // Circle — 待实现
void fun_d33(void) { draw_not_implemented(); }  // Square — 待实现
void fun_d34(void) { draw_not_implemented(); }  // S-Curve — 待实现
void fun_d35(void) { draw_not_implemented(); }  // Custom — 待实现
