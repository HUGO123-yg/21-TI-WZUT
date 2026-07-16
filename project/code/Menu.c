#include "zf_common_headfile.h"
#include "Rotation.h"

typedef void (*menu_action_t)(void);

typedef enum
{
    MENU_ITEM_ACTION,
    MENU_ITEM_SUBMENU,
    MENU_ITEM_BACK
} menu_item_type_t;

typedef struct menu_page menu_page_t;

typedef struct
{
    const char *name;
    menu_item_type_t type;
    menu_action_t action;
    const menu_page_t *submenu;
} menu_item_t;

struct menu_page
{
    const char *title;
    const menu_item_t *items;
    uint8 item_count;
};

typedef enum
{
    MENU_VIEW_WELCOME,
    MENU_VIEW_LIST,
    MENU_VIEW_ACTION
} menu_view_t;

#define MENU_ARRAY_SIZE(array) ((uint8)(sizeof(array) / sizeof((array)[0])))

/*
 * Add a normal menu option by copying one MENU_ACTION line in the menu
 * configuration below, then changing only its displayed name and callback.
 * The callback runs repeatedly while the action page is open. Put one-shot
 * work inside `if (Menu_IsActionFirstCall())`.
 */
#define MENU_ACTION(name_, callback_) \
    { (name_), MENU_ITEM_ACTION, (callback_), NULL }
#define MENU_SUBMENU(name_, page_) \
    { (name_), MENU_ITEM_SUBMENU, NULL, &(page_) }
#define MENU_BACK(name_) \
    { (name_), MENU_ITEM_BACK, NULL, NULL }

#define MENU_MAX_DEPTH 4U

static uint8 menu_action_first_call = 0U;

int Menu_IsActionFirstCall(void)
{
    return (menu_action_first_call != 0U);
}

static void nav_record_action(uint8 path_id, const char *title)
{
    ips_show_string(0, 16 * 0, title);

    if (Menu_IsActionFirstCall())
    {
        Init_Nag_Path(path_id);
        N.Nag_SystemRun_Index = 1;
    }

    ips_show_string(0, 16 * 1, "Distance");
    ips_show_float(8 * 10, 16 * 1, N.Mileage_All, 5, 3);
    ips_show_string(0, 16 * 2, "Angular");
    ips_show_float(8 * 10, 16 * 2, Nag_Yaw, 5, 3);
    ips_show_string(0, 16 * 3, "SaveIdx");
    ips_show_int(8 * 10, 16 * 3, N.Save_index, 5);
}

static void nav_save_action(const char *title)
{
    ips_show_string(0, 16 * 0, title);

    if (Menu_IsActionFirstCall() && N.Nag_SystemRun_Index == 1)
    {
        N.End_f = 1;
    }
}

static void nav_replay_action(uint8 path_id, const char *title)
{
    if (Menu_IsActionFirstCall())
    {
        Init_Nag_Path(path_id);
        N.Nag_SystemRun_Index = 2;
        fuxian = 1;
        target_speed = user_set_speed;
    }

    ips_show_string(0, 16 * 0, title);
    ips_show_string(0, 16 * 1, "BASE");
    ips_show_float(8 * 10, 16 * 1,
                   -roll_balance_cascade.angular_speed_cycle.out, 5, 3);
    ips_show_string(0, 16 * 2, "TRACK");
    ips_show_float(8 * 10, 16 * 2, track_cascade.track_cycle.out, 5, 3);
    ips_show_string(0, 16 * 3, "GD_SC");
    ips_show_float(8 * 10, 16 * 3, N.Final_Out, 5, 3);
    ips_show_string(0, 16 * 4, "Nag_Yaw");
    ips_show_float(8 * 10, 16 * 4, Nag_Yaw, 5, 3);
    ips_show_string(0, 16 * 5, "Angle_Run");
    ips_show_float(8 * 10, 16 * 5, N.Angle_Run, 5, 3);
    ips_show_string(0, 16 * 6, "SaveIdx:");
    ips_show_int(8 * 10, 16 * 6, N.Save_index, 5);
    ips_show_string(0, 16 * 7, "Nav0:");
    ips_show_float(8 * 10, 16 * 7, Nav_read[0] / 100.0f, 5, 2);
}

static void nav_clear_action(uint8 path_id, const char *title,
                             const char *done_text)
{
    ips_show_string(0, 16 * 0, title);

    if (Menu_IsActionFirstCall())
    {
        flash_Nag_Clear_Path(path_id);
    }

    ips_show_string(0, 16 * 2, done_text);
}

static void path1_record(void)
{
    nav_record_action(1U, "P1 recording....");
}

static void path1_save(void)
{
    nav_save_action("P1 SAVE....");
}

static void path1_replay(void)
{
    nav_replay_action(1U, "P1 Replay");
}

static void path1_clear(void)
{
    nav_clear_action(1U, "P1 Clear...", "P1 Data Cleared!");
}

static void path2_record(void)
{
    nav_record_action(2U, "P2 recording....");
}

static void path2_save(void)
{
    nav_save_action("P2 SAVE....");
}

static void path2_replay(void)
{
    nav_replay_action(2U, "P2 Replay");
}

static void path2_clear(void)
{
    nav_clear_action(2U, "P2 Clear...", "P2 Data Cleared!");
}

static void path3_record(void)
{
    nav_record_action(3U, "P3 recording....");
}

static void path3_save(void)
{
    nav_save_action("P3 SAVE....");
}

static void path3_replay(void)
{
    nav_replay_action(3U, "P3 Replay");
}

static void path3_clear(void)
{
    nav_clear_action(3U, "P3 Clear...", "P3 Data Cleared!");
}

static void empty_action(void)
{
}

static void rotation_test_action(void)
{
    if (Menu_IsActionFirstCall())
    {
        // 导航停止产生新目标；Body_ctrl 的旋转仲裁还会屏蔽残留差速。
        fuxian = 0U;
        target_speed = 0.0f;
        rotation_stop();
        (void)rotation_start(ROT_CW);
    }

    ips_show_string(0, 16 * 0, "Rotation 12s");
    ips_show_string(0, 16 * 1, "State:");
    ips_show_string(8 * 10, 16 * 1, rotation_state_name(rotation.state));
    ips_show_string(0, 16 * 2, "Duty:");
    ips_show_int(8 * 10, 16 * 2, rotation.turn_duty, 5);
    ips_show_string(0, 16 * 3, "Time:");
    ips_show_int(8 * 10, 16 * 3, rotation.elapsed_ms, 5);
}

/*
 * Menu configuration
 *
 * Normal action:
 *     MENU_ACTION("Displayed name", callback_function),
 *
 * Child menu:
 *     MENU_SUBMENU("Displayed name", child_menu),
 *
 * Return item:
 *     MENU_BACK("ESC"),
 */
static const menu_item_t path1_items[] =
{
    MENU_ACTION("Record",    path1_record),
    MENU_ACTION("SAVE",      path1_save),
    MENU_ACTION("Reproduce", path1_replay),
    MENU_ACTION("Clear",     path1_clear),
    MENU_ACTION("A_5",       empty_action),
    MENU_BACK("ESC")
};

static const menu_page_t path1_menu =
{
    "Page_2",
    path1_items,
    MENU_ARRAY_SIZE(path1_items)
};

static const menu_item_t path2_items[] =
{
    MENU_ACTION("Record",    path2_record),
    MENU_ACTION("SAVE",      path2_save),
    MENU_ACTION("Reproduce", path2_replay),
    MENU_ACTION("Clear",     path2_clear),
    MENU_ACTION("B_5",       empty_action),
    MENU_BACK("ESC")
};

static const menu_page_t path2_menu =
{
    "Page_2",
    path2_items,
    MENU_ARRAY_SIZE(path2_items)
};

static const menu_item_t path3_items[] =
{
    MENU_ACTION("Record",    path3_record),
    MENU_ACTION("SAVE",      path3_save),
    MENU_ACTION("Reproduce", path3_replay),
    MENU_ACTION("Clear",     path3_clear),
    MENU_ACTION("C_5",       empty_action),
    MENU_BACK("ESC")
};

static const menu_page_t path3_menu =
{
    "Page_2",
    path3_items,
    MENU_ARRAY_SIZE(path3_items)
};

static const menu_item_t test_d_items[] =
{
    MENU_ACTION("Rotation 12s", rotation_test_action),
    MENU_ACTION("D_2", empty_action),
    MENU_ACTION("D_3", empty_action),
    MENU_ACTION("D_4", empty_action),
    MENU_ACTION("D_5", empty_action),
    MENU_BACK("ESC")
};

static const menu_page_t test_d_menu =
{
    "Page_2",
    test_d_items,
    MENU_ARRAY_SIZE(test_d_items)
};

static const menu_item_t start_items[] =
{
    MENU_ACTION("Start_1", empty_action),
    MENU_ACTION("Start_2", empty_action),
    MENU_ACTION("Start_3", empty_action),
    MENU_ACTION("Start_4", empty_action),
    MENU_ACTION("E_5",     empty_action),
    MENU_BACK("ESC")
};

static const menu_page_t start_menu =
{
    "Page_2",
    start_items,
    MENU_ARRAY_SIZE(start_items)
};

static const menu_item_t main_items[] =
{
    MENU_SUBMENU("GO", path1_menu),
    MENU_SUBMENU("B",  path2_menu),
    MENU_SUBMENU("C",  path3_menu),
    MENU_SUBMENU("D",  test_d_menu),
    MENU_SUBMENU("E",  start_menu),
    MENU_BACK("ESC")
};

static const menu_page_t main_menu =
{
    "Page_1",
    main_items,
    MENU_ARRAY_SIZE(main_items)
};

static menu_view_t menu_view = MENU_VIEW_WELCOME;
static const menu_page_t *current_page = &main_menu;
static uint8 current_selection = 0U;
static menu_action_t current_action = NULL;
static uint8 menu_redraw = 1U;

static const menu_page_t *page_stack[MENU_MAX_DEPTH];
static uint8 selection_stack[MENU_MAX_DEPTH];
static uint8 menu_depth = 0U;

static void menu_draw_welcome(void)
{
    ips200_clear();
    ips200_show_string(100, 300, "Designed_by_WMCA");
}

static void menu_draw_page(void)
{
    uint8 index;

    ips200_clear();

    for (index = 0U; index < current_page->item_count; index++)
    {
        uint16 y = (uint16)(16U * (index + 1U));

        if (index == current_selection)
        {
            ips200_show_string(0, y, "->");
        }
        ips200_show_string(20, y, current_page->items[index].name);
    }

    ips200_show_string(8 * 23, 16 * 19, current_page->title);
}

static void menu_enter_submenu(const menu_page_t *submenu)
{
    if (menu_depth >= MENU_MAX_DEPTH)
    {
        return;
    }

    page_stack[menu_depth] = current_page;
    selection_stack[menu_depth] = current_selection;
    menu_depth++;

    current_page = submenu;
    current_selection = 0U;
    menu_redraw = 1U;
}

static void menu_go_back(void)
{
    if (menu_depth == 0U)
    {
        current_page = &main_menu;
        current_selection = 0U;
        menu_view = MENU_VIEW_WELCOME;
        menu_redraw = 1U;
        return;
    }

    menu_depth--;
    current_page = page_stack[menu_depth];
    current_selection = selection_stack[menu_depth];
    menu_redraw = 1U;
}

static void menu_activate_current_item(void)
{
    const menu_item_t *item = &current_page->items[current_selection];

    switch (item->type)
    {
        case MENU_ITEM_ACTION:
            current_action = item->action;
            menu_action_first_call = 1U;
            menu_view = MENU_VIEW_ACTION;
            menu_redraw = 0U;
            ips200_clear();
            break;

        case MENU_ITEM_SUBMENU:
            menu_enter_submenu(item->submenu);
            break;

        case MENU_ITEM_BACK:
            menu_go_back();
            break;

        default:
            break;
    }
}

static void menu_handle_list_keys(void)
{
    if (key1_take())
    {
        if (current_selection == 0U)
        {
            current_selection = (uint8)(current_page->item_count - 1U);
        }
        else
        {
            current_selection--;
        }
        menu_redraw = 1U;
    }

    if (key2_take())
    {
        current_selection++;
        if (current_selection >= current_page->item_count)
        {
            current_selection = 0U;
        }
        menu_redraw = 1U;
    }

    if (key3_take())
    {
        menu_activate_current_item();
    }
}

static void menu_handle_welcome(void)
{
    (void)key1_take();
    (void)key2_take();
    if (key3_take())
    {
        menu_view = MENU_VIEW_LIST;
        menu_redraw = 1U;
    }
}

static void menu_handle_action(void)
{
    (void)key1_take();
    (void)key2_take();
    if (key3_take())
    {
        if (current_action == rotation_test_action)
        {
            rotation_stop();
        }
        current_action = NULL;
        menu_view = MENU_VIEW_LIST;
        menu_redraw = 1U;
        return;
    }

    if (current_action != NULL)
    {
        current_action();
        menu_action_first_call = 0U;
    }
}

static void menu_render_if_needed(void)
{
    if (!menu_redraw)
    {
        return;
    }

    if (menu_view == MENU_VIEW_WELCOME)
    {
        menu_draw_welcome();
        menu_redraw = 0U;
    }
    else if (menu_view == MENU_VIEW_LIST)
    {
        menu_draw_page();
        menu_redraw = 0U;
    }
}

void Menu(void)
{
    control_background_task();

    switch (menu_view)
    {
        case MENU_VIEW_WELCOME:
            menu_handle_welcome();
            break;

        case MENU_VIEW_LIST:
            menu_handle_list_keys();
            if (menu_view == MENU_VIEW_ACTION)
            {
                menu_handle_action();
            }
            break;

        case MENU_VIEW_ACTION:
            menu_handle_action();
            break;

        default:
            current_page = &main_menu;
            current_selection = 0U;
            menu_depth = 0U;
            menu_view = MENU_VIEW_WELCOME;
            menu_redraw = 1U;
            break;
    }

    menu_render_if_needed();
    control_publish_main_command();
}
