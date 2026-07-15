#include "Menu.h"

#include <string.h>

#include "Control_system.h"
#include "Navigation.h"
#include "config.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"

typedef struct
{
    const char *label;
    menu_action_t action;
} menu_item_t;

static const menu_item_t dev_items[] =
{
    {"Stand / clear fault", MENU_ACTION_DEV_STAND},
    {"Stop balance", MENU_ACTION_DEV_STOP},
    {"Rotate CW 1 turn", MENU_ACTION_DEV_ROTATE_CW},
    {"Rotate CCW 1 turn", MENU_ACTION_DEV_ROTATE_CCW},
    {"Bridge left", MENU_ACTION_DEV_BRIDGE_LEFT},
    {"Bridge right", MENU_ACTION_DEV_BRIDGE_RIGHT},
    {"Bumpy module", MENU_ACTION_DEV_BUMPY},
    {"Jump module", MENU_ACTION_DEV_JUMP}
};

static const menu_item_t test_items[] =
{
    {"Race standby", MENU_ACTION_TEST_STAND},
    {"Record route 1", MENU_ACTION_TEST_RECORD_ROUTE_1},
    {"Record route 2", MENU_ACTION_TEST_RECORD_ROUTE_2},
    {"Record route 3", MENU_ACTION_TEST_RECORD_ROUTE_3},
    {"Stop / save record", MENU_ACTION_TEST_STOP_RECORDING},
    {"Replay route 1", MENU_ACTION_TEST_REPLAY_ROUTE_1},
    {"Replay route 2", MENU_ACTION_TEST_REPLAY_ROUTE_2},
    {"Replay route 3", MENU_ACTION_TEST_REPLAY_ROUTE_3},
    {"Stop replay", MENU_ACTION_TEST_STOP_REPLAY}
};

static menu_state_t menu_state;
static volatile uint8 menu_service_due;
static uint16 menu_tick_divider;
static uint16 menu_refresh_elapsed_ms;
static uint8 key4_long_handled;
static menu_page_t menu_drawn_page;

static uint8 menu_array_count(const menu_item_t *items, uint32 size_bytes)
{
    (void)items;
    return (uint8)(size_bytes / sizeof(menu_item_t));
}

static uint8 menu_current_item_count(void)
{
    if (MENU_PAGE_DEV == menu_state.page)
    {
        return menu_array_count(dev_items, sizeof(dev_items));
    }
    if (MENU_PAGE_TEST == menu_state.page)
    {
        return menu_array_count(test_items, sizeof(test_items));
    }
    return 2U;
}

static const menu_item_t *menu_current_items(void)
{
    if (MENU_PAGE_DEV == menu_state.page)
    {
        return dev_items;
    }
    if (MENU_PAGE_TEST == menu_state.page)
    {
        return test_items;
    }
    return 0;
}

static void menu_set_zero_command(void)
{
    balance_command_t command;

    memset(&command, 0, sizeof(command));
    control_system_set_command(&command);
}

static void menu_stop_navigation(void)
{
    const navigation_state_t *navigation;

    navigation = navigation_get_state();
    if (NAVIGATION_MODE_RECORDING == navigation->mode)
    {
        (void)navigation_stop_recording();
    }
    else if ((NAVIGATION_MODE_REPLAYING == navigation->mode)
             || (NAVIGATION_MODE_REPLAY_COMPLETE == navigation->mode))
    {
        navigation_stop_replay();
    }
}

static void menu_abort_module_actions(void)
{
    if (jump_ctrl_is_active())
    {
        (void)control_system_abort_jump();
    }
    (void)control_system_abort_rotation();
    (void)control_system_set_bridge_enabled(0U);
    (void)control_system_set_bumpy_enabled(0U);
    menu_stop_navigation();
    menu_set_zero_command();
    menu_state.active_action = MENU_ACTION_NONE;
}

static uint8 menu_balance_is_active(void)
{
    return control_system_get_state()->balance_enabled;
}

static uint8 menu_request_stand(uint8 clear_faults)
{
    menu_abort_module_actions();
    if (clear_faults)
    {
        control_system_clear_faults();
    }
    menu_state.hard_stopped = 0U;
    return control_system_request_stand();
}

static uint8 menu_start_recording(uint8 route_id)
{
    menu_stop_navigation();
    return (uint8)(NAVIGATION_STATUS_OK
                   == navigation_start_recording(route_id));
}

static uint8 menu_start_replay(uint8 route_id)
{
    if (!menu_request_stand(0U))
    {
        return 0U;
    }
    menu_stop_navigation();
    return (uint8)(NAVIGATION_STATUS_OK
                   == navigation_start_replay(route_id));
}

uint8 menu_execute_action(menu_action_t action)
{
    uint8 accepted;

    accepted = 0U;
    switch (action)
    {
        case MENU_ACTION_DEV_STAND:
        case MENU_ACTION_TEST_STAND:
            accepted = menu_request_stand(1U);
            break;

        case MENU_ACTION_DEV_STOP:
            menu_abort_module_actions();
            accepted = control_system_set_enabled(0U);
            menu_state.hard_stopped = 1U;
            break;

        case MENU_ACTION_DEV_ROTATE_CW:
            if (menu_balance_is_active())
            {
                accepted = control_system_start_rotation(
                    ROTATION_DIR_CW,
                    1.0f);
            }
            break;

        case MENU_ACTION_DEV_ROTATE_CCW:
            if (menu_balance_is_active())
            {
                accepted = control_system_start_rotation(
                    ROTATION_DIR_CCW,
                    1.0f);
            }
            break;

        case MENU_ACTION_DEV_BRIDGE_LEFT:
            if (menu_balance_is_active())
            {
                accepted = control_system_set_bridge_enabled(1U);
                if (accepted)
                {
                    accepted = control_system_start_bridge(1);
                }
                if (!accepted)
                {
                    (void)control_system_set_bridge_enabled(0U);
                }
            }
            break;

        case MENU_ACTION_DEV_BRIDGE_RIGHT:
            if (menu_balance_is_active())
            {
                accepted = control_system_set_bridge_enabled(1U);
                if (accepted)
                {
                    accepted = control_system_start_bridge(-1);
                }
                if (!accepted)
                {
                    (void)control_system_set_bridge_enabled(0U);
                }
            }
            break;

        case MENU_ACTION_DEV_BUMPY:
            if (menu_balance_is_active())
            {
                accepted = control_system_set_bumpy_enabled(1U);
                if (accepted)
                {
                    accepted = control_system_start_bumpy();
                }
                if (!accepted)
                {
                    (void)control_system_set_bumpy_enabled(0U);
                }
            }
            break;

        case MENU_ACTION_DEV_JUMP:
            if (menu_balance_is_active())
            {
                accepted = control_system_start_jump();
            }
            break;

        case MENU_ACTION_TEST_RECORD_ROUTE_1:
            accepted = menu_start_recording(1U);
            break;

        case MENU_ACTION_TEST_RECORD_ROUTE_2:
            accepted = menu_start_recording(2U);
            break;

        case MENU_ACTION_TEST_RECORD_ROUTE_3:
            accepted = menu_start_recording(3U);
            break;

        case MENU_ACTION_TEST_STOP_RECORDING:
            accepted = (uint8)(NAVIGATION_STATUS_OK
                == navigation_stop_recording());
            break;

        case MENU_ACTION_TEST_REPLAY_ROUTE_1:
            accepted = menu_start_replay(1U);
            break;

        case MENU_ACTION_TEST_REPLAY_ROUTE_2:
            accepted = menu_start_replay(2U);
            break;

        case MENU_ACTION_TEST_REPLAY_ROUTE_3:
            accepted = menu_start_replay(3U);
            break;

        case MENU_ACTION_TEST_STOP_REPLAY:
            navigation_stop_replay();
            accepted = menu_request_stand(0U);
            break;

        case MENU_ACTION_NONE:
        default:
            break;
    }

    menu_state.last_action = action;
    menu_state.last_result = accepted
        ? MENU_ACTION_RESULT_ACCEPTED
        : MENU_ACTION_RESULT_REJECTED;
    menu_state.active_action = accepted ? action : MENU_ACTION_NONE;
    menu_refresh_elapsed_ms = MENU_DISPLAY_REFRESH_MS;
    return accepted;
}

static void menu_update_action_state(void)
{
    const control_system_state_t *control;
    const rotation_ctrl_state_t *rotation;
    const jump_state_t *jump;
    const navigation_state_t *navigation;

    control = control_system_get_state();
    navigation = navigation_get_state();
    if ((MENU_ACTION_DEV_STAND == menu_state.active_action)
        || (MENU_ACTION_TEST_STAND == menu_state.active_action)
        || (MENU_ACTION_TEST_STOP_REPLAY == menu_state.active_action))
    {
        if (CONTROL_STARTUP_STANDING == control->startup_state)
        {
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_COMPLETED;
        }
        else if (CONTROL_STARTUP_FAULT == control->startup_state)
        {
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_REJECTED;
        }
    }
    else if ((MENU_ACTION_DEV_STOP == menu_state.active_action)
             || (MENU_ACTION_TEST_STOP_RECORDING
                 == menu_state.active_action))
    {
        menu_state.active_action = MENU_ACTION_NONE;
        menu_state.last_result = MENU_ACTION_RESULT_COMPLETED;
    }
    else if ((MENU_ACTION_TEST_RECORD_ROUTE_1 == menu_state.active_action)
             || (MENU_ACTION_TEST_RECORD_ROUTE_2
                 == menu_state.active_action)
             || (MENU_ACTION_TEST_RECORD_ROUTE_3
                 == menu_state.active_action))
    {
        if (NAVIGATION_MODE_ERROR == navigation->mode)
        {
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_REJECTED;
        }
        else if (NAVIGATION_MODE_RECORDING != navigation->mode)
        {
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_COMPLETED;
        }
    }
    else if ((MENU_ACTION_DEV_ROTATE_CW == menu_state.active_action)
        || (MENU_ACTION_DEV_ROTATE_CCW == menu_state.active_action))
    {
        rotation = control_system_get_rotation_state();
        if (ROTATION_RESULT_COMPLETED == rotation->result)
        {
            (void)control_system_release_rotation();
            (void)control_system_request_stand();
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_COMPLETED;
        }
        else if ((ROTATION_RESULT_TIMEOUT == rotation->result)
                 || (ROTATION_RESULT_FAULT == rotation->result))
        {
            (void)control_system_abort_rotation();
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_REJECTED;
        }
    }
    else if (MENU_ACTION_DEV_JUMP == menu_state.active_action)
    {
        jump = jump_ctrl_get_state();
        if (!jump->active)
        {
            (void)menu_request_stand(0U);
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = (JUMP_RESULT_COMPLETED == jump->result)
                ? MENU_ACTION_RESULT_COMPLETED
                : MENU_ACTION_RESULT_REJECTED;
        }
    }
    else if ((MENU_ACTION_DEV_BRIDGE_LEFT == menu_state.active_action)
             || (MENU_ACTION_DEV_BRIDGE_RIGHT == menu_state.active_action))
    {
        if (!control_system_get_state()->bridge_active)
        {
            (void)menu_request_stand(0U);
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_COMPLETED;
        }
    }
    else if (MENU_ACTION_DEV_BUMPY == menu_state.active_action)
    {
        if (!control_system_get_state()->bumpy_active)
        {
            (void)menu_request_stand(0U);
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_COMPLETED;
        }
    }
    else if ((MENU_ACTION_TEST_REPLAY_ROUTE_1 == menu_state.active_action)
             || (MENU_ACTION_TEST_REPLAY_ROUTE_2 == menu_state.active_action)
             || (MENU_ACTION_TEST_REPLAY_ROUTE_3 == menu_state.active_action))
    {
        if (NAVIGATION_MODE_REPLAY_COMPLETE == navigation->mode)
        {
            navigation_stop_replay();
            (void)control_system_request_stand();
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_COMPLETED;
        }
        else if (NAVIGATION_MODE_ERROR == navigation->mode)
        {
            menu_state.active_action = MENU_ACTION_NONE;
            menu_state.last_result = MENU_ACTION_RESULT_REJECTED;
        }
    }
}

static const char *menu_startup_text(control_startup_state_t state)
{
    switch (state)
    {
        case CONTROL_STARTUP_WAITING_DELAY:
            return "WAIT DELAY   ";
        case CONTROL_STARTUP_WAITING_WHEEL_FEEDBACK:
            return "WAIT WHEEL   ";
        case CONTROL_STARTUP_WAITING_UPRIGHT:
            return "WAIT UPRIGHT ";
        case CONTROL_STARTUP_STANDING:
            return "STANDING     ";
        case CONTROL_STARTUP_FAULT:
            return "FAULT        ";
        case CONTROL_STARTUP_DISABLED:
        default:
            return "STOPPED      ";
    }
}

static const char *menu_result_text(menu_action_result_t result)
{
    switch (result)
    {
        case MENU_ACTION_RESULT_ACCEPTED:
            return "RUNNING ";
        case MENU_ACTION_RESULT_COMPLETED:
            return "DONE    ";
        case MENU_ACTION_RESULT_REJECTED:
            return "REJECTED";
        case MENU_ACTION_RESULT_IDLE:
        default:
            return "IDLE    ";
    }
}

static void menu_draw(void)
{
#if MENU_DISPLAY_ENABLE
    const control_system_state_t *control;
    const menu_item_t *items;
    uint8 count;
    uint8 index;
    uint16 y;

    control = control_system_get_state();
    if (menu_drawn_page != menu_state.page)
    {
        ips200_clear();
        menu_drawn_page = menu_state.page;
    }
    ips200_show_string(0U, 0U, "MODE:");
    ips200_show_string(48U, 0U, menu_startup_text(control->startup_state));
    ips200_show_string(0U, 16U, "ACTION:");
    ips200_show_string(64U, 16U, menu_result_text(menu_state.last_result));

    if (MENU_PAGE_HOME == menu_state.page)
    {
        ips200_show_string(0U, 48U,
            (0U == menu_state.cursor) ? "> DEV" : "  DEV");
        ips200_show_string(0U, 64U,
            (1U == menu_state.cursor) ? "> TEST" : "  TEST");
    }
    else
    {
        ips200_show_string(0U, 32U,
            (MENU_PAGE_DEV == menu_state.page) ? "DEV" : "TEST");
        items = menu_current_items();
        count = menu_current_item_count();
        for (index = 0U; index < count; index++)
        {
            y = (uint16)(48U + (uint16)index * 16U);
            ips200_show_string(0U, y,
                (index == menu_state.cursor) ? ">" : " ");
            ips200_show_string(16U, y, items[index].label);
        }
    }
    ips200_show_string(0U, 304U, "K1^ K2v K3OK K4BACK");
#endif
}

static void menu_move_cursor(int8 direction)
{
    uint8 count;

    count = menu_current_item_count();
    if (direction < 0)
    {
        menu_state.cursor = (0U == menu_state.cursor)
            ? (uint8)(count - 1U)
            : (uint8)(menu_state.cursor - 1U);
    }
    else
    {
        menu_state.cursor = (uint8)((menu_state.cursor + 1U) % count);
    }
}

void menu_return_home(void)
{
    menu_abort_module_actions();
    (void)control_system_request_stand();
    menu_state.page = MENU_PAGE_HOME;
    menu_state.cursor = 0U;
    menu_refresh_elapsed_ms = MENU_DISPLAY_REFRESH_MS;
}

static void menu_enter_current(void)
{
    const menu_item_t *items;

    if (MENU_PAGE_HOME == menu_state.page)
    {
        menu_state.page = (0U == menu_state.cursor)
            ? MENU_PAGE_DEV
            : MENU_PAGE_TEST;
        menu_state.cursor = 0U;
        return;
    }

    items = menu_current_items();
    if (0 != items)
    {
        (void)menu_execute_action(items[menu_state.cursor].action);
    }
}

static uint8 menu_take_short_press(key_index_enum key)
{
    if (KEY_SHORT_PRESS == key_get_state(key))
    {
        key_clear_state(key);
        return 1U;
    }
    return 0U;
}

static void menu_handle_keys(void)
{
    key_state_enum key4_state;

    if (menu_take_short_press(KEY_1))
    {
        menu_move_cursor(-1);
        menu_refresh_elapsed_ms = MENU_DISPLAY_REFRESH_MS;
    }
    if (menu_take_short_press(KEY_2))
    {
        menu_move_cursor(1);
        menu_refresh_elapsed_ms = MENU_DISPLAY_REFRESH_MS;
    }
    if (menu_take_short_press(KEY_3))
    {
        menu_enter_current();
        menu_refresh_elapsed_ms = MENU_DISPLAY_REFRESH_MS;
    }

    key4_state = key_get_state(KEY_4);
    if (KEY_LONG_PRESS == key4_state)
    {
        if (!key4_long_handled)
        {
            menu_abort_module_actions();
            (void)control_system_set_enabled(0U);
            menu_state.hard_stopped = 1U;
            menu_state.page = MENU_PAGE_HOME;
            menu_state.cursor = 0U;
            menu_state.last_result = MENU_ACTION_RESULT_COMPLETED;
            key4_long_handled = 1U;
            menu_refresh_elapsed_ms = MENU_DISPLAY_REFRESH_MS;
        }
    }
    else if (KEY_RELEASE == key4_state)
    {
        key4_long_handled = 0U;
    }

    if (menu_take_short_press(KEY_4))
    {
        menu_return_home();
    }
}

void menu_init(void)
{
    memset(&menu_state, 0, sizeof(menu_state));
    menu_service_due = 0U;
    menu_tick_divider = 0U;
    menu_refresh_elapsed_ms = MENU_DISPLAY_REFRESH_MS;
    key4_long_handled = 0U;
    menu_drawn_page = (menu_page_t)0xFF;

    if (!MENU_ENABLE)
    {
        return;
    }

    key_init(MENU_KEY_SCAN_PERIOD_MS);
#if MENU_DISPLAY_ENABLE
    ips200_init(IPS200_TYPE_SPI);
#endif
    menu_state.initialized = 1U;
    menu_draw();
}

void menu_tick_1ms(void)
{
    if (!MENU_ENABLE || !menu_state.initialized)
    {
        return;
    }

    menu_tick_divider++;
    if (menu_tick_divider >= MENU_KEY_SCAN_PERIOD_MS)
    {
        menu_tick_divider = 0U;
        menu_service_due = 1U;
    }
}

void menu_task(void)
{
    if (!MENU_ENABLE || !menu_state.initialized || !menu_service_due)
    {
        return;
    }

    menu_service_due = 0U;
    key_scanner();
    menu_handle_keys();
    menu_update_action_state();
    if (menu_refresh_elapsed_ms < MENU_DISPLAY_REFRESH_MS)
    {
        menu_refresh_elapsed_ms += MENU_KEY_SCAN_PERIOD_MS;
    }
    if (menu_refresh_elapsed_ms >= MENU_DISPLAY_REFRESH_MS)
    {
        menu_refresh_elapsed_ms = 0U;
        menu_draw();
    }
}

const menu_state_t *menu_get_state(void)
{
    return &menu_state;
}
