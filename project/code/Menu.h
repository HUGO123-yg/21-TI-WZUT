#ifndef PROJECT_MENU_H
#define PROJECT_MENU_H

#include "zf_common_typedef.h"

typedef enum
{
    MENU_PAGE_HOME = 0,
    MENU_PAGE_DEV,
    MENU_PAGE_TEST
} menu_page_t;

typedef enum
{
    MENU_ACTION_NONE = 0,
    MENU_ACTION_DEV_STAND,
    MENU_ACTION_DEV_STOP,
    MENU_ACTION_DEV_ROTATE_CW,
    MENU_ACTION_DEV_ROTATE_CCW,
    MENU_ACTION_DEV_BRIDGE_LEFT,
    MENU_ACTION_DEV_BRIDGE_RIGHT,
    MENU_ACTION_DEV_BUMPY,
    MENU_ACTION_DEV_JUMP,
    MENU_ACTION_TEST_STAND,
    MENU_ACTION_TEST_RECORD_ROUTE_1,
    MENU_ACTION_TEST_RECORD_ROUTE_2,
    MENU_ACTION_TEST_RECORD_ROUTE_3,
    MENU_ACTION_TEST_STOP_RECORDING,
    MENU_ACTION_TEST_REPLAY_ROUTE_1,
    MENU_ACTION_TEST_REPLAY_ROUTE_2,
    MENU_ACTION_TEST_REPLAY_ROUTE_3,
    MENU_ACTION_TEST_STOP_REPLAY
} menu_action_t;

typedef enum
{
    MENU_ACTION_RESULT_IDLE = 0,
    MENU_ACTION_RESULT_ACCEPTED,
    MENU_ACTION_RESULT_COMPLETED,
    MENU_ACTION_RESULT_REJECTED
} menu_action_result_t;

typedef struct
{
    menu_page_t page;
    menu_action_t active_action;
    menu_action_t last_action;
    menu_action_result_t last_result;
    uint8 cursor;
    uint8 initialized;
    uint8 hard_stopped;
} menu_state_t;

void menu_init(void);

// The ISR only advances a divider and raises a service flag.
void menu_tick_1ms(void);

// Key scanning, display refresh and all menu actions run from the main loop.
void menu_task(void);

// Public for host tests and future UART/remote menu front ends.
uint8 menu_execute_action(menu_action_t action);
void menu_return_home(void);
const menu_state_t *menu_get_state(void);

#endif
