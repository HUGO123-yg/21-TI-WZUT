#ifndef CODE_REMOTE_CTRL_H_
#define CODE_REMOTE_CTRL_H_

#include "zf_common_typedef.h"

#define REMOTE_CTRL_CHANNEL_NUM          (6)

#define REMOTE_CTRL_CH_STEER            (0)
#define REMOTE_CTRL_CH_SPEED            (1)
#define REMOTE_CTRL_CH_ARM              (4)
#define REMOTE_CTRL_CH_ACTION           (5)

#define REMOTE_CTRL_CENTER_VALUE        (1024)
#define REMOTE_CTRL_DEADBAND            (80)
#define REMOTE_CTRL_EFFECTIVE_RANGE     (820)
#define REMOTE_CTRL_SWITCH_HIGH         (1500)
#define REMOTE_CTRL_FRAME_TIMEOUT_MS    (120)

#define REMOTE_CTRL_MAX_SPEED           (250.0f)
#define REMOTE_CTRL_MAX_STEER_DUTY      (900)
#define REMOTE_CTRL_SPEED_DIR           (1)
#define REMOTE_CTRL_STEER_DIR           (1)

typedef struct
{
    uint16 channel[REMOTE_CTRL_CHANNEL_NUM];
    uint8  online;
    uint8  detected;
    uint8  armed;
    uint8  fresh_frame;
    uint8  action_switch_last;
    uint32 lost_count_ms;
    int16  steer_adj;
    float  speed_target;
} remote_ctrl_struct;

extern remote_ctrl_struct remote_ctrl;

void  remote_ctrl_init(void);
void  remote_ctrl_update_1ms(void);
int16 remote_ctrl_get_steer_adj(void);
uint8 remote_ctrl_is_active(void);

#endif /* CODE_REMOTE_CTRL_H_ */
