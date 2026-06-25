#include "zf_common_headfile.h"

remote_ctrl_struct remote_ctrl;

static int16 remote_ctrl_channel_to_signed(uint16 value, int16 max_output)
{
    int16 offset = (int16)value - REMOTE_CTRL_CENTER_VALUE;

    if (func_abs(offset) <= REMOTE_CTRL_DEADBAND)
    {
        return 0;
    }

    if (offset > 0)
    {
        offset -= REMOTE_CTRL_DEADBAND;
    }
    else
    {
        offset += REMOTE_CTRL_DEADBAND;
    }

    int32 scaled = (int32)offset * max_output /
                   (REMOTE_CTRL_EFFECTIVE_RANGE - REMOTE_CTRL_DEADBAND);

    return (int16)func_limit_ab(scaled, -max_output, max_output);
}

static float remote_ctrl_channel_to_float(uint16 value, float max_output)
{
    int16 signed_value = remote_ctrl_channel_to_signed(value, 1000);
    return (float)signed_value * max_output / 1000.0f;
}

static void remote_ctrl_force_stop(void)
{
    if (one_bridge_is_active())
    {
        one_bridge_abort();
    }

    if (rotation_is_active())
    {
        rotation_stop();
    }

    if (stair_is_active())
    {
        stair_abort();
    }
    else if (jump_is_active())
    {
        jump_abort();
    }

    remote_ctrl.armed              = 0;
    remote_ctrl.steer_adj          = 0;
    remote_ctrl.speed_target       = 0.0f;
    remote_ctrl.action_switch_last = 1;
    target_speed                   = 0.0f;
    STOP_FLAG                      = 0;
}

void remote_ctrl_init(void)
{
    uint8 i;

    memset(&remote_ctrl, 0, sizeof(remote_ctrl));

    for (i = 0; i < REMOTE_CTRL_CHANNEL_NUM; i++)
    {
        remote_ctrl.channel[i] = REMOTE_CTRL_CENTER_VALUE;
    }
    remote_ctrl.action_switch_last = 1;
}

void remote_ctrl_update_1ms(void)
{
    uint8 i;
    uint8 has_new_frame = 0;
    if (uart_receiver.finsh_flag)
    {
        for (i = 0; i < REMOTE_CTRL_CHANNEL_NUM; i++)
        {
            remote_ctrl.channel[i] = uart_receiver.channel[i];
        }
        remote_ctrl.online        = uart_receiver.state;
        remote_ctrl.detected      = 1;
        remote_ctrl.lost_count_ms = 0;
        uart_receiver.finsh_flag  = 0;
        has_new_frame             = 1;
    }
    remote_ctrl.fresh_frame = has_new_frame;

    if (!has_new_frame && remote_ctrl.lost_count_ms < (REMOTE_CTRL_FRAME_TIMEOUT_MS + 1))
    {
        remote_ctrl.lost_count_ms++;
    }

    if (!remote_ctrl.detected)
    {
        return;
    }

    if (!remote_ctrl.online || remote_ctrl.lost_count_ms >= REMOTE_CTRL_FRAME_TIMEOUT_MS)
    {
        remote_ctrl.online = 0;
        remote_ctrl_force_stop();
        return;
    }

    if (remote_ctrl.channel[REMOTE_CTRL_CH_ARM] <= REMOTE_CTRL_SWITCH_HIGH)
    {
        remote_ctrl_force_stop();
        return;
    }

    if (!remote_ctrl.armed)
    {
        if (func_abs((int16)remote_ctrl.channel[REMOTE_CTRL_CH_SPEED] - REMOTE_CTRL_CENTER_VALUE) > REMOTE_CTRL_DEADBAND)
        {
            remote_ctrl_force_stop();
            return;
        }
        remote_ctrl.armed = 1;
    }

    STOP_FLAG = 1;

    if ((jump_cfg.state == JUMP_IDLE) &&
        !rotation_is_active() &&
        (straight_test.state != STRAIGHT_RUNNING) &&
        !stair_is_active() &&
        !one_bridge_is_active())
    {
        fuxian = 0;
        remote_ctrl.speed_target =
            (float)REMOTE_CTRL_SPEED_DIR *
            remote_ctrl_channel_to_float(remote_ctrl.channel[REMOTE_CTRL_CH_SPEED], REMOTE_CTRL_MAX_SPEED);
        remote_ctrl.steer_adj =
            (int16)(REMOTE_CTRL_STEER_DIR *
            remote_ctrl_channel_to_signed(remote_ctrl.channel[REMOTE_CTRL_CH_STEER], REMOTE_CTRL_MAX_STEER_DUTY));
        target_speed = remote_ctrl.speed_target;
    }
    else
    {
        remote_ctrl.steer_adj = 0;
    }

    if (remote_ctrl.channel[REMOTE_CTRL_CH_ACTION] > REMOTE_CTRL_SWITCH_HIGH)
    {
        if (!remote_ctrl.action_switch_last && jump_can_trigger())
        {
            (void)jump_trigger();
        }
        remote_ctrl.action_switch_last = 1;
    }
    else
    {
        remote_ctrl.action_switch_last = 0;
    }
}

int16 remote_ctrl_get_steer_adj(void)
{
    return remote_ctrl_is_active() ? remote_ctrl.steer_adj : 0;
}

uint8 remote_ctrl_is_active(void)
{
    return (remote_ctrl.online && remote_ctrl.armed) ? 1 : 0;
}
