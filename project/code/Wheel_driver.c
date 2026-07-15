#include "Wheel_driver.h"

#include <string.h>

#include "config.h"
#include "zf_driver_uart.h"

#define WHEEL_FRAME_HEADER       (0xA5U)
#define WHEEL_FRAME_LENGTH       (7U)
#define WHEEL_COMMAND_SET_DUTY   (0x01U)
#define WHEEL_COMMAND_GET_SPEED  (0x02U)

typedef struct
{
    uint8 receive_buffer[WHEEL_FRAME_LENGTH];
    uint8 receive_count;
    volatile int16 left_rpm;
    volatile int16 right_rpm;
    volatile uint32 valid_frame_count;
    volatile uint32 invalid_frame_count;
    volatile uint8 output_stop_locked;
    uint8 config_valid;
} wheel_driver_state_t;

static wheel_driver_state_t wheel_state;

static uint8 wheel_direction_is_valid(int32 direction)
{
    return (uint8)((1 == direction) || (-1 == direction));
}

uint8 wheel_driver_config_is_valid(void)
{
    return (uint8)(wheel_direction_is_valid(WHEEL_LEFT_COMMAND_DIRECTION)
        && wheel_direction_is_valid(WHEEL_RIGHT_COMMAND_DIRECTION)
        && wheel_direction_is_valid(WHEEL_LEFT_SPEED_DIRECTION)
        && wheel_direction_is_valid(WHEEL_RIGHT_SPEED_DIRECTION)
        && (WHEEL_DRIVER_BAUDRATE > 0U)
        && (WHEEL_MAX_COMMAND > 0)
        && (WHEEL_DIAMETER_M > 0.0f));
}

static int16 wheel_limit_command(int32 command)
{
    if (command > WHEEL_MAX_COMMAND)
    {
        return (int16)WHEEL_MAX_COMMAND;
    }
    if (command < -WHEEL_MAX_COMMAND)
    {
        return (int16)-WHEEL_MAX_COMMAND;
    }
    return (int16)command;
}

static void wheel_send_frame(uint8 command, int16 left_value, int16 right_value)
{
    uint8 frame[WHEEL_FRAME_LENGTH];
    uint8 index;

    if (!wheel_state.config_valid)
    {
        return;
    }

    frame[0] = WHEEL_FRAME_HEADER;
    frame[1] = command;
    frame[2] = (uint8)(((uint16)left_value >> 8) & 0xFFU);
    frame[3] = (uint8)((uint16)left_value & 0xFFU);
    frame[4] = (uint8)(((uint16)right_value >> 8) & 0xFFU);
    frame[5] = (uint8)((uint16)right_value & 0xFFU);
    frame[6] = 0U;
    for (index = 0U; index < WHEEL_FRAME_LENGTH - 1U; index++)
    {
        frame[6] = (uint8)(frame[6] + frame[index]);
    }
    uart_write_buffer(WHEEL_DRIVER_UART, frame, WHEEL_FRAME_LENGTH);
}

static uint8 wheel_frame_is_valid(const uint8 *frame)
{
    uint8 sum;
    uint8 index;

    if ((0 == frame) || (WHEEL_FRAME_HEADER != frame[0]))
    {
        return 0U;
    }

    sum = 0U;
    for (index = 0U; index < WHEEL_FRAME_LENGTH - 1U; index++)
    {
        sum = (uint8)(sum + frame[index]);
    }
    return (uint8)(sum == frame[WHEEL_FRAME_LENGTH - 1U]);
}

void wheel_driver_init(void)
{
    memset(&wheel_state, 0, sizeof(wheel_state));
    wheel_state.config_valid = wheel_driver_config_is_valid();
    if (!wheel_state.config_valid)
    {
        wheel_state.output_stop_locked = 1U;
        return;
    }
    uart_init(WHEEL_DRIVER_UART,
              WHEEL_DRIVER_BAUDRATE,
              WHEEL_DRIVER_TX_PIN,
              WHEEL_DRIVER_RX_PIN);
    uart_rx_interrupt(WHEEL_DRIVER_UART, 1U);
    wheel_driver_stop();
    wheel_driver_request_speed();
}

void wheel_driver_uart_isr(void)
{
    uint8 byte;
    uint8 *frame;

    while (uart_query_byte(WHEEL_DRIVER_UART, &byte))
    {
        if (0U == wheel_state.receive_count)
        {
            if (WHEEL_FRAME_HEADER == byte)
            {
                wheel_state.receive_buffer[0] = byte;
                wheel_state.receive_count = 1U;
            }
            continue;
        }

        wheel_state.receive_buffer[wheel_state.receive_count] = byte;
        wheel_state.receive_count++;
        if (wheel_state.receive_count < WHEEL_FRAME_LENGTH)
        {
            continue;
        }

        frame = wheel_state.receive_buffer;
        if (wheel_frame_is_valid(frame)
            && (WHEEL_COMMAND_GET_SPEED == frame[1]))
        {
            wheel_state.left_rpm = (int16)(((uint16)frame[2] << 8)
                                            | (uint16)frame[3]);
            wheel_state.right_rpm = (int16)(((uint16)frame[4] << 8)
                                             | (uint16)frame[5]);
            wheel_state.valid_frame_count++;
        }
        else
        {
            wheel_state.invalid_frame_count++;
        }

        wheel_state.receive_count = 0U;
        if (WHEEL_FRAME_HEADER == byte)
        {
            wheel_state.receive_buffer[0] = byte;
            wheel_state.receive_count = 1U;
        }
    }
}

void wheel_driver_set_command(int16 left_command, int16 right_command)
{
    int16 hardware_left;
    int16 hardware_right;

    if (wheel_state.output_stop_locked)
    {
        wheel_send_frame(WHEEL_COMMAND_SET_DUTY, 0, 0);
        return;
    }

    hardware_left = wheel_limit_command((int32)left_command
                                        * WHEEL_LEFT_COMMAND_DIRECTION);
    hardware_right = wheel_limit_command((int32)right_command
                                         * WHEEL_RIGHT_COMMAND_DIRECTION);
    wheel_send_frame(WHEEL_COMMAND_SET_DUTY, hardware_left, hardware_right);
}

void wheel_driver_stop(void)
{
    wheel_send_frame(WHEEL_COMMAND_SET_DUTY, 0, 0);
}

void wheel_driver_set_stop_lock(uint8 locked)
{
    wheel_state.output_stop_locked = (uint8)(0U != locked);
    if (wheel_state.output_stop_locked)
    {
        wheel_driver_stop();
    }
}

uint8 wheel_driver_is_stop_locked(void)
{
    return wheel_state.output_stop_locked;
}

void wheel_driver_request_speed(void)
{
    wheel_send_frame(WHEEL_COMMAND_GET_SPEED, 0, 0);
}

void wheel_driver_get_feedback(wheel_feedback_t *feedback)
{
    if (0 == feedback)
    {
        return;
    }

    feedback->left_rpm = (int16)(wheel_state.left_rpm
                                 * WHEEL_LEFT_SPEED_DIRECTION);
    feedback->right_rpm = (int16)(wheel_state.right_rpm
                                  * WHEEL_RIGHT_SPEED_DIRECTION);
    feedback->valid_frame_count = wheel_state.valid_frame_count;
    feedback->invalid_frame_count = wheel_state.invalid_frame_count;
}
