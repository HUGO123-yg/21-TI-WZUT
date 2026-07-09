/*********************************************************************************************************************
* CYT4BB Opensourec Library
* Copyright (c) 2022 SEEKFREE
*
* File Name:      hw_adapter
* Description:    Hardware API Mapping Document — TC264 (Source) to CYT4BB7 (Target)
*                 This is a PURE DOCUMENTATION header. No functions are implemented here.
*                 It records key API differences observed when porting code from the
*                 SEEKFREE TC264 library to the SEEKFREE CYT4BB7 library.
*
* Usage:          Include this file in project/code/ for reference during porting.
*                 Do NOT include it in compiled source unless you add real macros.
********************************************************************************************************************/

#ifndef _hw_adapter_h_
#define _hw_adapter_h_

// ============================================================================
// 1. UART: Debug / Communication
// ============================================================================
// Category: UART Initialization
// Source (TC264):
//     uart_init(UART_0, 115200, UART0_TX_P14_0, UART0_RX_P14_1);
//     uart_init(UART_3, 115200, UART3_TX_P20_0, UART3_RX_P20_3);
// Target (CYT4BB7):
//     uart_init(UART_0, 115200, UART0_TX_P00_1, UART0_RX_P00_0);
//     uart_init(UART_4, 115200, UART4_TX_P14_1, UART4_RX_P14_0);
// Action: ADAPT — Change pin definitions only.
// Notes:
//   - Both platforms use the same function signature:
//       void uart_init(uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin);
//   - TC264 has UART_0 ~ UART_3. CYT4BB7 has UART_0 ~ UART_6.
//   - Pin enums are completely different. Do NOT copy P20_0 / P20_1 from TC264;
//     on CYT4BB7 the corresponding debug-UART pins are typically P14_0 / P14_1
//     (for UART_4, which is the default debug UART in many CYT4BB examples).
//   - The write/read APIs (uart_write_byte, uart_read_byte, uart_query_byte)
//     are identical on both platforms.

// ============================================================================
// 2. PWM / Servo
// ============================================================================
// Category: PWM Output
// Source (TC264):
//     pwm_init(ATOM0_CH0_P21_2, 300, 5000);
//     pwm_set_duty(ATOM0_CH0_P21_2, 3700);
// Target (CYT4BB7):
//     pwm_init(TCPWM_CH10_P05_1, 300, 5000);
//     pwm_set_duty(TCPWM_CH10_P05_1, 3700);
// Action: ADAPT — Replace ATOM channel enums with TCPWM channel enums.
// Notes:
//   - Both platforms use the same function signatures:
//       void pwm_init(pwm_channel_enum pwmch, uint32 freq, uint32 duty);
//       void pwm_set_duty(pwm_channel_enum pwmch, uint32 duty);
//   - Both define PWM_DUTY_MAX as 10000.
//   - TC264 uses Infineon GTM-ATOM channels (ATOM0_CHx_Pxx_x).
//   - CYT4BB7 uses TCPWM channels (TCPWM_CHxx_Pxx_x).
//   - Servo control code that calculates duty based on angle can be reused
//     verbatim; only the pwm_channel_enum constants need to change.

// ============================================================================
// 3. PIT / Timer
// ============================================================================
// Category: Periodic Interrupt Timer (PIT)
// Source (TC264):
//     pit_ms_init(CCU60_CH0, 5);
//     pit_init(CCU60_CH0, 5000);   // unit: microseconds
// Target (CYT4BB7):
//     pit_ms_init(PIT_CH0, 1);
//     pit_init(PIT_CH0, 1000);     // unit: microseconds
// Action: ADAPT — Replace CCU6 channel enums with TCPWM PIT channel enums.
// Notes:
//   - Both platforms provide pit_init(), pit_ms_init(), pit_us_init()
//     with the SAME macro expansion:
//       #define pit_ms_init(pit_index, time)  pit_init((pit_index), (time*1000))
//       #define pit_us_init(pit_index, time)  pit_init((pit_index), (time))
//   - TC264 channels: CCU60_CH0, CCU60_CH1, CCU61_CH0, CCU61_CH1 (only 4).
//   - CYT4BB7 channels: PIT_CH0 ~ PIT_CH2, PIT_CH10 ~ PIT_CH21 (many more).
//   - ISR names differ slightly in documentation but the override pattern
//     is the same: define pit0_ch0_isr() in the core-specific ISR file.
//   - On CYT4BB7, PIT_CH0 and PIT_CH1 are often defined in main_cm7_0.c;
//     do NOT redefine them in cm7_0_isr.c or you will get linker errors.

// ============================================================================
// 4. IMU (Accelerometer + Gyroscope)
// ============================================================================
// Category: IMU Initialization and Data Read
// Source (TC264) — IMU660RA:
//     imu660ra_init();
//     imu660ra_get_gyro();
//     imu660ra_get_acc();
//     float g = imu660ra_gyro_transition(imu660ra_gyro_x);
// Target (CYT4BB7) — IMU660RB:
//     imu660rb_init();
//     imu660rb_get_gyro();
//     imu660rb_get_acc();
//     float g = imu660rb_gyro_transition(imu660ra_gyro_x);
// Action: REWRITE — Function names, global variables, and pin/SPI config differ.
// Notes:
//   - TC264 driver: IMU660RA (SPI_0, pins P20_11 ~ P20_14).
//   - CYT4BB7 driver: IMU660RB (SPI_2, pins P15_0 ~ P15_3 by default).
//   - Global variable names change:
//       imu660ra_gyro_x  ->  imu660rb_gyro_x
//       imu660ra_acc_x   ->  imu660rb_acc_x
//   - IMU660RA uses enum-based configuration for accel/gyro range.
//   - IMU660RB uses raw register values for range configuration.
//   - IMU660RB header mentions it can also use software IIC by setting
//     IMU660RB_USE_SOFT_IIC to 1; IMU660RA header has a similar switch.
//   - Transition macros differ: IMU660RA uses a runtime transition_factor[]
//     array, while IMU660RB provides inline functions.

// ============================================================================
// 5. Encoder (Quadrature / Direction)
// ============================================================================
// Category: Quadrature Encoder Input
// Source (TC264):
//     encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P00_7, TIM2_ENCODER_CH2_P00_8);
//     int16 count = encoder_get_count(TIM2_ENCODER);
// Target (CYT4BB7):
//     encoder_quad_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P07_6, TC_CH07_ENCODER_CH2_P07_7);
//     int16 count = encoder_get_count(TC_CH07_ENCODER);
// Action: ADAPT — Replace TIMx encoder enums with TC_CHxx encoder enums.
// Notes:
//   - Both platforms use the SAME function signatures:
//       void encoder_quad_init(encoder_index_enum encoder_n,
//                              encoder_channel1_enum count_pin,
//                              encoder_channel2_enum dir_pin);
//       int16 encoder_get_count(encoder_index_enum encoder_n);
//       void encoder_clear_count(encoder_index_enum encoder_n);
//   - TC264 uses TIM2 ~ TIM6 (GPT12 timer based).
//   - CYT4BB7 uses TCPWM quadrature-decoder channels:
//       TC_CH07, TC_CH20, TC_CH27, TC_CH58.
//   - Pin enums are completely different; verify against the target board
//     schematic when rewiring motor encoders.

// ============================================================================
// 6. GPIO
// ============================================================================
// Category: General Purpose Input / Output
// Source (TC264):
//     gpio_init(P20_6, GPI, GPIO_HIGH, GPI_PULL_UP);
//     uint8 level = gpio_get_level(P20_6);
//     gpio_set_level(P20_6, GPIO_HIGH);
// Target (CYT4BB7):
//     gpio_init(P20_3, GPI, GPIO_HIGH, GPI_PULL_UP);
//     uint8 level = gpio_get_level(P20_3);
//     gpio_set_level(P20_3, GPIO_HIGH);
// Action: NONE (API compatible) — Only pin constants differ.
// Notes:
//   - Function signatures are IDENTICAL:
//       void  gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8 dat, gpio_mode_enum pinconf);
//       uint8 gpio_get_level(gpio_pin_enum pin);
//       void  gpio_set_level(gpio_pin_enum pin, uint8 dat);
//       void  gpio_toggle_level(gpio_pin_enum pin);
//   - Macro helpers gpio_high() / gpio_low() exist on both.
//   - However, the gpio_pin_enum numeric encoding is DIFFERENT:
//       TC264:  pin = port * 32 + pin_index
//       CYT4BB7: pin = port * 8  + pin_index
//   - This means you cannot safely cast raw integers to gpio_pin_enum
//     across platforms. Always use the named constants (Pxx_x).
//   - Some pins available on TC264 do NOT exist on CYT4BB7 and vice versa.
//     Check zf_driver_gpio.h for the exact enum list.

// ============================================================================
// 7. Clock Initialization (Bonus)
// ============================================================================
// Category: System Clock
// Source (TC264):
//     clock_init();               // no arguments, fixed configuration
// Target (CYT4BB7):
//     clock_init(SYSTEM_CLOCK_250M);  // requires explicit clock enum
// Action: ADAPT — Add the clock frequency argument.
// Notes:
//   - TC264 clock_init() takes no parameters and uses a fixed PLL config.
//   - CYT4BB7 clock_init() expects a clock_enum argument such as
//     SYSTEM_CLOCK_250M or SYSTEM_CLOCK_200M.
//   - debug_init() is identical on both (no arguments).

#endif // _hw_adapter_h_
