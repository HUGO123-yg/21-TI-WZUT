#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// PIT_CH0 runs every 1 ms; five ticks produce a 200 Hz IMU update. The filter
// period is derived from these values so scheduler and estimator cannot drift.
#define IMU_SCHEDULER_TICK_PERIOD_S      (0.001f)
#define IMU_UPDATE_INTERVAL_TICKS        (5U)
#define IMU_UPDATE_PERIOD_S              \
    (IMU_SCHEDULER_TICK_PERIOD_S * (float)IMU_UPDATE_INTERVAL_TICKS)

// IMU660RB ranges selected by zf_device_imu660rb.h: +/-8 g and +/-2000 dps.
#define IMU_ACCEL_LSB_PER_G              (4098.0f)
#define IMU_GYRO_LSB_PER_DPS             (14.3f)

// Sensor-axis to vehicle-axis mapping. Axis indices are X=0, Y=1, Z=2.
#define IMU_BODY_X_SOURCE_AXIS           (0U)
#define IMU_BODY_Y_SOURCE_AXIS           (1U)
#define IMU_BODY_Z_SOURCE_AXIS           (2U)
#define IMU_BODY_X_DIRECTION             (1.0f)
#define IMU_BODY_Y_DIRECTION             (-1.0f)
#define IMU_BODY_Z_DIRECTION             (-1.0f)

// Additional software calibration in physical units. The 660RB driver already
// applies its fixed raw-count gyro offsets (-7, +6, +2).
#define IMU_ACCEL_BIAS_X_G               (0.0f)
#define IMU_ACCEL_BIAS_Y_G               (0.0f)
#define IMU_ACCEL_BIAS_Z_G               (0.0f)
#define IMU_GYRO_BIAS_X_DPS              (0.0f)
#define IMU_GYRO_BIAS_Y_DPS              (0.0f)
#define IMU_GYRO_BIAS_Z_DPS              (0.0f)

// Two-state Kalman filter parameters: angle and gyro bias.
#define IMU_KALMAN_Q_ANGLE               (0.001f)
#define IMU_KALMAN_Q_BIAS                (0.003f)
#define IMU_KALMAN_R_MEASUREMENT         (0.030f)
#define IMU_KALMAN_INITIAL_VARIANCE      (1.0f)

// Ignore accelerometer angle correction during strong dynamic acceleration.
#define IMU_ACCEL_CORRECTION_MIN_G       (0.80f)
#define IMU_ACCEL_CORRECTION_MAX_G       (1.20f)

// Control scheduler. The attitude estimator and fast balance controller share
// one 5 ms sample so the controller never mixes data from different periods.
#define CONTROL_FAST_INTERVAL_TICKS       IMU_UPDATE_INTERVAL_TICKS
#define CONTROL_FAST_PERIOD_S             IMU_UPDATE_PERIOD_S
#define CONTROL_SPEED_INTERVAL_STEPS      (4U)    // 50 Hz at a 200 Hz fast loop
#define CONTROL_LEG_INTERVAL_STEPS        (2U)    // 100 Hz at a 200 Hz fast loop

// The vehicle must be explicitly enabled after initialization. Keeping this at
// zero prevents uncalibrated gains or actuator directions from moving the car.
#define CONTROL_ENABLE_ON_BOOT            (0U)
#define CONTROL_FALL_PITCH_RAD             (0.61086524f) // 35 degrees
#define CONTROL_FALL_ROLL_RAD              (0.78539816f) // 45 degrees
#define CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS  (100U)

// Wheel driver protocol and logical-to-hardware signs. Logical positive means
// forward on both wheels. These signs match the last known driver wiring and
// must be checked with the vehicle lifted before balance control is enabled.
#define WHEEL_DRIVER_UART                  (UART_2)
#define WHEEL_DRIVER_BAUDRATE              (460800U)
#define WHEEL_DRIVER_TX_PIN                (UART2_TX_P10_1)
#define WHEEL_DRIVER_RX_PIN                (UART2_RX_P10_0)
#define WHEEL_LEFT_COMMAND_DIRECTION       (-1)
#define WHEEL_RIGHT_COMMAND_DIRECTION      (1)
#define WHEEL_LEFT_SPEED_DIRECTION         (1)
#define WHEEL_RIGHT_SPEED_DIRECTION        (-1)
#define WHEEL_MAX_COMMAND                  (3000)
#define WHEEL_DIAMETER_M                    (0.062f)

// Balance gains are deliberately zero until the motor direction, IMU sign and
// vehicle masses are verified. The control structure is operational, but zero
// gains plus CONTROL_ENABLE_ON_BOOT=0 make the default firmware stationary.
#define BALANCE_SPEED_KP                   (0.0f)
#define BALANCE_SPEED_KI                   (0.0f)
#define BALANCE_PITCH_KP                   (0.0f)
#define BALANCE_PITCH_KI                   (0.0f)
#define BALANCE_PITCH_KD                   (0.0f)
#define BALANCE_RATE_KP                    (0.0f)
#define BALANCE_RATE_KI                    (0.0f)
#define BALANCE_YAW_RATE_KP                (0.0f)
#define BALANCE_YAW_RATE_KI                (0.0f)
#define BALANCE_MAX_PITCH_REFERENCE_RAD    (0.10471976f) // 6 degrees
#define BALANCE_MAX_PITCH_RATE_RAD_S       (3.0f)
#define BALANCE_MAX_RATE_INTEGRAL          (500.0f)
#define BALANCE_MAX_YAW_COMMAND            (800.0f)
#define BALANCE_PITCH_ZERO_RAD              (0.0f)
#define BALANCE_WHEEL_OUTPUT_DIRECTION      (1.0f)

// Optional four-state inverted-pendulum feedback. Leave disabled until the
// physical model has been identified and K gains have been calculated. Gains
// include the conversion from model force/torque to wheel-driver command.
#define BALANCE_USE_STATE_FEEDBACK          (0U)
#define PENDULUM_K_POSITION                 (0.0f)
#define PENDULUM_K_SPEED                    (0.0f)
#define PENDULUM_K_PITCH                    (0.0f)
#define PENDULUM_K_PITCH_RATE               (0.0f)
#define PENDULUM_CART_EQUIVALENT_MASS_KG    (0.0f)   // awaiting measurement
#define PENDULUM_BODY_MASS_KG               (0.0f)   // awaiting measurement
#define PENDULUM_BODY_COM_HEIGHT_M          (0.049f) // nominal pose; verify per height
#define PENDULUM_GRAVITY_M_S2               (9.80665f)

// Leg control uses a coordinate system fixed to each pair of motor pivots:
// +x is vehicle-forward and +z points downward. Geometry is now measured, but
// servo output stays disabled until direction and end-stop checks are completed.
#define LEG_CONTROL_ENABLE                 (0U)
#define LEG_BASE_SPACING_M                 (0.038f)
#define LEG_LINK_A_PROXIMAL_M              (0.059f)
#define LEG_LINK_A_DISTAL_M                (0.090f)
#define LEG_LINK_B_PROXIMAL_M              (0.059f)
#define LEG_LINK_B_DISTAL_M                (0.090f)

// At the horizontal reference pose joint A points forward (0 rad) and joint B
// points rearward (pi rad). These branches retain that assembly mode while the
// wheel moves downward. Do not cross the near-straight-chain singularity.
#define LEG_BRANCH_A                       (-1)
#define LEG_BRANCH_B                       (1)
#define LEG_REFERENCE_JOINT_A_RAD          (0.0f)
#define LEG_REFERENCE_JOINT_B_RAD          (3.14159265f)
#define LEG_JOINT_A_MIN_RAD                (-0.10471976f) // allow old -160 trim
#define LEG_JOINT_A_MAX_RAD                (1.57079633f)
#define LEG_JOINT_B_MIN_RAD                (1.57079633f)
#define LEG_JOINT_B_MAX_RAD                (3.24631241f)  // allow old -160 trim

// Public leg commands are offsets from the horizontal reference pose. Forward
// kinematics computes its absolute (x,z), so the normal pose is exactly (0,0)
// to callers even though inverse kinematics uses pivot-relative coordinates.
#define LEG_DEFAULT_X_OFFSET_M             (0.0f)
#define LEG_DEFAULT_Z_OFFSET_M             (0.0f)
#define LEG_MAX_ABSOLUTE_Z_M               (0.145f)
#define LEG_ROLL_KP_M_PER_RAD              (0.0f)
#define LEG_ROLL_KD_M_PER_RAD_S            (0.0f)
#define LEG_ROLL_DIRECTION                  (1.0f)
#define LEG_MAX_ROLL_OFFSET_M              (0.0f)
#define LEG_MAX_TARGET_STEP_M              (0.001f)

// Servo channels and horizontal centres are confirmed by old/Common_peripherals.
// The old jump path used a 3000-count logical move for approximately 90 degrees,
// giving 1909.86 counts/rad. Output remains globally disabled until a lifted-car
// direction check confirms this provisional scale and the individual limits.
#define LEG_SERVO_COUNT                    (4U)
#define LEG_SERVO_FREQUENCY_HZ             (300U)
#define LEG_SERVO_1_PWM                    (TCPWM_CH10_P05_1)
#define LEG_SERVO_2_PWM                    (TCPWM_CH12_P05_3)
#define LEG_SERVO_3_PWM                    (TCPWM_CH09_P05_0)
#define LEG_SERVO_4_PWM                    (TCPWM_CH11_P05_2)
#define LEG_SERVO_1_CENTER                 (4400)
#define LEG_SERVO_2_CENTER                 (4400)
#define LEG_SERVO_3_CENTER                 (4700)
#define LEG_SERVO_4_CENTER                 (4200)
#define LEG_SERVO_1_DIRECTION              (1)
#define LEG_SERVO_2_DIRECTION              (-1)
#define LEG_SERVO_3_DIRECTION              (1)
#define LEG_SERVO_4_DIRECTION              (-1)
#define LEG_SERVO_1_PWM_PER_RAD            (1909.8593f)
#define LEG_SERVO_2_PWM_PER_RAD            (1909.8593f)
#define LEG_SERVO_3_PWM_PER_RAD            (1909.8593f)
#define LEG_SERVO_4_PWM_PER_RAD            (1909.8593f)
#define LEG_SERVO_1_ZERO_RAD               (0.0f)
#define LEG_SERVO_2_ZERO_RAD               (0.0f)
#define LEG_SERVO_3_ZERO_RAD               (3.14159265f)
#define LEG_SERVO_4_ZERO_RAD               (3.14159265f)
#define LEG_SERVO_1_PWM_MIN                (4200)
#define LEG_SERVO_1_PWM_MAX                (7400)
#define LEG_SERVO_2_PWM_MIN                (1400)
#define LEG_SERVO_2_PWM_MAX                (4600)
#define LEG_SERVO_3_PWM_MIN                (1700)
#define LEG_SERVO_3_PWM_MAX                (4900)
#define LEG_SERVO_4_PWM_MIN                (4000)
#define LEG_SERVO_4_PWM_MAX                (7200)
#define LEG_LEFT_JOINT_A_SERVO             (0U) // old steer_1, front/upper
#define LEG_LEFT_JOINT_B_SERVO             (2U) // old steer_3, rear/lower
#define LEG_RIGHT_JOINT_A_SERVO            (1U) // old steer_2, front/upper
#define LEG_RIGHT_JOINT_B_SERVO            (3U) // old steer_4, rear/lower

// Work-Flash layout used by the navigation route recorder. Pages 1 and 2
// contain alternating metadata copies. Keeping two copies prevents a power
// loss during a metadata update from destroying the only route directory.
#define NAV_FLASH_ROUTE_COUNT             (3U)
#define NAV_FLASH_META_PAGE_A             (1U)
#define NAV_FLASH_META_PAGE_B             (2U)

// Each route owns a fixed, descending page range. Fixed partitions make it
// impossible for a full route to overwrite another route or the metadata.
#define NAV_FLASH_ROUTE1_START_PAGE       (95U)
#define NAV_FLASH_ROUTE1_END_PAGE         (65U)
#define NAV_FLASH_ROUTE2_START_PAGE       (64U)
#define NAV_FLASH_ROUTE2_END_PAGE         (34U)
#define NAV_FLASH_ROUTE3_START_PAGE       (33U)
#define NAV_FLASH_ROUTE3_END_PAGE         (3U)

// The first 500 words in every 512-word page hold yaw samples. The remaining
// words contain page identity, sample count, generation, CRC, and commit data.
#define NAV_FLASH_SAMPLES_PER_PAGE        (500U)
#define NAV_FLASH_REPLAY_MAX_SAMPLES      (15500U)

// A new yaw sample is recorded after this much forward travel. The caller must
// pass distance_delta in the same distance unit used here.
#define NAV_FLASH_SAMPLE_DISTANCE         (5.0f)

// Yaw is stored as signed centi-degrees instead of float. This avoids storing
// compiler-dependent floating-point representations in persistent data.
#define NAV_FLASH_YAW_SCALE               (100.0f)

// Persistent format identifiers. Change FORMAT_VERSION whenever the on-Flash
// word layout changes incompatibly; old data will then be rejected safely.
#define NAV_FLASH_METADATA_MAGIC          (0x4E41564DUL)
#define NAV_FLASH_DATA_MAGIC              (0x4E415644UL)
#define NAV_FLASH_FORMAT_VERSION          (1U)
#define NAV_FLASH_COMMIT_MARKER           (0x434F4D54UL)

// Read every written page back and compare it before publishing metadata.
// Keep enabled on the vehicle; disabling it only reduces development latency.
#define NAV_FLASH_VERIFY_AFTER_WRITE      (1U)
#define NAV_FLASH_WRITE_RETRY_COUNT       (2U)

#endif
