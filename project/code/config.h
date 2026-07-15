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
// This is the same installed-IMU mapping used by the legacy balance core:
// X=raw X, Y=-raw Y and Z=-raw Z. Imu.c rejects directions other than +/-1.
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

// Estimate the remaining gyro bias while the vehicle is stationary at boot.
// Initialization deliberately fails if too few stationary samples are found;
// the control stack must not balance from an uncalibrated rate signal.
#define IMU_STARTUP_CALIBRATION_ENABLE          (1U)
#define IMU_STARTUP_CALIBRATION_SAMPLES         (200U)
#define IMU_STARTUP_CALIBRATION_MIN_VALID       (180U)
#define IMU_STARTUP_CALIBRATION_DELAY_MS        (5U)
#define IMU_STARTUP_CALIBRATION_MAX_GYRO_DPS    (5.0f)
#define IMU_STARTUP_CALIBRATION_MIN_G           (0.90f)
#define IMU_STARTUP_CALIBRATION_MAX_G           (1.10f)

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
#define CONTROL_WHEEL_REQUEST_INTERVAL_STEPS (4U) // request feedback at 50 Hz

// Balance is armed only by an explicit menu/control request. Boot initializes
// and monitors the sensors with both wheel commands held at zero.
#define CONTROL_DEFAULT_STAND_ON_BOOT      (0U)
#define CONTROL_STAND_ARM_DELAY_MS         (300U)
#define CONTROL_STAND_ARM_MAX_PITCH_ERROR_RAD (0.17453293f) // 10 degrees
#define CONTROL_STAND_ARM_MAX_ROLL_RAD     (0.34906585f) // 20 degrees
#define CONTROL_FALL_PITCH_ERROR_RAD       (0.61086524f) // 35 degrees
#define CONTROL_FALL_ROLL_RAD              (0.78539816f) // 45 degrees
#define CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS  (100U)

// Fault recovery is always explicit. The wheel driver remains stop-locked
// until the IMU is healthy, the chassis has been placed upright and the legs
// have returned to this safe pose. Software emergency stop holds the last
// valid leg PWM by default to avoid an uncontrolled chassis collapse; set the
// output-disable switch only when the mechanism has an independent support.
#define CONTROL_FAULT_RECOVERY_MAX_PITCH_ERROR_RAD \
    CONTROL_STAND_ARM_MAX_PITCH_ERROR_RAD
#define CONTROL_FAULT_RECOVERY_MAX_ROLL_RAD  CONTROL_STAND_ARM_MAX_ROLL_RAD
#define CONTROL_ESTOP_DISABLE_LEG_OUTPUT     (0U)

// Two top-level menus keep development actions separate from competition
// workflows. Keys are scanned from the main context; the 1 ms ISR only raises
// a service flag.
#define MENU_ENABLE                        (1U)
#define MENU_DISPLAY_ENABLE                (1U)
#define MENU_KEY_SCAN_PERIOD_MS            (10U)
#define MENU_DISPLAY_REFRESH_MS            (100U)
#define CONTROL_COMPETITION_MODULES_ON_BOOT (0U)

// Wheel driver protocol and logical-to-hardware signs. Logical positive means
// vehicle-forward on both wheels. These values reproduce the validated legacy
// paths CYT2_D_motor_ctrl(-left, +right) and speed=(raw_left-raw_right)/2.
// Wheel_driver.c refuses output if any direction is not exactly +1 or -1.
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

// Planar navigation runs continuously for logging and route recording. Wheel
// differential heading remains disabled until the effective track width has
// been measured on the assembled vehicle.
#define NAVIGATION_ENABLE                       (1U)
#define NAVIGATION_USE_WHEEL_YAW_CORRECTION     (0U)
#define NAVIGATION_WHEEL_TRACK_WIDTH_M          (0.0f) // awaiting measurement
#define NAVIGATION_WHEEL_YAW_RATE_WEIGHT        (0.05f)
#define NAVIGATION_STATIONARY_SPEED_M_S         (0.02f)
#define NAVIGATION_STATIONARY_GYRO_DPS          (1.0f)
#define NAVIGATION_GYRO_BIAS_LEARNING_RATE      (0.002f)

// Route replay computes an outer-loop yaw-rate command, but does not apply it
// to the wheel controller until signs, track width and gains are verified.
#define NAVIGATION_ROUTE_CONTROL_ENABLE         (0U)
#define NAVIGATION_HEADING_KP                    (2.0f)
#define NAVIGATION_MAX_YAW_RATE_RAD_S            (1.5f)

// Headless MT9V03X terrain recognition. The detector keeps an internal road
// region only to reject background pixels; it does not publish steering lines
// or draw to an LCD. Recognition results are observational and do not trigger
// bridge, bumpy-road, step or jump control until a separate arbiter is added.
#define TERRAIN_VISION_ENABLE                    (1U)
#define TERRAIN_VISION_DARK_SCENE_AVERAGE        (70U)
#define TERRAIN_VISION_THRESHOLD_FLOOR_DARK      (75U)
#define TERRAIN_VISION_THRESHOLD_FLOOR_NORMAL    (92U)
#define TERRAIN_VISION_BAND_DARK_PERCENT         (62U)
#define TERRAIN_VISION_BUMPY_MIN_WIDTH_PERCENT   (45U)
#define TERRAIN_VISION_BUMPY_MIN_STRIPS          (3U)
#define TERRAIN_VISION_SCORE_MAX                 (8U)
#define TERRAIN_VISION_BUMPY_CONFIRM_FRAMES      (2U)
#define TERRAIN_VISION_STEP_CONFIRM_FRAMES       (3U)
#define TERRAIN_VISION_BRIDGE_CONFIRM_FRAMES     (3U)
#define TERRAIN_VISION_OBSTACLE_CONFIRM_FRAMES   (3U)
#define TERRAIN_VISION_RELEASE_SCORE             (1U)
#define TERRAIN_VISION_EXPOSURE_UPDATE_FRAMES    (10U)
#define TERRAIN_VISION_EXPOSURE_MIN              (40U)
#define TERRAIN_VISION_EXPOSURE_MAX              (650U)
#define TERRAIN_VISION_EXPOSURE_STEP             (10U)

// Basic balance cascade converted from the validated legacy 660RB controller.
// The old angle/rate gains 700, 50 and 1.1 operated on degrees and raw gyro
// counts (14.3 LSB/(deg/s)); these values preserve the same small-signal wheel
// response in rad, rad/s and driver-command units at the current 200 Hz rate.
// The speed gain also includes the old 0.003 deg/output coupling and the wheel
// RPM-to-m/s conversion. Integral gains stay zero for the first hardware tune.
#define BALANCE_SPEED_KP                   (0.0806452f)
#define BALANCE_SPEED_KI                   (0.0f)
#define BALANCE_PITCH_KP                   (-48.9510f)
#define BALANCE_PITCH_KI                   (0.0f)
#define BALANCE_PITCH_KD                   (-0.0174825f)
#define BALANCE_RATE_KP                    (901.263f)
#define BALANCE_RATE_KI                    (0.0f)
#define BALANCE_YAW_RATE_KP                (0.0f)
#define BALANCE_YAW_RATE_KI                (0.0f)
#define BALANCE_MAX_PITCH_REFERENCE_RAD    (0.10471976f) // 6 degrees
#define BALANCE_MAX_PITCH_RATE_RAD_S       (3.0f)
#define BALANCE_MAX_RATE_INTEGRAL          (500.0f)
#define BALANCE_MAX_YAW_COMMAND            (800.0f)
#define BALANCE_PITCH_ZERO_RAD              (-0.10471976f) // legacy -6 degrees
#define BALANCE_WHEEL_OUTPUT_DIRECTION      (-1.0f)

// Zero-radius rotation is an outer angle loop that commands the existing yaw
// rate PI. Positive yaw is expected to be counter-clockwise, so clockwise is
// provisionally negative. Verify this sign with the vehicle lifted.
#define ROTATION_CONTROL_ENABLE             (1U)
#define ROTATION_CW_YAW_SIGN                (-1)
#define ROTATION_MAX_TURNS                  (5.0f)
#define ROTATION_MIN_YAW_RATE_RAD_S         (0.20f)
#define ROTATION_MAX_YAW_RATE_RAD_S         (1.20f)
#define ROTATION_ANGLE_KP_RAD_S_PER_RAD     (0.30f)
#define ROTATION_MAX_YAW_ACCEL_RAD_S2       (4.0f)
#define ROTATION_ANGLE_TOLERANCE_DEG        (3.0f)
#define ROTATION_SETTLE_YAW_RATE_RAD_S      (0.10f)
#define ROTATION_SETTLE_STEPS               (20U)   // 100 ms at 200 Hz

// Timeout scales with the requested turns. These values allow roughly two
// seconds of setup plus eight seconds per full turn before holding zero speed.
#define ROTATION_TIMEOUT_BASE_STEPS         (400U)
#define ROTATION_TIMEOUT_PER_TURN_STEPS     (1600U)

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
#define PENDULUM_BODY_COM_HEIGHT_M          (0.0f)   // not measured yet
#define PENDULUM_GRAVITY_M_S2               (9.80665f)

// Chassis ground clearance in the horizontal short-link reference pose. This
// is a packaging/terrain value, not the inverted-pendulum COM height.
#define BODY_REFERENCE_GROUND_CLEARANCE_M   (0.049f)
#define BODY_GROUND_CLEARANCE_TOLERANCE_M   (0.005f)

// Leg control uses a coordinate system fixed to each pair of motor pivots:
// +x is vehicle-forward and +z points downward. All four identical servos now
// have confirmed centres/travel; initialization holds the horizontal pose.
#define LEG_CONTROL_ENABLE                 (1U)
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
// The linkage can geometrically reach about 145 mm, but the confirmed servo-1
// operating limit is only +2000 PWM from horizontal (about 60 degrees). Keep
// normal targets below 140 mm so control never relies on the mechanical stop.
#define LEG_MECHANICAL_MAX_ABSOLUTE_Z_M    (0.145f)
#define LEG_SAFE_MAX_ABSOLUTE_Z_M          (0.140f)
#define LEG_MAX_ABSOLUTE_Z_M               LEG_SAFE_MAX_ABSOLUTE_Z_M
#define LEG_ROLL_KP_M_PER_RAD              (0.0f)
#define LEG_ROLL_KD_M_PER_RAD_S            (0.0f)
#define LEG_ROLL_DIRECTION                  (1.0f)
#define LEG_MAX_ROLL_OFFSET_M              (0.0f)
#define LEG_MAX_DIFFERENTIAL_Z_OFFSET_M    (0.030f)
#define LEG_MAX_TARGET_STEP_M              (0.001f)
#define LEG_FAULT_RECOVERY_X_OFFSET_M      LEG_DEFAULT_X_OFFSET_M
#define LEG_FAULT_RECOVERY_Z_OFFSET_M      LEG_DEFAULT_Z_OFFSET_M

// Subject-3 bumpy-road control. The official ribs are 20 mm high, 25 mm wide
// and spaced by about 100 mm. Two separated acceleration shocks are required
// for automatic entry so a single landing or bridge edge does not claim the
// speed-to-leg controller. A route/mileage trigger should still be preferred.
#define BUMPY_CONTROL_ENABLE                (1U)
#define BUMPY_AUTO_DETECT_ENABLE            (1U)
#define BUMPY_IMPACT_DETECT_DELTA_G         (0.20f)
#define BUMPY_IMPACT_RELEASE_DELTA_G        (0.08f)
#define BUMPY_STABLE_DELTA_G                (0.05f)
#define BUMPY_IMPACT_REQUIRED_COUNT         (2U)
#define BUMPY_IMPACT_REFRACTORY_STEPS       (10U)  // 50 ms at 200 Hz
#define BUMPY_DETECT_WINDOW_STEPS           (160U) // 800 ms

// The rules do not define the total ribbed-section length. This distance is a
// provisional route parameter and must be replaced by the measured course
// value. The timeout guarantees that a missed odometry update releases control.
#define BUMPY_CROSSING_DISTANCE_M           (1.00f)
#define BUMPY_CROSSING_TIMEOUT_STEPS        (1200U) // 6 s
#define BUMPY_RECOVER_STABLE_STEPS          (20U)   // 100 ms
#define BUMPY_RECOVER_TIMEOUT_STEPS         (200U)  // 1 s

// During crossing, the speed PI directly changes the common leg x offset.
// Balance_ctrl then fixes pitch reference at the calibrated zero and uses only
// the pitch/pitch-rate loops for wheel stabilization. Gains remain zero until
// leg-x direction and usable travel have been verified on the supported car.
#define BUMPY_SPEED_KP_M_PER_M_S            (0.0f)
#define BUMPY_SPEED_KI_M_PER_M              (0.0f)
#define BUMPY_SPEED_TO_LEG_DIRECTION        (1.0f)
#define BUMPY_MAX_SPEED_LEG_X_OFFSET_M      (0.020f)
#define BUMPY_MAX_TOTAL_LEG_X_OFFSET_M      (0.025f)
#define BUMPY_MAX_LEG_X_STEP_M              (0.00025f)

// Extending both legs by 20 mm creates clearance/compliance for the 20 mm
// ribs. Speed and steering are capped to reduce wheel unloading and yaw kicks.
#define BUMPY_BODY_Z_OFFSET_M               (0.020f)
#define BUMPY_MAX_SPEED_M_S                 (0.30f)
#define BUMPY_YAW_RATE_SCALE                (0.50f)
#define BUMPY_MAX_YAW_RATE_RAD_S            (0.60f)

// Single-side bridge control. The bridge layer leaves the pitch balance loop
// in charge of both wheels, limits forward/yaw commands, and asks Leg_ctrl for
// a symmetric left/right z difference. It is automatically held disabled when
// LEG_CONTROL_ENABLE is zero or leg_ctrl_init() is not ready.
#define BRIDGE_CONTROL_ENABLE               (1U)
#define BRIDGE_AUTO_DETECT_ENABLE           (1U)

// State-machine thresholds use the 200 Hz fast-control sample. Detection is
// deliberately shorter than the distance-based crossing state; do not exit a
// crossing just because the compensation itself has reduced roll toward zero.
#define BRIDGE_ROLL_DETECT_RAD              (0.08726646f) // 5 degrees
#define BRIDGE_ROLL_RECOVER_RAD             (0.01745329f) // 1 degree
#define BRIDGE_DETECT_STEPS                 (8U)          // 40 ms
#define BRIDGE_ENTER_STEPS                  (20U)         // 100 ms
#define BRIDGE_EXIT_STEPS                   (20U)         // 100 ms
#define BRIDGE_RECOVER_STEPS                (20U)         // 100 ms stable
#define BRIDGE_RECOVER_TIMEOUT_STEPS        (400U)        // 2 s guard
#define BRIDGE_CROSSING_TIMEOUT_STEPS       (800U)        // 4 s guard
#define BRIDGE_CROSSING_DISTANCE_M          (0.20f)       // verify on course

// Geometry feedforward estimates half of the left/right ground-height
// difference as 0.5 * support_span * tan(entry_roll). The 0.10 m span is a
// provisional chassis value and must be replaced by the measured distance
// between the two wheel contact lines. Change the direction sign only after a
// lifted-car roll-direction test.
#define BRIDGE_LATERAL_SUPPORT_SPAN_M       (0.10f)
#define BRIDGE_ROLL_TO_LEG_DIRECTION        (1.0f)
#define BRIDGE_FORCED_ENTRY_ROLL_RAD        BRIDGE_ROLL_DETECT_RAD

// Bridge-only roll PD is added to the latched geometry feedforward. Positive
// differential means left leg +z and right leg -z. Larger Kp levels the body
// more strongly; larger Kd adds damping. Excessive values cause side-to-side
// oscillation, so the total differential and its slew rate are both limited.
#define BRIDGE_ROLL_KP_M_PER_RAD            (0.0172f)
#define BRIDGE_ROLL_KD_M_PER_RAD_S          (0.000012f)
#define BRIDGE_MAX_DIFFERENTIAL_OFFSET_M    (0.015f)
#define BRIDGE_MAX_DIFFERENTIAL_STEP_M      (0.00025f)

// Keep both legs away from the horizontal reference while differential travel
// is active, cap bridge speed, and reduce steering that could twist a wheel off
// the narrow support. These are provisional low-speed test values.
#define BRIDGE_BODY_Z_OFFSET_M              (0.020f)
#define BRIDGE_MAX_SPEED_M_S                (0.26f)
#define BRIDGE_YAW_RATE_SCALE               (0.35f)
#define BRIDGE_MAX_YAW_RATE_RAD_S           (0.50f)

// Fixed-time jump script. The jump controller commands each target directly
// instead of waiting for contact or attitude events. +z extends the legs.
// These provisional targets are inside the measured five-bar workspace, but
// must still be checked with the vehicle supported before an on-ground jump.
#define JUMP_EXTEND_TIME_MS                 (100U)
#define JUMP_RETRACT_TIME_MS                (100U)
#define JUMP_BUFFER_TIME_MS                 (80U)
#define JUMP_LEG_X_OFFSET_M                 (0.0f)
#define JUMP_EXTEND_Z_OFFSET_M              (0.059f)
#define JUMP_RETRACT_Z_OFFSET_M             LEG_DEFAULT_Z_OFFSET_M
#define JUMP_BUFFER_Z_OFFSET_M              (0.020f)

// All four servos share the steer_1 calibration: horizontal 4500 and 3000 PWM
// counts per 90 degrees. Normal extension uses only 2000 counts; the measured
// opposite-side travel from horizontal is 1500 counts. Mirrored installations
// apply those relative travels in the opposite PWM direction.
#define LEG_SERVO_COUNT                    (4U)
#define LEG_SERVO_FREQUENCY_HZ             (300U)
#define LEG_SERVO_CALIBRATION_COMPLETE     (1U)
#define LEG_SERVO_REFERENCE_PWM            (4500)
#define LEG_SERVO_90_DEG_TRAVEL_PWM        (3000)
#define LEG_SERVO_PWM_PER_RAD              (1909.85932f)
#define LEG_SERVO_1_MECHANICAL_MIN_PWM     (3000)
#define LEG_SERVO_1_MECHANICAL_MAX_PWM     (7500)
#define LEG_SERVO_SAFE_EXTEND_TRAVEL_PWM   (2000)
#define LEG_SERVO_SAFE_RETRACT_TRAVEL_PWM  (1500)
#define LEG_SERVO_1_PWM                    (TCPWM_CH10_P05_1)
#define LEG_SERVO_2_PWM                    (TCPWM_CH12_P05_3)
#define LEG_SERVO_3_PWM                    (TCPWM_CH09_P05_0)
#define LEG_SERVO_4_PWM                    (TCPWM_CH11_P05_2)
#define LEG_SERVO_1_CENTER                 LEG_SERVO_REFERENCE_PWM
#define LEG_SERVO_2_CENTER                 LEG_SERVO_REFERENCE_PWM
#define LEG_SERVO_3_CENTER                 LEG_SERVO_REFERENCE_PWM
#define LEG_SERVO_4_CENTER                 LEG_SERVO_REFERENCE_PWM
#define LEG_SERVO_1_DIRECTION              (1)
#define LEG_SERVO_2_DIRECTION              (-1)
#define LEG_SERVO_3_DIRECTION              (1)
#define LEG_SERVO_4_DIRECTION              (-1)
// Joint B moves from pi toward pi/2 during extension, so the angle-to-PWM
// directions above produce effective extension signs +, -, -, + for 1..4.
#define LEG_SERVO_1_PWM_PER_RAD            LEG_SERVO_PWM_PER_RAD
#define LEG_SERVO_2_PWM_PER_RAD            LEG_SERVO_PWM_PER_RAD
#define LEG_SERVO_3_PWM_PER_RAD            LEG_SERVO_PWM_PER_RAD
#define LEG_SERVO_4_PWM_PER_RAD            LEG_SERVO_PWM_PER_RAD
#define LEG_SERVO_1_ZERO_RAD               (0.0f)
#define LEG_SERVO_2_ZERO_RAD               (0.0f)
#define LEG_SERVO_3_ZERO_RAD               (3.14159265f)
#define LEG_SERVO_4_ZERO_RAD               (3.14159265f)
#define LEG_SERVO_1_PWM_MIN                (LEG_SERVO_REFERENCE_PWM \
                                            - LEG_SERVO_SAFE_RETRACT_TRAVEL_PWM)
#define LEG_SERVO_1_PWM_MAX                (LEG_SERVO_REFERENCE_PWM \
                                            + LEG_SERVO_SAFE_EXTEND_TRAVEL_PWM)
#define LEG_SERVO_2_PWM_MIN                (LEG_SERVO_REFERENCE_PWM \
                                            - LEG_SERVO_SAFE_EXTEND_TRAVEL_PWM)
#define LEG_SERVO_2_PWM_MAX                (LEG_SERVO_REFERENCE_PWM \
                                            + LEG_SERVO_SAFE_RETRACT_TRAVEL_PWM)
#define LEG_SERVO_3_PWM_MIN                LEG_SERVO_2_PWM_MIN
#define LEG_SERVO_3_PWM_MAX                LEG_SERVO_2_PWM_MAX
#define LEG_SERVO_4_PWM_MIN                LEG_SERVO_1_PWM_MIN
#define LEG_SERVO_4_PWM_MAX                LEG_SERVO_1_PWM_MAX
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

// A new relative-yaw sample is recorded every 5 cm of forward travel. All
// navigation and route-recorder distance values use metres.
#define NAV_FLASH_SAMPLE_DISTANCE_M       (0.05f)

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
