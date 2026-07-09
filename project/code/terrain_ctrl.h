#ifndef _terrain_ctrl_h_
#define _terrain_ctrl_h_

#include "zf_common_headfile.h"
#include "bridge_ctrl.h"
#include "obstacle_ctrl.h"

// Terrain arbitration only distinguishes normal road, single-side bridge and bump road.
// Grass is intentionally handled as normal road.
typedef enum
{
    TERRAIN_NORMAL = 0,
    TERRAIN_BRIDGE,
    TERRAIN_OBSTACLE
} terrain_state_e;

void terrain_init(void);
void terrain_abort(void);
void terrain_run_1ms(float mileage, int16 speed, int16 balance_motor, float pitch_deg, float roll_deg, uint32 armed_ticks);

terrain_state_e terrain_get_state(void);
uint8 terrain_bridge_is_active(void);
uint8 terrain_obstacle_is_active(void);
uint16 terrain_get_bridge_count(void);
uint16 terrain_get_obstacle_score(void);
float terrain_get_course_mileage(void);
uint8 terrain_bridge_mileage_window_active(void);
uint8 terrain_obstacle_mileage_window_active(void);
void terrain_course_reset(float mileage);

#ifdef USE_TERRAIN_DEBUG
typedef struct
{
    uint32 tick;
    terrain_state_e terrain_state;
    bridge_state_e bridge_state;
    obstacle_state_e obstacle_state;
    float mileage;
    float course_mileage;
    int16 speed;
    int16 balance_motor;
    int16 left_motor;
    int16 right_motor;
    float pitch_deg;
    float roll_deg;
    float gyro_pitch_dps;
    float gyro_roll_dps;
    float angle_out;
    float speed_out;
    float track_out;
    float nav_out;
    uint16 bridge_count;
    uint16 obstacle_score;
    uint16 obstacle_stuck_count;
    uint8 obstacle_retry_count;
    uint8 bridge_window;
    uint8 obstacle_window;
} terrain_debug_sample_t;

void terrain_debug_update(uint32 tick,
                          int16 left_motor,
                          int16 right_motor,
                          float angle_out,
                          float speed_out,
                          float track_out,
                          float nav_out);
void terrain_debug_service(void);
const terrain_debug_sample_t *terrain_debug_get_sample(void);
#else
#define terrain_debug_update(tick, left_motor, right_motor, angle_out, speed_out, track_out, nav_out) ((void)0)
#define terrain_debug_service() ((void)0)
#define terrain_debug_get_sample() (0)
#endif

#endif
