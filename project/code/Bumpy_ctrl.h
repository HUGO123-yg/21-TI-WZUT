#ifndef PROJECT_BUMPY_CTRL_H
#define PROJECT_BUMPY_CTRL_H

#include "Balance_ctrl.h"
#include "Imu.h"
#include "zf_common_typedef.h"

typedef enum
{
    BUMPY_CTRL_STATUS_OK = 0,
    BUMPY_CTRL_STATUS_DISABLED,
    BUMPY_CTRL_STATUS_INVALID_ARGUMENT,
    BUMPY_CTRL_STATUS_INVALID_CONFIG
} bumpy_ctrl_status_t;

typedef enum
{
    BUMPY_PHASE_IDLE = 0,
    BUMPY_PHASE_DETECTING,
    BUMPY_PHASE_CROSSING,
    BUMPY_PHASE_RECOVERING
} bumpy_phase_t;

typedef struct
{
    bumpy_ctrl_status_t status;
    bumpy_phase_t phase;
    uint32 phase_elapsed_steps;
    uint32 impact_count;
    uint32 impact_refractory_steps;
    uint32 recover_stable_steps;
    uint8 enabled;
    uint8 forced_entry;
    uint8 impact_latched;
    float entry_distance_m;
    float crossing_distance_m;
    float impact_magnitude_g;
    float speed_error_m_s;
    float speed_leg_x_offset_m;
    float shaped_speed_m_s;
    float shaped_yaw_rate_rad_s;
} bumpy_ctrl_state_t;

bumpy_ctrl_status_t bumpy_ctrl_init(void);
bumpy_ctrl_status_t bumpy_ctrl_set_enabled(uint8 enabled);

// Route/mileage control should call this at the first rib. Automatic impact
// detection remains available as a fallback when the route location is unknown.
bumpy_ctrl_status_t bumpy_ctrl_force_enter(float traveled_distance_m);
bumpy_ctrl_status_t bumpy_ctrl_abort(void);

// Runs at CONTROL_FAST_PERIOD_S. In crossing mode the speed loop changes the
// common leg x offset and asks Balance_ctrl to keep pitch/rate control separate.
bumpy_ctrl_status_t bumpy_ctrl_update(const imu_data_t *imu,
                                      float traveled_distance_m,
                                      float measured_speed_m_s,
                                      uint8 run_speed_loop,
                                      const balance_command_t *requested,
                                      balance_command_t *shaped);

// Active means this module currently owns speed-to-leg command shaping.
uint8 bumpy_ctrl_is_active(void);
// Monitoring also includes the impact-confirmation phase.
uint8 bumpy_ctrl_is_monitoring(void);
const bumpy_ctrl_state_t *bumpy_ctrl_get_state(void);

#endif
