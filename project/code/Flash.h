#ifndef PROJECT_FLASH_H
#define PROJECT_FLASH_H

#include "zf_common_typedef.h"

// Result codes returned by the project-level navigation Flash module.
typedef enum
{
    NAV_FLASH_STATUS_OK = 0,
    NAV_FLASH_STATUS_NOT_INITIALIZED,
    NAV_FLASH_STATUS_INVALID_ARGUMENT,
    NAV_FLASH_STATUS_INVALID_CONFIG,
    NAV_FLASH_STATUS_NOT_FOUND,
    NAV_FLASH_STATUS_BUSY,
    NAV_FLASH_STATUS_FULL,
    NAV_FLASH_STATUS_CORRUPT,
    NAV_FLASH_STATUS_IO_ERROR
} nav_flash_status_t;

// Public state is intentionally small; page indexes and buffers remain private
// so menu and control code cannot modify an in-progress Flash transaction.
typedef enum
{
    NAV_FLASH_STATE_UNINITIALIZED = 0,
    NAV_FLASH_STATE_IDLE,
    NAV_FLASH_STATE_RECORDING,
    NAV_FLASH_STATE_FLUSH_PENDING,
    NAV_FLASH_STATE_ABORT_PENDING,
    NAV_FLASH_STATE_COMMIT_COMPLETE,
    NAV_FLASH_STATE_ABORTED,
    NAV_FLASH_STATE_LOADING,
    NAV_FLASH_STATE_REPLAY_READY,
    NAV_FLASH_STATE_ERROR
} nav_flash_state_t;

typedef struct
{
    uint8 route_id;
    uint8 valid;
    uint16 reserved;
    uint32 sample_count;
    uint32 data_generation;
} nav_flash_route_info_t;

// Initialize project-level metadata after the low-level flash_init() call.
// This function only reads Work Flash and must be called from the main context.
nav_flash_status_t nav_flash_init(void);

// Start replacing one route. Samples are kept in RAM until a full page or
// record_stop requests a flush. route_id is in the range 1..3.
nav_flash_status_t nav_flash_record_start(uint8 route_id);

// Add forward distance in metres and record yaw whenever
// NAV_FLASH_SAMPLE_DISTANCE_M is crossed. This function performs no Flash I/O
// and may run in a periodic ISR. distance_delta_m must be finite and non-negative.
nav_flash_status_t nav_flash_record_sample(float yaw_deg,
                                            float distance_delta_m);

// Request final-page and metadata commit. Completion is asynchronous; call
// nav_flash_service() from the main loop until COMMIT_COMPLETE or ERROR.
nav_flash_status_t nav_flash_record_stop(void);

// Cancel the current uncommitted recording. If data pages have already been
// written over an older route, service() asynchronously invalidates that old
// route before reporting ABORTED so metadata never references partial data.
// This function performs no Flash I/O and is safe to call outside main context.
nav_flash_status_t nav_flash_record_abort(void);

// Execute at most one pending page write and any resulting metadata commit.
// This function uses blocking driver calls and must never run in an ISR.
nav_flash_status_t nav_flash_service(void);

// Load and validate a complete route into RAM before enabling vehicle motion.
// No Flash reads are required later while replaying the route.
nav_flash_status_t nav_flash_load_route(uint8 route_id);

// Read a loaded yaw sample in degrees. The returned data is valid only while
// NAV_FLASH_STATE_REPLAY_READY is reported.
nav_flash_status_t nav_flash_get_sample(uint32 index, float *yaw_deg);

// Invalidate a route in the redundant metadata. Data pages are left in place
// to avoid 31 unnecessary erases and are overwritten by the next recording.
nav_flash_status_t nav_flash_delete_route(uint8 route_id);

nav_flash_status_t nav_flash_get_route_info(
    uint8 route_id,
    nav_flash_route_info_t *info);

nav_flash_state_t nav_flash_get_state(void);
nav_flash_status_t nav_flash_get_last_status(void);
uint32 nav_flash_get_loaded_sample_count(void);

#endif
