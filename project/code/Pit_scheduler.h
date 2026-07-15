#ifndef PROJECT_PIT_SCHEDULER_H
#define PROJECT_PIT_SCHEDULER_H

#include "zf_common_typedef.h"

typedef struct
{
    uint32 core_clock_hz;
    uint32 profiling_available;
    uint32 pit_isr_sample_count;
    uint32 pit_isr_last_cycles;
    uint32 pit_isr_max_cycles;
    uint32 pit_isr_budget_exceeded_count;
    uint32 control_tick_sample_count;
    uint32 control_tick_last_cycles;
    uint32 control_tick_max_cycles;
    uint32 control_tick_budget_exceeded_count;
    uint32 pending_ticks;
    uint32 pending_tick_high_watermark;
    uint32 dropped_tick_count;
} pit_scheduler_stats_t;

// Enables the Cortex-M7 DWT cycle counter and assigns PendSV the lowest
// exception priority. Call once after clock initialization and before PIT_CH0.
void pit_scheduler_init(void);

// These calls bracket the PIT workload. Profiler bookkeeping and the C function
// epilogue are outside the interval; nested higher-priority interrupt time is
// deliberately included to give a conservative response-time measurement.
uint32 pit_scheduler_measure_begin(void);
void pit_scheduler_record_pit_isr(uint32 start_cycles);

// Queues one logical 1 ms control tick and pends the low-priority worker.
void pit_scheduler_schedule_tick_from_isr(void);

// PendSV entry point. It runs a bounded number of queued control ticks and
// repends itself when work remains, so higher-priority peripheral ISRs can run.
void pit_scheduler_run_deferred(void);

// Stable address for the debugger/watch window. At 250 MHz, divide cycle
// values by 250 to obtain microseconds.
const volatile pit_scheduler_stats_t *pit_scheduler_get_stats(void);
void pit_scheduler_reset_measurements(void);

#endif
