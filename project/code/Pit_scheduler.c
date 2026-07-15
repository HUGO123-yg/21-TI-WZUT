#include "Pit_scheduler.h"

#include <string.h>

#include "Control_system.h"
#include "config.h"
#include "cy_device_headers.h"
#include "syslib/cy_syslib.h"
#include "system/system_cyt4bb.h"

#define PIT_DWT_UNLOCK_KEY (0xC5ACCE55UL)

#if PIT_CONTROL_DEFER_ENABLE \
    && ((PIT_CONTROL_PENDING_TICK_LIMIT == 0U) \
        || (PIT_CONTROL_MAX_TICKS_PER_PENDSV == 0U))
#error "Deferred PIT scheduling requires non-zero queue and service limits."
#endif

static volatile pit_scheduler_stats_t pit_stats;
static uint32 pit_isr_budget_cycles;
#if PIT_CONTROL_DEFER_ENABLE
static uint32 control_tick_budget_cycles;
#endif

static uint32 pit_scheduler_us_to_cycles(uint32 time_us)
{
    uint32 cycles_per_us;

    cycles_per_us = (SystemCoreClock + 999999UL) / 1000000UL;
    return cycles_per_us * time_us;
}

#if PIT_CONTROL_DEFER_ENABLE
static void pit_scheduler_record_control_tick(uint32 start_cycles)
{
    uint32 elapsed_cycles;

    if (!pit_stats.profiling_available)
    {
        return;
    }

    elapsed_cycles = DWT->CYCCNT - start_cycles;
    pit_stats.control_tick_last_cycles = elapsed_cycles;
    pit_stats.control_tick_sample_count++;
    if (elapsed_cycles > pit_stats.control_tick_max_cycles)
    {
        pit_stats.control_tick_max_cycles = elapsed_cycles;
    }
    if (elapsed_cycles > control_tick_budget_cycles)
    {
        pit_stats.control_tick_budget_exceeded_count++;
    }
}

static uint8 pit_scheduler_take_pending_tick(void)
{
    uint32 interrupt_state;
    uint8 tick_available;

    interrupt_state = Cy_SysLib_EnterCriticalSection();
    tick_available = (uint8)(pit_stats.pending_ticks > 0U);
    if (tick_available)
    {
        pit_stats.pending_ticks--;
    }
    Cy_SysLib_ExitCriticalSection(interrupt_state);
    return tick_available;
}
#endif

void pit_scheduler_init(void)
{
    memset((void *)&pit_stats, 0, sizeof(pit_stats));
    pit_stats.core_clock_hz = SystemCoreClock;
    pit_isr_budget_cycles = pit_scheduler_us_to_cycles(
        PIT_ISR_WCET_BUDGET_US);
#if PIT_CONTROL_DEFER_ENABLE
    control_tick_budget_cycles = pit_scheduler_us_to_cycles(
        PIT_CONTROL_TICK_WCET_BUDGET_US);
#endif

#if PIT_RUNTIME_PROFILING_ENABLE
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = PIT_DWT_UNLOCK_KEY;
    if (0U == (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk))
    {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        __DSB();
        __ISB();
        pit_stats.profiling_available = 1U;
    }
#endif

#if PIT_CONTROL_DEFER_ENABLE
    NVIC_SetPriority(PendSV_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
#endif
}

uint32 pit_scheduler_measure_begin(void)
{
    return pit_stats.profiling_available ? DWT->CYCCNT : 0U;
}

void pit_scheduler_record_pit_isr(uint32 start_cycles)
{
    uint32 elapsed_cycles;

    if (!pit_stats.profiling_available)
    {
        return;
    }

    elapsed_cycles = DWT->CYCCNT - start_cycles;
    pit_stats.pit_isr_last_cycles = elapsed_cycles;
    pit_stats.pit_isr_sample_count++;
    if (elapsed_cycles > pit_stats.pit_isr_max_cycles)
    {
        pit_stats.pit_isr_max_cycles = elapsed_cycles;
    }
    if (elapsed_cycles > pit_isr_budget_cycles)
    {
        pit_stats.pit_isr_budget_exceeded_count++;
    }
}

void pit_scheduler_schedule_tick_from_isr(void)
{
#if PIT_CONTROL_DEFER_ENABLE
    if (pit_stats.pending_ticks < PIT_CONTROL_PENDING_TICK_LIMIT)
    {
        pit_stats.pending_ticks++;
        if (pit_stats.pending_ticks > pit_stats.pending_tick_high_watermark)
        {
            pit_stats.pending_tick_high_watermark = pit_stats.pending_ticks;
        }
    }
    else
    {
        pit_stats.dropped_tick_count++;
    }
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
#else
    control_system_tick_1ms();
#endif
}

void pit_scheduler_run_deferred(void)
{
#if PIT_CONTROL_DEFER_ENABLE
    uint32 processed_ticks;
    uint32 start_cycles;

    processed_ticks = 0U;
    while ((processed_ticks < PIT_CONTROL_MAX_TICKS_PER_PENDSV)
           && pit_scheduler_take_pending_tick())
    {
        start_cycles = pit_scheduler_measure_begin();
        control_system_tick_1ms();
        pit_scheduler_record_control_tick(start_cycles);
        processed_ticks++;
    }
    if (pit_stats.pending_ticks > 0U)
    {
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
#endif
}

const volatile pit_scheduler_stats_t *pit_scheduler_get_stats(void)
{
    return &pit_stats;
}

void pit_scheduler_reset_measurements(void)
{
    uint32 interrupt_state;

    interrupt_state = Cy_SysLib_EnterCriticalSection();
    pit_stats.pit_isr_sample_count = 0U;
    pit_stats.pit_isr_last_cycles = 0U;
    pit_stats.pit_isr_max_cycles = 0U;
    pit_stats.pit_isr_budget_exceeded_count = 0U;
    pit_stats.control_tick_sample_count = 0U;
    pit_stats.control_tick_last_cycles = 0U;
    pit_stats.control_tick_max_cycles = 0U;
    pit_stats.control_tick_budget_exceeded_count = 0U;
    pit_stats.pending_tick_high_watermark = pit_stats.pending_ticks;
    pit_stats.dropped_tick_count = 0U;
    Cy_SysLib_ExitCriticalSection(interrupt_state);
}
