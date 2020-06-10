
#include "freertos_main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"
#include "portable.h"

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    (void) pcTaskName;
    (void) pxTask;

    //
    // Run time stack overflow checking is performed if
    // configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    // function is called if a stack overflow is detected.
    //
    while (1)
    {
        __asm("BKPT #0\n") ; // Break into the debugger
    }
}

void vApplicationMallocFailedHook(void) {
    //
    // Called if a call to pvPortMalloc() fails because there is insufficient
    // free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    // internally by FreeRTOS API functions that create tasks, queues, software
    // timers, and semaphores.  The size of the FreeRTOS heap is set by the
    // configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
    //
    while (1);
}

//*****************************************************************************
//
// Sleep function called from FreeRTOS IDLE task.
// Do necessary application specific Power down operations here
// Return 0 if this function also incorporates the WFI, else return value same
// as idleTime
//
//*****************************************************************************
uint32_t am_freertos_sleep(uint32_t idleTime)
{
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    return 0;
}

//*****************************************************************************
//
// Recovery function called from FreeRTOS IDLE task, after waking up from Sleep
// Do necessary 'wakeup' operations here, e.g. to power up/enable peripherals etc.
//
//*****************************************************************************
void am_freertos_wakeup(uint32_t idleTime)
{
    return;
}

#if configUSE_TICKLESS_IDLE == 2
uint32_t ulTimerCountsForOneTick = 0;
/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * resolution of the Tick timer.
 */
static uint32_t xMaximumPossibleSuppressedTicks = 0;
static uint32_t g_lastSTimerVal = 0;

void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    uint32_t ulReloadValue;
    uint32_t New_Timer, Delta_Sleep;
    TickType_t xModifiableIdleTime;
    uint32_t elapsed_time;
    /* Make sure the SysTick reload value does not overflow the counter. */
    if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }


    /* Calculate the reload value required to wait xExpectedIdleTime
    tick periods.  -1 is used because this code will execute part way
    through one of the tick periods. */
    ulReloadValue =  ulTimerCountsForOneTick * ( xExpectedIdleTime - 1 );

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    __disable_interrupt();
    __DSB();
    __ISB();

#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK
    // Adjust for the time already elapsed
    elapsed_time = am_hal_stimer_counter_get() - g_lastSTimerVal;
#else
    am_hal_ctimer_stop(configCTIMER_NUM, AM_HAL_CTIMER_BOTH);
    // Adjust for the time already elapsed
    elapsed_time = am_hal_ctimer_read(configCTIMER_NUM, AM_HAL_CTIMER_BOTH);
#endif


    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    /* Abandon low power entry if the sleep time is too short */
    if( (eTaskConfirmSleepModeStatus() == eAbortSleep) || ((elapsed_time + ulTimerCountsForOneTick) > ulReloadValue) )
    {
#ifndef AM_FREERTOS_USE_STIMER_FOR_TICK
        am_hal_ctimer_start(configCTIMER_NUM, AM_HAL_CTIMER_BOTH);
#endif
        /* Re-enable interrupts - see comments above __disable_irq() call
        above. */
        __enable_interrupt();

    }
    else
    {
        // Adjust for the time already elapsed
        ulReloadValue -= elapsed_time;
        // Initialize new timeout value
#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK
        am_hal_stimer_compare_delta_set(0, ulReloadValue);
#else
        am_hal_ctimer_clear(configCTIMER_NUM, AM_HAL_CTIMER_BOTH);
        am_hal_ctimer_compare_set(configCTIMER_NUM, AM_HAL_CTIMER_BOTH, 0, ulReloadValue);
        am_hal_ctimer_start(configCTIMER_NUM, AM_HAL_CTIMER_BOTH);
#endif

        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
        set its parameter to 0 to indicate that its implementation contains
        its own wait for interrupt or wait for event instruction, and so wfi
        should not be executed again.  However, the original expected idle
        time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;

        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );       // Turn OFF all Periphials in this function

        if( xModifiableIdleTime > 0 )
        {
                __DSB();
                __WFI();
                __ISB();
        }

        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );       // Turn ON all Periphials in this function

        // Any interrupt may have woken us up

        // Before renable interrupts, check how many ticks the processor has been in SLEEP
        // Adjust xTickCount via vTaskStepTick( Delta_Sleep )
        // to keep xTickCount up to date, as if ticks have been running all along

#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK
        New_Timer = am_hal_stimer_counter_get();
        Delta_Sleep = (signed long) New_Timer - (signed long) g_lastSTimerVal;
        g_lastSTimerVal = New_Timer - Delta_Sleep%ulTimerCountsForOneTick;
#else
        am_hal_ctimer_stop(configCTIMER_NUM, AM_HAL_CTIMER_BOTH);
        New_Timer = am_hal_ctimer_read(configCTIMER_NUM, AM_HAL_CTIMER_BOTH);
        // INTSTAT check is needed to handle a possible case where the we came here without timer
        // incrementing at all....the value will still say 0, but it does not mean it expired
        if ((New_Timer == 0) && ((am_hal_ctimer_int_status_get(false) & (1 << configCTIMER_NUM*2))))
        {
            // The timer ran to completion and reset itself
            Delta_Sleep = ulReloadValue;
            // Clear the INTSTAT to prevent interrupt handler from counting an extra tick
            am_hal_ctimer_int_clear((1 << configCTIMER_NUM*2));
        } else
        {
            Delta_Sleep = New_Timer; // Indicates the time elapsed since we slept
        }
#endif

        Delta_Sleep /= ulTimerCountsForOneTick;

        // Correct System Tick after Sleep
        vTaskStepTick( Delta_Sleep );

        /* Restart System Tick */
#ifdef AM_FREERTOS_USE_STIMER_FOR_TICK

        // Clear the interrupt - to avoid extra tick counting in ISR
        am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
        am_hal_stimer_compare_delta_set(0, ulTimerCountsForOneTick);
#else
        am_hal_ctimer_clear(configCTIMER_NUM, AM_HAL_CTIMER_BOTH);
        am_hal_ctimer_compare_set(configCTIMER_NUM, AM_HAL_CTIMER_BOTH, 0, ulTimerCountsForOneTick);


        am_hal_ctimer_start(configCTIMER_NUM, AM_HAL_CTIMER_BOTH);
#endif
        /* Re-enable interrupts - see comments above __disable_irq() call above. */
        __enable_interrupt();

    }
}

#endif /* #if configUSE_TICKLESS_IDLE = 2 */

void task_alloc_stack(void *pvParameters) {
    while(1) {}
}

void task_log1(void *pvParameters) {
    int count = 0;
    while(1) {
        CWM_OS_dbgPrintf("%s: %d\n", __func__, count++);
        vTaskDelay(100);
    }
}

void task_log2(void *pvParameters) {
    int count = 0;
    while(1) {
        CWM_OS_dbgPrintf("%s: %d\n", __func__, count++);
        vTaskDelay(300);
    }
}

//*****************************************************************************
//
// Main Function
//
//*****************************************************************************

TaskHandle_t hTaskAllocStack;
TaskHandle_t hTaskLog1, hTaskLog2;

int main(void)
{
    board_init();

    xTaskCreate(task_alloc_stack, "alloc_stack", 1024, NULL, 1, &hTaskAllocStack);
    xTaskCreate(task_log1, "log1", 256, NULL, 1, &hTaskLog1);
    xTaskCreate(task_log2, "log2", 256, NULL, 1, &hTaskLog2);

    vTaskStartScheduler();
}

