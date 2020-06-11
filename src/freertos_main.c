
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

