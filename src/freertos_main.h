

#ifndef FREERTOS_MAIN_H
#define FREERTOS_MAIN_H

//*****************************************************************************
//
// Required built-ins.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

//*****************************************************************************
//
// Standard AmbiqSuite includes.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
//#include "am_devices.h"
#include "am_util.h"

//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

//*****************************************************************************
//
// Task include files.
//
//*****************************************************************************

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void disable_print_interface(void);

// board_apollo3.c
void board_init();
int CWM_OS_dbgPrintf(const char * format,...);


#endif // FREERTOS_LOWPOWER_H
