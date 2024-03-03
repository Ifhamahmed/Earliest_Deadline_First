/*
                    Library Implementing the Real-Time, Dynamic Priority Scheduling Algorithm: Earliest Deadline First (EDF) 

    Author: Ifham Ahmed
    Institution: Albert Ludwigs Universität Freiburg
    Module: Praktikum on "Advanced Real-Time Scheduling with FreeRTOS"

    Notice: 

        This library is intended for advanced users only. Do not attempt to change the files unless you have a good understanding 
        of Real-Time scheduling and FreeRTOS.

    Description:

        Library Implementation for the Extended EDF Scheduler. Implements a more efficient EDF Scheduler with task lists for
        suspended, blocked and ready tasks. This implementation makes use of FreeRTOS Notification signals to invoke the 
        scheduler when required. This library also provides Total Bandwidth Server (TBS) capabilities while also making it
        possible to instead use a generic aperiodic server. In addition, the library also provides WCET overflow and Deadline
        overflow detection. Incase any of these overflows occur, an appropriate action is taken by the scheduler. 
        For WCET Overflows: The task period is doubled for one instance. For Deadline overflows: the task is deleted since
        a deadline overflow would mean that the task cannot be scheduled among the current task set.

    Instructions to use on ESP32:

        1) Make the necessary settings in the Config header file (Enable or Disable TBS) and also the trace features
            Special trace features are available for ESP32 using the High Precision Hardware Timers. If this is not required,
            trace features using FreeRTOS ticks are also available. However tracing based on ticks is not very accurate as multiple
            tasks can run in a single tick and this cannot be captured due to the interrupt granularity. As such, using ESP timers
            are more accurate
        2) Call the EDFInit() function once before creating the tasks and beginning the scheduler
        3) Create the tasks with the EDFCreatePeriodicTask() and EDFCreateAperiodicTask() functions
        4) The parameters for the task are to be passed using the respective task config structures (// TODO)
        5) Once the tasks are all created, call the EDFStartScheduling() function (and suspend the app_main() task in ESP-IDF)
        6) If it is needed to stop the scheduler, the app_main() task must be woken up and all the created tasks must be 
            deleted using the EDFDeleteAllTasks() function. The scheduler cannot be stopped and resumed as this would corrupt all
            the timing information.
    
    Information on porting:

        It is possible to port this library to other micro-controllers as the library only depends on FreeRTOS and not any third party libraries
        apart from the standart C libraries: stdlib.h, string.h which are available on most platforms

        The library also only uses basic datatypes and structures already provided by FreeRTOS. However, some changes might be required such as 
        tracing and some due to datatype definitions if the Micro-controller does not support original FreeRTOS datatypes or requires modifications.

*/

#ifndef _EXTEDFLIB_H_
#define _EXTEDFLIB_H_


// ****************************** EDF DEFINES *****************************//
#define TASK_NUM_START                      4 // 2 for MAIN TASK and 3 for IDLE TASK as TCB Numbers
#define IDLE_TASK_NUM                       TASK_NUM_START - 1
#define MAIN_TASK_NUM                       IDLE_TASK_NUM - 1
#define MAX_NUM_OF_PERIODIC_TASKS           10
#define MAX_NUM_OF_APERIODIC_TASKS          4
#define TOTAL_NUM_OF_TASKS                  MAX_NUM_OF_APERIODIC_TASKS + MAX_NUM_OF_PERIODIC_TASKS
#define SCHED_TASK_NUM                      TOTAL_NUM_OF_TASKS + TASK_NUM_START
#define APERIODIC_SERVER_NUM                SCHED_TASK_NUM + 1
// Set task priorities here
#define MAX_SYS_PRIO                        configMAX_PRIORITIES - 5
#define SCHED_PRIO                          MAX_SYS_PRIO + 1
#define RUNNING_TASK_PRIO                   MAX_SYS_PRIO
#define LOWEST_SYS_PRIO                     tskIDLE_PRIORITY
#define APERIODIC_PRIO                      LOWEST_SYS_PRIO + 1
#define BLOCKED_TASK_PRIO                   APERIODIC_PRIO + 1
// Task Local Storage
#define LOCAL_STORAGE_INDEX                 0
// Aperiodic Server Stack Size
#define APERIODIC_SERVER_STACK              3000
// Periodic Utilization Limit
#define UP_LIMIT                            0.9f

// *********************************************************************** //
// *********************** Common Defines Include ************************ //
#include "commonDefines.h"
// *********************************************************************** //
// *********************************************************************** //
// *********************** System Includes ******************************* //
#include <stdlib.h>
#include <string.h>
// *********************************************************************** //
// *********************** FREERTOS Includes ***************************** //
//#include "traceMacros.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/list.h"
#include "freertos/event_groups.h"
// *********************************************************************** //

// ************************* Data Structures ***************************** //
// Scheduler Signals
#define SWITCH_ON_BLOCK                 (1 << 0)
#define SWITCH_ON_READY                 (1 << 1)
#define SWITCH_ON_SUSPEND               (1 << 2)
#if USE_WCET_CHECKS == 1
#define SWITCH_ON_WCET_OVERFLOW         (1 << 3)
#else
#define SWITCH_ON_WCET_OVERFLOW         0x00
#endif

#if USE_DEADLINE_CHECKS == 1
#define SWITCH_ON_DEADLINE_OVERFLOW     (1 << 4)
#else
#define SWITCH_ON_DEADLINE_OVERFLOW     0x00
#endif

#if USE_WCET_CHECKS == 1
#define SWITCH_ON_WCET_WAKEUP           (1 << 5)
#else
#define SWITCH_ON_WCET_WAKEUP           0x00
#endif

#define ALL_SWITCHES                    SWITCH_ON_BLOCK | SWITCH_ON_READY | SWITCH_ON_SUSPEND | SWITCH_ON_WCET_OVERFLOW | SWITCH_ON_DEADLINE_OVERFLOW | SWITCH_ON_WCET_WAKEUP


// Task states periodic
typedef enum taskStatus
{
    TASK_BLOCKED = 1,
    TASK_READY,
    TASK_SUSPENDED,
    TASK_RUNNING
} taskStatus;

#if USE_TBS == 0
// task states aperiodic
typedef enum taskStatusA
{
    TASK_NOT_ARRIVED = 1,
    TASK_ARRIVED,
    TASK_EXECUTED
} taskStatusA;
#endif
/*
Structures to hold extended TCB
*/
typedef struct extTCB
{
    TaskHandle_t cTaskHandle;
    const char * taskName;
    uint32_t stackSize;
    void (*instanceFunc)(void*);
    void *instanceParams;
    TickType_t WCET;
    TickType_t measuredExecTime;
    TickType_t period;
    TickType_t phase;
    TickType_t relDeadline;
    TickType_t relArrivalTime;
    TickType_t absDeadline;
    ListItem_t xTCBListItem;
    BaseType_t xPriority; 
    BaseType_t xTaskNumber;
    taskStatus status; 

    #if USE_TBS == 1
    BaseType_t executedTBSTask;
    BaseType_t isPeriodic;
    #endif

    #if USE_WCET_CHECKS == 1
    BaseType_t WCETExceeded;
    TickType_t nextUnblockTime; // incase of WCET Overflow
    #endif

    #if USE_DEADLINE_CHECKS == 1
    BaseType_t deadlineExceeded;
    #endif
} extTCB_t;

#if USE_TBS == 0
typedef struct extTCBA
{
    const char * taskName;
    void (*instanceFunc)(void*);
    void *instanceParams;
    TickType_t WCET;
    TickType_t measuredExecTime;
    BaseType_t executedThisInstance;
    BaseType_t xTaskNumber;
    TickType_t phase;
    ListItem_t xTCBAListItem;
    uint32_t stackSize;
    taskStatusA status;
} extTCBA_t;
#endif
// TODO 9: (Low) Add a task config structure to pass to the task creation functions rather than having many function arguments
// ************************************************************************ //
// *************************** Globals ************************************ //
/*
Some of these variables and arrays are externed by the trace macros
*/

#ifdef TRACE_CONFIG
#define TRACE_ARRAY_WIDTH                   MAX_NUM_OF_APERIODIC_TASKS + MAX_NUM_OF_PERIODIC_TASKS + 2
extern int task_nums[TRACE_ARRAY_SIZE][TRACE_ARRAY_WIDTH];
extern int trcIndex;
#endif

#ifdef ESP_TRACE_CONFIG
#define TRACE_TIMER_TIMEOUT                 5 * 1000 * 1000 // 5 seconds max
#define MAX_NUM_OF_EVENTS                   1000           

extern int64_t eventArray[MAX_NUM_OF_EVENTS][3];     // Structure of eventArray at each index  | -- Task Number -- | -- Entry Time -- | -- Exit Time -- |
extern int64_t eventIndex;
extern esp_timer_handle_t traceTimer;
#endif

// ********************** Function Declarations *************************** //
void EDFCreatePeriodicTask(const char* taskName, int stackSize, void (*instanceFunc)(void*), int timePeriod, int relDeadline, int phase, TaskHandle_t *handle, void *instanceParams, TickType_t WCETinTicks);
void EDFCreateAperiodicTask(const char* taskName, void (*instanceFunc)(void*), void *instanceParams, int stackSize, TickType_t WCET, TickType_t arrivalTime);
void EDFStartScheduling();
void EDFDeleteAllTasks();
void EDFInit();

// TODO 10: (Low) Moved Function declarations for internal functions into the source file
// ********************** Idle Hook Declaration **************************** //
#ifdef TRACE_CONFIG
void vApplicationIdleHook(void);
#endif

#ifdef ESP_TRACE_CONFIG
void espTimerCallback(void *arg);
void vApplicationIdleHook(void);
#endif
// ************************************************************************ //
// ********************** Tick Hook Declaration **************************** //
void vApplicationTickHook(void);
// ************************************************************************ //

#endif // _EXTEDFLIB_H_
