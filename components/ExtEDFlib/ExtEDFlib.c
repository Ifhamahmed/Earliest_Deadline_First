// ************************* File Includes *************************** // 
#include "ExtEDFlib.h"
// ******************************************************************* //
// *************************** Globals ************************************ //
/*
Some of these variables and arrays are externed to the trace macros
*/

#ifdef TRACE_CONFIG
int task_nums[TRACE_ARRAY_SIZE][TRACE_ARRAY_WIDTH];
int trcIndex = 0;
#endif

#ifdef ESP_TRACE_CONFIG
esp_timer_handle_t traceTimer;
int64_t eventArray[MAX_NUM_OF_EVENTS][3];
int64_t eventIndex = 0;
void espTimerCallback(void *arg)  {return;}
void vApplicationIdleHook(void) {return;}
#endif

// ******************* Extended EDF Variables *********************** //
// Task Lists
// Initial Lists

// Final Ready, Blocked and Suspended Lists
static List_t xTCBReadyListVar;
static List_t xTCBBlockedListVar;
static List_t xTCBSuspendedListVar;
static List_t xTCBAperiodicListVar;
static List_t xTCBInitListVar;

static List_t * xTCBReadyList = &xTCBReadyListVar;
static List_t * xTCBBlockedList = &xTCBBlockedListVar;
static List_t * xTCBSuspendedList = &xTCBSuspendedListVar;
static List_t * xTCBAperiodicList = &xTCBAperiodicListVar;
static List_t * xTCBInitList = &xTCBInitListVar;

// Current and previous Task handles, used to control priorities
static extTCB_t * xTCBToBlock;
static extTCB_t * xTCBToSuspend;
static extTCB_t * xTCBToReady;
static extTCB_t * firstTaskToExecute;

#if USE_WCET_CHECKS == 1
static extTCB_t * xTCBWCETOverflow;
#endif 

#if USE_DEADLINE_CHECKS == 1
static extTCB_t * xTCBDeadlineOverflow;
#endif

// Library Task Handles
static TaskHandle_t EDFGenHandle = NULL;
static TaskHandle_t EDFSchedulerHandle = NULL;
static TaskHandle_t EDFAperiodicServerHandle = NULL;

#if USE_TBS == 0
static extTCBA_t * aperiodicTCBQueue[MAX_NUM_OF_APERIODIC_TASKS];
// Aperiodic Server 
BaseType_t aperiodicJobPointer = 0;
#else
static TickType_t d_k = 0;
#endif

// System Start Time
static TickType_t xSysStartTime = 0;
// Number of tasks created
static BaseType_t xNoOfPeriodicTasks = 0;
static BaseType_t xNoOfAperiodicTasks = 0;
//
static BaseType_t startEDF = pdFALSE;
// For resuming suspended tasks
static TickType_t EarliestSchedWakeUp = 0;

// periodic scheduler wake up frequency in ticks
// to check for timing errors
static TickType_t schedulerWakeUpFreq = 10;
static TickType_t schedulerSleepCount = 0;

// ******************************************************************** //

// Stats
static float pUtilizationFactor = 0.0f;
static float apUtilizationFactor = 0.0f;
static unsigned int periodicCount = 0;
static unsigned int aperiodicCount = 0;
static unsigned int schedulerCount = 0;
static unsigned int idleCount = 0;
static unsigned int totalCount = 0;
// Utilization Factor based on WCET
static float Up_accepted = 0.0f;
// *********************************************************************** //

// ******************* Private Function Declarations ***************** //
static void generatorTaskEDF(void *pvParameters);
static void addTCBToList(extTCB_t * xTCB);
#if USE_TBS == 0
static void addTCBAToList(extTCBA_t * xTCBA);
#else
static void addTBSTCBToList(extTCB_t * xTCB);
#endif
static void deleteTCBFromList(extTCB_t * xTCB);
static void swapLists(List_t ** aTCBList, List_t ** bTCBList);
static BaseType_t max(TickType_t r, TickType_t d);

// EDF Scheduler Functions
static void EDFSchedulerInit();
static float EDFSchedulabilityCheck(TickType_t period, TickType_t WCET);
static void EDFPeriodicWrapper(void *pvParameters);
static void EDFAperiodicServer(void *pvParameters);
static void EDFSchedulerFunctionOpt(uint32_t events, extTCB_t ** firstTaskToRun);
static void EDFInsertTaskToReadyList(extTCB_t * xTCB);
#if USE_TBS == 0
static void EDFWakeAperiodicServer();
#endif
static BaseType_t EDFGetNextTaskToRun(extTCB_t ** nextTaskToRun, BaseType_t preEmptionReq);

#if USE_WCET_CHECKS == 1
static void EDFWakeSuspendedTasksDueToWCET();
#endif

// ******************************************************************* //
// ************* Functions called from Trace Macros ****************** //
// Not defined as static to enable call from trace macros
// Not to be called by the user
void EDFWakeScheduler(uint32_t event);
void EDFMovedTaskToReadyState(TaskHandle_t xTaskToReadyState);
void EDFTaskSuspended(TaskHandle_t xTaskToSuspend);
void EDFTaskBlocked();
void EDFTaskResumed(TaskHandle_t xTaskToResume);

#ifdef TRACE_CONFIG
// Trace Macro Functions
void tickTrace(BaseType_t xTickCount, BaseType_t xTaskNumber, BaseType_t xTCBNumber);
void switchedOutTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber);
void switchedInTrace(BaseType_t xtaskNumber, BaseType_t xTCBNumber);
#endif

#ifdef ESP_TRACE_CONFIG
void ESPSwitchedOutTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber);
void ESPSwitchedInTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber);
#endif
// ******************************************************************* //
// ****************** Private Functions Definitions ****************** //
#if USE_TBS == 1
static BaseType_t max(TickType_t r, TickType_t d)
{
    if (r > d) return r;
    else if (r < d) return d;
    else return r; // or d
}
#endif

static void EDFPeriodicWrapper(void *pvParameters)
{
    extTCB_t * curTask = (extTCB_t *)pvParameters;
    TickType_t prevArrivalTime = xTaskGetTickCount();

    curTask->relArrivalTime = xSysStartTime;
    curTask->absDeadline = xSysStartTime + curTask->relDeadline + curTask->phase;

    if (curTask->phase != 0)
    {
        vTaskDelayUntil(&curTask->relArrivalTime, curTask->phase);
    }

    for (;;)
    {
        printf("[INFO] In Task %s with number: %d, prevArrivalTime: %ld, relArrival Time: %ld, current time: %ld, absDeadline: %ld, period: %ld, next wakeup: %ld, priority: %d\n", curTask->taskName, curTask->xTaskNumber, prevArrivalTime, curTask->relArrivalTime, xTaskGetTickCount(), curTask->absDeadline, curTask->period, curTask->relArrivalTime + curTask->period, curTask->xPriority);

        // Execute task function
        curTask->instanceFunc(curTask->instanceParams);
        // Specify absolute deadline of next instance
        curTask->absDeadline = curTask->relArrivalTime + curTask->relDeadline + curTask->period;
        prevArrivalTime = curTask->relArrivalTime;

        #if USE_TBS == 1
        // Mark TBS Task as executed
        if (curTask->isPeriodic == pdFALSE)
        {
            curTask->executedTBSTask = pdTRUE;
        }
        #endif
        printf("[INFO] Task %s instance completed (next absDeadline: %ld) at %ld\n", curTask->taskName, curTask->absDeadline, xTaskGetTickCount());
        vTaskDelayUntil(&curTask->relArrivalTime, curTask->period);

        // reset measured exec time after waking up
        curTask->measuredExecTime = 0;
    }
}

#if USE_TBS == 0
static void EDFAperiodicServer(void *pvParameters)
{
    extTCBA_t * xTCBA;
    TickType_t xLastWakeUpTime;
    BaseType_t aperiodicJob = 0;
    for (;;)
    {
        if (xNoOfAperiodicTasks > 0)
        {
            xTCBA = aperiodicTCBQueue[aperiodicJob];
            // Delay Server until arrival of aperiodic task
            vTaskDelayUntil(&xLastWakeUpTime, xTCBA->phase + xSysStartTime);
            
            // Change task Number
            vTaskSetTaskNumber(EDFAperiodicServerHandle, xTCBA->xTaskNumber);
            // After wake up, execute the aperiodic task
            printf("[INFO] In Task \"%s\" with Number: %d....\n", xTCBA->taskName, xTCBA->xTaskNumber);

            // execute aperiodic funtion
            xTCBA->executedThisInstance = pdFALSE;
            xTCBA->instanceFunc(xTCBA->instanceParams);
            xTCBA->executedThisInstance = pdTRUE;

            printf("[INFO] Task \"%s\" Completed Execution.\n", xTCBA->taskName);

            aperiodicJob++;
            xNoOfAperiodicTasks--;
        }
        else
        {
            // No more aperiodic tasks to run
            vTaskSuspend(NULL);
        }
    }
}
#endif

static void generatorTaskEDF(void *pvParameters)
{
    extTCB_t * xTCB;
    ListItem_t * xTCBListItem = listGET_HEAD_ENTRY(xTCBInitList);
    const ListItem_t * xTCBListEndMarker = listGET_END_MARKER(xTCBInitList);
    BaseType_t taskCreated = pdFALSE;
    for (;;)
    {
        // create all periodic tasks
        while (xTCBListItem != xTCBListEndMarker)
        {
            xTCB = listGET_LIST_ITEM_OWNER(xTCBListItem);
            printf("[INFO] Creating %s with priority %d, stack size %ld, period: %ld, releaseTime: %ld\n", xTCB->taskName, xTCB->xPriority, xTCB->stackSize, xTCB->period, xTCB->relArrivalTime);
            taskCreated = xTaskCreate(EDFPeriodicWrapper, xTCB->taskName, xTCB->stackSize, (void *) xTCB, xTCB->xPriority, &(xTCB->cTaskHandle));

            if (taskCreated == pdPASS)
            {
                printf("[INFO] %s created with priority %d and TCB Pointer: %p\n", xTCB->taskName, xTCB->xPriority, xTCB->cTaskHandle);
                vTaskSetThreadLocalStoragePointer(xTCB->cTaskHandle, LOCAL_STORAGE_INDEX, xTCB);
            }
            else
            {
                printf("[INFO] Could not allocate memory\n");
                abort();
            }
            vTaskSetTaskNumber(xTCB->cTaskHandle, xTCB->xTaskNumber);
            xTCBListItem = listGET_NEXT(xTCBListItem);
        }

        #if USE_TBS == 0
        if (xNoOfAperiodicTasks > 0)
        {
            // create server with lowest possible priority with respect to all other tasks
            taskCreated = xTaskCreate(EDFAperiodicServer, "Aperiodic Server", APERIODIC_SERVER_STACK, NULL, APERIODIC_PRIO, &EDFAperiodicServerHandle);
            vTaskSetTaskNumber(EDFAperiodicServerHandle, aperiodicTCBQueue[0]->xTaskNumber);
            //vTaskSuspend(EDFAperiodicServerHandle);
            if (taskCreated == pdTRUE)
            {
                printf("[INFO] Created Server for %d Aperiodic Tasks.\n", xNoOfAperiodicTasks);
            }
            else
            {
                printf("[INFO] Could not allocate memory for server.\n");
            }
        }
        #endif
        xSysStartTime = xTaskGetTickCount(); // get start time for tasks
        printf("[INFO] System Start Time: %ld\n", xSysStartTime);
        startEDF = pdTRUE;
        vTaskDelete(NULL);
    }
}

static void EDFSchedulerTask(void *pvParameters)
{
    uint32_t schedEvents;
    for (;;)
    {
        xTaskNotifyWait(0x00, ALL_SWITCHES, &schedEvents, portMAX_DELAY);
        //printf("Events: Ox%X\n", schedEvents);
        EDFSchedulerFunctionOpt(schedEvents, &firstTaskToExecute);

        #if USE_WCET_CHECKS == 1
        if ((schedEvents & SWITCH_ON_WCET_WAKEUP) == SWITCH_ON_WCET_WAKEUP)
        {
            EDFWakeSuspendedTasksDueToWCET();
        }
        #endif
    }
}

static void addTCBToList(extTCB_t * xTCB)
{
    vListInitialiseItem(&xTCB->xTCBListItem);

    listSET_LIST_ITEM_OWNER(&xTCB->xTCBListItem, xTCB);
    listSET_LIST_ITEM_VALUE(&xTCB->xTCBListItem, xTCB->absDeadline);

    // insert to initial list
    vListInsert(xTCBInitList, &xTCB->xTCBListItem);
}

#if USE_TBS == 0
static void addTCBAToList(extTCBA_t * xTCBA)
{
    vListInitialiseItem(&xTCBA->xTCBAListItem);
    listSET_LIST_ITEM_OWNER(&xTCBA->xTCBAListItem, xTCBA);
    listSET_LIST_ITEM_VALUE(&xTCBA->xTCBAListItem, xTCBA->phase);
    vListInsert(xTCBAperiodicList, &xTCBA->xTCBAListItem);
}

#else

static void addTBSTCBToList(extTCB_t * xTCB)
{
    vListInitialiseItem(&xTCB->xTCBListItem);
    listSET_LIST_ITEM_OWNER(&xTCB->xTCBListItem, xTCB);
    listSET_LIST_ITEM_VALUE(&xTCB->xTCBListItem, xTCB->absDeadline);
    // insert to initial list

    vListInsert(xTCBInitList, &xTCB->xTCBListItem);
}
#endif

static void deleteTCBFromList(extTCB_t * xTCB)
{
    uxListRemove(&xTCB->xTCBListItem);
    vPortFree(xTCB);
}

static void swapLists(List_t ** aTCBList, List_t ** bTCBList)
{
    List_t * tmp = *aTCBList;
    *aTCBList = *bTCBList;
    *bTCBList = tmp;
}

static void EDFSchedulerInit()
{
    extTCB_t * xTCB;
    ListItem_t * xListItem;
    const ListItem_t * xListEndMarker;

    // get first task to run
    xListItem = listGET_HEAD_ENTRY(xTCBInitList);

    // change ready task priority
    xTCB = (extTCB_t *)listGET_LIST_ITEM_OWNER(xListItem);
    xTCB->xPriority = RUNNING_TASK_PRIO;
    firstTaskToExecute = xTCB;
}

#if USE_WCET_CHECKS == 1
static void EDFWakeSuspendedTasksDueToWCET()
{
    // TODO 1: (Medium) Add feature to update sched wake up time for WCET WAKEUP based on next earliest unblock time 
    ListItem_t * xTCBListItem = listGET_HEAD_ENTRY(xTCBSuspendedList);
    const ListItem_t * xListEndMarker = listGET_END_MARKER(xTCBSuspendedList);

    extTCB_t * xTCB;
    TickType_t xCurTick = xTaskGetTickCount();

    while (xTCBListItem != xListEndMarker)
    {
        xTCB = listGET_LIST_ITEM_OWNER(xTCBListItem);
        xTCBListItem = listGET_NEXT(xTCBListItem);

        // Resume Task at
        if ((xTCB->status == TASK_SUSPENDED) & (xTCB->WCETExceeded == pdTRUE))
        {
            // Method used by Robin kase in his thesis
            // find task unblocking time
            if (((long int)xTCB->nextUnblockTime - (long int)xCurTick) <= 0)
            {
                //unblock task
                xTCB->WCETExceeded = pdFALSE;
                xTCB->relArrivalTime = xCurTick;
                xTCB->absDeadline = xTCB->relArrivalTime + xTCB->period;
                printf("[INFO] Suspended Task '%s' Resumed at %ld with absDeadline: %ld, period: %ld. Task resumed......\n", xTCB->taskName, xCurTick, xTCB->absDeadline, xTCB->period);
                vTaskResume(xTCB->cTaskHandle);
            }
        }
    }
}
#endif

static BaseType_t EDFGetNextTaskToRun(extTCB_t ** nextTaskToRun, BaseType_t preEmptionReq)
{
    // TODO 2: (Low) Optimize the function to make it more compact to reduce runtime. Reduced scheduler overhead required
    // Make function such that the current running task needs to be removed from the ready list before calling this function
    BaseType_t preemptionRequired = pdFALSE;
    extTCB_t * xTCBNextInit = NULL;
    extTCB_t * nextTaskToRunPrev = NULL;
    ListItem_t * xTCBToRun;
    ListItem_t * xTCBToRunInit;

    // check if init list is empty, get head entry of init list
    if (!listLIST_IS_EMPTY(xTCBInitList))
    {
        xTCBToRunInit = listGET_HEAD_ENTRY(xTCBInitList);
        xTCBNextInit = listGET_LIST_ITEM_OWNER(xTCBToRunInit);
    }

    // get head entry of ready list
    if (!listLIST_IS_EMPTY(xTCBReadyList))
    {
        ListItem_t * xTCBToRun = listGET_HEAD_ENTRY(xTCBReadyList);
        *nextTaskToRun = listGET_LIST_ITEM_OWNER(xTCBToRun);
        preemptionRequired = pdTRUE;
    }
    else
    {
        //printf("Ready List empty\n");
        preemptionRequired = pdFALSE;
    }

    if ((xTCBNextInit != NULL) & (preemptionRequired == pdTRUE))
    {
        if ((xTCBNextInit->absDeadline + xSysStartTime) < (*nextTaskToRun)->absDeadline)
        {
            // correct arrival time and absDeadline
            xTCBNextInit->relArrivalTime = xSysStartTime;
            xTCBNextInit->absDeadline = xSysStartTime + xTCBNextInit->relDeadline + xTCBNextInit->phase;
            // remove from init list
            uxListRemove(&xTCBNextInit->xTCBListItem);
            // insert into final ready list
            listSET_LIST_ITEM_VALUE(&xTCBNextInit->xTCBListItem, xTCBNextInit->absDeadline);
            vListInsert(xTCBReadyList, &xTCBNextInit->xTCBListItem);
            *nextTaskToRun = xTCBNextInit;
        }
    }
    else if ((xTCBNextInit != NULL) & (preemptionRequired == pdFALSE))
    {
        // empty ready list
        // correct arrival time and absDeadline
        xTCBNextInit->relArrivalTime = xSysStartTime;
        xTCBNextInit->absDeadline = xSysStartTime + xTCBNextInit->relDeadline + xTCBNextInit->phase;
        // remove from init list
        uxListRemove(&xTCBNextInit->xTCBListItem);
        // insert into final ready list
        listSET_LIST_ITEM_VALUE(&xTCBNextInit->xTCBListItem, xTCBNextInit->absDeadline);
        vListInsert(xTCBReadyList, &xTCBNextInit->xTCBListItem);
        *nextTaskToRun = xTCBNextInit;
        preemptionRequired = pdTRUE;
    }

    if (preEmptionReq == pdFALSE)
    {
        return preemptionRequired;
    }
    else
    {
        return preEmptionReq;
    }
}

static BaseType_t EDFGetNextTaskToRunOpt(extTCB_t ** nextTaskToRun)
{
    extTCB_t * xTCBNextInit = NULL;
    extTCB_t * nextTCB = NULL;
    ListItem_t * xTCBToRun;
    ListItem_t * xTCBToRunInit;
    BaseType_t preemptionRequired = pdFALSE;

    // check if init list is empty, get head entry of init list
    if (!listLIST_IS_EMPTY(xTCBInitList))
    {
        xTCBToRunInit = listGET_HEAD_ENTRY(xTCBInitList);
        xTCBNextInit = listGET_LIST_ITEM_OWNER(xTCBToRunInit);
    }

    // get head entry of ready list
    if (!listLIST_IS_EMPTY(xTCBReadyList))
    {
        ListItem_t * xTCBToRun = listGET_HEAD_ENTRY(xTCBReadyList);
        nextTCB = listGET_LIST_ITEM_OWNER(xTCBToRun);
    }
    else
    {
        //printf("Ready List empty\n");
    }

    // ************** //
    // The following code gradually empties the init list until all the initial task start-times have been corrected
    // the function is called whenever a task completes execution, starting with the first task

    if (nextTCB != NULL)
    {
        // ready list not empty
        if (xTCBNextInit != NULL)
        {
            // Init list not empty so check if head entry of init list needs to be executed next
            if ((xTCBNextInit->absDeadline + xSysStartTime) < nextTCB->absDeadline)
            {
                // correct arrival time and absDeadline
                xTCBNextInit->relArrivalTime = xSysStartTime;
                xTCBNextInit->absDeadline = xSysStartTime + xTCBNextInit->relDeadline + xTCBNextInit->phase;
                // remove from init list
                uxListRemove(&xTCBNextInit->xTCBListItem);
                // insert into final ready list
                listSET_LIST_ITEM_VALUE(&xTCBNextInit->xTCBListItem, xTCBNextInit->absDeadline);
                vListInsert(xTCBReadyList, &xTCBNextInit->xTCBListItem);
                nextTCB = xTCBNextInit;
            }
        }
    }
    else
    {
        // ready list empty
        if (xTCBNextInit != NULL)
        {
            // init list not empty
            // correct arrival time and absDeadline
            xTCBNextInit->relArrivalTime = xSysStartTime;
            xTCBNextInit->absDeadline = xSysStartTime + xTCBNextInit->relDeadline + xTCBNextInit->phase;
            // remove from init list
            uxListRemove(&xTCBNextInit->xTCBListItem);
            // insert into final ready list
            listSET_LIST_ITEM_VALUE(&xTCBNextInit->xTCBListItem, xTCBNextInit->absDeadline);
            vListInsert(xTCBReadyList, &xTCBNextInit->xTCBListItem);
            nextTCB = xTCBNextInit;
        }
        else
        {
            // init list also empty
            nextTCB = NULL;
        }
    }

    if (*nextTaskToRun != NULL)
    {
        if (nextTCB != NULL)
        {
            if ((*nextTaskToRun)->absDeadline > nextTCB->absDeadline)
            {
                *nextTaskToRun = nextTCB;
            }
        }
        preemptionRequired = pdTRUE;
    }
    else
    {
        if (nextTCB != NULL)
        {
            *nextTaskToRun = nextTCB;
            preemptionRequired = pdTRUE;
        }
        else
        {
            preemptionRequired = pdFALSE;
        }
    }

    return preemptionRequired;
}

static void EDFSchedulerFunctionOpt(uint32_t schedEvents, extTCB_t ** firstTaskToRun)
{
    if(startEDF == pdFALSE)
    {
        return;
    }
    if (xTaskGetTickCount() == EarliestSchedWakeUp)
    {
        EarliestSchedWakeUp = 0;
    }

    // scheduler variables, keep static to remember state
    static extTCB_t * currentRunningTask = NULL;

    if (*firstTaskToRun != NULL)
    {
        currentRunningTask = *firstTaskToRun;
        currentRunningTask->status = TASK_RUNNING;
        *firstTaskToRun = NULL;
    }

    // variable to decide if a preemption is required
    BaseType_t preemptionRequired = pdFALSE;
    // variable to collect next task to run
    extTCB_t * nextTaskToRun = NULL;

    if ((schedEvents & SWITCH_ON_BLOCK) == SWITCH_ON_BLOCK)
    {
        //printf("Sched Block, Triggered by: %s with status: %d\n", currentRunningTask->taskName, currentRunningTask->status);

        // delegated code from Task Blocking Macro
        // Done to ensure the variable currentRunningTask is not visible to any code outside the scheduler
        // Needed as Task Blocking Macro does not have access to the calling TCB
        if (currentRunningTask->status == TASK_RUNNING)
        {
            currentRunningTask->status = TASK_BLOCKED;
            listSET_LIST_ITEM_VALUE(&currentRunningTask->xTCBListItem, currentRunningTask->absDeadline);
            uxListRemove(&currentRunningTask->xTCBListItem);
            vListInsert(xTCBBlockedList, &currentRunningTask->xTCBListItem);
        }

        // Change task priority and select next task to run
        // following function only called if the currentRunningTask is removed from the ready list, which is done above
        preemptionRequired = EDFGetNextTaskToRunOpt(&nextTaskToRun);

        #if USE_TBS == 1
        if ((currentRunningTask->isPeriodic == pdFALSE) & (currentRunningTask->executedTBSTask == pdTRUE))
        {
            // TBS Task execution completed
            // printf("[INFO] TBS Task '%s' has completed execution, will be deleted\n", currentRunningTask->taskName);
            if(currentRunningTask->cTaskHandle != NULL){
                vTaskDelete(currentRunningTask->cTaskHandle);
                currentRunningTask->cTaskHandle = NULL;
            }
            deleteTCBFromList(currentRunningTask);
            currentRunningTask = NULL;
        }
        else
        {
        #endif 
            currentRunningTask->xPriority = BLOCKED_TASK_PRIO;
            vTaskPrioritySet(currentRunningTask->cTaskHandle, BLOCKED_TASK_PRIO);
            currentRunningTask = NULL;
        
        #if USE_TBS == 1
        }
        #endif
        // request serviced
    }
    if ((schedEvents & SWITCH_ON_READY) == SWITCH_ON_READY)
    {
        // preemption is performed only when a task is moved into the ready state

        if (xTCBToReady != NULL)
        {
            //printf("Sched Ready: nextTaskName: %s, absD: %ld\n", xTCBToReady->taskName, xTCBToReady->absDeadline);
            if (currentRunningTask != NULL)
            {
                if (currentRunningTask->absDeadline > xTCBToReady->absDeadline)
                {
                    // need preemption as task moved into ready state has earlier deadline
                    preemptionRequired = pdTRUE;
                    nextTaskToRun = xTCBToReady;
                }
            }
            else
            {
                preemptionRequired = pdTRUE;
                nextTaskToRun = xTCBToReady;
            }
            // once preemption decision has been made, set passing variable as NULL to indicate the request as serviced
            xTCBToReady = NULL;
        }
    }
    if ((schedEvents & SWITCH_ON_SUSPEND) == SWITCH_ON_SUSPEND)
    {
        if (xTCBToSuspend != NULL)
        {
            if ((xTCBToSuspend->status == TASK_SUSPENDED) 
            #if USE_WCET_CHECKS == 1
            & (xTCBToSuspend->WCETExceeded == pdFALSE)
            #endif
            )
            {
                //printf("Sched Suspend, triggered by: %s\n", xTCBToSuspend->taskName);
                // change priorty 
                xTCBToSuspend->xPriority = BLOCKED_TASK_PRIO;
                vTaskPrioritySet(xTCBToSuspend->cTaskHandle, BLOCKED_TASK_PRIO);

                // Function can be called as the suspended task is removed from the ready list by the Suspend Macro
                preemptionRequired = EDFGetNextTaskToRunOpt(&nextTaskToRun);

                // request serviced
                xTCBToSuspend = NULL;
            }
        }
    }
    #if USE_WCET_CHECKS == 1
    if ((schedEvents & SWITCH_ON_WCET_OVERFLOW) == SWITCH_ON_WCET_OVERFLOW)
    {
        if (xTCBWCETOverflow != NULL)
        {
            if ((xTCBWCETOverflow->status == TASK_SUSPENDED) & (xTCBWCETOverflow->WCETExceeded == pdTRUE))
            {
                // insert task to suspended list
                uxListRemove(&xTCBWCETOverflow->xTCBListItem);
                vListInsert(xTCBSuspendedList, &xTCBWCETOverflow->xTCBListItem);

                printf("[INFO] Task '%s' crossed WCET, WCET: %ld, measured Execution Time: %ld. NextUnblockTime: %ld with period doubled.Task Suspended....\n", xTCBWCETOverflow->taskName, xTCBWCETOverflow->WCET, xTCBWCETOverflow->measuredExecTime, xTCBWCETOverflow->nextUnblockTime);
                xTCBWCETOverflow->measuredExecTime = 0;
                //xTCB->WCETExceeded = pdFALSE;
                xTCBWCETOverflow->xPriority = BLOCKED_TASK_PRIO;
                vTaskPrioritySet(xTCBWCETOverflow->cTaskHandle, BLOCKED_TASK_PRIO);
                vTaskSuspend(xTCBWCETOverflow->cTaskHandle);

                // Task removed from ready list above
                preemptionRequired = EDFGetNextTaskToRunOpt(&nextTaskToRun);
                if (xTCBWCETOverflow->cTaskHandle == currentRunningTask->cTaskHandle)
                {
                    currentRunningTask = NULL;
                }
                // request serviced
                xTCBWCETOverflow = NULL;
            }
        }
    }
    #endif

    #if USE_DEADLINE_CHECKS == 1
    if ((schedEvents & SWITCH_ON_DEADLINE_OVERFLOW) == SWITCH_ON_DEADLINE_OVERFLOW)
    {
        if (xTCBDeadlineOverflow != NULL)
        {
            if (xTCBDeadlineOverflow->deadlineExceeded == pdTRUE)
            {
                // deadline missed
                // For now, print message and delete the task
                printf("[INFO] Task '%s' missed its deadline of %ld with current time: %ld and TCB Address: %p and thus cannot be scheduled using EDF. Task Deleted....\n", xTCBDeadlineOverflow->taskName, xTCBDeadlineOverflow->absDeadline, xTaskGetTickCount(), xTCBDeadlineOverflow->cTaskHandle);

                if(xTCBDeadlineOverflow->cTaskHandle != NULL){
                    if (currentRunningTask->cTaskHandle == xTCBDeadlineOverflow->cTaskHandle)
                    {
                        // Set to NULL as current Running Task will be removed from the system
                        currentRunningTask = NULL;
                    }
                    vTaskDelete(xTCBDeadlineOverflow->cTaskHandle);
                    xTCBDeadlineOverflow->cTaskHandle = NULL;
                }
                deleteTCBFromList(xTCBDeadlineOverflow);

                // Task has been removed from the system
                preemptionRequired = EDFGetNextTaskToRunOpt(&nextTaskToRun);

                // request serviced
                xTCBDeadlineOverflow = NULL;
            }
        }
    }
    #endif

    // action to perform if preemption is required
    if (preemptionRequired == pdTRUE)
    {
        if (currentRunningTask != NULL)
        {
            // Change task status to TASK_READY if currentRunningTask has not completed execution
            if (currentRunningTask->status == TASK_RUNNING)
            {
                currentRunningTask->status = TASK_READY;
            }
            //printf("Task to be blocked: %s with status: %d\n", currentRunningTask->taskName, currentRunningTask->status);
            currentRunningTask->xPriority = BLOCKED_TASK_PRIO;
            vTaskPrioritySet(currentRunningTask->cTaskHandle, BLOCKED_TASK_PRIO);
        }
        if (nextTaskToRun != NULL)
        {
            //printf("selected task: %s with absDeadline: %ld and status: %d. curTime: %ld and wakeUp: %ld\n", nextTaskToRun->taskName, nextTaskToRun->absDeadline, nextTaskToRun->status, xTaskGetTickCount(), nextTaskToRun->absDeadline - nextTaskToRun->period);
            currentRunningTask = nextTaskToRun;
            currentRunningTask->xPriority = RUNNING_TASK_PRIO;
            currentRunningTask->status = TASK_RUNNING;
            vTaskPrioritySet(currentRunningTask->cTaskHandle, RUNNING_TASK_PRIO);
        }
        preemptionRequired = pdFALSE;
        nextTaskToRun = NULL;
    }
}

static void EDFInsertTaskToReadyList(extTCB_t * xTCB)
{
    if (startEDF != pdTRUE)
    {
        return;
    }

    if ((xTCB->status != TASK_READY) & (xTCB->status != TASK_RUNNING))
    {
        // FreeRTOS moves tasks to ready state when there is a priority change
        // To account for this
        #if USE_WCET_CHECKS == 1
        if ((xTCB->status == TASK_SUSPENDED) & (xTCB->WCETExceeded == pdTRUE))
        {
            return;
        }
        #endif

        xTCB->status = TASK_READY;
        uxListRemove(&xTCB->xTCBListItem);
        vListInsert(xTCBReadyList, &xTCB->xTCBListItem);

        // delegate preemption decision to the scheduler, only let the scheduler know that a task has been moved into the ready state
        if (xTCBToReady == NULL)
        {
            xTCBToReady = xTCB;
            EDFWakeScheduler(SWITCH_ON_READY);
        }
        else
        {
            if (xTCBToReady->absDeadline > xTCB->absDeadline)
            {
                xTCBToReady = xTCB;
                EDFWakeScheduler(SWITCH_ON_READY);
            }
        }
    }
}

static float EDFSchedulabilityCheck(TickType_t period, TickType_t WCET)
{
    // check task schedulability based on FreeRTOS Ticks
    // Use WCET passed as task param to calculate this
    float Up;

    Up = Up_accepted + (float) WCET / (float) period;
    if (Up <= UP_LIMIT)
    {
        Up_accepted = Up;
        printf("Current Periodic utilization: %0.2f\n", Up_accepted);
        
    }
    return Up;
}

#if USE_TBS == 0
static void EDFWakeAperiodicServer()
{
    if (startEDF == pdTRUE)
    {
        xTaskResumeFromISR(EDFAperiodicServerHandle);
        return;
    }
}
#endif

// ****************** Public Function Definitions ******************** //
// ******************************************************  EDF Scheduler ********************************************************//


void EDFCreatePeriodicTask(const char* taskName, 
                            int stackSize, 
                            void (*instanceFunc)(void*), 
                            int timePeriod,
                            int relDeadline, 
                            int phase, 
                            TaskHandle_t *handle, 
                            void *instanceParams, 
                            TickType_t WCETinTicks)
{
    configASSERT(xNoOfPeriodicTasks <= MAX_NUM_OF_PERIODIC_TASKS);
    configASSERT(relDeadline <= timePeriod);

    float periodicUtilization = EDFSchedulabilityCheck(timePeriod / portTICK_PERIOD_MS, WCETinTicks);
    if (periodicUtilization > 1.0f)
    {
        printf("Task \"%s\" Failed schedulability check. Predicted CPU Utilization: %.2f!!\n", taskName, periodicUtilization);
        return;
    }

    extTCB_t * taskNode = (extTCB_t *)malloc(sizeof(extTCB_t));

    if (taskNode == NULL)
    {
        printf("Could not allocate Memory......\n");
        return;
    }

    taskNode->taskName = taskName;
    taskNode->instanceFunc = instanceFunc;
    taskNode->instanceParams = instanceParams;
    taskNode->measuredExecTime = 0;
    taskNode->WCET = WCETinTicks;
    taskNode->period = timePeriod / portTICK_PERIOD_MS;
    taskNode->phase = phase / portTICK_PERIOD_MS;
    taskNode->relDeadline = relDeadline / portTICK_PERIOD_MS;
    taskNode->absDeadline = taskNode->phase + taskNode->relDeadline;// temporary abs deadline to sort tasks
    taskNode->xTaskNumber = xNoOfPeriodicTasks + TASK_NUM_START;
    #if USE_TBS == 1
    taskNode->executedTBSTask = pdFALSE;
    taskNode->isPeriodic = pdTRUE;
    #endif
    taskNode->cTaskHandle = NULL;
    taskNode->stackSize = stackSize;
    #if USE_WCET_CHECKS == 1
    taskNode->WCETExceeded = pdFALSE;
    #endif

    #if USE_DEADLINE_CHECKS == 1
    taskNode->deadlineExceeded = pdFALSE;
    #endif

    taskNode->xPriority = BLOCKED_TASK_PRIO;
    taskNode->status = TASK_BLOCKED;
    xNoOfPeriodicTasks++;

    addTCBToList(taskNode);
}

void EDFCreateAperiodicTask(const char* taskName, 
                            void (*instanceFunc)(void*), 
                            void *instanceParams,
                            int stackSize, 
                            TickType_t WCETinTicks,
                            TickType_t arrivalTime)
{
    configASSERT(xNoOfAperiodicTasks <= MAX_NUM_OF_APERIODIC_TASKS);

    #if USE_TBS == 1
    extTCB_t * taskNode = (extTCB_t *)malloc(sizeof(extTCB_t));
    #else
    extTCBA_t * taskNode = (extTCBA_t *)malloc(sizeof(extTCBA_t));
    #endif

    if (taskNode == NULL)
    {
        printf("Could not allocate Memory......\n");
        return;
    }

    taskNode->status = TASK_BLOCKED;
    taskNode->stackSize = stackSize;
    taskNode->instanceFunc = instanceFunc;
    taskNode->instanceParams = instanceParams;
    taskNode->measuredExecTime = 0;
    taskNode->taskName = taskName;
    taskNode->WCET = WCETinTicks;
    taskNode->xTaskNumber = MAX_NUM_OF_PERIODIC_TASKS + TASK_NUM_START + xNoOfAperiodicTasks;
    taskNode->phase = arrivalTime / portTICK_PERIOD_MS;

    #if USE_TBS == 1
    // TBS Additions
    d_k = max(taskNode->phase, d_k) + (TickType_t)(((float) pdTICKS_TO_MS(WCETinTicks)) / (float) (1 - Up_accepted));

    printf("%s with release time: %ld, deadline: %ld, Up_acc: %0.2f\n", taskName, arrivalTime, d_k, Up_accepted);

    taskNode->period = d_k;
    taskNode->relDeadline = d_k;
    taskNode->absDeadline = taskNode->phase + taskNode->relDeadline;// temporary abs deadline to sort tasks
    taskNode->cTaskHandle = NULL;
    
    #if USE_WCET_CHECKS == 1 & USE_WCET_CHECKS_TBS == 1
    taskNode->WCETExceeded = pdFALSE;
    #endif

    #if USE_DEADLINE_CHECKS == 1 & USE_DEADLINE_CHECKS_TBS == 1
    taskNode->deadlineExceeded = pdFALSE;
    #endif
    taskNode->xPriority = BLOCKED_TASK_PRIO;
    taskNode->isPeriodic = pdFALSE;
    taskNode->executedTBSTask = pdFALSE;

    addTBSTCBToList(taskNode);
    #else
    aperiodicTCBQueue[xNoOfAperiodicTasks] = taskNode;
    #endif
    xNoOfAperiodicTasks++;
}

// First function to be called from the module
void EDFInit()
{
    #ifdef TRACE_CONFIG
    memset(task_nums, 0, sizeof(task_nums));
    #endif

    #ifdef ESP_TRACE_CONFIG
    memset(eventArray, 0, sizeof(eventArray));
    const esp_timer_create_args_t timer_args = {
            .callback = &espTimerCallback,
            .name = "traceTimer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &traceTimer));
    ESP_ERROR_CHECK(esp_timer_start_once(traceTimer, TRACE_TIMER_TIMEOUT));
    #endif

    vTaskPrioritySet(NULL, MAX_SYS_PRIO + 2);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    vListInitialise(xTCBBlockedList);
    vListInitialise(xTCBReadyList);
    vListInitialise(xTCBSuspendedList);
    vListInitialise(xTCBAperiodicList);
    vListInitialise(xTCBInitList);
}

void EDFStartScheduling()
{
    configASSERT((xNoOfAperiodicTasks > 0) || (xNoOfPeriodicTasks > 0));

    #if USE_TBS == 1
    if ((xNoOfPeriodicTasks > 0) | (xNoOfAperiodicTasks > 0))
    #else
    if (xNoOfPeriodicTasks > 0)
    #endif
    {
        EDFSchedulerInit();
    }

    // create Generator Task
    xTaskCreate(generatorTaskEDF, "EDF Gen Task", 2000, NULL, SCHED_PRIO, &EDFGenHandle);
    xTaskCreate(EDFSchedulerTask, "EDF Scheduler", 2000, NULL, SCHED_PRIO, &EDFSchedulerHandle);
    vTaskSetTaskNumber(EDFSchedulerHandle, SCHED_TASK_NUM);
}

void EDFDeleteAllTasks()
{
    extTCB_t * xTCB;
    TaskHandle_t xHandle;
    ListItem_t * xTCBListItem = listGET_HEAD_ENTRY(xTCBReadyList);
    const ListItem_t * xTCBListEndMarker = listGET_END_MARKER(xTCBReadyList);

    printf("[INFO] Deleting all Tasks............\n");
    while (xTCBListItem != xTCBListEndMarker)
    {
        xTCB = listGET_LIST_ITEM_OWNER(xTCBListItem);
        xTCBListItem = listGET_NEXT(xTCBListItem);
        xHandle = xTCB->cTaskHandle;
        deleteTCBFromList(xTCB);
        if (xHandle != NULL)
        {
            vTaskDelete(xHandle);
        }
    }

    xTCBListItem = listGET_HEAD_ENTRY(xTCBBlockedList);
    xTCBListEndMarker = listGET_END_MARKER(xTCBBlockedList);
    while (xTCBListItem != xTCBListEndMarker)
    {
        xTCB = listGET_LIST_ITEM_OWNER(xTCBListItem);
        xTCBListItem = listGET_NEXT(xTCBListItem);
        xHandle = xTCB->cTaskHandle;
        deleteTCBFromList(xTCB);
        if (xHandle != NULL)
        {
            vTaskDelete(xHandle);
        }
    }

    xTCBListItem = listGET_HEAD_ENTRY(xTCBSuspendedList);
    xTCBListEndMarker = listGET_END_MARKER(xTCBSuspendedList);
    while (xTCBListItem != xTCBListEndMarker)
    {
        xTCB = listGET_LIST_ITEM_OWNER(xTCBListItem);
        xTCBListItem = listGET_NEXT(xTCBListItem);
        xHandle = xTCB->cTaskHandle;
        deleteTCBFromList(xTCB);
        if (xHandle != NULL)
        {
            vTaskDelete(xHandle);
        }
    }

    vTaskDelete(EDFSchedulerHandle);
    EDFSchedulerHandle = NULL;
    startEDF = pdFALSE;

    #ifdef TRACE_CONFIG
    printf("[INFO] Overhead and CPU Utilization Information.........\n");
    printf("[INFO] Periodic Utilization: %f, %d\n", ((float) periodicCount) / ((float) totalCount), periodicCount);
    printf("[INFO] Aperiodic Utilization: %f, %d\n", ((float) aperiodicCount) / ((float) totalCount), aperiodicCount);
    printf("[INFO] Idle Utilization: %f, %d\n", ((float) idleCount) / ((float) totalCount), idleCount);
    printf("[INFO] Scheduler INFO: %f, %d\n", ((float) (totalCount - idleCount - aperiodicCount - periodicCount)) / ((float) totalCount), schedulerCount);
    printf("[INFO] Stats END\n");
    #endif

    #ifdef ESP_TRACE_CONFIG
    ESP_ERROR_CHECK(esp_timer_stop(traceTimer));
    ESP_ERROR_CHECK(esp_timer_delete(traceTimer));
    #endif
}
// ******************************************************************** //
// **************** EDF Functions called from Trace Macros ************ //
void EDFWakeScheduler(uint32_t event)
{
    if ((startEDF == pdTRUE) & (EDFSchedulerHandle != NULL))
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(EDFSchedulerHandle, event, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }
    return;
}

void EDFMovedTaskToReadyState(TaskHandle_t xTaskToReadyState)
{
    // get pointer to task that was moved to ready state
    extTCB_t * xTCB = (extTCB_t *)pvTaskGetThreadLocalStoragePointer(xTaskToReadyState, LOCAL_STORAGE_INDEX);
    if ((startEDF == pdTRUE) & (EDFSchedulerHandle != NULL))
    {
        if (xTCB != NULL)
        {
            EDFInsertTaskToReadyList(xTCB);
        }
        return;
    }
    return;
}

void EDFTaskSuspended(TaskHandle_t xTaskToSuspend)
{
    if ((startEDF == pdTRUE) & (EDFSchedulerHandle != NULL))
    {

        #if USE_TBS == 0
        // if vTaskSuspend called from the aperiodic server, do nothing
        if (xTaskGetCurrentTaskHandle() == EDFAperiodicServerHandle)
        {
            return;
        }
        #endif
        // get pointer to task that was suspended
        extTCB_t * xTCB = (extTCB_t *)pvTaskGetThreadLocalStoragePointer(xTaskToSuspend, LOCAL_STORAGE_INDEX);

        if (xTCB != NULL)
        {
            #if USE_WCET_CHECKS == 1
            if (xTCB->WCETExceeded == pdTRUE)
            {
                // do not add to list as this was already done by the scheduler
                return;
            }
            #endif

            if (xTCB->status != TASK_SUSPENDED)
            {
                xTCB->status = TASK_SUSPENDED;
                uxListRemove(&xTCB->xTCBListItem);
                vListInsert(xTCBSuspendedList, &xTCB->xTCBListItem);
                xTCBToSuspend = xTCB;
                EDFWakeScheduler(SWITCH_ON_SUSPEND);
            }
        }
        return;
    }
    return;
}

void EDFTaskBlocked()
{
    if ((startEDF == pdTRUE) & (EDFSchedulerHandle != NULL))
    {
        // as task TCB not availabe to macro, call scheduler and ask to block current running task
        EDFWakeScheduler(SWITCH_ON_BLOCK);
        return;
    }
    return;
}

// assuming task goes to ready queue after resumption
void EDFTaskResumed(TaskHandle_t xTaskToResume)
{
    if ((startEDF == pdTRUE) & (EDFSchedulerHandle != NULL))
    {
        // get pointer to task that was resumed
        extTCB_t * xTCBToResume = (extTCB_t *)pvTaskGetThreadLocalStoragePointer(xTaskToResume, LOCAL_STORAGE_INDEX);

        if (xTCBToResume != NULL)
        {
            EDFInsertTaskToReadyList(xTCBToResume);
        }
        return;
    }
    return;
}

// ******************************************************************* //
// ********************* Tick hook Function ************************** //
void vApplicationTickHook(void)
{

    TaskHandle_t curTaskHandle = xTaskGetCurrentTaskHandle();

    if (startEDF == pdFALSE || curTaskHandle == EDFGenHandle)
    {
        return;
    }

    #if USE_WCET_CHECKS == 1
    if (EarliestSchedWakeUp == xTaskGetTickCountFromISR())
    {
        EDFWakeScheduler(SWITCH_ON_WCET_WAKEUP);
    }
    #endif

    /*#if USE_TBS == 0
    if (xNoOfAperiodicTasks > 0)
    {
        if (xTaskGetTickCountFromISR() >= (aperiodicTCBQueue[aperiodicJobPointer]->phase + xSysStartTime))
        {
            vTaskSetTaskNumber(EDFAperiodicServerHandle, aperiodicTCBQueue[aperiodicJobPointer]->xTaskNumber);
            EDFWakeAperiodicServer();
        }
    }
    #endif*/

    TaskHandle_t idleTaskHandle = xTaskGetIdleTaskHandle();
    extTCB_t * curTaskTCB = (extTCB_t *)pvTaskGetThreadLocalStoragePointer(curTaskHandle, LOCAL_STORAGE_INDEX);

    if ((curTaskHandle != NULL) & (curTaskHandle != idleTaskHandle) & (curTaskHandle != EDFSchedulerHandle) & (curTaskHandle != EDFAperiodicServerHandle))
    {
        curTaskTCB->measuredExecTime++;

        #if USE_WCET_CHECKS == 1
        if ((curTaskTCB->measuredExecTime > curTaskTCB->WCET) & (curTaskTCB->status == TASK_RUNNING) & (curTaskTCB->WCETExceeded == pdFALSE)
        #if USE_TBS == 1 & USE_WCET_CHECKS_TBS == 1
        & (curTaskTCB->isPeriodic == pdTRUE)
        #endif
        )
        {
            curTaskTCB->WCETExceeded = pdTRUE;
            // Calculate next unblock time here and wake up scheduler
            curTaskTCB->status = TASK_SUSPENDED;

            curTaskTCB->nextUnblockTime = curTaskTCB->relArrivalTime + curTaskTCB->period;

            xTCBWCETOverflow = curTaskTCB;

            // wake up scheduler at unblock time
            if (curTaskTCB->nextUnblockTime < EarliestSchedWakeUp || EarliestSchedWakeUp == 0)
            {
                EarliestSchedWakeUp = curTaskTCB->nextUnblockTime;
            }
            EDFWakeScheduler(SWITCH_ON_WCET_OVERFLOW);
        }
        #endif

        #if USE_DEADLINE_CHECKS == 1
        if ((curTaskTCB->status == TASK_RUNNING)
        #if USE_TBS == 1 & USE_DEADLINE_CHECKS_TBS == 1
        & (curTaskTCB->isPeriodic == pdTRUE)
        #endif
        )
        {
            if (((long int)curTaskTCB->absDeadline - (long int)xTaskGetTickCountFromISR()) < 0)
            {
                curTaskTCB->deadlineExceeded = pdTRUE;
                xTCBDeadlineOverflow = curTaskTCB;
                EDFWakeScheduler(SWITCH_ON_DEADLINE_OVERFLOW);
            }
        }
        #endif
    }
}
// ******************************************************************* //

#ifdef TRACE_CONFIG
// ********************* Idle Hook Definition ************************ //
void vApplicationIdleHook(void)
{
    if (trcIndex < TRACE_ARRAY_SIZE)
    {
        task_nums[trcIndex][SCHED_TASK_NUM - 4] = IDLE_TASK_NUM;
    }
}

// ******************** Tracing Functions  *****************************//

void tickTrace(BaseType_t xTickCount, BaseType_t xTaskNumber, BaseType_t xTCBNumber)
{
    trcIndex = xTickCount;
    if (trcIndex < TRACE_ARRAY_SIZE)
    {
        // Periodic, Aperiodic Tasks and the Scheduler
        if (xTaskNumber != 0)
        {
            if (xTaskNumber < SCHED_TASK_NUM)
            {
                task_nums[trcIndex][xTaskNumber - TASK_NUM_START] = xTaskNumber;
            }
            else
            {
                task_nums[trcIndex][SCHED_TASK_NUM - 3] = xTaskNumber;
            }
        }
        // Idle Task
        else
        {
            if (xTCBNumber == IDLE_TASK_NUM)
            {
                task_nums[trcIndex][SCHED_TASK_NUM - 4] = IDLE_TASK_NUM;
            }
        }
    }
}

void switchedOutTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber)
{
    if (trcIndex < TRACE_ARRAY_SIZE)
    {
        // Periodic, Aperiodic Tasks and the Scheduler
        if (xTaskNumber != 0)
        {
            if (xTaskNumber < SCHED_TASK_NUM)
            {
                task_nums[trcIndex][xTaskNumber - TASK_NUM_START] = xTaskNumber;
            }
            else
            {
                task_nums[trcIndex][SCHED_TASK_NUM - 3] = xTaskNumber;
            }
        }
        // Idle Task
        else
        {
            if (xTCBNumber == IDLE_TASK_NUM)
            {
                task_nums[trcIndex][SCHED_TASK_NUM - 4] = IDLE_TASK_NUM;
            }
        }
    }
}

void switchedInTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber)
{
    if (trcIndex < TRACE_ARRAY_SIZE)
    {
        // Periodic, Aperiodic Tasks and the Scheduler
        if (xTaskNumber != 0)
        {
            if (xTaskNumber < SCHED_TASK_NUM)
            {
                task_nums[trcIndex][xTaskNumber - TASK_NUM_START] = xTaskNumber;
            }
            else
            {
                task_nums[trcIndex][SCHED_TASK_NUM - 3] = xTaskNumber;
            }
        }
        // Idle Task
        else
        {
            if (xTCBNumber == IDLE_TASK_NUM)
            {
                task_nums[trcIndex][SCHED_TASK_NUM - 4] = IDLE_TASK_NUM;
            }
        }
    }
}
#endif
// ********************************************************************* //
// ******************* ESP TRACE Funtions ****************************** //
#ifdef ESP_TRACE_CONFIG

void ESPSwitchedOutTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber)
{
    if (eventIndex < MAX_NUM_OF_EVENTS)
    {
        // Periodic, Aperiodic Tasks and the Scheduler
        if (xTaskNumber != 0)
        {
            eventArray[eventIndex][2] = esp_timer_get_time();
            eventIndex++;
        }
        // Idle Task
        else if ((xTCBNumber == IDLE_TASK_NUM) && (xTaskNumber == 0))
        {
            eventArray[eventIndex][2] = esp_timer_get_time();
            eventIndex++;
        }
    }
}

void ESPSwitchedInTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber)
{
    if (eventIndex < MAX_NUM_OF_EVENTS)
    {
        // Periodic, Aperiodic Tasks and the Scheduler
        if (xTaskNumber != 0)
        {
            eventArray[eventIndex][0] = xTaskNumber;
            eventArray[eventIndex][1] = esp_timer_get_time();
        }
        // Idle Task
        else if ((xTCBNumber == IDLE_TASK_NUM) && (xTaskNumber == 0))
        {
            eventArray[eventIndex][0] = IDLE_TASK_NUM;
            eventArray[eventIndex][1] = esp_timer_get_time();
        }
    }
}

#endif
// ******************************************************************** //