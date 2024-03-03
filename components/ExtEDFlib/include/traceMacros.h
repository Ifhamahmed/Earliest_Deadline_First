#ifndef _TRACE_MACROS_H_
#define _TRACE_MACROS_H_

#include "commonDefines.h"

#ifdef TRACE_CONFIG

#define traceTASK_INCREMENT_TICK(xTickCount)\
extern void tickTrace(BaseType_t xTickCount, BaseType_t xTaskNumber, BaseType_t xTCBNumber);\
tickTrace(xTickCount, pxCurrentTCB[0]->uxTaskNumber, pxCurrentTCB[0]->uxTCBNumber);

#define traceTASK_SWITCHED_OUT()\
extern void switchedOutTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber);\
switchedOutTrace(pxCurrentTCB[0]->uxTaskNumber, pxCurrentTCB[0]->uxTCBNumber);

#define traceTASK_SWITCHED_IN()\
extern void switchedInTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber);\
switchedInTrace(pxCurrentTCB[0]->uxTaskNumber, pxCurrentTCB[0]->uxTCBNumber);

#endif

#ifdef ESP_TRACE_CONFIG

#define traceTASK_SWITCHED_OUT()\
extern void ESPSwitchedOutTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber);\
ESPSwitchedOutTrace(pxCurrentTCB[0]->uxTaskNumber, pxCurrentTCB[0]->uxTCBNumber);

#define traceTASK_SWITCHED_IN()\
extern void ESPSwitchedInTrace(BaseType_t xTaskNumber, BaseType_t xTCBNumber);\
ESPSwitchedInTrace(pxCurrentTCB[0]->uxTaskNumber, pxCurrentTCB[0]->uxTCBNumber);

#endif

#define traceMOVED_TASK_TO_READY_STATE(xTask)\
extern void EDFMovedTaskToReadyState(TaskHandle_t xTaskToReadyState);\
EDFMovedTaskToReadyState(xTask);

#define traceTASK_DELAY_UNTIL( xAbsTick )\
extern void EDFTaskBlocked();\
EDFTaskBlocked();

#define traceTASK_SUSPEND(xTask)\
extern void EDFTaskSuspended(TaskHandle_t xTaskToSuspend);\
EDFTaskSuspended(xTask);

#define traceTASK_RESUME(xTask)\
extern void EDFTaskResumed(TaskHandle_t xTaskToResume);\
EDFTaskResumed(xTask);



#endif //_TRACE_MACROS_H_