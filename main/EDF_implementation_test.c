/* FreeRTOS Real Time Stats Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "ExtEDFlib.h"
#include "esp_err.h"


#define NUM_OF_INSTR        100000  //Actual CPU cycles used will depend on compiler optimization
#define NUM_OF_PERIODIC_GEN_TASKS 10

static char task_names[NUM_OF_PERIODIC_GEN_TASKS][configMAX_TASK_NAME_LEN];
static int time_period[NUM_OF_PERIODIC_GEN_TASKS] = {100, 200, 400, 800, 1200, 1600, 2000, 2400, 2800, 3200};
static int time_periods[NUM_OF_PERIODIC_GEN_TASKS] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
static int phase[NUM_OF_PERIODIC_GEN_TASKS] = {0, 50};
static TaskHandle_t taskHandles[NUM_OF_PERIODIC_GEN_TASKS];

// User defined parameter struct
typedef struct taskParams
{
    int mulFactor;
} taskParams;

static void periodicTask(void *params)
{
    //Consume CPU cycles
    int mulFactor = ((taskParams *)params)->mulFactor;
    for (int i = 0; i < (NUM_OF_INSTR * mulFactor); i++) 
    {
        __asm__ __volatile__("NOP");
    }
}

static void aperiodicTask(void *params)
{
    //Consume CPU cycles
    int mulFactor = ((taskParams *)params)->mulFactor;
    for (int i = 0; i < (NUM_OF_INSTR * mulFactor); i++) 
    {
        __asm__ __volatile__("NOP");
    }
}

#ifdef TRACE_CONFIG
static void printFunc1(void *pvParameters)
{
    printf("********\n");
    printf("[INFO] Schedule Data Gathered. Printing data......\n");

    for (int i = 0; i < TRACE_ARRAY_SIZE; i++)
    {
        printf("Tick %d:   ", i+1);
        for (int j = 0; j < TRACE_ARRAY_WIDTH; j++)
        {
            printf("%d, ", task_nums[i][j]);
        }
        printf("\n");
    }
    printf("\n[INFO] Schedule Printed!!!\n");
    printf("\n********");

    vTaskDelete(NULL);
}
#endif

#ifdef ESP_TRACE_CONFIG
static void printFunc2(void *pvParameters)
{
    printf("********\n");
    printf("[INFO] Schedule Data Gathered. Printing data......\n");

    for (int i = 0; i < MAX_NUM_OF_EVENTS; i++)
    {
        if ((i > 1) && !eventArray[i][0] && !eventArray[i][1] && !eventArray[i][2]) break;
        printf("Event No: %d, Task No: %lld, Entry at: %lld us, Exit at: %lld us after startup\n", i, eventArray[i][0], eventArray[i][1], eventArray[i][2]);
    }
    printf("\n[INFO] Schedule Printed!!!\n");
    printf("\n********");

    vTaskDelete(NULL);
}
#endif


void app_main(void)
{
    //Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(100));

    EDFInit();

    //TaskHandle_t curTask = xTaskGetCurrentTaskHandle();
    //vTaskSetTaskNumber(curTask, 7);
    
    //Create aperiodic/periodic tasks here
    taskParams *instanceParams1 = (taskParams *)malloc(sizeof(taskParams));
    taskParams *instanceParams3 = (taskParams *)malloc(sizeof(taskParams));
    taskParams *instanceParams = (taskParams *)malloc(sizeof(taskParams));
    for (int i = 0; i < NUM_OF_PERIODIC_GEN_TASKS; i++) {

        if (i == 2)
        {
            instanceParams3->mulFactor = 5;
            snprintf(task_names[i], configMAX_TASK_NAME_LEN, "Periodic %d", i+1);
            EDFCreatePeriodicTask(task_names[i], 2000, periodicTask, time_period[i], time_period[i], 0, (&taskHandles[i]), (void *)instanceParams3, 6);
        }
        else if (i > 0)
        {
            instanceParams->mulFactor = 5;
            snprintf(task_names[i], configMAX_TASK_NAME_LEN, "Periodic %d", i+1);
            EDFCreatePeriodicTask(task_names[i], 2000, periodicTask, time_period[i], time_period[i], 0, (&taskHandles[i]), (void *)instanceParams, 6);
        }
        else
        {
            instanceParams1->mulFactor = 3;
            snprintf(task_names[i], configMAX_TASK_NAME_LEN, "Periodic %d", i+1);
            EDFCreatePeriodicTask(task_names[i], 2000, periodicTask, time_period[i], time_period[i], 0, (&taskHandles[i]), (void *)instanceParams1, 10);
        }
    }

    taskParams *aperiodicInstanceParams1 = (taskParams *)malloc(sizeof(taskParams));
    aperiodicInstanceParams1->mulFactor = 20;
    EDFCreateAperiodicTask("Aperiodic 1", aperiodicTask, (void *)aperiodicInstanceParams1, 2000, 5, 200);

    taskParams *aperiodicInstanceParams2 = (taskParams *)malloc(sizeof(taskParams));
    aperiodicInstanceParams2->mulFactor = 20;
    EDFCreateAperiodicTask("Aperiodic 2", aperiodicTask, (void *) aperiodicInstanceParams2, 2000, 4, 1400);

    /*aperiodicInstanceParams->mulFactor = 15;
    EDFCreateAperiodicTask("Aperiodic 3", aperiodicTask, (void *) aperiodicInstanceParams, 4);

    aperiodicInstanceParams->mulFactor = 15;
    EDFCreateAperiodicTask("Aperiodic 4", aperiodicTask, (void *) aperiodicInstanceParams, 4);*/

    EDFStartScheduling();
    vTaskDelay(4000 / portTICK_PERIOD_MS);

    // In this time, the created tasks will run and data will be collected on the schedule
    EDFDeleteAllTasks();
    xTaskCreate(printFunc2, "Print Task", 2048, NULL, 1, NULL);
}
