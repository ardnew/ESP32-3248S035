/* FreeRTOS Real Time Stats Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"


#define SOC_NUM_CORES 2
#define STATS_TICKS pdMS_TO_TICKS(10000)
#define ARRAY_SIZE_OFFSET 5 // Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

#if NUM_TASKS_TRACED

SemaphoreHandle_t sync_lvgl_task;
SemaphoreHandle_t sync_stats_task;

// void vTaskGetRunTimeStats(char *pcWriteBuffer)
// {
//     TaskStatus_t *pxTaskStatusArray;
//     volatile UBaseType_t uxArraySize, x;
//     unsigned long ulTotalRunTime, ulStatsAsPercentage;

//     /* Make sure the write buffer does not contain a string. */
//     *pcWriteBuffer = 0x00;

//     /* Take a snapshot of the number of tasks in case it changes while this
//     function is executing. */
//     uxArraySize = uxTaskGetNumberOfTasks();

//     /* Allocate a TaskStatus_t structure for each task.  An array could be
//     allocated statically at compile time. */
//     pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

//     if (pxTaskStatusArray != NULL)
//     {
//         /* Generate raw status information about each task. */
//         uxArraySize = uxTaskGetSystemState(pxTaskStatusArray,
//                                            uxArraySize,
//                                            &ulTotalRunTime);

//         /* For percentage calculations. */
//         ulTotalRunTime /= 100UL;

//         /* Avoid divide by zero errors. */
//         if (ulTotalRunTime > 0)
//         {
//             /* For each populated position in the pxTaskStatusArray array,
//             format the raw data as human readable ASCII data. */
//             for (x = 0; x < uxArraySize; x++)
//             {
//                 /* What percentage of the total run time has the task used?
//                 This will always be rounded down to the nearest integer.
//                 ulTotalRunTimeDiv100 has already been divided by 100. */
//                 ulStatsAsPercentage =
//                     pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

//                 if (ulStatsAsPercentage > 0UL)
//                 {
//                     sprintf(pcWriteBuffer, "%stt%lutt%lu%%rn",
//                             pxTaskStatusArray[x].pcTaskName,
//                             pxTaskStatusArray[x].ulRunTimeCounter,
//                             ulStatsAsPercentage);
//                 }
//                 else
//                 {
//                     /* If the percentage is zero here then the task has
//                     consumed less than 1% of the total run time. */
//                     sprintf(pcWriteBuffer, "%stt%lutt<1%%rn",
//                             pxTaskStatusArray[x].pcTaskName,
//                             pxTaskStatusArray[x].ulRunTimeCounter);
//                 }

//                 pcWriteBuffer += strlen((char *)pcWriteBuffer);
//             }
//         }

//         /* The array is no longer needed, free the memory it consumes. */
//         vPortFree(pxTaskStatusArray);
//     }
// }

static const char *task_state_str(eTaskState state)
{
  switch (state)
  {
  case eRunning:
    return "Running";
  case eReady:
    return "Ready";
  case eBlocked:
    return "Blocked";
  case eSuspended:
    return "Suspended";
  case eDeleted:
    return "Deleted";
  default:
    return "Unknown";
  }
}

/**
 * @brief   Function to print the CPU usage of tasks over a given duration.
 *
 * This function will measure and print the CPU usage of tasks over a specified
 * number of ticks (i.e. real time stats). This is implemented by simply calling
 * uxTaskGetSystemState() twice separated by a delay, then calculating the
 * differences of task run times before and after the delay.
 *
 * @note    If any tasks are added or removed during the delay, the stats of
 *          those tasks will not be printed.
 * @note    This function should be called from a high priority task to minimize
 *          inaccuracies with delays.
 * @note    When running in dual core mode, each core will correspond to 50% of
 *          the run time.
 *
 * @param   xTicksToWait    Period of stats measurement
 *
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_NO_MEM        Insufficient memory to allocated internal arrays
 *  - ESP_ERR_INVALID_SIZE  Insufficient array size for uxTaskGetSystemState. Trying increasing ARRAY_SIZE_OFFSET
 *  - ESP_ERR_INVALID_STATE Delay duration too short
 */
static esp_err_t print_real_time_stats(TickType_t xTicksToWait)
{
  TaskStatus_t *start_array = NULL, *end_array = NULL;
  UBaseType_t start_array_size, end_array_size;
  configRUN_TIME_COUNTER_TYPE start_run_time, end_run_time;
  esp_err_t ret;

  // Allocate array to store current task states
  start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
  start_array = malloc(sizeof(TaskStatus_t) * start_array_size);
  if (start_array == NULL)
  {
    ret = ESP_ERR_NO_MEM;
    goto exit;
  }
  // Get current task states
  start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
  if (start_array_size == 0)
  {
    ret = ESP_ERR_INVALID_SIZE;
    goto exit;
  }

  vTaskDelay(xTicksToWait);

  // Allocate array to store tasks states post delay
  end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
  end_array = malloc(sizeof(TaskStatus_t) * end_array_size);
  if (end_array == NULL)
  {
    ret = ESP_ERR_NO_MEM;
    goto exit;
  }
  // Get post delay task states
  end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
  if (end_array_size == 0)
  {
    ret = ESP_ERR_INVALID_SIZE;
    goto exit;
  }

  // Calculate total_elapsed_time in units of run time stats clock period.
  uint32_t total_elapsed_time = (end_run_time - start_run_time);
  if (total_elapsed_time == 0)
  {
    ret = ESP_ERR_INVALID_STATE;
    goto exit;
  }

  printf("[%2s!%1s] %-16s %10s (%s)\t<%9s|%-5s> => <%9s|%-5s>\n",
         "ID", "C", "Name:", "Ticks", "CPU", "Before", "Prio", "After", "Prio");

  // Match each task in start_array to those in the end_array
  for (int i = 0; i < start_array_size; i++)
  {
    int k = -1;
    for (int j = 0; j < end_array_size; j++)
    {
      if (start_array[i].xHandle == end_array[j].xHandle)
      {
        k = j;
        // Mark that task have been matched by overwriting their handles
        start_array[i].xHandle = NULL;
        end_array[j].xHandle = NULL;
        break;
      }
    }
    // Check if matching task found
    if (k >= 0)
    {
      uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
      uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * SOC_NUM_CORES);

      // printf(
      //   "| %s | %"PRIu32" | %"PRIu32"%%\n",
      //   start_array[i].pcTaskName,
      //   task_elapsed_time,
      //   percentage_time
      // );

      printf(
          "[%02u!%1u] %-16s %10" PRIu32 " (%02" PRIu32 "%%)\t<%9s|%2u:%-2u> => <%9s|%2u:%-2u>\n",
          start_array[i].xTaskNumber,
          start_array[i].xCoreID >= configNUM_CORES ? 0 : start_array[i].xCoreID + 1,
          start_array[i].pcTaskName,
          task_elapsed_time,
          percentage_time,
          task_state_str(start_array[i].eCurrentState),
          start_array[i].uxBasePriority,
          start_array[i].uxCurrentPriority,
          task_state_str(end_array[k].eCurrentState),
          end_array[k].uxBasePriority,
          end_array[k].uxCurrentPriority);
    }
  }

  // Print unmatched tasks
  for (int i = 0; i < start_array_size; i++)
  {
    if (start_array[i].xHandle != NULL)
    {
      printf("| %s | Deleted\n", start_array[i].pcTaskName);
    }
  }
  for (int i = 0; i < end_array_size; i++)
  {
    if (end_array[i].xHandle != NULL)
    {
      printf("| %s | Created\n", end_array[i].pcTaskName);
    }
  }
  ret = ESP_OK;

exit: // Common return path
  free(start_array);
  free(end_array);
  return ret;
}

// static void spin_task(void *arg)
// {
//     xSemaphoreTake(sync_lvgl_task, portMAX_DELAY);
//     while (1) {
//         //Consume CPU cycles
//         for (int i = 0; i < SPIN_ITER; i++) {
//             __asm__ __volatile__("NOP");
//         }
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

void stats_task(void *arg)
{
  xSemaphoreTake(sync_stats_task, portMAX_DELAY);

  // Start all the spin tasks
  for (int i = 0; i < NUM_TASKS_TRACED; i++)
  {
    xSemaphoreGive(sync_lvgl_task);
  }

  // Print real time stats periodically
  while (1)
  {
    printf("\n\nGetting real time stats over %" PRIu32 " ticks\n", STATS_TICKS);
    if (print_real_time_stats(STATS_TICKS) == ESP_OK)
    {
      printf("Real time stats obtained\n");
    }
    else
    {
      printf("Error getting real time stats\n");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

#endif // NUM_TASKS_TRACED

// void app_main(void)
// {
//     //Allow other core to finish initialization
//     vTaskDelay(pdMS_TO_TICKS(100));

//     //Create semaphores to synchronize
//     sync_lvgl_task = xSemaphoreCreateCounting(NUM_TASKS_TRACED, 0);
//     sync_stats_task = xSemaphoreCreateBinary();

//     //Create spin tasks
//     for (int i = 0; i < NUM_TASKS_TRACED; i++) {
//         snprintf(task_names[i], configMAX_TASK_NAME_LEN, "spin%d", i);
//         xTaskCreatePinnedToCore(spin_task, task_names[i], 1024, NULL, SPIN_TASK_PRIO, NULL, tskNO_AFFINITY);
//     }

//     //Create and start stats task
//     xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, STATS_TASK_PRIO, NULL, tskNO_AFFINITY);
//     xSemaphoreGive(sync_stats_task);
// }
