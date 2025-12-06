/**
 * @file 		  rt_stats.cpp
 *
 * Author		: Bernd Waldmann
 * Created		: 29-Nov-2025
 * Tabsize		: 4
 * 
 * This Revision: $Id: rt_stats.cpp 1941 2025-12-04 19:13:36Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief simple display of task statistics (runtime, percent load, stack watermark)
 * call `update_realtime_stats()` from loop(), every few seconds.
 * 
 */

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ansi.h"
#include "rt_stats.h"


#define ARRAY_SIZE_OFFSET   5   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE


static TaskStatus_t *tasks_status_1 = NULL, *tasts_status_2 = NULL;
static UBaseType_t tasks_status_1_size, tasks_status_2_size;
static uint32_t start_run_time, end_run_time;


/**
 * @brief Print task statistics, based on two snapshots in `tasks_status_1` and `tasts_status_2`
 * 
 */
static void print_stats()
{
    //Calculate total_elapsed_time in units of run time stats clock period.
    uint32_t total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
        return;
    }

    printf("\n| %-20s | Run Time | Percent | Watermark\n","Task");
    //Match each task in tasks_status_1 to those in the tasts_status_2
    for (int i = 0; i < tasks_status_1_size; i++) {
        int k = -1;
        for (int j = 0; j < tasks_status_2_size; j++) {
            if (tasks_status_1[i].xHandle == tasts_status_2[j].xHandle) {
                k = j;
                break;
            }
        }
        //Check if matching task found
        if (k >= 0) {
            uint32_t task_elapsed_time = tasts_status_2[k].ulRunTimeCounter - tasks_status_1[i].ulRunTimeCounter;
            uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * portNUM_PROCESSORS);
            uint32_t watermark = tasks_status_1[i].usStackHighWaterMark;
            printf("%s| %-20s | %8" PRIu32 " | %6" PRIu32 "%% | %6" PRIu32 " %s\n", 
                (percentage_time >= 5) ? ANSI_BOLD : "",
                tasks_status_1[i].pcTaskName, 
                task_elapsed_time, 
                percentage_time,
                watermark,
                (percentage_time >= 5) ? ANSI_RESET : ""
            );
        }
    }
}


/**
 * @brief Fill array with task information. Call this twice, with a pause in between
 * 
 * @param array 
 * @param run_time 
 * @return uint32_t 
 */
static uint32_t fill_stats( TaskStatus_t** array, uint32_t* run_time )
{
    uint32_t array_size;
    if (*array) free(*array);
    array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    *array = (TaskStatus_t*) malloc(sizeof(TaskStatus_t) * array_size);
    array_size = uxTaskGetSystemState(*array, array_size, run_time);
    return array_size;
}


/**
 * @brief Show realtime task statistics. Call this from loop(), every few seconds.
 * It will calculate task runtime etc. relative to the previous call, and print
 * a table with all tasks, to stdout
 * 
 */
void update_realtime_stats()
{
    if (tasks_status_1==NULL) {
        // first time call to this function. Fill 1st snapshot and return
        tasks_status_1_size = fill_stats( &tasks_status_1, &start_run_time );
        return;
    }

    tasks_status_2_size = fill_stats( &tasts_status_2, &end_run_time );
    if (0==tasks_status_2_size) return;

    // we have both valid tasks_status_1 and tasts_status_2, let's print it
    print_stats();
    // now move snapshot 2 to 1
    free(tasks_status_1);
    tasks_status_1 = tasts_status_2;
    tasts_status_2 = NULL;
    tasks_status_1_size = tasks_status_2_size;
    tasks_status_2_size = 0;
    start_run_time = end_run_time;
}
