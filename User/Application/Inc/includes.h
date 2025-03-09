/**
 * @file    includes.h
 * @author  Deadline039
 * @brief   Include files
 * @version 1.0
 * @date    2024-04-03
 */

#ifndef __INCLUDES_H
#define __INCLUDES_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <bsp.h>

#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"

#include "queue.h"
#include "semphr.h "

void freertos_start(void);

/*控制摩擦轮任务与实行任务的消息队列*/
extern QueueHandle_t Queue_From_Fir;

void task_AK80ctrl(void *pvParameters);
void task_djictrl(void *pvParameters);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDES_H */
