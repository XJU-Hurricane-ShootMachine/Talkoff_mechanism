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

/*定义摩擦带电机以及俯仰角AK80-8D电机的参数，便于队列传输*/
typedef struct {
    float Fir_Speed[3];
    float AK_PitchAngle;
} Firection_Parameter_t;

typedef struct motor_pid {
    pid_t pid_pos;
    pid_t pid_spd;
} dji_pid;


extern Firection_Parameter_t Firection_Send;

extern TaskHandle_t task_AK80ctrl_handle;
extern TaskHandle_t task_djictrl_handle;

void task_AK80ctrl(void *pvParameters);
void task_djictrl(void *pvParameters);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDES_H */
