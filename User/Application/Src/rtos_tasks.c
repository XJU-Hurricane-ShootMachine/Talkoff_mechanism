/**
 * @file    rtos_tasks.c
 * @author  Deadline039
 * @brief   RTOS tasks.
 * @version 1.0
 * @date    2024-01-31
 */

#include "includes.h"
#include "uart2_calbackl.h"
#include "remote_ctrl.h"
#include "talk_off.h"

#include "queue.h"
#include "semphr.h "

/*控制电机以及接收遥控器的消息队列*/
QueueHandle_t Queue_From_Fir;

static TaskHandle_t start_task_handle;
void start_task(void *pvParameters);

static TaskHandle_t task_message_handle;
void task_message(void *pvParameters);

// static TaskHandle_t task_sendmage_handle;
// void task_sendmage(void *pvParameters);

static TaskHandle_t task_AK80ctrl_handle;
static TaskHandle_t task_djictrl_handle;

/*****************************************************************************/

/**
 * @brief FreeRTOS start up.
 *
 */
void freertos_start(void) {
    xTaskCreate(start_task, "start_task", 512, NULL, 2, &start_task_handle);
    vTaskStartScheduler();
}

/**
 * @brief Start up task.
 *
 * @param pvParameters Start parameters.
 */
void start_task(void *pvParameters) {
    UNUSED(pvParameters);
    taskENTER_CRITICAL();
    Queue_From_Fir = xQueueCreate(2, sizeof(int16_t));
    xTaskCreate(task_message, "task_message", 256, NULL, 1,
                &task_message_handle);
    // xTaskCreate(task_sendmage,"task_sendmage",256,  NULL,&task_sendmage_handle);
    xTaskCreate(task_AK80ctrl, "task_AK80ctrl", 256, NULL, 2,
                &task_AK80ctrl_handle);
    xTaskCreate(task_djictrl, "task_djictrl", 256, NULL, 2,
                &task_djictrl_handle);
    vTaskDelete(start_task_handle);
    taskEXIT_CRITICAL();
}

/**
  * @brief 按键回调函数
  * 
  * @param key 键值
  */
void task_key(uint8_t key) {
    if (key == 2) {
        int16_t key_flag = 1; /* 电机开始向下压 */
        xQueueSend(Queue_From_Fir, &key_flag, 10);
        xQueueSend(Queue_From_Fir, &key_flag, 10);
    }
    if (key == 3) {
        int16_t key_flag = 0;
        xQueueSend(Queue_From_Fir, &key_flag, 10);
        xQueueSend(Queue_From_Fir, &key_flag, 10);
    }
    return;
}

/**
 * @brief 串口轮询数据任务
 *
 * @param pvParameters  parameters.
 */
void task_message(void *pvParameters) {
    UNUSED(pvParameters);
    // message_add_polling_handle(&usart2_handle);     /* 使用串口二接收数据 */
    // message_register_recv_callback(MSG_REMOTE, remote_receive_callback);
    // remote_register_key_callback(1, task_key);
    // remote_register_key_callback(2, task_key);
    uint8_t number = 0;
    while (1) {
        number = key_scan(0);
        if (number) {
            task_key(number);
        }
        // key_scan(0);
        // message_polling_data();
        vTaskDelay(400);
    }
}

// void task_sendmage(void *pvParameters){
//     UNUSED(pvParameters);

//     message_register_send_handle(MSG_CHASSIS, &usart1_handle);     /* 注册数据发送句柄1 */

//     while(1){
//
//     }
// }