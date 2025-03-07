/**
 * @file    remote_ctrl.c
 * @author  Deadline039
 * @brief   遥控器
 * @version 0.1
 * @date    2024-04-19
 */

 #include "remote_ctrl.h"
 #include "msg_protocol.h"
 #include "FreeRTOS.h"
 #include "./LED/led.h"
 #include "task.h"
 
 /* 遥控器按键回调函数 */
 static remote_key_callback_t p_key_callback[19];
 
 /* 遥控器上报任务句柄 */
 TaskHandle_t remote_report_task_handle = {0};
 
 /* 底盘状态 */
 extern uint8_t g_chassis_status;
 
 /**
  * @brief 遥控器接收数据
  */
 uint8_t g_remote_key = 0;
 uint8_t g_remote_left_x = 12;
 uint8_t g_remote_left_y = 12;
 uint8_t g_remote_right_x = 12;
 uint8_t g_remote_right_y = 12;
 
 /**
  * @brief 遥控器下标数组下标
  */
 enum {
     ARRAY_KEY = 0x00U,
     ARRAY_LEFT_X,
     ARRAY_LEFT_Y,
     ARRAY_RIGHT_X,
     ARRAY_RIGHT_Y,
 };
 
 /**
  * @brief 串口遥控器回调
  *
  * @param msg_length 数据长度
  * @param msg_type 数据类型
  * @param msg_data 数据区内容
  */
 void remote_receive_callback(uint8_t msg_length, message_type_t msg_type,
                              void *msg_data) {
     static uint8_t key_up = 1; /* 按键按松开标志, 避免连续调用回调函数 */
 
     if (msg_length != 5 || msg_type != MSG_DATA_UINT8) {
         return;
     }
     LED0_TOGGLE();
 
     uint8_t *remote_data = (uint8_t *)msg_data;
     g_remote_key = remote_data[ARRAY_KEY];
     g_remote_left_x = remote_data[ARRAY_LEFT_X];
     g_remote_left_y = remote_data[ARRAY_LEFT_Y];
     g_remote_right_x = remote_data[ARRAY_RIGHT_X];
     g_remote_right_y = remote_data[ARRAY_RIGHT_Y];
 
     if (g_remote_key == 0) {
         key_up = 1;
         return;
     }
 
     if (key_up && p_key_callback[g_remote_key] != NULL) {
         key_up = 0;
         p_key_callback[g_remote_key](g_remote_key);
     }
 }
 
 /**
  * @brief 注册遥控器按键回调函数指针
  *
  * @param key 要注册的按键(1 ~ 18)
  * @param callback 回调函数指针
  */
 void remote_register_key_callback(uint8_t key, remote_key_callback_t callback) {
     if (key < 1 || key > 18) {
         return;
     }
     p_key_callback[key] = callback;
 }
 
 /**
  * @brief 取消注册遥控器按键回调函数指针
  *
  * @param key 要取消注册的按键(1 ~ 18)
  */
 void remote_unregister_key_callback(uint8_t key) {
     if (key < 1 || key > 18) {
         return;
     }
     p_key_callback[key] = NULL;
 }
 