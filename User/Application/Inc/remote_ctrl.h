/**
 * @file    remote_ctrl.h
 * @author  Deadline039
 * @brief   遥控器公开内容
 * @version 0.1
 * @date    2024-04-20
 * @note    遥控器按键可以通过注册回调函数使用,
 *          也可以在别的函数里轮询`g_remote_key`的值来调操作.
 */

 #ifndef __REMOTE_CTRL_H
 #define __REMOTE_CTRL_H
 
//  #include "includes.h"
 #include "msg_protocol.h"




 /* 按键回调函数 */
 typedef void (*remote_key_callback_t)(uint8_t /* key */);
 
 extern uint8_t g_remote_key;
 extern uint8_t g_remote_left_x;
 extern uint8_t g_remote_left_y;
 extern uint8_t g_remote_right_x;
 extern uint8_t g_remote_right_y;
 
 void remote_receive_callback(uint8_t msg_length, message_type_t msg_type,
                              void *msg_data);
 void remote_report_task(void *pvParameters);
 void remote_register_key_callback(uint8_t key, remote_key_callback_t callback);
 void remote_unregister_key_callback(uint8_t key);
 
 #endif /* __REMOTE_CTRL_H */
 