/**
 * @file    msg_protocol.h
 * @author  Deadline039
 * @brief   消息协议
 * @version 1.0
 * @date    2024-03-01
 * @note    接收暂只支持DMA, 如果使用中断自行到`uart.h`编写相应代码
 *
 *****************************************************************************
*                             ##### 如何使用 ####
* (#) 在`message_mean_t`枚举中添加要使用的数据通信类型
*
*
* (#) 发送
*      (##) 调用`message_register_uart_handle`注册串口消息通讯句柄, 发送将会
*           使用这个函数注册的句柄
*      (##) 调用`message_send_data`来发送数据. 如果要更改串口, 重新调用
*           `message_register_uart_handle`更改发送串口句柄
*      (##) `message_send_data`函数需要指定`data_mean`(也就是`message_mean_t`
*           枚举中的类型), `data_type`(数据类型, 整数, 浮点或者字符串等),
*           `data`(数据指针, 也就是要发送的数据), 以及`data_len`, 数据长度
* (#) 接收
*      (##) 调用`message_add_polling_handle`添加要轮询的串口
*      (##) 调用`message_register_recv_callback`注册接收回调函数, 当收到消息
*           以后会调用回调函数.
*      (##) 需要持续调用`message_polling_data`来轮询消息, 可以放到RTOS的一个任
*           务或者定时器里. 当收到消息后根据`data_mean`来调用相应的回调函数
*      (##) 回调函数参数形式必须是void (uint8_t, message_type_t, void*)
*           第一个参数是消息长度, 第二个参数是数据类型(整数, 浮点或者字符串等),
*           第三个参数是数据区内容, 无返回值
*      (##) `message_polling_data`仅支持DMA接收, 如果是串口接收需要自行编写回调
*           函数与接收逻辑
*      (##) 调用`message_remove_polling_handle`删除要轮询的串口
******************************************************************************
*    Date    | Version |   Author    | Version Info
* -----------+---------+-------------+----------------------------------------
* 2024-04-13 |   1.0   | Deadline039 | 初版
*/

#ifndef __MSG_PROTOCOL_H
#define __MSG_PROTOCOL_H

#include <bsp.h>

#define MSG_MAX_DATA_LENGTH          16 /* 最大数据长度, 超出长度会造成缓冲区溢出 */
/* 如果是数组, 可以调用此宏定义获取长度; 堆分配的内存勿用!! */
#define MSG_GET_DATA_ARRAY_LENGTH(X) (sizeof(X))

#if (MSG_MAX_DATA_LENGTH <= 1)
#error Max data length must be more than 1.
#elif (MSG_MAX_DATA_LENGTH >= 250)
#error Max data length must be less than 250.
#endif /* MSG_MAX_DATA_LENGTH */

#define MSG_NO_DATA           0x00 /* 没有收到消息 */
#define MSG_DATA_OVER         0xFF /* 数据长度溢出 */
#define MSG_DATA_LENGTH_ERROR 0xFE /* 实际接收长度与消息中的长度不一 */
#define MSG_DATA_VERIFY_ERROR 0xFD /* 接收校验错误(最后一个字节不是0xFF) */

/**
 * @brief 数据含义
 */
typedef enum {
    MSG_REMOTE = 0x00U, /*!< 遥控器通信数据 遥控器->主板 */
    MSG_CHASSIS,        /*!< 主板底盘通信   底盘<->主板 */

    // MSG_DT35,               /*接收DT35信息*/
    MSG_MEAN_LENGTH_RESERVE /*!< 保留位, 用于定义数据长度 */

} message_mean_t;

/**
 * @brief 数据类型
 */
typedef enum {
    MSG_DATA_UINT8 = 0x00U,
    MSG_DATA_INT8,
    MSG_DATA_UINT16,
    MSG_DATA_INT16,
    MSG_DATA_INT32,
    MSG_DATA_UINT32,
    MSG_DATA_INT64,
    MSG_DATA_UINT64,
    MSG_DATA_FP32,
    MSG_DATA_FP64,
    MSG_DATA_STRING
} message_type_t;

/**
 * @brief 回调函数指针定义
 *
 * @param msg_length 消息帧长度
 * @param msg_type 数据类型
 * @param[in] msg_data 消息数据接收区
 */
typedef void (*msg_recv_callback_t)(uint8_t /* msg_length */,
                                    message_type_t /* msg_type */,
                                    void * /* msg_data */);

void message_register_recv_callback(message_mean_t msg_mean,
                                    msg_recv_callback_t msg_callback);
void message_register_send_handle(message_mean_t msg_mean,
                                  UART_HandleTypeDef *msg_send_handle);
void message_send_data(message_mean_t data_mean, message_type_t data_type,
                       void *data, size_t data_len);

void message_add_polling_handle(UART_HandleTypeDef *uart_handle);
void message_remove_polling_handle(UART_HandleTypeDef *uart_handle);

uint8_t message_polling_data(void);

#endif /* __MSG_PROTOCOL_H */
