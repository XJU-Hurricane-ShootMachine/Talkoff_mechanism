/**
 * @file    msg_protocol.c
 * @author  Deadline039
 * @brief   消息协议以及收发
 * @version 1.0
 * @date    2024-03-01
 * @note    接收暂只支持DMA, 如果使用中断自行到`uart.h`编写相应代码
 */

#include "msg_protocol.h"
#include "string.h"


#if SYS_SUPPORT_OS
#include "FreeRTOS.h"
#endif /* SYS_SUPPORT_OS */

/**
 * @brief 回调函数指针
 */
static msg_recv_callback_t p_receive_callback[MSG_MEAN_LENGTH_RESERVE] = {NULL};

/**
 * @brief 串口发送句柄指针
 */
static UART_HandleTypeDef *p_send_handle[MSG_MEAN_LENGTH_RESERVE] = {NULL};

/**
 * @brief 注册接收回调函数指针
 *
 * @param msg_mean 数据含义
 * @param msg_callback 回调指针
 * @note 如果更换回调函数, 重新调用该函数即可
 */
void message_register_recv_callback(message_mean_t msg_mean,
                                    msg_recv_callback_t msg_callback) {
    p_receive_callback[msg_mean] = msg_callback;
}

/**
 * @brief 注册数据发送句柄
 *
 * @param msg_mean 数据含义
 * @param msg_send_handle 发送串口句柄
 */
void message_register_send_handle(message_mean_t msg_mean,
                                UART_HandleTypeDef *msg_send_handle) {
    p_send_handle[msg_mean] = msg_send_handle;
}

/**
 * @brief 填充并发送数据, 支持多种类型
 *
 * @param data_mean 数据含义
 * @param data_type 数据类型
 * @param data 数据内容
 * @param data_len 数据长度(字节数), 使用`MSG_GET_DATA_ARRAY_LENGTH`宏获取即可
 */
void message_send_data(message_mean_t data_mean, message_type_t data_type,
                    void *data, size_t data_len) {
    if (data == NULL || data_len == 0) {
        return;
    }
    if (data_len > MSG_MAX_DATA_LENGTH) {
        return;
    }
    if (p_send_handle[data_mean] == NULL) {
        return;
    }
#if SYS_SUPPORT_OS
    uint8_t *data_buf =
        (uint8_t *)pvPortMalloc(data_len + 3); /* 发送数据的字节流 */
#else                                          /* SYS_SUPPORT_OS */
    uint8_t *data_buf = (uint8_t *)malloc(data_len + 3); /* 发送数据的字节流 */
#endif                                         /* SYS_SUPPORT_OS */

    /* 复制数据到字节流, 前面空出两个字节用来标记数据 */
    memcpy((data_buf + 2), data, data_len);
    /* 第一个字节, 高四位标记含义, 低四位标记数据类型 */
    *data_buf = (uint8_t)(data_mean << 4) | data_type;
    /* 第二个字节, 标记数据长度 */
    *(data_buf + 1) = (uint8_t)data_len;
    /* 最后一个字节, 标记数据末尾 */
    *(data_buf + data_len + 2) = 0xFF;

    if (p_send_handle[data_mean]->hdmatx != NULL) {
        uart_dmatx_write(p_send_handle[data_mean], data_buf, data_len + 3);
        uart_dmatx_send(p_send_handle[data_mean]);
    } else {
        HAL_UART_Transmit(p_send_handle[data_mean], data_buf, data_len + 3,
                        0xFFFF);
    }
#if SYS_SUPPORT_OS
    vPortFree(data_buf);
#else  /* SYS_SUPPORT_OS */
    free(data_buf);
#endif /* SYS_SUPPORT_OS */
}

/**
 * @brief 消息轮询节点链表
 */
typedef struct polling_list_node {
    UART_HandleTypeDef *huart;      /*!< 串口句柄 */
    struct polling_list_node *next; /*!< 链表下一个节点 */
} polling_list_node_t;

/* 串口轮询链表头指针 */
static polling_list_node_t *p_polling_list_head = NULL;

/**
 * @brief 添加要轮询的串口句柄, 仅支持DMA
 *
 * @param uart_handle 要添加的串口句柄
 */
void message_add_polling_handle(UART_HandleTypeDef *uart_handle) {
    if (uart_handle == NULL) {
        return;
    }

    polling_list_node_t *node = p_polling_list_head;
    while ((node != NULL) && (node->huart != uart_handle)) {
        node = node->next;
    }

    if (node != NULL) {
        return; /* 句柄已存在, 不添加 */
    }
#if SYS_SUPPORT_OS
    polling_list_node_t *new_node =
        (polling_list_node_t *)pvPortMalloc(sizeof(polling_list_node_t));
#else  /* SYS_SUPPORT_OS */
    polling_list_node_t *new_node =
        (polling_list_node_t *)malloc(sizeof(polling_list_node_t));
#endif /* SYS_SUPPORT_OS */

    if (new_node == NULL) {
        return;
    }

    new_node->huart = uart_handle;
    new_node->next = p_polling_list_head;
    p_polling_list_head = new_node;
}

/**
 * @brief 移除要轮询的串口句柄, 仅支持DMA
 *
 * @param uart_handle 要移除的串口句柄
 */
void message_remove_polling_handle(UART_HandleTypeDef *uart_handle) {
    if (uart_handle == NULL) {
        return;
    }

    polling_list_node_t *current_node = p_polling_list_head;
    polling_list_node_t *previous_node = p_polling_list_head;

    while ((current_node != NULL) && (current_node->huart != uart_handle)) {
        previous_node = current_node;
        current_node = current_node->next;
    }

    if (current_node == NULL) {
        /* 句柄不存在 */
        return;
    }

    if (previous_node == current_node) {
        p_polling_list_head = previous_node->next;
    }

    previous_node->next = current_node->next;
#if SYS_SUPPORT_OS
    vPortFree(current_node);
#else  /* SYS_SUPPORT_OS */
    free(current_node);
#endif /* SYS_SUPPORT_OS */
}

/**
 * @brief 轮询数据, 并调用相应的函数
 *
 * @return 长度或者状态标记
 *  @retval `0-MSG_NO_DATA` - 没有收到数据, 或者没有接收的串口句柄
 *  @retval `255-MSG_DATA_OVER` - 长度溢出
 *  @retval `254-MSG_DATA_ERROR` - 校验错误, 末尾没有收到0xFF
 *  @retval 1~250 - 本次收到的有效数据
 * @note 事先在message_mean_t定义数据类型, 并注册相应的回调函数.
 *       这个函数挂在一个while循环或者定时器中一直轮询就可以,
 *       不要改动这个函数的任何内容
 */
uint8_t message_polling_data(void) {
    /* 轮询链表的指针 */
    static polling_list_node_t *current_node = NULL;

    if (current_node == NULL) {
        current_node = p_polling_list_head;
        return MSG_NO_DATA;
    }

    /* 数据数组 */
    uint8_t data_buf[MSG_MAX_DATA_LENGTH + 3];

    uint32_t data_len =
        uart_dmarx_read(current_node->huart, data_buf, sizeof(data_buf));

    current_node = current_node->next;

    /* 无数据 */
    if (data_len == 0) {
        return MSG_NO_DATA;
    }

    uint8_t msg_len = *(data_buf + 1);

    /* 数据长度超过最大长度, 为避免异常内存操作, 直接返回 */
    if (msg_len > MSG_MAX_DATA_LENGTH) {
        return MSG_DATA_OVER;
    }

    /* 数据帧中定义的长度与实际接收到的长度不一致 */
    if (data_len != (msg_len + 3)) {
        return MSG_DATA_LENGTH_ERROR;
    }

    /* 校验字节错误, 最后一位不是0xFF */
    if (*(data_buf + msg_len + 2) != 0xFF) {
        return MSG_DATA_VERIFY_ERROR;
    }

    if ((data_buf[0] >> 4) >= MSG_MEAN_LENGTH_RESERVE) {
        /* 消息下标越界 */
        return 0;
    }

    if (p_receive_callback[(data_buf[0] >> 4)] != NULL) {
        p_receive_callback[(data_buf[0] >> 4)](msg_len, data_buf[0] & 0x0F,
                                            data_buf + 2);
    }

    return (uint8_t)msg_len; /* 返回消息长度 */
}
