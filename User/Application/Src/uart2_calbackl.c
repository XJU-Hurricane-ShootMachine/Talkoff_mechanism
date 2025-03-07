#include "uart2_calbackl.h"
#include "includes.h"

/* 注册串口一定要在stm32f4xx_hal_conf.h开启对应的注册USE_HAL_SPI_REGISTER_CALLBACKS，使能为1
 */

static UART_HandleTypeDef *uart_handle;
static uint8_t g_U2_recv_buf[40]; /* 中断接收到的一个字符 */

uint8_t USART2_RX_STA = 0;
uint8_t USART2_START = 0;

static uint8_t num[5] = {0};

/**
 * @brief 串口2接收回调
 *
 * @param byte 串口收到的字节
 */
void uart2_callback(uint8_t byte) {
    if (byte == 's') {
        USART2_START = 1;
    }
    if (byte == 'e') {
        USART2_RX_STA = 0;
        USART2_START = 0;
    }

    if (USART2_START) {
        LED0_TOGGLE();
        g_U2_recv_buf[USART2_RX_STA++] = byte;
    }
}

void message_polling(void) {
    static uint8_t dma_rx_buf[30];
    uint32_t len = uart_dmarx_read(uart_handle, dma_rx_buf, sizeof(dma_rx_buf));

    if (len == 0) {
        return;
    }

    for (size_t i = 0; i < len; i++) {
        uart2_callback(dma_rx_buf[i]);
    }
}

/**
 * @brief 注册通信串口2
 *
 * @param huart 串口句柄
 */
void U2_register_uart(UART_HandleTypeDef *huart) {
    if (huart == NULL) {
        return;
    }

    uart_handle = huart;
}
