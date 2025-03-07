#ifndef _CHASSIS_H
#define _CHASSIS_H

#include <bsp.h>


/** Export Types *************************************************************/

/**
 * @brief 主板<-->底盘 串口消息内容
 */
typedef enum {
    /* 底盘-->主板 */
    CHASSIS_INIT = 0U,  /*!< 底盘复位信息 */
    CHASSIS_RECEIVED,   /*!< 底盘收到消息 */
    CHASSIS_NOT_ARRIVE, /*!< 底盘未到达 */
    CHASSIS_ARRIVE,     /*!< 底盘到达 */
    CHASSIS_MOVE,       /*!< 底盘横走 */
    CHASSIS_ERROR,      /*!< 底盘错误 */

    /* 主板-->底盘 */
    CHASSIS_MAIN_INIT, /*!< 主板复位信息 */
    CHASSIS_GET_STATE, /*!< 主板获取信息 */
    CHASSIS_PATH_DATA, /*!< 定点与路径 */
} chassis_message_t;

/**
 * @brief 点坐标信息
 */
typedef struct {
    float x, y;  /* !< 坐标  */
    float angle; /* !< 角度  */
} pos_point_t;

/** Export Variables *********************************************************/

extern TaskHandle_t message_polling_task_handle; /* 串口消息轮询任务句柄 */
extern TaskHandle_t chassis_ctrl_task_handle; /* 底盘控制任务句柄 */
extern TaskHandle_t chassis_auto_run_handle;  /* 底盘自动运行 */

/** Export Function **********************************************************/

void chassis_ctrl_task(void *pvParameters);
void chassis_auto_run(void *pvParameters);
void chassis_switch_mode(uint8_t key);


#endif /* _CHASSIS_H */