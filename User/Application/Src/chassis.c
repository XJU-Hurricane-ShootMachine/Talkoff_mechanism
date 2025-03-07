/**
 * @file    chassis.c
 * @author  Deadline039
 * @brief   舵轮底盘控制任务
 * @version 1.0
 * @date    2024-04-20
 */

#include "chassis.h"
#include "msg_protocol.h"
#include "includes.h"

TaskHandle_t chassis_ctrl_task_handle = {0}; /* 手动控制任务 */
TaskHandle_t chassis_auto_run_handle = {0};  /* 自动控制任务 */

/** 底盘自动控制 **************************************************************/

static void main_message_process(chassis_message_t main_message,
                                 uint8_t point_number);

/**
  * @brief 底盘状态
  */
static enum {
    STATUS_GO_POINT,   /*!< 底盘当前在跑点 */
    STATUS_GO_PATH,    /*!< 底盘当前在跑路径 */
    STATUS_GO_STORAGE, /*!< 底盘当前在归仓路径 */
    STATUS_ARRIVE,     /*!< 底盘已到达, 停止 */
    STATUS_MAIN_ERROR  /*!< 主板出错, 停止运动 */
} chassis_status = STATUS_ARRIVE;

static void send_chassis_status(void);
/* 接收到的点位 */
static pos_point_t receive_point;
/**
  * 主板发送的跑点的信息, 0~4代表5个框, 5代表Ⅲ区中央, 6代表Ⅰ区到Ⅲ区的路径,
  * 7代表重试的路径, 8无意义, 避免误操作
  */
static uint8_t msg_point_number = 8;

/**
  * @brief 底盘收到消息后回调函数
  *
  * @param msg_length 消息帧长度
  * @param msg_type 数据类型
  * @param[in] msg_data 消息数据接收区
  */
static void chassis_msg_callback(uint8_t msg_length, message_type_t msg_type,
                                 void *msg_data) {
    UNUSED(msg_length);
}

/**
  * @brief 处理主板消息
  *
  * @param point_number `CHASSIS_PATH_DATA`的点信息
  */
void main_message_process(chassis_message_t main_message,
                          uint8_t point_number) {

    chassis_message_t send_data;

    switch (main_message) {
        case CHASSIS_GET_STATE: {
            send_chassis_status();
        } break;

        case CHASSIS_MAIN_INIT: {
            if (chassis_status != STATUS_ARRIVE) {
                /* 在跑的过程中主板复位, 可能有问题, 停止 */
                chassis_status = STATUS_MAIN_ERROR;
            } else {
                send_data = CHASSIS_INIT;
                message_send_data(MSG_CHASSIS, MSG_DATA_UINT8, &send_data,
                                  sizeof(send_data));
            }
        } break;

        case CHASSIS_PATH_DATA: {
            // if (chassis_status != STATUS_ARRIVE) {
            //     send_data = CHASSIS_NOT_ARRIVE;
            //     message_send_data(MSG_CHASSIS, MSG_DATA_UINT8, &send_data,
            //                       sizeof(send_data));
            //     log_message(LOG_WARNING,
            //                 "Chassis is running now, the point %d "
            //                 "will be ignored. ",
            //                 point_number);

            //     return;
            // }

            msg_point_number = point_number;
            send_data = CHASSIS_RECEIVED;
            message_send_data(MSG_CHASSIS, MSG_DATA_UINT8, &send_data,
                              sizeof(send_data));

            if (point_number <= 5) {
                /* 归仓点 */
                chassis_status = STATUS_GO_STORAGE;
            } else if (point_number <= 7) {
                /* 路径点 */
                chassis_status = STATUS_GO_PATH;
            }
        } break;

        default: {
            log_message(
                LOG_WARNING,
                "Unknow message received, which is %d, abort this message. ",
                main_message);
        } break;
    }
}

/**
  * @brief 底盘状态上报
  *
  */
void send_chassis_status(void) {
    chassis_message_t send_data;

    switch (chassis_status) {
        case STATUS_GO_POINT:
        case STATUS_GO_PATH:
        case STATUS_GO_STORAGE: {
            send_data = CHASSIS_NOT_ARRIVE;
        } break;

        case STATUS_ARRIVE: {
            send_data = CHASSIS_ARRIVE;
        } break;

        default: {
            send_data = CHASSIS_ERROR;
        } break;
    }
    message_send_data(MSG_CHASSIS, MSG_DATA_UINT8, &send_data,
                      sizeof(send_data));
}

/**
  * @brief 主板出错, LED1闪烁, 舵轮停止运动
  *
  */
void main_error(void) {
    while (1) {
        LED1_TOGGLE();
        log_message(
            LOG_ERROR,
            "Main board reset, may error occurred. Please reset this board. ");
        steering_wheel_ctrl(0, 0, 0);
        HAL_Delay(1000);
    }
}

/**
  * @brief 自动运行任务
  *
  * @param pvParameters 传入参数(未用到)
  */
void chassis_auto_run(void *pvParameters) {
    UNUSED(pvParameters);

    message_register_recv_callback(MSG_CHASSIS, chassis_msg_callback);

    /** 发送初始化完毕信息 *****************************************/
    message_register_send_handle(MSG_CHASSIS, &usart6_handle);
    chassis_message_t send_data = CHASSIS_INIT;
    message_send_data(MSG_CHASSIS, MSG_DATA_UINT8, &send_data,
                      sizeof(send_data));

    bool is_arrive = false;

    send_data = CHASSIS_ARRIVE; /* 到达后给主板发送消息 */

    while (1) {
        switch (chassis_status) {
            case STATUS_GO_POINT: {
                /* 跑点 */
                is_arrive =
                    go_single_point(receive_point.x, receive_point.y,
                                    receive_point.angle, ACTION_PLANE_POS);
            } break;

            case STATUS_GO_PATH: {
                /* 跑路径 */
                if (msg_point_number == 6) {
                    /* 一区到三区的路径 */
                    is_arrive = points_inturn_move(red_path_I_to_III, 9);
                } else if (msg_point_number == 7) {
                    /* 重试的路径 */
                    // is_arrive = points_inturn_move();
                }
            } break;

            case STATUS_GO_STORAGE: {
                /* 跑归仓点 */
                if (msg_point_number <= 5) {
                    is_arrive = goto_storage_area(msg_point_number);
                }
            } break;

            case STATUS_ARRIVE: {
                /* 到达点位, 保持速度为0 */
                steering_wheel_ctrl(0, 0, 0);
            } break;

            case STATUS_MAIN_ERROR: {
                main_error();
            } break;

            default: {
            } break;
        }

        vTaskDelay(10);

        if (!(is_arrive && (chassis_status != STATUS_ARRIVE))) {
            continue;
        }

        /* 刚跑完, 状态还没标记为到达 */
        if ((chassis_status == STATUS_GO_PATH) &&
            (msg_point_number == 6 || msg_point_number == 7)) {
            /* 跑路径到Ⅲ区 */
            while (go_IIIcenter_clear_action(1715.0f, 1920.0f, -90.0f) == false)
                ;
        } else if ((chassis_status == STATUS_GO_STORAGE) &&
                   (msg_point_number == 5)) {
            /* 回到Ⅲ区中央 */
            act_position_reset_data();
        }

        send_data = CHASSIS_ARRIVE;
        message_send_data(MSG_CHASSIS, MSG_DATA_UINT8, &send_data,
                          sizeof(send_data));
        chassis_status = STATUS_ARRIVE;
        msg_point_number = 8;
    }
}

/** 模式切换 ******************************************************************/

/**
  * @brief 遥控器切换模式
  *
  * @param key 按下的按键
  * @note key15 手动, LED1灭;
  *       key16 自动, LED1亮
  */
void chassis_switch_mode(uint8_t key) {
    if (key == 15) {
        vTaskSuspend(chassis_auto_run_handle);
        vTaskResume(chassis_ctrl_task_handle);
        LED1_OFF();
    } else if (key == 16) {
        vTaskSuspend(chassis_ctrl_task_handle);
        vTaskResume(chassis_auto_run_handle);
        LED1_ON();
    }
}

/** 底盘手动控制 **************************************************************/

/**
  * @brief 底盘手动控制任务, 创建后挂起, 用遥控器控制
  *
  * @param pvParameters 传入参数(未使用)
  */
void chassis_ctrl_task(void *pvParameters) {
    UNUSED(pvParameters);
    vTaskSuspend(chassis_ctrl_task_handle);

    /* x速度, y速度, 自转速度 */
    int8_t chassis_speed_x = 0, chassis_speed_y = 0, chassis_speed_round = 0;

    while (1) {
        if (g_remote_left_x > 14 || g_remote_left_x < 10) {
            chassis_speed_x = (-1) * (g_remote_left_x - 12);
        } else {
            chassis_speed_x = 0;
        }

        if (g_remote_left_y > 14 || g_remote_left_y < 10) {
            chassis_speed_y = g_remote_left_y - 12;
        } else {
            chassis_speed_y = 0;
        }

        if (g_remote_right_x > 14 || g_remote_right_x < 10) {
            chassis_speed_round = g_remote_right_x - 12;
        } else {
            chassis_speed_round = 0;
        }

        steering_wheel_ctrl(chassis_speed_x, chassis_speed_y,
                            chassis_speed_round);
        vTaskDelay(20);
    }
}
