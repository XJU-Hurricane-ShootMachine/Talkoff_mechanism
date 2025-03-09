/**
 * @file    take_off.c
 * @brief   底盘起跳的主要控制逻辑实现
 * @version 0.2
 * @date    2025-03-05
 */

#include <includes.h>
#include "remote_ctrl.h"
#include "talk_off.h"

#define CANid_AK_1   100 /* CANid需要在上位机上查看 */
#define CANid_AK_2   104

#define dji_pushdown 90.0 /* 电机下压距离 */
#define ak_pushdown  180.0

ak_motor_handle_t ak_motor_1;
ak_motor_handle_t ak_motor_2;
dji_motor_handle_t dji_motor_1;
dji_motor_handle_t dji_motor_2;

dji_pid_t motor2; /* dji电机1 */
dji_pid_t motor1;

/**
 * @brief 大疆pid计算
 * 
 * @param real_angle 期望的角度
 */
void dji_out(float real_angle) {
    static float angle_out[2] = {0.0, 0.0};
    static float speed_out[2] = {0.0, 0.0};
    angle_out[0] =
        pid_calc(&motor1.pid_pos, real_angle, dji_motor_1.rotor_degree);
    speed_out[0] =
        pid_calc(&motor1.pid_spd, angle_out[0], dji_motor_1.speed_rpm);
    angle_out[1] =
        pid_calc(&motor2.pid_pos, real_angle, dji_motor_2.rotor_degree);
    speed_out[1] =
        pid_calc(&motor2.pid_spd, angle_out[1], dji_motor_2.speed_rpm);
    dji_motor_set_current(can1_selected, DJI_MOTOR_GROUP1,
                          (int16_t)speed_out[0], (int16_t)speed_out[1], 0.0,
                          0.0);
}

/**
 * @brief AK电机控制
 * 
 * @param pvParameters pvParameters
 */
void task_AK80ctrl(void *pvParameters) {
    UNUSED(pvParameters);
    ak_motor_init(&ak_motor_1, CANid_AK_1, AK80_8, AK_MODE_SERVO,
                  can1_selected);
    ak_motor_init(&ak_motor_2, CANid_AK_2, AK80_8, AK_MODE_SERVO,
                  can1_selected);

    int16_t ak_flag = 0;

    while (1) {
        xQueueReceive(Queue_From_Fir, &ak_flag, 1);
        if (ak_flag) { /* 电机向下压 */
            ak_servo_set_pos_spd(&ak_motor_1, ak_pushdown, 327000, 327000);
            ak_servo_set_pos_spd(&ak_motor_2, (-ak_pushdown), 327000, 327000);
        } else {
            ak_servo_set_pos_spd(&ak_motor_1, 0, 327000, 327000);
            ak_servo_set_pos_spd(&ak_motor_2, 0, 327000, 327000);
        }
    }
}

/**
 * @brief 大疆电机控制任务
 * 
 * @param pvParameters pvParameters
 */
void task_djictrl(void *pvParameters) {
    UNUSED(pvParameters);
    dji_motor_init(&dji_motor_1, DJI_M3508, CAN_Motor1_ID, can1_selected);
    dji_motor_init(&dji_motor_2, DJI_M3508, CAN_Motor2_ID, can1_selected);
    pid_init(&motor1.pid_pos, 16384, 5000, 1.0, 8000, POSITION_PID, 30.0f,
             0.02f, 0.5f); /* 电机1的速度和角度pid */
    pid_init(&motor1.pid_spd, 8192, 8192, 30, 8000, POSITION_PID, 5.0f, 0.01f,
             0.5f);

    pid_init(&motor2.pid_pos, 16384, 5000, 1.0, 8000, POSITION_PID, 8.0f,
             0.001f, 0.0f); /* 电机2的速度和角度pid */
    pid_init(&motor2.pid_spd, 8192, 8192, 30, 8000, POSITION_PID, 5.0f, 0.001f,
             0.2f);
    int16_t ak_flag = 0;
    while (1) {
        xQueueReceive(Queue_From_Fir, &ak_flag, 1);
        if (ak_flag) {
            dji_out(dji_pushdown); /* 向下压过程 */
        } else {
            dji_out(0.0);
        }
    }
}