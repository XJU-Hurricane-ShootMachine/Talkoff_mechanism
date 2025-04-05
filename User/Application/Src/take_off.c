/**
 * @file    take_off.c
 * @brief   底盘起跳的主要控制逻辑实现
 * @version 0.2
 * @date    2025-03-05
 */

#include <includes.h>
#include "remote_ctrl.h"
#include "talk_off.h"

#define CANid_AK_1   104 /* CANid需要在上位机上查看 */
#define CANid_AK_2   105

#define dji_pushdown 90.0   /* 电机下压距离 角度 */
#define ak_pushdown  72000U /* 300圈 */

ak_motor_handle_t ak_motor_1;
ak_motor_handle_t ak_motor_2;
dji_motor_handle_t dji_motor_1;
dji_motor_handle_t dji_motor_2;

dji_pid_t motor2; /* dji电机1 */
dji_pid_t motor1;

int16_t angle_gap = 0;

/**
 * @brief 两个电机轮流转动至指定角度
 *
 * @param motor1
 * @param motor2
 * @param pos1
 * @param pos2
 * @note 通过轮流在前发送消息，减少两个电机之间的角度差
 */
void ak_set_pos_spd(ak_motor_handle_t *motor1, ak_motor_handle_t *motor2,
                    float pos1, float pos2) {

    static uint8_t falg = 0;
    if (falg) {
        ak_servo_set_pos_spd(motor1, pos1, 327670, 327000);
        ak_servo_set_pos_spd(motor2, pos2, 327670, 327000);
        falg = 0;
    } else {
        ak_servo_set_pos_spd(motor2, pos2, 327670, 327000);
        ak_servo_set_pos_spd(motor1, pos1, 327670, 327000);
        falg = 1;
    }
}

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
 * @brief AK电机控制 可以转动超过99圈
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
    int32_t ak_agler = ak_pushdown % 35640; /* 圈数较99的余读数  */
    uint8_t ak_push_flag = 0; /* 按压标志位，在压下之后才会进行300圈回正 */
    while (1) {
        xQueueReceive(Queue_From_Fir, &ak_flag, 1);

        angle_gap = (int16_t)ak_motor_1.pos + (int16_t)ak_motor_2.pos;

        UNUSED(
            angle_gap); /* 避免优化，因为上文这个变量只赋值了没有使用，编译器会优化掉 */

        if (ak_flag) { /* 电机向下压 */
            if (ak_push_flag == 0) {
                ak_agler = ak_pushdown; /* 当回零300圈之后 重新给角度赋值 */
            }
            if (ak_agler <
                35640) { /* 因为任务时刻在运行，所以要以小角度保持一直转动 */

                angle_gap = (int16_t)ak_motor_1.pos + (int16_t)ak_motor_2.pos;
                ak_servo_set_pos_spd(&ak_motor_2, -ak_agler, 2164, 32760);
                ak_servo_set_pos_spd(&ak_motor_1, ak_agler, 2164,
                                     32760); /* 21690 */
                vTaskDelay(10);
            } else {
                for (uint8_t i = 0; i < ak_pushdown / 35640;
                     i++) { /* 在for循环里转完所有的 99 圈 */
                    do {
                        angle_gap =
                            (int16_t)ak_motor_1.pos + (int16_t)ak_motor_2.pos;

                        ak_servo_set_pos_spd(&ak_motor_1, 35640, 2164, 327000);
                        ak_servo_set_pos_spd(&ak_motor_2, -35640, 2164, 327000);
                        vTaskDelay(10);
                    } while (ak_motor_1.spd > 200 ||
                             (int16_t)ak_motor_1.pos != 3200);
                    do {
                        ak_servo_set_origin(
                            &ak_motor_1,
                            AK_ORIGIN_TEMPORARY); /* 设置临时原点 */
                        ak_servo_set_origin(&ak_motor_2, AK_ORIGIN_TEMPORARY);
                        vTaskDelay(30);
                    } while (ak_motor_1.pos != 0);
                }
                ak_agler = ak_pushdown % 35640; /* 这里再将值赋为余数 */
            }
            ak_push_flag = 1; /* 将标志位赋1，可以进行300圈回正 */
        }
        /* 按下回拉 不进行按压 这里角度为余数 不会进行300圈回零*/
        else {
            if (ak_push_flag) {
                ak_agler = ak_pushdown;
            }
            if (ak_agler < 35640) { /* 不满99圈直接归零 */
                angle_gap = (int16_t)ak_motor_1.pos + (int16_t)ak_motor_2.pos;
                ak_servo_set_pos_spd(&ak_motor_2, 0, 2164, 32760);
                ak_servo_set_pos_spd(&ak_motor_1, 0, 2164, 32760); /* 21690 */
                vTaskDelay(10);
            } else {
                /* 在这里先将小角度归零 */
                while ((int32_t)ak_motor_1.pos != 0) {
                    angle_gap =
                        (int16_t)ak_motor_1.pos + (int16_t)ak_motor_2.pos;

                    ak_servo_set_pos_spd(&ak_motor_2, 0, 2164, 32760);
                    ak_servo_set_pos_spd(&ak_motor_1, 0, 2164, 32760);
                    vTaskDelay(10);
                }
                for (uint8_t i = 0; i < ak_pushdown / 35640;
                     i++) { /* 在for循环里转完所有的 99 圈 */
                    do {
                        angle_gap =
                            (int16_t)ak_motor_1.pos + (int16_t)ak_motor_2.pos;

                        ak_servo_set_pos_spd(&ak_motor_1, -35640, 2164, 327000);
                        ak_servo_set_pos_spd(&ak_motor_2, 35640, 2164, 327000);
                        vTaskDelay(10);
                    } while (ak_motor_1.spd < -200 ||
                             (int16_t)ak_motor_1.pos != -3200);
                    do {
                        ak_servo_set_origin(
                            &ak_motor_1,
                            AK_ORIGIN_TEMPORARY); /* 设置临时原点 */
                        ak_servo_set_origin(&ak_motor_2, AK_ORIGIN_TEMPORARY);
                        vTaskDelay(30);
                    } while ((int16_t)ak_motor_1.pos != 0);
                }
                ak_agler = ak_pushdown % 35640;
            }
            ak_push_flag = 0;
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
    pid_init(&motor2.pid_pos, 16384, 5000, 1.0, 8000, POSITION_PID, 30.0f,
             0.001f, 0.0f); /* 电机2的速度和角度pid */
    pid_init(&motor2.pid_spd, 8192, 8192, 30, 8000, POSITION_PID, 5.0f, 0.01f,
             0.2f);
    int16_t dji_flag = 0;
    while (1) {
        xQueueReceive(Queue_From_Fir, &dji_flag, 1);
        if (dji_flag) {
            dji_out(dji_pushdown); /* 向下压过程 */
        } else {
            dji_out(0.0);
        }
    }
}