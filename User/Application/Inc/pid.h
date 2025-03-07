/**
 * @file    pid.h
 * @author  Deadline039
 * @brief   pid类封装
 * @version 0.1
 * @date    2023-10-27
 */
#ifndef __PID_H
#define __PID_H

#include "stdint.h"


/**
 * @brief PID类型, 位置PID或者增量PID
 */
typedef enum {
    POSITION_PID = 0x00U,
    DELTA_PID
} pid_mode_t;

/**
 * @brief PID, 可以为速度环, 也可以为角度环
 *
 */
typedef struct {
    float kp, ki, kd; /* pid三参数 */

    float set[3]; /* 目标值, 包含本次, 上次, 上上次 */
    float get[3]; /* 测量值, 包含本次, 上次, 上上次 */
    float err[3]; /* 差值, 包含本次, 上次, 上上次 */

    float pout, iout, dout; /* pid输出 */

    pid_mode_t pid_mode; /* PID模式 */

    /* 位置模式 */
    float pos_out;     /* 本次输出 */
    float pos_lastout; /* 上次输出 */

    /* 增量模式 */
    float delta_u;       /* 本次增量值 */
    float delta_out;     /* 本次增量输出 = delta_lastout + delta_u */
    float delta_lastout; /* 上次增量输出 */

    float max_output;     /* 输出限幅 */
    float integral_limit; /* 积分限幅 */
    float deadband;       /* 死区(绝对值) */
    float max_error;      /* 最大误差 */
    
    float angle_err;
    float angle_err_err;

    /*这三个个参数均为角度环计算电机角度（角度制）的参数*/

} pid_t;

void pid_init(pid_t *pid, uint16_t maxout_p, uint16_t intergralLim_p,
              float deadband_p, uint16_t maxerr_p, pid_mode_t pid_mode_p,
              float kp_p, float ki_p, float kd_p);
void pid_reset(pid_t *pid, float kp_p, float ki_p, float kd_p);
float pid_calc(pid_t *pid, float target_p, float measure_p);

#endif 
