/**
 * @file    pid.c
 * @author  Deadline039
 * @brief   pid类实现
 * @version 1.0
 * @date    2023-10-27
 */

#include "pid.h"

#include "my_math.h"
#include "stdint.h"

/**
 * @brief PID状态记录
 */
enum {
    LLAST = 0, /*!< 上上次 */
    LAST = 1,  /*!< 上次 */
    NOW = 2    /*!< 本次 */
};

/**
 * @brief 死区限制
 *
 * @param a 传入的值
 * @param abs_max 限制值
 */
static inline void abs_limit(float *a, float abs_max) {
    if (*a > abs_max) {
        *a = abs_max;
    }
    if (*a < -abs_max) {
        *a = -abs_max;
    }
}

/**
 * @brief PID初始化
 *
 * @param pid PID结构体指针
 * @param maxout_p 输出限幅
 * @param intergralLim_p 积分限幅
 * @param deadband_p 死区, PID计算的最小误差
 * @param maxerr_p 最大误差
 * @param pid_mode_p PID模式
 *  @arg `POSITION_PID`, 位置式PID;
 *  @arg `DELTA_PID`, 增量式PID
 * @param kp_p P参数
 * @param ki_p I参数
 * @param kd_p D参数
 */
void pid_init(pid_t *pid, uint16_t maxout_p, uint16_t intergralLim_p,
              float deadband_p, uint16_t maxerr_p, pid_mode_t pid_mode_p,
              float kp_p, float ki_p, float kd_p) {
    pid->max_output = maxout_p;
    pid->integral_limit = intergralLim_p;
    pid->deadband = deadband_p;
    pid->max_error = maxerr_p;
    pid->pid_mode = pid_mode_p;

    pid->kp = kp_p;
    pid->ki = ki_p;
    pid->kd = kd_p;
    pid->pos_out = 0;
    pid->delta_out = 0;
}

/**
 * @brief PID参数调整
 *
 * @param pid PID结构体指针
 * @param kp_p P参数
 * @param ki_p I参数
 * @param kd_p D参数
 */
void pid_reset(pid_t *pid, float kp_p, float ki_p, float kd_p) {
    pid->kp = kp_p;
    pid->ki = ki_p;
    pid->kd = kd_p;
}

/**
 * @brief PID计算
 *
 * @param pid PID结构体指针
 * @param target_p 目标值
 * @param measure_p 电机测量值
 * @return float PID计算的结果
 */
float pid_calc(pid_t *pid, float target_p, float measure_p) {
    pid->get[NOW] = measure_p;
    pid->set[NOW] = target_p;
    pid->err[NOW] = target_p - measure_p;

    if ((math_compare_float(pid->max_error, 0.0) != MATH_FP_EQUATION) &&
        my_abs(pid->err[NOW]) > pid->max_error) {
        return 0.0;
    }

    if ((math_compare_float(pid->deadband, 0.0) &&
         my_abs(pid->err[NOW]) < pid->deadband)) {
        return 0.0;
    }

    if (pid->pid_mode == POSITION_PID) {
        /* 位置式PID */
        pid->pout = pid->kp * pid->err[NOW];
        pid->iout += pid->ki * pid->err[NOW];
        pid->dout = pid->kd * (pid->err[NOW] - pid->err[LAST]);
        abs_limit(&pid->iout, pid->integral_limit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&pid->pos_out, pid->max_output);
        pid->pos_lastout = pid->pos_out;
    } else if (pid->pid_mode == DELTA_PID) {
        /* 增量式PID */
        pid->pout = pid->kp * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->ki * pid->err[NOW];
        pid->dout =
            pid->kd * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);
        abs_limit(&pid->iout, pid->integral_limit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->delta_lastout + pid->delta_u;
        abs_limit(&pid->delta_out, pid->max_output);
        pid->delta_lastout = pid->delta_out;
    }

    /* 状态转移 */
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];

    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];

    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}



