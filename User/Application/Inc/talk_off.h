/**
 * @file    take_off.c
 * @brief   底盘起跳的主要控制逻辑封装
 * @version 0.1
 * @date    2025-03-09
 */
#ifndef __TALK_OFF_H
#define __TALK_OFF_H

typedef struct motor_pid {
    pid_t pid_pos;
    pid_t pid_spd;
} dji_pid_t;   



#endif 
