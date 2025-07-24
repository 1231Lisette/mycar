#ifndef __PID_CONFIG_H
#define __PID_CONFIG_H

#include "stm32f10x.h"

// PID参数结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float max_output;   // 输出限幅
    float max_integral; // 积分限幅
} PIDParams;

// PID控制器结构体
typedef struct {
    PIDParams params;   // PID参数
    float target;       // 目标值
    float actual;       // 实际值
    float error;        // 误差
    float last_error;   // 上次误差
    float integral;     // 积分项
    float output;       // 输出值
} PIDController;

// 函数声明
void PID_Init(PIDController *pid);
void PID_SetParams(PIDController *pid, float kp, float ki, float kd, float max_output, float max_integral);
void PID_Reset(PIDController *pid);
float PID_Calculate(PIDController *pid, float target, float actual);
void PID_SaveParams(void);
void PID_LoadParams(void);

#endif /* __PID_CONFIG_H */ 
