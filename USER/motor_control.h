// USER/motor_control.h
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "AllHeader.h"

// 转向状态枚举
typedef enum {
    TURN_IDLE,
    TURN_LEFT_90,
    TURN_RIGHT_90
} TurnState;

// 基础运动控制函数
void motor_forward(int speed);
void motor_backward(int speed);
void motor_turn_left(int speed1, int speed2);
void motor_turn_right(int speed1, int speed2);
void motor_stop(void);

// 精确转向控制函数
void motor_turn_left_90_precise(void);
void motor_turn_right_90_precise(void);
uint8_t motor_update_precise_turn(void);
TurnState motor_get_turn_state(void);
void motor_cancel_turn(void);

// PID参数设置函数
void motor_set_turn_pid_params(float kp, float ki, float kd);

#endif /* __MOTOR_CONTROL_H */

