// USER/mode_choose.h
#ifndef __MODE_CHOOSE_H
#define __MODE_CHOOSE_H

#include "AllHeader.h"

typedef enum {
    MODE_HMI,   // Controlled by Human-Machine Interface (USART1)
    MODE_ROS    // Controlled by ROS (USART3)
} ControlMode;

void mode_init(void);
void set_control_mode(ControlMode mode);
ControlMode get_control_mode(void);

#endif /* __MODE_CHOOSE_H */ 
