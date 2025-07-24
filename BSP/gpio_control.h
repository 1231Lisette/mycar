// BSP/gpio_control.h
#ifndef __GPIO_CONTROL_H
#define __GPIO_CONTROL_H

#include "AllHeader.h"

void LED_Init(void);
void Buzzer_Init(void);
void Control_LED(uint8_t state);
void Control_Buzzer(uint8_t state);

void Laser_Init(void);
void Control_Laser(uint8_t state);

#endif /* __GPIO_CONTROL_H */ 