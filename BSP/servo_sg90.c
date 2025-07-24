// BSP/servo_sg90.c
#include "servo_sg90.h"

// SG90 Servo PWM configuration
// Timer Clock: 72MHz
// Prescaler: 71 -> Timer Freq = 72,000,000 / (71 + 1) = 1,000,000 Hz = 1MHz
// Period (ARR): 19999 -> PWM Freq = 1,000,000 / (19999 + 1) = 50 Hz (20ms)
// Pulse Width (CCR):
// 0.5ms -> 500 counts (0 degrees)
// 2.5ms -> 2500 counts (180 degrees)
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

/**
 * @brief  Initializes TIM4 for dual Servo control on PB8 and PB9.
 * @note   Configures TIM4_CH3 and TIM4_CH4 for PWM output.
 * @retval None
 */
void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // Enable GPIOB and TIM4 clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // Configure PB8 (TIM4_CH3) and PB9 (TIM4_CH4) as AF push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Time base configuration for TIM4
    TIM_TimeBaseStructure.TIM_Period = 19999; // 20ms period
    TIM_TimeBaseStructure.TIM_Prescaler = 71; // 1MHz clock
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM1 Mode configuration: Channel 3
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // Initial pulse
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // PWM1 Mode configuration: Channel 4
    TIM_OCInitStructure.TIM_Pulse = 0; // Initial pulse
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    // TIM4 enable counter
    TIM_Cmd(TIM4, ENABLE);
}

/**
 * @brief  Sets the angle of a specific servo.
 * @param  servo_id: 1 for Servo1 (PB8), 2 for Servo2 (PB9).
 * @param  angle: The desired angle (0-180 degrees).
 * @retval None
 */
void Servo_Set_Angle(uint8_t servo_id, uint8_t angle)
{
    uint32_t pulse;

    // Constrain angle to 0-180
    if (angle > 180) {
        angle = 180;
    }

    // Convert angle to pulse width
    pulse = SERVO_MIN_PULSE + (uint32_t)((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180.0);

    if (servo_id == 1) {
        TIM_SetCompare3(TIM4, pulse);
    } else if (servo_id == 2) {
        TIM_SetCompare4(TIM4, pulse);
    }
} 