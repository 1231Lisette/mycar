// BSP/gpio_control.c
#include "gpio_control.h"

/**
 * @brief  Initializes the LED on pin PB0.
 * @note   Configures PB0 as a push-pull output.
 * @retval None
 */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIOB clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure PB0 as push-pull output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Turn off LED by default
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

/**
 * @brief  Initializes the Buzzer on pins PB5 and PB6.
 * @note   Configures PB6 as push-pull output (for signal) and
 *         PB5 as push-pull output (to act as GND).
 * @retval None
 */
void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIOB clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure PB6 as push-pull output for buzzer signal
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure PB5 as push-pull output for buzzer GND
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Set PB5 low to act as GND
    GPIO_ResetBits(GPIOB, GPIO_Pin_5);
    // Turn off buzzer by default
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
}

/**
 * @brief  Controls the LED.
 * @param  state: 0 to turn off, 1 to turn on.
 * @retval None
 */
void Control_LED(uint8_t state)
{
    if (state)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    }
}

/**
 * @brief  Controls the Buzzer.
 * @param  state: 0 to turn off, 1 to turn on.
 * @retval None
 */
void Control_Buzzer(uint8_t state)
{
    if (state)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_6); // High level to trigger
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    }
} 

/**
 * @brief  Initializes the Laser on pin PB1.
 * @note   Configures PB1 as a push-pull output.
 * @retval None
 */
void Laser_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIOB clock, it should be already enabled by LED/Buzzer init, but good practice to ensure.
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure PB1 as push-pull output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Turn off Laser by default
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
}

/**
 * @brief  Controls the Laser.
 * @param  state: 0 to turn off, 1 to turn on.
 * @retval None
 */
void Control_Laser(uint8_t state)
{
    if (state)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    }
} 