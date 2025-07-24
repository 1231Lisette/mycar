// USER/init.c
#include "init.h"

/**
 * @brief  Master initialization function.
 * @note   This function calls all necessary hardware and module initializations.
 * @retval None
 */
void master_init(void)
{
    // Initialize basic system components (like NVIC priority groups)
    bsp_init();
    
    // Initialize delay functions
    delay_init();
    
    // Initialize USART1 for HMI/Debug communication
    uart_init(115200);
    printf("USART1 (HMI/Debug) initialized.\r\n");

    // Initialize USART2 for Motor Driver communication
    Motor_Usart_init();
    printf("USART2 (Motor Driver) initialized.\r\n");

    // Initialize USART3 for ROS communication
    uart3_init(115200);
    printf("USART3 (ROS) initialized.\r\n");
    
    // Initialize Timer for system tick
    TIM3_Init();
    printf("TIM3 (System Tick) initialized.\r\n");

    // Initialize LED and Buzzer
    LED_Init();
    printf("LED initialized on PB0.\r\n");
    Buzzer_Init();
    printf("Buzzer initialized on PB5 (GND) and PB6 (Signal).\r\n");
    Laser_Init();
    printf("Laser initialized on PB1.\r\n");
    Servo_Init();
    printf("Servos initialized on TIM4 (PB8, PB9).\r\n");

    // Perform initial motor setup
    // Set motor to stop state
    Contrl_Pwm(0, 0, 0, 0);
    delay_ms(100);
    
    // Disable data upload by default
    send_upload_data(false, false, false);
    delay_ms(10);
    
    // Set motor parameters (assuming L type 520 motor)
    send_motor_type(1);
    delay_ms(100);
    send_pulse_phase(40);
    delay_ms(100);
    send_pulse_line(11);
    delay_ms(100);
    send_wheel_diameter(67.00);
    delay_ms(100);
    send_motor_deadzone(1900);
    delay_ms(100);

    printf("All initializations complete. System starting...\r\n");
} 
