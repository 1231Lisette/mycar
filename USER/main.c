#include "AllHeader.h"

/**
 * @brief  Main program.
 * @retval None
 */
int main(void)
{
	// Initializes all peripherals and modules
	master_init();

	// Initialize control mode to HMI by default
	mode_init();
	
	// Enable encoder data upload for precise turning
	send_upload_data(true, false, false);
	delay_ms(100);
	
	while (1)
	{
		// Process encoder data if available
		if (g_recv_flag) {
			Deal_data_real();
			g_recv_flag = 0;
		}
		
		
		// Poll for commands from HMI (USART1) and ROS (USART3)
		hmi_command_poll();
		ros_command_poll();
		
		// A heartbeat LED can be placed here to indicate the system is running.
		// For example, every 500000 main loop cycles, toggle the LED.
		static int loop_count = 0;
		if(++loop_count > 500000) {
			loop_count = 0;
			// LED_G_Tog; // You can uncomment this if you have a status LED
		}
	}
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif
