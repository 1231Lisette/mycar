#include "usart.h"	  
/* usart.c */
volatile uint8_t dbg_cmd_ready = 0;
volatile  uint8_t test_flag;
char dbg_cmd_buf[50];
//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	
//Add the following code to support the printf function without selecting use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数   Support functions required by the standard library
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式  Define _sys_exit() to avoid using semihosting mode
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 Redefine fputc function
int fputc(int ch, FILE *f)
{      
	  while((USART1->SR&0X40)==0);
		USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/**************************************************************************
Function: Serial port 1 initialization
Input   : bound：Baud rate
Output  : none
函数功能：串口1初始化
入口参数：bound：波特率
返回  值：无
**************************************************************************/
     
void uart_init(u32 bound)//usart1
{
  //GPIO端口设置	 GPIO port settings
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟	Enable USART1, GPIOA clock
  
		//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出	Multiplexed push-pull output
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9	Initialize GPIOA.9
	   
	  //USART1_RX	  GPIOA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入	Floating Input
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  Initialize GPIOA.10
   
  //USART 初始化设置	USART initialization settings
	USART_InitStructure.USART_BaudRate = bound;//串口波特率	Serial port baud rate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式	The word length is 8-bit data format
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位	One stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位	No parity bit
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制	No hardware flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式	Transceiver mode

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
 
	USART_Init(USART1, &USART_InitStructure); //初始化串口1	Initialize serial port 1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//串口接受中断	Disable serial port receive interrupt
	USART_Cmd(USART1, ENABLE);                    //使能串口1 	Enable serial port 1

}

/**
 * @Brief: UART1发送数据		UART1 sends data
 * @Note: 
 * @Parm: ch:待发送的数据 	ch: data to be sent
 * @Retval: 
 */
void USART1_Send_U8(uint8_t ch)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, ch);
}

/**
 * @Brief: UART1发送数据		UART1 sends data
 * @Note: 
 * @Parm: BufferPtr:待发送的数据  Length:数据长度		BufferPtr: data to be sent Length: data length
 * @Retval: 
 */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		USART1_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}


static char hmi_cmd_buffer[50];
static volatile uint8_t hmi_cmd_ready = 0;

void parse_hmi_command(void); // Forward declaration

/**
 * @brief  USART1 Interrupt Service Routine.
 * @note   Handles commands from HMI/PC Serial Assistant.
 *         It only receives bytes, forms a command string, and sets a flag.
 *         Actual command parsing is done in the main loop poll function.
 */
void USART1_IRQHandler(void)
{
	uint16_t usart_sr = USART1->SR;
	uint8_t usart_dr = USART1->DR;

	if ((usart_sr & USART_FLAG_RXNE) != RESET) {
		char ch = (char)usart_dr;
		static uint8_t idx = 0;

		// Ignore line endings, good for various serial terminals
		if (ch == '\r' || ch == '\n') return; 

		if (ch == '$') {
			idx = 0;
			memset(hmi_cmd_buffer, 0, sizeof(hmi_cmd_buffer));
		} else if (ch == '#') {
			if (idx > 0) { // Ensure command is not empty
				hmi_cmd_buffer[idx] = '\0';
				hmi_cmd_ready = 1; // Set flag to be processed in main loop
			}
			idx = 0;
		} else if (idx < sizeof(hmi_cmd_buffer) - 1) {
			hmi_cmd_buffer[idx++] = ch;
		}
	}
}

/**
 * @brief  Polls and parses commands received from the HMI (USART1).
 * @note   This function should be called repeatedly in the main loop.
 */
void hmi_command_poll(void)
{
    if (!hmi_cmd_ready) return;
    hmi_cmd_ready = 0; // Clear the flag immediately

    // Handle Mode Switch commands: $MODE:ROS# or $MODE:HMI#
    if (strncmp(hmi_cmd_buffer, "MODE:", 5) == 0) {
        if (strcmp(hmi_cmd_buffer + 5, "ROS") == 0) {
            set_control_mode(MODE_ROS);
        } else if (strcmp(hmi_cmd_buffer + 5, "HMI") == 0) {
            set_control_mode(MODE_HMI);
        }
    }
    // Handle Manual Movement commands (only if in HMI mode)
    // Format: $MOVE:FWD,100# $MOVE:STOP#
    else if (get_control_mode() == MODE_HMI && strncmp(hmi_cmd_buffer, "MOVE:", 5) == 0) 
	{
        char* cmd = hmi_cmd_buffer + 5;
		
        if (strncmp(cmd, "FWD,", 4) == 0) 
		{
            motor_forward(atoi(cmd + 4));
        } 
		else if (strncmp(cmd, "BWD,", 4) == 0) 
		{
            motor_backward(atoi(cmd + 4));
        } 
		else if (strncmp(cmd, "LFT,", 4) == 0) 
		{
            int speed1 = 0, speed2 = 0;
            // Use sscanf for safer parsing
            if (sscanf(cmd + 4, "%d,%d", &speed1, &speed2) == 2) {
			    motor_turn_left(speed1, speed2);
            }
        } 
		else if (strncmp(cmd, "RGT,", 4) == 0) 
		{
            int speed1 = 0, speed2 = 0;
            // Use sscanf for safer parsing
            if (sscanf(cmd + 4, "%d,%d", &speed1, &speed2) == 2) {
			    motor_turn_right(speed1, speed2);
            }
        } 
		else if (strcmp(cmd, "STOP") == 0) 
		{
            motor_stop();
        }
    }
    // Handle Precise Turn commands: $TURN:LEFT90# $TURN:RIGHT90# $TURN:CANCEL#
    else if (get_control_mode() == MODE_HMI && strncmp(hmi_cmd_buffer, "TURN:", 5) == 0) 
	{
        char* cmd = hmi_cmd_buffer + 5;
		
        if (strcmp(cmd, "LEFT90") == 0) 
		{
            motor_turn_left_90_precise();
        } 
		else if (strcmp(cmd, "RIGHT90") == 0) 
		{
            motor_turn_right_90_precise();
        } 
		else if (strcmp(cmd, "CANCEL") == 0) 
		{
            motor_cancel_turn();
        }
    }
    // Handle PID parameter adjustment: $PID:2.0,0.1,0.05#
    else if (get_control_mode() == MODE_HMI && strncmp(hmi_cmd_buffer, "PID:", 4) == 0) 
	{
        char* cmd = hmi_cmd_buffer + 4;
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;
        
        // Use sscanf for safer parsing
        if (sscanf(cmd, "%f,%f,%f", &kp, &ki, &kd) == 3) {
            motor_set_turn_pid_params(kp, ki, kd);
        } else {
            printf("> ERROR: Invalid PID format. Use: $PID:kp,ki,kd#\r\n");
        }
    }
}
