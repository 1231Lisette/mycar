#include "usart3.h"


// 用于调试输出
#define DEBUG_TO_USART1 1

// Ring buffer for USART3 RX data
#define USART3_RX_BUFFER_SIZE 256
static volatile uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
static volatile uint16_t usart3_rx_head = 0;
static volatile uint16_t usart3_rx_tail = 0;

/**
 * @brief  Put a byte into the ring buffer (called from ISR)
 */
static inline void USART3_RxBuffer_Write(uint8_t ch)
{
    uint16_t next_head = (usart3_rx_head + 1) % USART3_RX_BUFFER_SIZE;
    if(next_head != usart3_rx_tail)  // ensure no overflow
    {
        usart3_rx_buffer[usart3_rx_head] = ch;
        usart3_rx_head = next_head;
    }
    // else overflow, byte lost
}

/**
 * @brief  Check if there is data available in the ring buffer.
 * @retval 1 if data is available, 0 otherwise.
 */
uint8_t USART3_Is_Data_Available(void)
{
    return (usart3_rx_head != usart3_rx_tail);
}

/**
 * @brief  Try to get a byte from the ring buffer (non-blocking)
 * @param  p_ch: Pointer to a variable to store the received byte.
 * @retval 1 if a byte was received, 0 if buffer was empty.
 */
uint8_t USART3_Try_Receive_Byte(uint8_t* p_ch)
{
    if(usart3_rx_head != usart3_rx_tail)
    {
        *p_ch = usart3_rx_buffer[usart3_rx_tail];
        usart3_rx_tail = (usart3_rx_tail + 1) % USART3_RX_BUFFER_SIZE;
        return 1;
    }
    return 0;
}

/**
 * @brief  Get a byte from the ring buffer (blocking)
 * @retval Received byte
 */
uint8_t USART3_Receive_Byte(void)
{
    // Wait until data is available
    while(usart3_rx_head == usart3_rx_tail);
    uint8_t ch = usart3_rx_buffer[usart3_rx_tail];
    usart3_rx_tail = (usart3_rx_tail + 1) % USART3_RX_BUFFER_SIZE;
    return ch;
}

/* 兼容uint8_t数组发送的封装，等价于 USART3_Send_Array */
void USART3_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
    USART3_Send_Array(BufferPtr, Length);
}

void uart3_init(uint32_t bound)
{
    // GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // USART3_TX   PB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3_RX   PB.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART 初始化设置
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
    
    #if DEBUG_TO_USART1
    printf("USART3 initialized at %d baud\r\n", (int)bound);
    #endif
}

/**
 * @Brief: UART3发送单个字节
 * @Note: 
 * @Parm: ch: 待发送的数据
 * @Retval: None
 */
void USART3_Send_Byte(uint8_t ch)
{
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    USART_SendData(USART3, ch);
}

/**
 * @Brief: UART3发送数组
 * @Note: 
 * @Parm: BufferPtr: 待发送的数据指针, Length: 数据长度
 * @Retval: None
 */
void USART3_Send_Array(uint8_t *BufferPtr, uint16_t Length)
{
    while(Length--)
    {
        USART3_Send_Byte(*BufferPtr);
        BufferPtr++;
    }
}

/**
 * @Brief: UART3发送字符串
 * @Note: 
 * @Parm: str: 待发送的字符串
 * @Retval: None
 */
void USART3_Send_String(char *str)
{
    while(*str)
    {
        USART3_Send_Byte(*str++);
    }
}

/**
 * @brief  UART3 接收指定长度数据（阻塞式）
 * @param  buffer: 数据缓存指针
 * @param  length: 需要接收的字节数
 */
void USART3_Receive_Data(void *buffer, uint16_t length)
{
    uint8_t *ptr = (uint8_t *)buffer;
    while(length--)
    {
        *ptr++ = USART3_Receive_Byte();
    }
}

/**
 * @brief  USART3中断服务程序 (修改后版本)
 * @note   此版本通过查找帧头'S'和固定长度来同步数据，更加健壮
 */
void USART3_IRQHandler(void)
{
	// 使用在pc_command_parser.h中定义的帧长度
	static uint8_t rx_buffer[RPI_FRAME_LEN];
	static uint8_t rx_count = 0;

	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
    {
		uint8_t ch = USART_ReceiveData(USART3);

		// 如果收到了帧头 'S'，则从头开始接收
		if (ch == 'S') 
        {
			rx_count = 0;
			rx_buffer[rx_count++] = ch;
		} 
        // 如果已经收到了帧头，则继续接收后续字节
        else if (rx_count > 0 && rx_count < RPI_FRAME_LEN) 
        {
			rx_buffer[rx_count++] = ch;

			// 当接收的字节数达到帧长度时
			if (rx_count == RPI_FRAME_LEN) 
            {
				// (可选但推荐) 最后检查一次帧尾是否为 '\n'
				if (rx_buffer[RPI_FRAME_LEN - 1] == '\n') 
                {
					// 数据帧看起来是完整的，调用回调函数进行处理
					USART3_Rx_Callback(rx_buffer);
				}
				// 无论帧是否有效，都重置计数器，准备接收下一个新帧
				rx_count = 0; 
			}
		}
		
        // 清除中断挂起位
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}
