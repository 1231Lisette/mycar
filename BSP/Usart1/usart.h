#ifndef __USART_H
#define __USART_H
#include "AllHeader.h"

// USART1 (Debug/Log Output)
void uart_init(u32 bound);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void USART1_IRQHandler(void);
void hmi_command_poll(void); // New poll function for HMI commands

// USART3 (ROS Communication)
void uart3_init(u32 bound);
void USART3_Send_U8(uint8_t ch);
void USART3_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void USART3_IRQHandler(void);

// Shared Buffers/Flags
extern volatile uint8_t dbg_cmd_ready;   // 1 indicates a full command is ready to be parsed
extern char dbg_cmd_buf[50];
extern uint8_t ros_rx_buffer[128];
extern uint8_t ros_rx_index;

#endif



