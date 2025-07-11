#ifndef __USART_H
#define __USART_H
#include "AllHeader.h"



void uart_init(u32 bound);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void uart_init(u32 bound);
void uart3_init(u32 bound);  // 添加UART3初始化声明
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void USART3_Send_U8(uint8_t ch);  // 添加UART3发送单字节声明
void USART3_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);  // 添加UART3发送数组声明

#endif


