#ifndef __USART_H
#define __USART_H
#include "AllHeader.h"



void uart_init(u32 bound);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void uart_init(u32 bound);
void uart3_init(u32 bound);  // ���UART3��ʼ������
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void USART3_Send_U8(uint8_t ch);  // ���UART3���͵��ֽ�����
void USART3_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);  // ���UART3������������

#endif


