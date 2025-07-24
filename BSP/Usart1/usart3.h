#ifndef __USART3_H
#define __USART3_H

#include "stm32f10x.h"
#include "pc_command_parser.h" // Include the new parser
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "usart.h"

void uart3_init(uint32_t bound);
void USART3_Send_Byte(uint8_t ch);
void USART3_Send_Array(uint8_t *BufferPtr, uint16_t Length);
void USART3_Send_String(char *str);
void USART3_Receive_Data(void *buffer, uint16_t length);
void USART3_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
uint8_t USART3_Receive_Byte(void);
void USART3_IRQHandler(void);
// 新增非阻塞函数声明
uint8_t USART3_Is_Data_Available(void);
uint8_t USART3_Try_Receive_Byte(uint8_t* p_ch);
#endif /* __USART3_H */ 
