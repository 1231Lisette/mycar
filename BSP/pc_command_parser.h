// BSP/pc_command_parser.h
#ifndef __PC_COMMAND_PARSER_H
#define __PC_COMMAND_PARSER_H

#include "AllHeader.h"
#include "stm32f10x.h"

/**
 * @brief 定义从树莓派接收的数据帧的准确长度
 * S(1)+servo1(1)+servo2(1)+L(1)+led(1)+A(1)+laser(1)+V(1)+4*vel(8)+R(1)+buzzer(1)+\n(1) = 19 字节
 */
#define RPI_FRAME_LEN 19

/**
 * @brief 轮询并解析来自ROS(树莓派)的指令
 * @note  此函数应在主循环中被反复调用。
 * 在main.c中已有名为ros_command_poll的调用，故函数名保持一致。
 */
void ros_command_poll(void);

/**
 * @brief USART3中断服务程序的回调函数
 * @note  当接收到完整的一帧数据后，此函数被调用
 * @param buffer: 指向接收数据缓冲区的指针
 */
void USART3_Rx_Callback(uint8_t* buffer);
#endif /* __PC_COMMAND_PARSER_H */ 