// BSP/pc_command_parser.h
#ifndef __PC_COMMAND_PARSER_H
#define __PC_COMMAND_PARSER_H

#include "AllHeader.h"
#include "stm32f10x.h"

/**
 * @brief �������ݮ�ɽ��յ�����֡��׼ȷ����
 * S(1)+servo1(1)+servo2(1)+L(1)+led(1)+A(1)+laser(1)+V(1)+4*vel(8)+R(1)+buzzer(1)+\n(1) = 19 �ֽ�
 */
#define RPI_FRAME_LEN 19

/**
 * @brief ��ѯ����������ROS(��ݮ��)��ָ��
 * @note  �˺���Ӧ����ѭ���б��������á�
 * ��main.c��������Ϊros_command_poll�ĵ��ã��ʺ���������һ�¡�
 */
void ros_command_poll(void);

/**
 * @brief USART3�жϷ������Ļص�����
 * @note  �����յ�������һ֡���ݺ󣬴˺���������
 * @param buffer: ָ��������ݻ�������ָ��
 */
void USART3_Rx_Callback(uint8_t* buffer);
#endif /* __PC_COMMAND_PARSER_H */ 