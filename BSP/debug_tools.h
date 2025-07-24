#ifndef __DEBUG_TOOLS_H
#define __DEBUG_TOOLS_H

#include "AllHeader.h"

// 调试级别定义
#define DEBUG_LEVEL_ERROR   1
#define DEBUG_LEVEL_WARN    2
#define DEBUG_LEVEL_INFO    3
#define DEBUG_LEVEL_DEBUG   4

// 当前调试级别（可以修改这个值来控制输出详细程度）
#define CURRENT_DEBUG_LEVEL DEBUG_LEVEL_INFO

// 调试输出宏
#if CURRENT_DEBUG_LEVEL >= DEBUG_LEVEL_ERROR
#define DEBUG_ERROR(fmt, ...) printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DEBUG_ERROR(fmt, ...)
#endif

#if CURRENT_DEBUG_LEVEL >= DEBUG_LEVEL_WARN
#define DEBUG_WARN(fmt, ...) printf("[WARN] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DEBUG_WARN(fmt, ...)
#endif

#if CURRENT_DEBUG_LEVEL >= DEBUG_LEVEL_INFO
#define DEBUG_INFO(fmt, ...) printf("[INFO] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(fmt, ...)
#endif

#if CURRENT_DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG
#define DEBUG_PRINT(fmt, ...) printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...)
#endif

// 函数进入和退出跟踪
#define FUNC_ENTER() DEBUG_PRINT("ENTER: %s", __FUNCTION__)
#define FUNC_EXIT()  DEBUG_PRINT("EXIT: %s", __FUNCTION__)

// 硬件状态检查宏
#define CHECK_USART1_STATUS() Debug_Check_USART1_Status()
#define CHECK_USART2_STATUS() Debug_Check_USART2_Status()
#define CHECK_USART3_STATUS() Debug_Check_USART3_Status()
#define CHECK_TIMER_STATUS()  Debug_Check_Timer_Status()
#define CHECK_GPIO_STATUS()   Debug_Check_GPIO_Status()

// 变量监控宏
#define WATCH_VAR(var) printf("[WATCH] %s = %d\r\n", #var, (int)var)
#define WATCH_VAR_HEX(var) printf("[WATCH] %s = 0x%X\r\n", #var, (unsigned int)var)
#define WATCH_VAR_FLOAT(var) printf("[WATCH] %s = %.2f\r\n", #var, (float)var)

// 调试函数声明
void Debug_Check_USART1_Status(void);
void Debug_Check_USART2_Status(void);
void Debug_Check_USART3_Status(void);
void Debug_Check_Timer_Status(void);
void Debug_Check_GPIO_Status(void);
void Debug_System_Info(void);
void Debug_Memory_Info(void);
void Debug_All_Hardware(void);

#endif /* __DEBUG_TOOLS_H */ 

