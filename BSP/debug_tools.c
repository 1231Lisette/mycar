#include "debug_tools.h"

// 检查USART1状态
void Debug_Check_USART1_Status(void)
{
    printf("=== USART1 Status Check ===\r\n");
    
    // 检查时钟使能
    if(RCC->APB2ENR & RCC_APB2ENR_USART1EN)
        printf("✓ USART1 Clock: ENABLED\r\n");
    else
        printf("✗ USART1 Clock: DISABLED\r\n");
    
    if(RCC->APB2ENR & RCC_APB2ENR_IOPAEN)
        printf("✓ GPIOA Clock: ENABLED\r\n");
    else
        printf("✗ GPIOA Clock: DISABLED\r\n");
    
    // 检查GPIO配置
    printf("PA9 (TX) Config: 0x%X\r\n", (GPIOA->CRH >> 4) & 0xF);
    printf("PA10(RX) Config: 0x%X\r\n", (GPIOA->CRH >> 8) & 0xF);
    
    // 检查USART寄存器
    printf("USART1->CR1: 0x%X\r\n", USART1->CR1);
    printf("USART1->CR2: 0x%X\r\n", USART1->CR2);
    printf("USART1->SR:  0x%X\r\n", USART1->SR);
    printf("USART1->BRR: 0x%X\r\n", USART1->BRR);
    
    // 检查中断使能
    if(USART1->CR1 & USART_CR1_RXNEIE)
        printf("✓ RXNE Interrupt: ENABLED\r\n");
    else
        printf("✗ RXNE Interrupt: DISABLED\r\n");
    
    // 检查NVIC中断
    if(NVIC->ISER[USART1_IRQn >> 5] & (1 << (USART1_IRQn & 0x1F)))
        printf("✓ NVIC Interrupt: ENABLED\r\n");
    else
        printf("✗ NVIC Interrupt: DISABLED\r\n");
    
    printf("==========================\r\n");
}

// 检查USART2状态
void Debug_Check_USART2_Status(void)
{
    printf("=== USART2 Status Check ===\r\n");
    
    if(RCC->APB1ENR & RCC_APB1ENR_USART2EN)
        printf("✓ USART2 Clock: ENABLED\r\n");
    else
        printf("✗ USART2 Clock: DISABLED\r\n");
    
    printf("PA2 (TX) Config: 0x%X\r\n", (GPIOA->CRL >> 8) & 0xF);
    printf("PA3 (RX) Config: 0x%X\r\n", (GPIOA->CRL >> 12) & 0xF);
    
    printf("USART2->CR1: 0x%X\r\n", USART2->CR1);
    printf("USART2->SR:  0x%X\r\n", USART2->SR);
    printf("==========================\r\n");
}

// 检查USART3状态
void Debug_Check_USART3_Status(void)
{
    printf("=== USART3 Status Check ===\r\n");
    
    if(RCC->APB1ENR & RCC_APB1ENR_USART3EN)
        printf("✓ USART3 Clock: ENABLED\r\n");
    else
        printf("✗ USART3 Clock: DISABLED\r\n");
    
    printf("PB10(TX) Config: 0x%X\r\n", (GPIOB->CRH >> 8) & 0xF);
    printf("PB11(RX) Config: 0x%X\r\n", (GPIOB->CRH >> 12) & 0xF);
    
    printf("USART3->CR1: 0x%X\r\n", USART3->CR1);
    printf("USART3->SR:  0x%X\r\n", USART3->SR);
    printf("==========================\r\n");
}

// 检查定时器状态
void Debug_Check_Timer_Status(void)
{
    printf("=== Timer Status Check ===\r\n");
    
    if(RCC->APB1ENR & RCC_APB1ENR_TIM3EN)
        printf("✓ TIM3 Clock: ENABLED\r\n");
    else
        printf("✗ TIM3 Clock: DISABLED\r\n");
    
    printf("TIM3->CR1: 0x%X\r\n", TIM3->CR1);
    printf("TIM3->PSC: %d\r\n", TIM3->PSC);
    printf("TIM3->ARR: %d\r\n", TIM3->ARR);
    printf("TIM3->CNT: %d\r\n", TIM3->CNT);
    
    if(TIM3->DIER & TIM_DIER_UIE)
        printf("✓ TIM3 Update Interrupt: ENABLED\r\n");
    else
        printf("✗ TIM3 Update Interrupt: DISABLED\r\n");
    
    printf("========================\r\n");
}

// 检查GPIO状态
void Debug_Check_GPIO_Status(void)
{
    printf("=== GPIO Status Check ===\r\n");
    
    printf("GPIOA->CRL: 0x%08X\r\n", GPIOA->CRL);
    printf("GPIOA->CRH: 0x%08X\r\n", GPIOA->CRH);
    printf("GPIOA->IDR: 0x%04X\r\n", GPIOA->IDR);
    printf("GPIOA->ODR: 0x%04X\r\n", GPIOA->ODR);
    
    printf("GPIOB->CRH: 0x%08X\r\n", GPIOB->CRH);
    printf("GPIOB->IDR: 0x%04X\r\n", GPIOB->IDR);
    
    printf("=======================\r\n");
}

// 系统信息
void Debug_System_Info(void)
{
    printf("=== System Information ===\r\n");
    printf("System Core Clock: %d Hz\r\n", SystemCoreClock);
    printf("RCC->CFGR: 0x%08X\r\n", RCC->CFGR);
    printf("RCC->CR:   0x%08X\r\n", RCC->CR);
    printf("RCC->APB1ENR: 0x%08X\r\n", RCC->APB1ENR);
    printf("RCC->APB2ENR: 0x%08X\r\n", RCC->APB2ENR);
    printf("=========================\r\n");
}

// 内存信息
/*void Debug_Memory_Info(void)
{
    printf("=== Memory Information ===\r\n");
    
    // 获取栈指针
    register uint32_t sp asm("sp");
    printf("Current Stack Pointer: 0x%08X\r\n", sp);
    
    // 估算栈使用量（假设栈从0x20005000开始）
    uint32_t stack_start = 0x20005000;
    uint32_t stack_used = stack_start - sp;
    printf("Estimated Stack Used: %d bytes\r\n", stack_used);
    
    // 全局变量信息
    extern uint8_t dbg_cmd_ready;
    extern char dbg_cmd_buf[50];
    printf("dbg_cmd_ready: %d\r\n", dbg_cmd_ready);
    printf("dbg_cmd_buf: %s\r\n", dbg_cmd_buf);
    
    printf("========================\r\n");
}*/

// 检查所有硬件
void Debug_All_Hardware(void)
{
    Debug_System_Info();
    Debug_Check_USART1_Status();
    Debug_Check_USART2_Status();
    Debug_Check_USART3_Status();
    Debug_Check_Timer_Status();
    Debug_Check_GPIO_Status();
    //Debug_Memory_Info();
} 

