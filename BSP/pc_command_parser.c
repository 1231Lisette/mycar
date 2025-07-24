#include "pc_command_parser.h"
#include "motor_control.h"
#include "gpio_control.h"
#include "servo_sg90.h"      // 包含舵机控制头文件
#include "app_motor_usart.h" // 包含Contrl_Speed等电机控制函数
#include <string.h>          // for memcpy
#include <stdint.h>          // for uint16_t

// 全局变量，用于存储从中断接收的数据帧
static volatile uint8_t rpi_cmd_buffer[RPI_FRAME_LEN];
static volatile uint8_t rpi_cmd_ready = 0;

/**
 * @brief  从两个 uint8_t 字节中解包恢复出速度值(double)
 * @note   此算法基于您提供的 pi_to_32.txt 文件。
 * 格式: 1位符号, 10位整数, 5位小数。
 * @param  high_byte: 高位字节
 * @param  low_byte: 低位字节
 * @retval 恢复出的速度值 (double)
 */
static double unpack_velocity(uint8_t high_byte, uint8_t low_byte) {
    uint16_t packed = ((uint16_t)high_byte << 8) | (uint16_t)low_byte;
    uint16_t sign = (packed >> 15) & 0x1;
    uint16_t integer_part = (packed >> 5) & 0x3FF; // 10-bit integer part
    uint16_t frac_part    =  packed        & 0x1F;  // 5-bit fractional part

    double value = (double)integer_part + (double)frac_part / 32.0;
    return sign ? -value : value;
}

/**
 * @brief  USART3中断回调函数，在接收完一帧后被调用
 * @note   将接收到的数据复制到本地缓冲区，并设置就绪标志
 * @param  buffer: 指向中断服务程序中接收缓冲区的指针
 */
void USART3_Rx_Callback(uint8_t* buffer)
{
    if (!rpi_cmd_ready)
    {
        memcpy((void*)rpi_cmd_buffer, buffer, RPI_FRAME_LEN);
        rpi_cmd_ready = 1;
    }
}

/**
 * @brief  轮询并解析从树莓派(USART3)接收的指令
 * @note   此函数在主循环中被调用，检查就绪标志，验证数据帧，并执行指令
 */
void ros_command_poll(void)
{
    if (!rpi_cmd_ready) return; // 如果没有新指令，直接返回

    // 1. 验证数据帧结构 (检查各个位置的标志字母)
    if (rpi_cmd_buffer[0]  == 'S' &&  // 舵机数据开始
        rpi_cmd_buffer[3]  == 'L' &&  // LED数据
        rpi_cmd_buffer[5]  == 'A' &&  // 激光数据
        rpi_cmd_buffer[7]  == 'V' &&  // 速度数据开始
        rpi_cmd_buffer[16] == 'R' &&  // 蜂鸣器数据
        rpi_cmd_buffer[18] == '\n')   // 帧结束符
    {
        // 2. 解包：从缓冲区中提取每个设备的控制值
        uint8_t servo1_angle = rpi_cmd_buffer[1];
        uint8_t servo2_angle = rpi_cmd_buffer[2];
        uint8_t led_state = rpi_cmd_buffer[4];
        uint8_t laser_state = rpi_cmd_buffer[6];
        uint8_t buzzer_state = rpi_cmd_buffer[17];

        // 解包四个轮子的速度
        double vel_fl = unpack_velocity(rpi_cmd_buffer[8],  rpi_cmd_buffer[9]);
        double vel_fr = unpack_velocity(rpi_cmd_buffer[10], rpi_cmd_buffer[11]);
        double vel_bl = unpack_velocity(rpi_cmd_buffer[12], rpi_cmd_buffer[13]);
        double vel_br = unpack_velocity(rpi_cmd_buffer[14], rpi_cmd_buffer[15]);

        // 3. 控制：根据解包后的值驱动各个外设
        
        // --- 舵机控制 ---
        Servo_Set_Angle(1, servo1_angle);
        Servo_Set_Angle(2, servo2_angle);

        // --- LED 控制 ---
        Control_LED(led_state);

        // --- 激光控制 ---
        Control_Laser(laser_state);

        // --- 电机速度控制 (调用移动函数) ---
        // 注意: unpack_velocity返回的是double类型，而Contrl_Speed需要int16_t。
        // 这里需要一个“缩放因子”来做单位转换。这个值的确定需要调试。
        // 例如：如果树莓派发来的速度单位是米/秒(m/s)，而电机驱动板需要的是厘米/秒(cm/s)，
        // 那么这个缩放因子就应该是 100.0。请根据您的实际情况修改。
        const double scaling_factor = 1000.0; // 这是一个需要您调试和调整的示例值！
        
        Contrl_Speed((int16_t)(vel_fl * scaling_factor), 
                     (int16_t)(vel_fr * scaling_factor), 
                     (int16_t)(vel_bl * scaling_factor), 
                     (int16_t)(vel_br * scaling_factor));

        // --- 蜂鸣器控制 ---
        Control_Buzzer(buzzer_state);
    }
    
    // 处理完毕，清除标志位，等待下一条指令
    rpi_cmd_ready = 0;
}