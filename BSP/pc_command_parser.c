#include "pc_command_parser.h"
#include "motor_control.h"
#include "gpio_control.h"
#include "servo_sg90.h"      // �����������ͷ�ļ�
#include "app_motor_usart.h" // ����Contrl_Speed�ȵ�����ƺ���
#include <string.h>          // for memcpy
#include <stdint.h>          // for uint16_t

// ȫ�ֱ��������ڴ洢���жϽ��յ�����֡
static volatile uint8_t rpi_cmd_buffer[RPI_FRAME_LEN];
static volatile uint8_t rpi_cmd_ready = 0;

/**
 * @brief  ������ uint8_t �ֽ��н���ָ����ٶ�ֵ(double)
 * @note   ���㷨�������ṩ�� pi_to_32.txt �ļ���
 * ��ʽ: 1λ����, 10λ����, 5λС����
 * @param  high_byte: ��λ�ֽ�
 * @param  low_byte: ��λ�ֽ�
 * @retval �ָ������ٶ�ֵ (double)
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
 * @brief  USART3�жϻص��������ڽ�����һ֡�󱻵���
 * @note   �����յ������ݸ��Ƶ����ػ������������þ�����־
 * @param  buffer: ָ���жϷ�������н��ջ�������ָ��
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
 * @brief  ��ѯ����������ݮ��(USART3)���յ�ָ��
 * @note   �˺�������ѭ���б����ã���������־����֤����֡����ִ��ָ��
 */
void ros_command_poll(void)
{
    if (!rpi_cmd_ready) return; // ���û����ָ�ֱ�ӷ���

    // 1. ��֤����֡�ṹ (������λ�õı�־��ĸ)
    if (rpi_cmd_buffer[0]  == 'S' &&  // ������ݿ�ʼ
        rpi_cmd_buffer[3]  == 'L' &&  // LED����
        rpi_cmd_buffer[5]  == 'A' &&  // ��������
        rpi_cmd_buffer[7]  == 'V' &&  // �ٶ����ݿ�ʼ
        rpi_cmd_buffer[16] == 'R' &&  // ����������
        rpi_cmd_buffer[18] == '\n')   // ֡������
    {
        // 2. ������ӻ���������ȡÿ���豸�Ŀ���ֵ
        uint8_t servo1_angle = rpi_cmd_buffer[1];
        uint8_t servo2_angle = rpi_cmd_buffer[2];
        uint8_t led_state = rpi_cmd_buffer[4];
        uint8_t laser_state = rpi_cmd_buffer[6];
        uint8_t buzzer_state = rpi_cmd_buffer[17];

        // ����ĸ����ӵ��ٶ�
        double vel_fl = unpack_velocity(rpi_cmd_buffer[8],  rpi_cmd_buffer[9]);
        double vel_fr = unpack_velocity(rpi_cmd_buffer[10], rpi_cmd_buffer[11]);
        double vel_bl = unpack_velocity(rpi_cmd_buffer[12], rpi_cmd_buffer[13]);
        double vel_br = unpack_velocity(rpi_cmd_buffer[14], rpi_cmd_buffer[15]);

        // 3. ���ƣ����ݽ�����ֵ������������
        
        // --- ������� ---
        Servo_Set_Angle(1, servo1_angle);
        Servo_Set_Angle(2, servo2_angle);

        // --- LED ���� ---
        Control_LED(led_state);

        // --- ������� ---
        Control_Laser(laser_state);

        // --- ����ٶȿ��� (�����ƶ�����) ---
        // ע��: unpack_velocity���ص���double���ͣ���Contrl_Speed��Ҫint16_t��
        // ������Ҫһ�����������ӡ�������λת�������ֵ��ȷ����Ҫ���ԡ�
        // ���磺�����ݮ�ɷ������ٶȵ�λ����/��(m/s)���������������Ҫ��������/��(cm/s)��
        // ��ô����������Ӿ�Ӧ���� 100.0�����������ʵ������޸ġ�
        const double scaling_factor = 1000.0; // ����һ����Ҫ�����Ժ͵�����ʾ��ֵ��
        
        Contrl_Speed((int16_t)(vel_fl * scaling_factor), 
                     (int16_t)(vel_fr * scaling_factor), 
                     (int16_t)(vel_bl * scaling_factor), 
                     (int16_t)(vel_br * scaling_factor));

        // --- ���������� ---
        Control_Buzzer(buzzer_state);
    }
    
    // ������ϣ������־λ���ȴ���һ��ָ��
    rpi_cmd_ready = 0;
}