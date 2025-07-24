#include "pid_config.h"
#include "stm32f10x_flash.h"

// Flash存储地址（根据实际STM32型号调整）
#define FLASH_PARAMS_ADDR 0x0801F000  // 存储PID参数的Flash地址

// 默认PID参数
static PIDParams default_params = {
    .kp = 1.0f,
    .ki = 0.1f,
    .kd = 0.01f,
    .max_output = 1000.0f,
    .max_integral = 500.0f
};

// 初始化PID控制器
void PID_Init(PIDController *pid)
{
    pid->params = default_params;
    PID_Reset(pid);
}

// 设置PID参数
void PID_SetParams(PIDController *pid, float kp, float ki, float kd, float max_output, float max_integral)
{
    pid->params.kp = kp;
    pid->params.ki = ki;
    pid->params.kd = kd;
    pid->params.max_output = max_output;
    pid->params.max_integral = max_integral;
}

// 重置PID控制器
void PID_Reset(PIDController *pid)
{
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

// PID计算
float PID_Calculate(PIDController *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;
    
    // 计算误差
    float error = target - actual;
    
    // 计算积分项
    pid->integral += error;
    
    // 积分限幅
    if(pid->integral > pid->params.max_integral)
        pid->integral = pid->params.max_integral;
    else if(pid->integral < -pid->params.max_integral)
        pid->integral = -pid->params.max_integral;
    
    // 计算微分项
    float derivative = error - pid->last_error;
    
    // PID输出计算
    pid->output = pid->params.kp * error + 
                 pid->params.ki * pid->integral +
                 pid->params.kd * derivative;
    
    // 输出限幅
    if(pid->output > pid->params.max_output)
        pid->output = pid->params.max_output;
    else if(pid->output < -pid->params.max_output)
        pid->output = -pid->params.max_output;
    
    // 保存误差
    pid->last_error = error;
    
    return pid->output;
}

// 将PID参数保存到Flash
void PID_SaveParams(void)
{
    FLASH_Unlock();
    FLASH_ErasePage(FLASH_PARAMS_ADDR);
    
    uint32_t *data = (uint32_t *)&default_params;
    for(int i = 0; i < sizeof(PIDParams)/4; i++)
    {
        FLASH_ProgramWord(FLASH_PARAMS_ADDR + i*4, data[i]);
    }
    
    FLASH_Lock();
}

// 从Flash加载PID参数
void PID_LoadParams(void)
{
    PIDParams *flash_params = (PIDParams *)FLASH_PARAMS_ADDR;
    
    // 检查Flash是否已经写入过参数
    if(flash_params->kp != 0xFFFFFFFF)
    {
        default_params = *flash_params;
    }
} 
