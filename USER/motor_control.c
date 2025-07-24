// USER/motor_control.c
#include "motor_control.h"

// 转向控制相关参数
#define WHEEL_DIAMETER_MM 67.0f  // 轮子直径(mm)
#define WHEEL_TRACK_MM 127.0f    // 轮距(mm) - 需要根据实际小车测量
#define WHEEL_BASE_MM 100.0f     // 轴距(mm) - 前后轮中心距离
#define ENCODER_PPR 11           // 编码器每圈脉冲数
#define GEAR_RATIO 40.0f         // 减速比

// PID控制参数
#define TURN_KP 2.0f             // 比例系数
#define TURN_KI 0.1f             // 积分系数
#define TURN_KD 0.05f            // 微分系数
#define TURN_MAX_OUTPUT 200      // 最大输出限制
#define TURN_MAX_INTEGRAL 100    // 积分限幅

// 差速转向PID控制器
typedef struct {
    float kp, ki, kd;
    float target_angle;          // 目标角度（度）
    float current_angle;         // 当前角度（度）
    float error;                 // 误差
    float last_error;            // 上次误差
    float integral;              // 积分项
    float output;                // 输出
    float max_output;            // 最大输出
    float max_integral;          // 积分限幅
} TurnPIDController;

static TurnPIDController turn_pid = {
    .kp = TURN_KP,
    .ki = TURN_KI,
    .kd = TURN_KD,
    .max_output = TURN_MAX_OUTPUT,
    .max_integral = TURN_MAX_INTEGRAL
};


static TurnState current_turn_state = TURN_IDLE;
static int turn_start_encoder[4] = {0, 0, 0, 0};
static float turn_start_angle = 0.0f;

/**
 * @brief  计算编码器差值
 * @param  current: 当前编码器值
 * @param  start: 起始编码器值
 * @retval 编码器差值
 */
static int get_encoder_delta(int current, int start)
{
    int delta = current - start;
    // 处理编码器溢出
    if (delta > 32767) delta -= 65536;
    if (delta < -32768) delta += 65536;
    return delta;
}

/**
 * @brief  计算当前转向角度
 * @retval 当前转向角度（度）
 */
static float calculate_current_turn_angle(void)
{
    // 计算左右轮编码器差值
    int left_delta = (get_encoder_delta(Encoder_Now[0], turn_start_encoder[0]) + 
                     get_encoder_delta(Encoder_Now[2], turn_start_encoder[2])) / 2;
    int right_delta = (get_encoder_delta(Encoder_Now[1], turn_start_encoder[1]) + 
                      get_encoder_delta(Encoder_Now[3], turn_start_encoder[3])) / 2;
    
    // 计算差速转向角度
    // 角度 = (右轮距离 - 左轮距离) / 轮距 * 180 / π
    float left_distance = (float)left_delta / (ENCODER_PPR * GEAR_RATIO) * WHEEL_DIAMETER_MM * 3.14159f;
    float right_distance = (float)right_delta / (ENCODER_PPR * GEAR_RATIO) * WHEEL_DIAMETER_MM * 3.14159f;
    float angle = (right_distance - left_distance) / WHEEL_TRACK_MM * 180.0f / 3.14159f;
    
    return angle;
}

/**
 * @brief  重置PID控制器
 */
static void reset_turn_pid(void)
{
    turn_pid.error = 0.0f;
    turn_pid.last_error = 0.0f;
    turn_pid.integral = 0.0f;
    turn_pid.output = 0.0f;
}

/**
 * @brief  PID计算
 * @param  target: 目标角度
 * @param  current: 当前角度
 * @retval PID输出
 */
static float calculate_turn_pid(float target, float current)
{
    turn_pid.target_angle = target;
    turn_pid.current_angle = current;
    
    // 计算误差
    float error = target - current;
    
    // 计算积分项
    turn_pid.integral += error;
    
    // 积分限幅
    if(turn_pid.integral > turn_pid.max_integral)
        turn_pid.integral = turn_pid.max_integral;
    else if(turn_pid.integral < -turn_pid.max_integral)
        turn_pid.integral = -turn_pid.max_integral;
    
    // 计算微分项
    float derivative = error - turn_pid.last_error;
    
    // PID输出计算
    turn_pid.output = turn_pid.kp * error + 
                     turn_pid.ki * turn_pid.integral +
                     turn_pid.kd * derivative;
    
    // 输出限幅
    if(turn_pid.output > turn_pid.max_output)
        turn_pid.output = turn_pid.max_output;
    else if(turn_pid.output < -turn_pid.max_output)
        turn_pid.output = -turn_pid.max_output;
    
    // 保存误差
    turn_pid.last_error = error;
    turn_pid.error = error;
    
    return turn_pid.output;
}

/**
 * @brief  开始精确左转90度（差速转向 + PID）
 */
void motor_turn_left_90_precise(void)
{
    if (current_turn_state != TURN_IDLE) return;
    
    // 记录起始编码器值和角度
    for (int i = 0; i < 4; i++) {
        turn_start_encoder[i] = Encoder_Now[i];
    }
    turn_start_angle = 0.0f;
    
    // 设置目标角度
    turn_pid.target_angle = -90.0f;  // 左转为负角度
    current_turn_state = TURN_LEFT_90;
    
    // 重置PID控制器
    reset_turn_pid();
    
    printf("> Starting precise left turn 90 degrees (differential + PID)...\r\n");
}

/**
 * @brief  开始精确右转90度（差速转向 + PID）
 */
void motor_turn_right_90_precise(void)
{
    if (current_turn_state != TURN_IDLE) return;
    
    // 记录起始编码器值和角度
    for (int i = 0; i < 4; i++) {
        turn_start_encoder[i] = Encoder_Now[i];
    }
    turn_start_angle = 0.0f;
    
    // 设置目标角度
    turn_pid.target_angle = 90.0f;   // 右转为正角度
    current_turn_state = TURN_RIGHT_90;
    
    // 重置PID控制器
    reset_turn_pid();
    
    printf("> Starting precise right turn 90 degrees (differential + PID)...\r\n");
}

/**
 * @brief  更新精确转向状态（需要在主循环中调用）
 * @retval 1: 转向完成, 0: 转向进行中
 */
uint8_t motor_update_precise_turn(void)
{
    if (current_turn_state == TURN_IDLE) return 1;
    
    // 计算当前角度
    float current_angle = calculate_current_turn_angle();
    
    // PID计算
    float pid_output = calculate_turn_pid(turn_pid.target_angle, current_angle);
    
    // 应用PID输出到电机
    int base_speed = 100;  // 基础速度
    int left_speed = base_speed - (int)pid_output;
    int right_speed = base_speed + (int)pid_output;
    
    // 限制速度范围
    if (left_speed > 200) left_speed = 200;
    if (left_speed < -200) left_speed = -200;
    if (right_speed > 200) right_speed = 200;
    if (right_speed < -200) right_speed = -200;
    
    // 控制电机
    Contrl_Speed(left_speed, right_speed, left_speed, right_speed);
    
    // 检查是否达到目标角度（允许±2度误差）
    if (fabs(current_angle - turn_pid.target_angle) < 2.0f) {
        motor_stop();
        current_turn_state = TURN_IDLE;
        printf("> Precise differential turn completed. Angle: %.1f°\r\n", current_angle);
        return 1; // 转向完成
    }
    
    return 0; // 转向进行中
}

/**
 * @brief  获取当前转向状态
 * @retval 当前转向状态
 */
TurnState motor_get_turn_state(void)
{
    return current_turn_state;
}

/**
 * @brief  取消当前转向操作
 */
void motor_cancel_turn(void)
{
    if (current_turn_state != TURN_IDLE) {
        motor_stop();
        current_turn_state = TURN_IDLE;
        reset_turn_pid();
        printf("> Turn operation cancelled.\r\n");
    }
}

/**
 * @brief  设置转向PID参数
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 */
void motor_set_turn_pid_params(float kp, float ki, float kd)
{
    turn_pid.kp = kp;
    turn_pid.ki = ki;
    turn_pid.kd = kd;
    printf("> Turn PID parameters updated: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", kp, ki, kd);
}

/**
 * @brief  Move forward with a specified speed.
 * @param  speed: Target speed for all motors.
 */
void motor_forward(int speed)
{
    Contrl_Speed(speed, speed, speed, speed);
}

/**
 * @brief  Move backward with a specified speed.
 * @param  speed: Target speed for all motors (will be negated).
 */
void motor_backward(int speed)
{
    Contrl_Speed(-speed, -speed, -speed, -speed);
}

/**
 * @brief  Turn left using differential speed.
 * @param  speed: Speed for the differential steering.
 */
void motor_turn_left(int speed1, int speed2)
{
    Contrl_Speed(-speed1, -speed1, speed2, speed2);
}

/**
 * @brief  Turn right using differential speed.
 * @param  speed: Speed for the differential steering.
 */
void motor_turn_right(int speed1, int speed2)
{
    Contrl_Speed(speed1, speed1, -speed2, -speed2);
}

/**
 * @brief  Stop all motors immediately.
 */
void motor_stop(void)
{
    Contrl_Speed(0, 0, 0, 0);
} 
