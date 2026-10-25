/**
 * @file motor_tb6612.c
 * @brief TB6612FNG 驱动实现。
 */
#include "motor_tb6612.h"

#include "car_physical_config.h"
#include "chesscar_common.h"

typedef struct {
    GPIO_TypeDef *in1Port;
    uint16_t in1Pin;
    GPIO_TypeDef *in2Port;
    uint16_t in2Pin;
    uint32_t pwmChannel;
    volatile int8_t appliedDirection;
} Motor_Side;

static Motor_Side g_left = {
    BSP_MOTOR_LEFT_IN1_PORT, BSP_MOTOR_LEFT_IN1_PIN,
    BSP_MOTOR_LEFT_IN2_PORT, BSP_MOTOR_LEFT_IN2_PIN,
    BSP_MOTOR_PWM_CHANNEL_LEFT, 0
};

static Motor_Side g_right = {
    BSP_MOTOR_RIGHT_IN1_PORT, BSP_MOTOR_RIGHT_IN1_PIN,
    BSP_MOTOR_RIGHT_IN2_PORT, BSP_MOTOR_RIGHT_IN2_PIN,
    BSP_MOTOR_PWM_CHANNEL_RIGHT, 0
};

static uint32_t Motor_AbsSpeed(int16_t speed)
{
    return (speed < 0) ? (uint32_t)(-(int32_t)speed) : (uint32_t)speed;
}

/*
 * STM32 通用定时器 PWM 默认为向上计数 + PWM Mode 1：
 * CNT < CCR 时输出有效电平，所以 CCR 直接就是「高电平计数」，
 * duty 越大占空比越高，不需要像向下计数那样做 (ARR - duty) 反转。
 *
 * 这一点和本工程早期的 MSPM0 版本不同——MSPM0 用的是 EDGE_ALIGN
 * 向下计数，CCR 保存的是「周期末尾剩余计数」，必须写 SPEED_MAX - duty。
 * 移植时如果照抄那段公式，会出现「低速命令反而接近满占空比」的严重故障。
 */
static void Motor_SetPwm(const Motor_Side *side, uint32_t duty)
{
    if (duty > (uint32_t)CAR_PWM_OUTPUT_LIMIT) {
        duty = (uint32_t)CAR_PWM_OUTPUT_LIMIT;
    }
    __HAL_TIM_SET_COMPARE(&BSP_MOTOR_PWM_TIM, side->pwmChannel, duty);
}

static void Motor_ApplySide(Motor_Side *side, int16_t speed)
{
    speed = (int16_t)ChessCar_ClampInt32(speed,
                                         -CAR_PWM_OUTPUT_LIMIT,
                                         CAR_PWM_OUTPUT_LIMIT);

    /*
     * 先把 PWM 降到 0 再改方向，避免换向瞬间 H 桥两臂同时导通
     * 造成穿通电流。TB6612 内部虽有死区时间，但软件侧同样不该抢跑。
     */
    Motor_SetPwm(side, 0U);
    HAL_GPIO_WritePin(side->in1Port, side->in1Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(side->in2Port, side->in2Pin, GPIO_PIN_RESET);

    if (speed > 0) {
        /* IN1=H IN2=L → CW */
        HAL_GPIO_WritePin(side->in1Port, side->in1Pin, GPIO_PIN_SET);
        side->appliedDirection = 1;
    } else if (speed < 0) {
        /* IN1=L IN2=H → CCW */
        HAL_GPIO_WritePin(side->in2Port, side->in2Pin, GPIO_PIN_SET);
        side->appliedDirection = -1;
    }
    /* 零速保留最后方向，仅用于描述滑行时的历史方向。 */

    Motor_SetPwm(side, Motor_AbsSpeed(speed));
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&BSP_MOTOR_PWM_TIM, BSP_MOTOR_PWM_CHANNEL_LEFT);
    HAL_TIM_PWM_Start(&BSP_MOTOR_PWM_TIM, BSP_MOTOR_PWM_CHANNEL_RIGHT);

#if BSP_MOTOR_HAS_STBY
    /* STBY 必须拉高，否则两路输出始终是高阻，电机完全不动。 */
    HAL_GPIO_WritePin(BSP_MOTOR_STBY_PORT, BSP_MOTOR_STBY_PIN, GPIO_PIN_SET);
#endif

    g_left.appliedDirection = 0;
    g_right.appliedDirection = 0;
    Motor_Stop();
}

void Motor_SetSides(int16_t left, int16_t right)
{
    Motor_ApplySide(&g_left, left);
    Motor_ApplySide(&g_right, right);
}

void Motor_SetLeft(int16_t speed)  { Motor_ApplySide(&g_left, speed); }
void Motor_SetRight(int16_t speed) { Motor_ApplySide(&g_right, speed); }

void Motor_Stop(void)
{
    /* IN1=L IN2=L → Stop（高阻滑行）。 */
    Motor_SetSides(0, 0);
}

void Motor_Brake(void)
{
    /* IN1=H IN2=H → Short brake。PWM 置 0 后再拉高两个方向脚。 */
    Motor_SetPwm(&g_left, 0U);
    Motor_SetPwm(&g_right, 0U);
    HAL_GPIO_WritePin(g_left.in1Port, g_left.in1Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(g_left.in2Port, g_left.in2Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(g_right.in1Port, g_right.in1Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(g_right.in2Port, g_right.in2Pin, GPIO_PIN_SET);
}

void Motor_Standby(void)
{
#if BSP_MOTOR_HAS_STBY
    Motor_Stop();
    HAL_GPIO_WritePin(BSP_MOTOR_STBY_PORT, BSP_MOTOR_STBY_PIN, GPIO_PIN_RESET);
#else
    Motor_Stop();
#endif
}

int8_t Motor_GetLeftDirection(void)  { return g_left.appliedDirection; }
int8_t Motor_GetRightDirection(void) { return g_right.appliedDirection; }
