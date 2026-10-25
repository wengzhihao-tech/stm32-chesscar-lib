/**
 * @file encoder_tim.c
 * @brief 编码器模式测速实现（含 16 位计数器溢出累计）。
 */
#include "encoder_tim.h"

#include "car_physical_config.h"

/*
 * 方向符号：小车向前推时计数应增加。
 * 架空正转测试后若某侧计数变负，只改对应的这一个宏，不要动算法。
 */
#ifndef ENCODER_LEFT_DIRECTION_SIGN
#define ENCODER_LEFT_DIRECTION_SIGN  (1)
#endif
#ifndef ENCODER_RIGHT_DIRECTION_SIGN
#define ENCODER_RIGHT_DIRECTION_SIGN (-1)
#endif

static volatile int32_t g_leftOverflow;
static volatile int32_t g_rightOverflow;

void Encoder_Init(void)
{
    /* TIM_CHANNEL_ALL 会同时配置 CH1/CH2 为编码器输入。 */
    HAL_TIM_Encoder_Start(&BSP_ENCODER_LEFT_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&BSP_ENCODER_RIGHT_TIM, TIM_CHANNEL_ALL);

    /* 使能 Update 中断用于捕捉 16 位计数器的溢出/下溢。 */
    __HAL_TIM_CLEAR_FLAG(&BSP_ENCODER_LEFT_TIM, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&BSP_ENCODER_RIGHT_TIM, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&BSP_ENCODER_LEFT_TIM, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&BSP_ENCODER_RIGHT_TIM, TIM_IT_UPDATE);

    Encoder_Reset();
}

void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&BSP_ENCODER_LEFT_TIM, 0);
    __HAL_TIM_SET_COUNTER(&BSP_ENCODER_RIGHT_TIM, 0);
    g_leftOverflow = 0;
    g_rightOverflow = 0;
}

/*
 * 把 16 位 CNT 与溢出圈数合成 32 位累计值。
 *
 * 读取顺序很关键：先读溢出计数、再读 CNT、再复查溢出计数。
 * 如果两次溢出计数不一致，说明读 CNT 期间正好发生了溢出中断，
 * 此时用新的溢出值重读一次 CNT，避免拼出一个跳变几万的假计数。
 */
static int32_t Encoder_Compose(TIM_HandleTypeDef *htim,
                               volatile int32_t *overflow,
                               int32_t sign)
{
    int32_t ovBefore;
    int32_t ovAfter;
    uint32_t cnt;

    do {
        ovBefore = *overflow;
        cnt = __HAL_TIM_GET_COUNTER(htim);
        ovAfter = *overflow;
    } while (ovBefore != ovAfter);

    return sign * (ovAfter * BSP_ENCODER_TIM_PERIOD + (int32_t)cnt);
}

int32_t Encoder_GetLeftCount(void)
{
    return Encoder_Compose(&BSP_ENCODER_LEFT_TIM, &g_leftOverflow,
                           ENCODER_LEFT_DIRECTION_SIGN);
}

int32_t Encoder_GetRightCount(void)
{
    return Encoder_Compose(&BSP_ENCODER_RIGHT_TIM, &g_rightOverflow,
                           ENCODER_RIGHT_DIRECTION_SIGN);
}

void Encoder_OnTimerOverflow(TIM_HandleTypeDef *htim)
{
    if (htim == 0) {
        return;
    }

    /*
     * 编码器模式下 DIR 位指示当前计数方向：
     * 向下计数时发生的 Update 事件是下溢，溢出圈数要减 1。
     */
    if (htim->Instance == BSP_ENCODER_LEFT_TIM.Instance) {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
            g_leftOverflow--;
        } else {
            g_leftOverflow++;
        }
    } else if (htim->Instance == BSP_ENCODER_RIGHT_TIM.Instance) {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
            g_rightOverflow--;
        } else {
            g_rightOverflow++;
        }
    }
}
