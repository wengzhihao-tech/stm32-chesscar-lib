/**
 * @file encoder_tim.h
 * @brief STM32 定时器编码器模式测速（TI1 and TI2 四倍频 + 32 位软件累计）。
 *
 * 为什么要做软件累计：
 *   F103 的 TIM2/TIM3 是 16 位计数器，向前跑约 65536 个计数就回绕。
 *   13PPR × 4 × 20.409 ≈ 1061 计数/圈，满量程只有约 62 圈，
 *   跑几十秒就会溢出。因此必须在溢出中断里累计圈数，对外提供 int32_t。
 *
 * 必须在 CubeMX 中：
 *   - Combined Channels → Encoder Mode
 *   - Encoder Mode 选 "TI1 and TI2"（这才是 4 倍频；TI1 或 TI2 只有 2 倍频）
 *   - Counter Period = 65535
 *   - 使能对应定时器的 global interrupt（为了 Update 中断）
 */
#ifndef ENCODER_TIM_H
#define ENCODER_TIM_H

#include "bsp_config.h"
#include <stdint.h>

/** 启动两路编码器接口并清零累计值。 */
void Encoder_Init(void);

/** 清零左右累计计数（标定或重新起步时调用）。 */
void Encoder_Reset(void);

/**
 * @brief 读取累计计数（向前为正）。
 *
 * 返回值已合并溢出圈数，是单调的 32 位有符号累计量，
 * 调用者应自行做周期差分，**不要**在外部清零。
 */
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);

/**
 * @brief 定时器溢出回调钩子。
 *
 * 在 CubeMX 生成的 HAL_TIM_PeriodElapsedCallback 里调用本函数，
 * 或直接使用本文件提供的弱符号实现。
 */
void Encoder_OnTimerOverflow(TIM_HandleTypeDef *htim);

#endif /* ENCODER_TIM_H */
