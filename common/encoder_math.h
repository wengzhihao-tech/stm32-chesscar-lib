/**
 * @file encoder_math.h
 * @brief 编码器脉冲、轮速与里程之间的数学换算（平台无关）。
 *
 * 全部距离用 cm、速度用 cm/s、时间用 ms。函数只做数值换算，
 * 不读取任何硬件计数器，因此可直接在 PC 上做单元测试。
 *
 * 累计计数允许为负，用于表达倒车。
 */
#ifndef ENCODER_MATH_H
#define ENCODER_MATH_H

#include "chesscar_common.h"

typedef struct {
    float wheelDiameterCm;   /**< 轮胎安装后实测外径，cm。 */
    float gearRatio;         /**< 减速比，电机轴转数 / 车轮转数。 */
    uint32_t pulsesPerRound; /**< 电机轴每圈计数（已含倍频），STM32 编码器模式为 PPR*4。 */
    float samplePeriodMs;    /**< 速度环采样周期，ms，必须与实际调用周期一致。 */
} Encoder_Config;

/** 单周期计数差 → 速度 cm/s。 */
float Encoder_CountToSpeedCmps(int32_t count, const Encoder_Config *config);

/** 目标速度 cm/s → 单周期应有计数差，用于前馈或自检。 */
int32_t Encoder_SpeedCmpsToCount(float speedCmps, const Encoder_Config *config);

/** 累计计数 → 里程 cm，保留正负号表示前进/后退。 */
float Encoder_CountToDistanceCm(int32_t count, const Encoder_Config *config);

/** 速度 cm/s → 电机轴转速 rpm，便于与电机规格书对照。 */
float Encoder_SpeedCmpsToMotorRpm(float speedCmps, const Encoder_Config *config);

/**
 * @brief 单位计数对应的速度量化步长 cm/s。
 *
 * 低速时反馈抖动的根源：10ms 一拍的计数是整数，每增加 1 个计数速度就跳一格。
 * 用这个值判断「当前目标速度是否低于可分辨下限」，低于时应降低 Kd 或延长周期。
 */
float Encoder_SpeedQuantumCmps(const Encoder_Config *config);

#endif /* ENCODER_MATH_H */
