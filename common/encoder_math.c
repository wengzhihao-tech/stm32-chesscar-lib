/**
 * @file encoder_math.c
 * @brief 编码器换算实现。
 */
#include "encoder_math.h"

/*
 * 所有换算都要除以轮径、每圈计数、采样周期等参数。
 * 只要其中一个为 0 或负数，结果就没有物理意义，统一在入口拦截，
 * 调用者可用返回 0 判断配置是否尚未初始化。
 */
static bool Encoder_IsConfigValid(const Encoder_Config *config)
{
    return (config != 0) &&
           (config->wheelDiameterCm > 0.0f) &&
           (config->gearRatio > 0.0f) &&
           (config->pulsesPerRound > 0U) &&
           (config->samplePeriodMs > 0.0f);
}

float Encoder_CountToDistanceCm(int32_t count, const Encoder_Config *config)
{
    float wheelCircleCm;

    if (!Encoder_IsConfigValid(config)) {
        return 0.0f;
    }

    /*
     * 距离 = 计数 / (每圈计数 × 减速比) × 车轮周长
     * count 保留符号，所以返回距离也保留方向。
     */
    wheelCircleCm = config->wheelDiameterCm * CHESSCAR_PI;
    return ((float)count /
            ((float)config->pulsesPerRound * config->gearRatio)) * wheelCircleCm;
}

float Encoder_CountToSpeedCmps(int32_t count, const Encoder_Config *config)
{
    float distanceCm;

    if (!Encoder_IsConfigValid(config)) {
        return 0.0f;
    }

    /* 速度 = 本周期距离 / 本周期时间；乘 1000 把 cm/ms 换成 cm/s。 */
    distanceCm = Encoder_CountToDistanceCm(count, config);
    return distanceCm * 1000.0f / config->samplePeriodMs;
}

int32_t Encoder_SpeedCmpsToCount(float speedCmps, const Encoder_Config *config)
{
    float wheelCircleCm;
    float rounds;

    if (!Encoder_IsConfigValid(config)) {
        return 0;
    }

    /*
     * 本周期距离 = 目标速度 × 采样周期
     * 车轮圈数   = 本周期距离 / 车轮周长
     * 计数       = 车轮圈数 × 每圈计数 × 减速比
     */
    wheelCircleCm = config->wheelDiameterCm * CHESSCAR_PI;
    rounds = (speedCmps * config->samplePeriodMs / 1000.0f) / wheelCircleCm;
    return (int32_t)(rounds * (float)config->pulsesPerRound * config->gearRatio);
}

float Encoder_SpeedCmpsToMotorRpm(float speedCmps, const Encoder_Config *config)
{
    float wheelCircleCm;

    if (!Encoder_IsConfigValid(config)) {
        return 0.0f;
    }

    /* 车轮 rpm = 速度 / 周长 × 60；电机轴 rpm 再乘减速比。 */
    wheelCircleCm = config->wheelDiameterCm * CHESSCAR_PI;
    return (speedCmps / wheelCircleCm) * 60.0f * config->gearRatio;
}

float Encoder_SpeedQuantumCmps(const Encoder_Config *config)
{
    return Encoder_CountToSpeedCmps(1, config);
}
