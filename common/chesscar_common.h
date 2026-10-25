/**
 * @file chesscar_common.h
 * @brief 光电象棋小车各层共享的基础类型、限幅工具与单位约定。
 *
 * 本文件只放不依赖任何芯片寄存器的轻量定义，使 common/ 下的算法层
 * 可以同时编译到 STM32（HAL 库）、MSPM0（DriverLib）或 PC 单元测试环境。
 *
 * 单位约定（全工程统一，不要在局部改写）：
 *   距离 cm   速度 cm/s   角度 deg   角速度 deg/s   时间 ms
 *
 * 禁止在本文件中包含 stm32fxxx_hal.h 或任何 GPIO/TIM/ADC 访问代码。
 */
#ifndef CHESSCAR_COMMON_H
#define CHESSCAR_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#ifndef CHESSCAR_ABS
#define CHESSCAR_ABS(x) (((x) >= 0) ? (x) : -(x))
#endif

#ifndef CHESSCAR_PI
#define CHESSCAR_PI (3.1415926f)
#endif

/**
 * @brief 浮点限幅。
 *
 * 常用于 PID 积分限幅、PWM 输出限幅、速度目标限幅。
 */
static inline float ChessCar_ClampFloat(float value, float minValue, float maxValue)
{
    if (value > maxValue) {
        return maxValue;
    }
    if (value < minValue) {
        return minValue;
    }
    return value;
}

/**
 * @brief 32 位整数限幅。
 *
 * 常用于把电机输出限制在 [-PWM_FULL, +PWM_FULL] 范围内。
 */
static inline int32_t ChessCar_ClampInt32(int32_t value, int32_t minValue, int32_t maxValue)
{
    if (value > maxValue) {
        return maxValue;
    }
    if (value < minValue) {
        return minValue;
    }
    return value;
}

/**
 * @brief 一阶低通滤波：y(n) = k * x(n) + (1 - k) * y(n-1)。
 *
 * k 越小滤波越强、相位滞后越大。编码器低速反馈量化跳变明显时很有用，
 * 但不要用它掩盖机械问题或采样周期抖动。
 */
static inline float ChessCar_LowPass(float previous, float current, float k)
{
    k = ChessCar_ClampFloat(k, 0.0f, 1.0f);
    return k * current + (1.0f - k) * previous;
}

/**
 * @brief 死区补偿：在非零输出上叠加一个起转 PWM，抵消齿轮箱与轮胎静摩擦。
 *
 * 输出为 0 时保持 0，不要给静止电机施加补偿电流。
 */
static inline int32_t ChessCar_ApplyDeadzone(int32_t output, int32_t deadzonePwm, int32_t outputLimit)
{
    if (output == 0 || deadzonePwm <= 0) {
        return ChessCar_ClampInt32(output, -outputLimit, outputLimit);
    }
    if (output > 0) {
        return ChessCar_ClampInt32(output + deadzonePwm, -outputLimit, outputLimit);
    }
    return ChessCar_ClampInt32(output - deadzonePwm, -outputLimit, outputLimit);
}

#endif /* CHESSCAR_COMMON_H */
