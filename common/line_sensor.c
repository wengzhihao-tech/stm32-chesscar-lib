/**
 * @file line_sensor.c
 * @brief 光电探头归一化与加权偏差实现。
 */
#include "line_sensor.h"

void LineSensor_LoadDefaultConfig(LineSensor_Config *config)
{
    uint8_t i;
    static const int8_t defaultWeight[LINE_SENSOR_CHANNELS] = {
        -7, -5, -3, -1, 1, 3, 5, 7
    };

    if (config == 0) {
        return;
    }

    for (i = 0U; i < LINE_SENSOR_CHANNELS; ++i) {
        config->weight[i] = defaultWeight[i];
        /*
         * 未标定时给一组保守默认：
         * 多数反射式探头在白底反射强（ADC 低）、黑线吸收（ADC 高），
         * 具体极性必须现场确认，否则偏差符号会整体反相。
         */
        config->whiteReference[i] = 600U;
        config->blackReference[i] = 3000U;
    }
    config->hitThreshold = (int16_t)(LINE_SENSOR_NORM_MAX / 2);
}

bool LineSensor_Init(LineSensor_State *s, const LineSensor_Config *config)
{
    uint8_t i;

    if ((s == 0) || (config == 0)) {
        return false;
    }

    s->config = *config;
    s->mask = 0U;
    s->hitCount = 0U;
    s->error = 0.0f;
    s->lastValidError = 0.0f;
    s->lineFound = false;
    s->allBlack = false;
    s->allWhite = true;
    for (i = 0U; i < LINE_SENSOR_CHANNELS; ++i) {
        s->normalized[i] = 0;
    }
    return true;
}

/*
 * 加权平均：把所有命中探头的权重求和后除以命中数，
 * 而不是只取最靠边的那一颗。这样在跨两颗探头之间时偏差连续变化，
 * 外环微分项不会出现阶跃跳变。
 */
static void LineSensor_ComputeError(LineSensor_State *s)
{
    int32_t weightedSum = 0;
    uint8_t count = 0U;
    uint8_t i;

    for (i = 0U; i < LINE_SENSOR_CHANNELS; ++i) {
        if ((s->mask & (uint8_t)(1U << i)) != 0U) {
            weightedSum += s->config.weight[i];
            count++;
        }
    }

    s->hitCount = count;
    s->allBlack = (count == LINE_SENSOR_CHANNELS);
    s->allWhite = (count == 0U);

    if (count == 0U) {
        /* 丢线：沿用最近一次有效偏差继续找线，不要把偏差清零停在原地。 */
        s->lineFound = false;
        s->error = s->lastValidError;
        return;
    }

    s->lineFound = true;
    s->error = (float)weightedSum / (float)count;
    s->lastValidError = s->error;
}

void LineSensor_UpdateMask(LineSensor_State *s, uint8_t mask)
{
    if (s == 0) {
        return;
    }
    s->mask = mask;
    LineSensor_ComputeError(s);
}

void LineSensor_UpdateAnalog(LineSensor_State *s, const uint16_t *adcValues)
{
    uint8_t i;

    if ((s == 0) || (adcValues == 0)) {
        return;
    }

    s->mask = 0U;
    for (i = 0U; i < LINE_SENSOR_CHANNELS; ++i) {
        int32_t white = (int32_t)s->config.whiteReference[i];
        int32_t black = (int32_t)s->config.blackReference[i];
        int32_t span = black - white;
        int32_t norm;

        if (span == 0) {
            /* 该路未标定或探头失效：归一化为 0，等效于永不命中。 */
            s->normalized[i] = 0;
            continue;
        }

        /*
         * 归一化到 0~1000：0 = 完全白底，1000 = 完全黑线。
         * span 为负时（白底 ADC 更高的探头）公式同样成立，
         * 因为分子分母同时变号。
         */
        norm = ((int32_t)adcValues[i] - white) * LINE_SENSOR_NORM_MAX / span;
        norm = ChessCar_ClampInt32(norm, 0, LINE_SENSOR_NORM_MAX);
        s->normalized[i] = (int16_t)norm;

        if (norm >= s->config.hitThreshold) {
            s->mask |= (uint8_t)(1U << i);
        }
    }

    LineSensor_ComputeError(s);
}

void LineSensor_Calibrate(LineSensor_State *s, const uint16_t *adcValues, bool isBlack)
{
    uint8_t i;

    if ((s == 0) || (adcValues == 0)) {
        return;
    }

    for (i = 0U; i < LINE_SENSOR_CHANNELS; ++i) {
        if (isBlack) {
            s->config.blackReference[i] = adcValues[i];
        } else {
            s->config.whiteReference[i] = adcValues[i];
        }
    }
}

float LineSensor_GetError(const LineSensor_State *s)
{
    return (s != 0) ? s->error : 0.0f;
}
uint8_t LineSensor_GetMask(const LineSensor_State *s)
{
    return (s != 0) ? s->mask : 0U;
}
bool LineSensor_IsLineFound(const LineSensor_State *s)
{
    return (s != 0) && s->lineFound;
}
bool LineSensor_IsAllBlack(const LineSensor_State *s)
{
    return (s != 0) && s->allBlack;
}
bool LineSensor_IsAllWhite(const LineSensor_State *s)
{
    return (s != 0) && s->allWhite;
}
