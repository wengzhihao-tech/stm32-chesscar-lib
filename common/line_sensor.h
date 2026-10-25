/**
 * @file line_sensor.h
 * @brief 8 路光电/灰度探头的归一化、加权偏差计算与丢线处理（平台无关）。
 *
 * 两种使用方式：
 *   A. 数字量 DO：直接喂 8 位掩码 LineSensor_UpdateMask()
 *   B. 模拟量 AO：喂 ADC 原始值 LineSensor_UpdateAnalog()，模块内部按
 *      黑/白标定值归一化到 0~1000 后再按阈值生成掩码
 *
 * 探头编号约定：**bit0 = 最左侧探头**（车头视角），bit7 = 最右侧。
 * 权重左负右正，因此偏差 error > 0 表示线在车的右边，车需要右转。
 *
 * 参考：博客园 8 路灰度教学工程的加权平均法
 *       https://www.cnblogs.com/zw-awa/p/23366990
 *       要点：权重取 -7,-5,-3,-1,1,3,5,7，error = 权重和 / 命中数。
 */
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "chesscar_common.h"

#define LINE_SENSOR_CHANNELS      (8U)
#define LINE_SENSOR_NORM_MAX      (1000)  /**< 归一化满量程。 */

typedef struct {
    /** 每路探头的黑线 ADC 读数（标定得到）。 */
    uint16_t blackReference[LINE_SENSOR_CHANNELS];
    /** 每路探头的白底 ADC 读数（标定得到）。 */
    uint16_t whiteReference[LINE_SENSOR_CHANNELS];
    /** 归一化值超过此阈值判为压线，建议 500（即黑白中点）。 */
    int16_t hitThreshold;
    /** 探头权重，左负右正；默认 -7,-5,-3,-1,1,3,5,7。 */
    int8_t weight[LINE_SENSOR_CHANNELS];
} LineSensor_Config;

typedef struct {
    LineSensor_Config config;
    uint8_t mask;                                  /**< 当前命中掩码，bit0 为最左。 */
    int16_t normalized[LINE_SENSOR_CHANNELS];      /**< 归一化后的 0~1000 值。 */
    uint8_t hitCount;                              /**< 本次命中探头数。 */
    float error;                                   /**< 加权偏差，左负右正。 */
    float lastValidError;                          /**< 最近一次有效偏差，丢线时沿用。 */
    bool lineFound;                                /**< 本次是否检测到线。 */
    bool allBlack;                                 /**< 全黑：到达终点标记或压上横线。 */
    bool allWhite;                                 /**< 全白：丢线。 */
} LineSensor_State;

/** 载入默认权重与阈值；黑白标定值仍需调用者填写或现场标定。 */
void LineSensor_LoadDefaultConfig(LineSensor_Config *config);

bool LineSensor_Init(LineSensor_State *s, const LineSensor_Config *config);

/** 数字量输入：直接给 8 位掩码（DO 读到黑线为 1）。 */
void LineSensor_UpdateMask(LineSensor_State *s, uint8_t mask);

/** 模拟量输入：给 8 路 ADC 原始值，内部按标定值归一化后生成掩码。 */
void LineSensor_UpdateAnalog(LineSensor_State *s, const uint16_t *adcValues);

/**
 * @brief 现场标定一侧参考值。
 *
 * 用法：把车放在纯白底上调用 LineSensor_Calibrate(s, adc, false)，
 *       再把 8 路全部压到黑线上调用 LineSensor_Calibrate(s, adc, true)。
 */
void LineSensor_Calibrate(LineSensor_State *s, const uint16_t *adcValues, bool isBlack);

float LineSensor_GetError(const LineSensor_State *s);
uint8_t LineSensor_GetMask(const LineSensor_State *s);
bool LineSensor_IsLineFound(const LineSensor_State *s);
bool LineSensor_IsAllBlack(const LineSensor_State *s);
bool LineSensor_IsAllWhite(const LineSensor_State *s);

#endif /* LINE_SENSOR_H */
