/**
 * @file adc_line_sensor.h
 * @brief 8 路光电探头 ADC DMA 驱动（STM32 HAL 侧）。
 *
 * 负责启动 ADC1 DMA Circular 扫描并维护一个"最新已完整采样"的快照缓冲，
 * 调用者只需在控制周期（10ms）取快照并传给 LineSensor_UpdateAnalog()。
 *
 * 仅在 BSP_LINE_SENSOR_USE_ADC == 1 时生效；数字量 DO 版本不需要本文件。
 *
 * =======================================================================
 * CubeMX 配置要点（对应 bsp_config.h 注释第 6 项）：
 *
 *   ADC1 → Parameter Settings：
 *     - Scan Conversion Mode       : Enable     ← 多通道必须开；不开则只转换 Rank1
 *     - Continuous Conversion Mode : Enable     ← 不开则只做一轮扫描后停机
 *     - Number of Conversions      : 8
 *     - Rank 1~8 依次配置 8 路探头对应的通道
 *     - Sampling Time              : 239.5 Cycles（灰度探头信号源阻抗较高）
 *
 *   ADC1 → DMA Settings：
 *     - Add DMA Request : ADC1
 *     - Mode            : Circular
 *     - Peripheral Data Width : Half Word  ← ADC 12 位数据存在 16 位寄存器里
 *     - Memory Data Width     : Half Word  ← 两边必须一致，否则只读到低 8 位
 *     - 注意：F103 上 ADC1 固定用 DMA1_Channel1，
 *       与本工程的 USART1（DMA1_Ch4/Ch5）不冲突
 *
 *   F103 专项提示：
 *     - F103 没有"DMA Continuous Requests"这个开关（F4 以上才有），
 *       Circular 模式下 ADC 每次转换完毕自动触发下一次 DMA 请求，
 *       按参考 F4 教程找不到该选项时可以忽略。
 *
 * =======================================================================
 * 典型接入流程：
 *
 *   // main.c：在 MX_DMA_Init() 和 MX_ADC1_Init() 完成之后调用一次
 *   ADC_LineSensor_Init();
 *
 *   // examples/hal_callbacks.c 已包含下面这段，不需要在 main.c 再添加：
 *   // void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
 *   //     if (hadc->Instance == ADC1) ADC_LineSensor_OnConvCplt();
 *   // }
 *
 *   // app_chesscar.c（10ms 控制节拍）：
 *   uint16_t raw[BSP_LINE_SENSOR_CHANNELS];
 *   if (ADC_LineSensor_GetRaw(raw))
 *       LineSensor_UpdateAnalog(&g_lineSensor, raw);
 */
#ifndef ADC_LINE_SENSOR_H
#define ADC_LINE_SENSOR_H

#include "bsp_config.h"
#include <stdbool.h>
#include <stdint.h>

#if BSP_LINE_SENSOR_USE_ADC

/**
 * @brief 启动 ADC1 DMA Circular 扫描。
 *
 * 在 MX_DMA_Init() 和 MX_ADC1_Init() 之后调用一次。
 * Circular 模式下 DMA 会持续采样，无需再次调用。
 *
 * 重要顺序：MX_DMA_Init() 必须在 MX_ADC1_Init() 之前调用
 * （CubeMX 默认生成顺序已保证这一点）。
 */
void ADC_LineSensor_Init(void);

/**
 * @brief 取最新一次完整扫描的 8 路 ADC 原始值（12 位右对齐，范围 0~4095）。
 *
 * @param outBuf  调用者分配的 BSP_LINE_SENSOR_CHANNELS 个 uint16_t。
 * @return true   至少已完成一次完整扫描，outBuf 已更新；
 *         false  DMA 尚未触发过 ConvCplt（刚上电），outBuf 不变。
 *
 * 线程安全说明：outBuf 从内部快照缓冲拷出，主循环侧无需关中断。
 * 快照在 ConvCplt 中断里原子更新（memcpy 16 字节），
 * 对 10ms 控制周期而言最多持有上一帧数据，可以接受。
 */
bool ADC_LineSensor_GetRaw(uint16_t *outBuf);

/**
 * @brief 由 HAL_ADC_ConvCpltCallback 调用，把 DMA 缓冲快照到安全区。
 *
 * 请勿在其他地方重复调用。若工程中还有其他 ADC，请在
 * examples/hal_callbacks.c 的 HAL_ADC_ConvCpltCallback 里
 * 按 hadc->Instance 区分后调用。
 */
void ADC_LineSensor_OnConvCplt(void);

#endif /* BSP_LINE_SENSOR_USE_ADC */

#endif /* ADC_LINE_SENSOR_H */
