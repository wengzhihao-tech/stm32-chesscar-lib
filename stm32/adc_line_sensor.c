/**
 * @file adc_line_sensor.c
 * @brief 8 路光电探头 ADC DMA 驱动实现。
 *
 * 核心思路：
 *   - DMA 在 Circular 模式下持续把 ADC 结果写入 s_dmaBuf（硬件直写）；
 *   - 每扫完 8 路，HAL 触发 ConvCpltCallback；
 *   - 回调里把 s_dmaBuf 快照到 s_snapBuf，主循环只从 s_snapBuf 读。
 *
 * 为什么需要双缓冲（见 docs/PITFALLS.md 坑 29）：
 *   ADC DMA Circular 模式下，DMA 硬件在主循环读取期间可能正在写 s_dmaBuf，
 *   直接读会得到"前 N 路是新值、后 M 路是上轮值"的混合帧，
 *   归一化结果出现跳变，表现和探头松动一模一样，极难排查。
 *   在中断里 memcpy 16 字节（F103 @ 72MHz 约 0.2~0.4µs）是最简单的解法。
 */
#include "adc_line_sensor.h"

#if BSP_LINE_SENSOR_USE_ADC

#include <string.h>

/* ---- 内部状态 ---- */

/** DMA 直接写入的原始缓冲（Circular 模式下由硬件持续更新）。*/
static uint16_t s_dmaBuf[BSP_LINE_SENSOR_CHANNELS];

/** 每次 ConvCplt 时拷出的完整快照，供主循环读取。*/
static uint16_t s_snapBuf[BSP_LINE_SENSOR_CHANNELS];

/** 防止上电后主循环在 DMA 首次完成前读到未初始化的 s_snapBuf。*/
static volatile bool s_dataReady = false;

/* ---- 公共接口 ---- */

void ADC_LineSensor_Init(void)
{
    /*
     * HAL_ADC_Start_DMA 在 Circular 模式下只需调用一次：
     * 每次 DMA 传输完成后，DMA 控制器自动把地址指针回绕到缓冲头部，
     * 无需在 ConvCplt 回调里重新启动。
     *
     * 参数说明：
     *   &BSP_LINE_SENSOR_ADC  — hadc1，在 bsp_config.h 中声明
     *   (uint32_t *)s_dmaBuf  — DMA 目标内存（必须 4 字节对齐；静态数组已满足）
     *   BSP_LINE_SENSOR_CHANNELS — 传输元素数（每个元素 Half Word，即 16 位）
     */
    HAL_ADC_Start_DMA(&BSP_LINE_SENSOR_ADC,
                      (uint32_t *)s_dmaBuf,
                      BSP_LINE_SENSOR_CHANNELS);
}

bool ADC_LineSensor_GetRaw(uint16_t *outBuf)
{
    if (!s_dataReady) {
        return false;
    }
    /* 从快照缓冲拷出；主循环侧不需要关中断，因为快照已经是完整的一帧。*/
    memcpy(outBuf, s_snapBuf, BSP_LINE_SENSOR_CHANNELS * sizeof(uint16_t));
    return true;
}

void ADC_LineSensor_OnConvCplt(void)
{
    /*
     * 在中断上下文里把 DMA 缓冲快照一次。
     *
     * 此时 DMA 刚完成一次完整扫描（8 路全部写入 s_dmaBuf），
     * Circular 模式下 DMA 指针即将回绕到 s_dmaBuf[0] 重新开始写；
     * 我们在它开始覆盖之前把 8 个值拷到快照缓冲。
     *
     * F103 的 ADC 转换速率8 通道x239.5 采样周期 @ 12MHz ADC 时钟）
     * 约每 160µs 完成一轮，远快于 10ms 控制周期，所以主循环每 10ms
     * 拿到的是最近 160µs 内的采样结果。
     */
    memcpy(s_snapBuf, s_dmaBuf, BSP_LINE_SENSOR_CHANNELS * sizeof(uint16_t));
    s_dataReady = true;
}

#endif /* BSP_LINE_SENSOR_USE_ADC */
