/**
 * @file hal_callbacks.c
 * @brief CubeMX HAL 回调的统一挂接点。
 *
 * 把这些回调集中放在一个文件里，可以避免每次 CubeMX 重新生成代码时
 * 在 main.c 的 USER CODE 区反复手工补回调而漏掉某一个。
 *
 * 使用方法：把本文件加入工程，确保 main.c 里**没有**重复定义同名回调。
 */
#include "app_chesscar.h"
#include "bsp_config.h"
#include "encoder_tim.h"
#include "uart_link.h"
#if BSP_LINE_SENSOR_USE_ADC
#include "adc_line_sensor.h"
#endif

/**
 * @brief 定时器周期中断。
 *
 * 这里同时承担两件事：
 *   - TIM4：1ms 控制节拍
 *   - TIM2/TIM3：编码器 16 位计数器溢出累计
 *
 * 中断内只做标志位与计数，绝不做 PID、浮点除法或 I2C 刷屏。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim4.Instance) {
        App_OnTick1ms();
        return;
    }

    /* 编码器定时器溢出。 */
    Encoder_OnTimerOverflow(htim);
}

/**
 * @brief 串口空闲事件 / 接收完成回调。
 *
 * size 为本次收到的字节数。只搬运进环形缓冲，解析放主循环。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    UartLink_OnRxEvent(huart, size);
}

/**
 * @brief 串口错误回调。
 *
 * ORE（溢出）在实际调试中非常常见：上位机在 STM32 复位期间持续发数据就会触发。
 * 不在这里重启接收，串口就会「只能用一次」然后彻底哑掉。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UartLink_OnError(huart);
}

/**
 * @brief ADC 转换完成回调。
 *
 * ADC1 在 Circular DMA 模式下每完成一次完整的 8 通道扫描触发一次。
 * 只在模拟量探头模式（BSP_LINE_SENSOR_USE_ADC == 1）下编译。
 *
 * 若工程中还有其他 ADC 实例，在此处按 hadc->Instance 区分后分别处理。
 */
#if BSP_LINE_SENSOR_USE_ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        ADC_LineSensor_OnConvCplt();
    }
}
#endif /* BSP_LINE_SENSOR_USE_ADC */
