/**
 * @file uart_link.h
 * @brief STM32 HAL 串口不定长收发（ReceiveToIdle_DMA + 环形缓冲）。
 *
 * 方案选择说明：
 *   - 传统 RXNE 中断每收 1 字节进一次中断，115200 下满载时中断压力大，
 *     会影响 10ms 控制节拍的实时性。
 *   - IDLE 空闲中断 + DMA：DMA 搬数据不占 CPU，总线静默一个帧时间后
 *     触发一次回调，115200 下约为最后一个字符后 100us 通知应用。
 *   - HAL 已封装为 HAL_UARTEx_ReceiveToIdle_DMA + HAL_UARTEx_RxEventCallback，
 *     不需要自己写 IDLE 标志处理。
 *
 * 关键坑（务必注意）：
 *   1. DMA 设为 Circular 时，必须关掉半传输中断（HT），否则一帧会回调两次。
 *   2. 回调里**只做数据搬运进环形缓冲**，协议解析放到主循环，
 *      不要在中断里跑 CRC 校验和业务逻辑。
 *   3. 发生 ORE/FE/NE 错误后 HAL 会中止传输，必须重新启动接收，
 *      本模块的 UartLink_OnError 负责这件事。
 *
 * 参考：
 *   - ST 官方社区《STM32 hal库串口空闲中断最新用法》
 *     https://shequ.stmicroelectronics.cn/thread-637349-1-1.html
 *   - MaJerle/stm32-usart-uart-dma-rx-tx
 *     https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 */
#ifndef UART_LINK_H
#define UART_LINK_H

#include "bsp_config.h"
#include "link_protocol.h"

/** 初始化环形缓冲、绑定协议上下文并启动 DMA 接收。 */
void UartLink_Init(LinkProtocol_ReceiveCallback onFrame);

/**
 * @brief 在主循环中调用：把环形缓冲里的字节喂给协议状态机。
 *
 * 必须每个主循环都调用，否则环形缓冲会被 DMA 覆盖写满。
 */
void UartLink_Poll(void);

/** 发送一个协议帧（阻塞式，长度很短，1~40 字节，耗时约 3ms@115200）。 */
bool UartLink_SendFrame(uint8_t type, uint8_t flags,
                        const uint8_t *payload, uint8_t length);

/** 诊断：返回 CRC 错误累计数。持续增长说明线路有干扰或波特率不匹配。 */
uint16_t UartLink_GetCrcErrorCount(void);
uint16_t UartLink_GetValidFrameCount(void);
/** 诊断：环形缓冲溢出次数。非零说明主循环调用 UartLink_Poll 太慢。 */
uint16_t UartLink_GetOverrunCount(void);

/** 在 HAL_UARTEx_RxEventCallback 中调用。 */
void UartLink_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size);
/** 在 HAL_UART_ErrorCallback 中调用，用于重启接收。 */
void UartLink_OnError(UART_HandleTypeDef *huart);

#endif /* UART_LINK_H */
