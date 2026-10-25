/**
 * @file uart_link.c
 * @brief 串口 DMA 空闲中断接收 + 环形缓冲实现。
 */
#include "uart_link.h"

#include <string.h>

static uint8_t g_dmaBuffer[BSP_LINK_RX_DMA_BUFFER_SIZE];
static uint8_t g_ringBuffer[BSP_LINK_RING_BUFFER_SIZE];
static volatile uint16_t g_ringHead;   /* 中断写入位置 */
static volatile uint16_t g_ringTail;   /* 主循环读取位置 */
static volatile uint16_t g_overrunCount;

static LinkProtocol_Context g_link;

static bool UartLink_RawSend(const uint8_t *data, uint8_t length)
{
    /*
     * 协议帧最长 40 字节，115200 下约 3.5ms，阻塞发送对 10ms 控制周期
     * 影响可接受。若要发高频遥测，应改为 HAL_UART_Transmit_DMA 并加发送队列。
     */
    return HAL_UART_Transmit(&BSP_LINK_UART, (uint8_t *)data, length, 50U) == HAL_OK;
}

void UartLink_Init(LinkProtocol_ReceiveCallback onFrame)
{
    g_ringHead = 0U;
    g_ringTail = 0U;
    g_overrunCount = 0U;

    LinkProtocol_Init(&g_link, UartLink_RawSend, onFrame);

    HAL_UARTEx_ReceiveToIdle_DMA(&BSP_LINK_UART, g_dmaBuffer, BSP_LINK_RX_DMA_BUFFER_SIZE);

    /*
     * 关掉 DMA 半传输中断。
     * 不关的话，一帧数据在到达缓冲区一半时就会触发一次 RxEventCallback，
     * 同一帧被处理两次，表现为「每条指令都执行两遍」。
     */
    __HAL_DMA_DISABLE_IT(BSP_LINK_UART.hdmarx, DMA_IT_HT);
}

void UartLink_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t i;

    if (huart->Instance != BSP_LINK_UART.Instance) {
        return;
    }

    /* 中断里只搬字节，不做任何解析。 */
    for (i = 0U; i < size; ++i) {
        uint16_t next = (uint16_t)((g_ringHead + 1U) % BSP_LINK_RING_BUFFER_SIZE);
        if (next == g_ringTail) {
            /* 环形缓冲满：主循环处理太慢，记录并丢弃最新字节。 */
            g_overrunCount++;
            break;
        }
        g_ringBuffer[g_ringHead] = g_dmaBuffer[i];
        g_ringHead = next;
    }

    /* 重新武装接收。NORMAL 模式下必须重启，Circular 模式下也建议显式重启。 */
    HAL_UARTEx_ReceiveToIdle_DMA(&BSP_LINK_UART, g_dmaBuffer, BSP_LINK_RX_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(BSP_LINK_UART.hdmarx, DMA_IT_HT);
}

void UartLink_OnError(UART_HandleTypeDef *huart)
{
    if (huart->Instance != BSP_LINK_UART.Instance) {
        return;
    }

    /*
     * ORE/FE/NE 等错误后 HAL 会把状态置为 READY 并停止接收。
     * 不重启的话串口会「只能用一次」，这是新手最常踩的坑。
     */
    HAL_UART_AbortReceive(&BSP_LINK_UART);
    HAL_UARTEx_ReceiveToIdle_DMA(&BSP_LINK_UART, g_dmaBuffer, BSP_LINK_RX_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(BSP_LINK_UART.hdmarx, DMA_IT_HT);
}

void UartLink_Poll(void)
{
    while (g_ringTail != g_ringHead) {
        uint8_t byte = g_ringBuffer[g_ringTail];
        g_ringTail = (uint16_t)((g_ringTail + 1U) % BSP_LINK_RING_BUFFER_SIZE);
        LinkProtocol_ProcessByte(&g_link, byte);
    }
}

bool UartLink_SendFrame(uint8_t type, uint8_t flags,
                        const uint8_t *payload, uint8_t length)
{
    uint8_t sequence = LinkProtocol_AllocateSequence(&g_link);
    return LinkProtocol_SendFrame(&g_link, type, sequence, flags, payload, length);
}

uint16_t UartLink_GetCrcErrorCount(void)   { return g_link.crcErrorCount; }
uint16_t UartLink_GetValidFrameCount(void) { return g_link.validFrameCount; }
uint16_t UartLink_GetOverrunCount(void)    { return g_overrunCount; }
