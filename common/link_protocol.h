/**
 * @file link_protocol.h
 * @brief STM32 与上位机/视觉板之间的二进制帧协议（帧头 + CRC16 + 逐字节状态机）。
 *
 * 来源：从本机 smartcar 工程的 vision_protocol 移植，已在 MaixCAM 视觉链路上验证。
 *
 * 帧格式（小端，固定宽度整数，不传结构体）：
 *   [0] SOF1 = 0xA5
 *   [1] SOF2 = 0x5A
 *   [2] type      消息类型
 *   [3] sequence  递增序号，用于 ACK 与丢帧诊断
 *   [4] flags     ACK 请求 / ACK 响应 / 错误标志
 *   [5] length    payload 字节数，最大 32
 *   [6..] payload
 *   [末尾 2 字节] CRC16-CCITT-FALSE，低字节在前
 *
 * CRC 覆盖 type/sequence/flags/length/payload，**不含两个帧头字节**。
 *
 * 为什么用状态机逐字节喂而不是整包强转：
 *   串口会粘包、拆包、丢字节；直接把缓冲区强转成结构体会受大小端与
 *   结构体对齐影响，换编译器就崩。逐字节状态机可以在任何噪声后重新同步。
 */
#ifndef LINK_PROTOCOL_H
#define LINK_PROTOCOL_H

#include "chesscar_common.h"

#define LINK_PROTOCOL_SOF1             (0xA5U)
#define LINK_PROTOCOL_SOF2             (0x5AU)
#define LINK_PROTOCOL_MAX_PAYLOAD      (32U)
#define LINK_PROTOCOL_FRAME_OVERHEAD   (8U)

#define LINK_FLAG_ACK_REQUEST          (0x01U)
#define LINK_FLAG_ACK                  (0x02U)
#define LINK_FLAG_ERROR                (0x04U)

typedef enum {
    LINK_MSG_HELLO          = 0x01,
    LINK_MSG_HEARTBEAT      = 0x02,
    LINK_MSG_CAR_STATUS     = 0x10, /**< STM32 → 上位机：速度、里程、故障码。 */
    LINK_MSG_VISION_RESULT  = 0x11, /**< 视觉板 → STM32：通用目标识别结果。 */
    LINK_MSG_LINE_RESULT    = 0x13, /**< 视觉板 → STM32：黑线循迹结果，与通用结果分开。 */
    LINK_MSG_CHESS_MOVE     = 0x18, /**< 上位机 → STM32：象棋走子指令（起点/终点格）。 */
    LINK_MSG_TASK_COMMAND   = 0x20,
    LINK_MSG_COMMAND_ACK    = 0x21,
    LINK_MSG_PID_TUNE       = 0x30, /**< 上位机 → STM32：在线改 PID 增益。 */
    LINK_MSG_ERROR          = 0x7F
} LinkProtocol_MessageType;

typedef struct {
    uint8_t type;
    uint8_t sequence;
    uint8_t flags;
    uint8_t length;
    uint8_t payload[LINK_PROTOCOL_MAX_PAYLOAD];
} LinkProtocol_Frame;

/** 底层整帧入队回调，**禁止阻塞**（用 DMA 或环形缓冲发送）。 */
typedef bool (*LinkProtocol_SendCallback)(const uint8_t *data, uint8_t length);
/** 收到一个 CRC 正确的完整帧时回调。 */
typedef void (*LinkProtocol_ReceiveCallback)(const LinkProtocol_Frame *frame);

typedef enum {
    LINK_PARSER_WAIT_SOF1 = 0,
    LINK_PARSER_WAIT_SOF2,
    LINK_PARSER_READ_TYPE,
    LINK_PARSER_READ_SEQUENCE,
    LINK_PARSER_READ_FLAGS,
    LINK_PARSER_READ_LENGTH,
    LINK_PARSER_READ_PAYLOAD,
    LINK_PARSER_READ_CRC_LOW,
    LINK_PARSER_READ_CRC_HIGH
} LinkProtocol_ParserState;

typedef struct {
    LinkProtocol_SendCallback send;
    LinkProtocol_ReceiveCallback receive;
    LinkProtocol_Frame rxFrame;
    LinkProtocol_ParserState parserState;
    uint16_t calculatedCrc;
    uint16_t receivedCrc;
    uint16_t validFrameCount;   /**< 诊断用：CRC 正确的完整帧数。 */
    uint16_t crcErrorCount;     /**< 诊断用：CRC 错误数，持续增长说明线路有干扰。 */
    uint16_t lengthErrorCount;  /**< 诊断用：超长负载数。 */
    uint8_t payloadIndex;
    uint8_t nextTxSequence;
} LinkProtocol_Context;

void LinkProtocol_Init(LinkProtocol_Context *ctx,
                       LinkProtocol_SendCallback sendCallback,
                       LinkProtocol_ReceiveCallback receiveCallback);

/** 从 UART 环形缓冲逐字节喂入，可处理拆包与粘包。 */
void LinkProtocol_ProcessByte(LinkProtocol_Context *ctx, uint8_t byte);

/** 批量喂入，内部仍是逐字节状态机。 */
void LinkProtocol_ProcessBuffer(LinkProtocol_Context *ctx, const uint8_t *data, uint16_t length);

uint8_t LinkProtocol_AllocateSequence(LinkProtocol_Context *ctx);

bool LinkProtocol_SendFrame(LinkProtocol_Context *ctx, uint8_t type,
                            uint8_t sequence, uint8_t flags,
                            const uint8_t *payload, uint8_t length);

/** CRC16-CCITT-FALSE：poly=0x1021，init=0xFFFF，结果不反转、不异或。 */
uint16_t LinkProtocol_Crc16(const uint8_t *data, uint8_t length);

#endif /* LINK_PROTOCOL_H */
