/**
 * @file link_protocol.c
 * @brief 二进制帧编解码与 CRC 校验实现。
 */
#include "link_protocol.h"

static uint16_t LinkProtocol_CrcUpdate(uint16_t crc, uint8_t data)
{
    uint8_t bit;

    crc ^= (uint16_t)data << 8U;
    for (bit = 0U; bit < 8U; ++bit) {
        crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                                      : (uint16_t)(crc << 1U);
    }
    return crc;
}

uint16_t LinkProtocol_Crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t index;

    if ((data == 0) && (length != 0U)) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        crc = LinkProtocol_CrcUpdate(crc, data[index]);
    }
    return crc;
}

void LinkProtocol_Init(LinkProtocol_Context *ctx,
                       LinkProtocol_SendCallback sendCallback,
                       LinkProtocol_ReceiveCallback receiveCallback)
{
    if (ctx == 0) {
        return;
    }

    ctx->send = sendCallback;
    ctx->receive = receiveCallback;
    ctx->parserState = LINK_PARSER_WAIT_SOF1;
    ctx->calculatedCrc = 0xFFFFU;
    ctx->receivedCrc = 0U;
    ctx->validFrameCount = 0U;
    ctx->crcErrorCount = 0U;
    ctx->lengthErrorCount = 0U;
    ctx->payloadIndex = 0U;
    ctx->nextTxSequence = 0U;
}

/*
 * 复位解析器时的小技巧：
 * 如果导致复位的那个字节恰好等于 SOF1，就把它当作下一帧的起点，
 * 这样在粘包或丢字节后能少等一帧完成重新同步。
 */
static void LinkProtocol_ResetParser(LinkProtocol_Context *ctx, uint8_t lastByte)
{
    ctx->parserState = (lastByte == LINK_PROTOCOL_SOF1) ? LINK_PARSER_WAIT_SOF2
                                                        : LINK_PARSER_WAIT_SOF1;
    ctx->payloadIndex = 0U;
    ctx->calculatedCrc = 0xFFFFU;
}

void LinkProtocol_ProcessByte(LinkProtocol_Context *ctx, uint8_t byte)
{
    if (ctx == 0) {
        return;
    }

    switch (ctx->parserState) {
        case LINK_PARSER_WAIT_SOF1:
            if (byte == LINK_PROTOCOL_SOF1) {
                ctx->parserState = LINK_PARSER_WAIT_SOF2;
            }
            break;

        case LINK_PARSER_WAIT_SOF2:
            if (byte == LINK_PROTOCOL_SOF2) {
                ctx->parserState = LINK_PARSER_READ_TYPE;
                ctx->calculatedCrc = 0xFFFFU;
            } else if (byte != LINK_PROTOCOL_SOF1) {
                ctx->parserState = LINK_PARSER_WAIT_SOF1;
            }
            break;

        case LINK_PARSER_READ_TYPE:
            ctx->rxFrame.type = byte;
            ctx->calculatedCrc = LinkProtocol_CrcUpdate(ctx->calculatedCrc, byte);
            ctx->parserState = LINK_PARSER_READ_SEQUENCE;
            break;

        case LINK_PARSER_READ_SEQUENCE:
            ctx->rxFrame.sequence = byte;
            ctx->calculatedCrc = LinkProtocol_CrcUpdate(ctx->calculatedCrc, byte);
            ctx->parserState = LINK_PARSER_READ_FLAGS;
            break;

        case LINK_PARSER_READ_FLAGS:
            ctx->rxFrame.flags = byte;
            ctx->calculatedCrc = LinkProtocol_CrcUpdate(ctx->calculatedCrc, byte);
            ctx->parserState = LINK_PARSER_READ_LENGTH;
            break;

        case LINK_PARSER_READ_LENGTH:
            ctx->rxFrame.length = byte;
            ctx->calculatedCrc = LinkProtocol_CrcUpdate(ctx->calculatedCrc, byte);
            ctx->payloadIndex = 0U;
            if (byte > LINK_PROTOCOL_MAX_PAYLOAD) {
                ctx->lengthErrorCount++;
                LinkProtocol_ResetParser(ctx, byte);
            } else {
                ctx->parserState = (byte == 0U) ? LINK_PARSER_READ_CRC_LOW
                                                : LINK_PARSER_READ_PAYLOAD;
            }
            break;

        case LINK_PARSER_READ_PAYLOAD:
            ctx->rxFrame.payload[ctx->payloadIndex++] = byte;
            ctx->calculatedCrc = LinkProtocol_CrcUpdate(ctx->calculatedCrc, byte);
            if (ctx->payloadIndex >= ctx->rxFrame.length) {
                ctx->parserState = LINK_PARSER_READ_CRC_LOW;
            }
            break;

        case LINK_PARSER_READ_CRC_LOW:
            ctx->receivedCrc = byte;
            ctx->parserState = LINK_PARSER_READ_CRC_HIGH;
            break;

        case LINK_PARSER_READ_CRC_HIGH:
            ctx->receivedCrc |= (uint16_t)byte << 8U;
            if (ctx->receivedCrc == ctx->calculatedCrc) {
                ctx->validFrameCount++;
                if (ctx->receive != 0) {
                    ctx->receive(&ctx->rxFrame);
                }
            } else {
                /* CRC 错误帧绝不能刷新上层的目标有效期，直接丢弃。 */
                ctx->crcErrorCount++;
            }
            LinkProtocol_ResetParser(ctx, byte);
            break;

        default:
            LinkProtocol_ResetParser(ctx, byte);
            break;
    }
}

void LinkProtocol_ProcessBuffer(LinkProtocol_Context *ctx, const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if ((ctx == 0) || (data == 0)) {
        return;
    }
    for (i = 0U; i < length; ++i) {
        LinkProtocol_ProcessByte(ctx, data[i]);
    }
}

uint8_t LinkProtocol_AllocateSequence(LinkProtocol_Context *ctx)
{
    uint8_t sequence;

    if (ctx == 0) {
        return 0U;
    }
    sequence = ctx->nextTxSequence;
    ctx->nextTxSequence++; /* 溢出自然回绕 */
    return sequence;
}

bool LinkProtocol_SendFrame(LinkProtocol_Context *ctx, uint8_t type,
                            uint8_t sequence, uint8_t flags,
                            const uint8_t *payload, uint8_t length)
{
    uint8_t frame[LINK_PROTOCOL_MAX_PAYLOAD + LINK_PROTOCOL_FRAME_OVERHEAD];
    uint16_t crc;
    uint8_t index;

    if ((ctx == 0) || (ctx->send == 0) ||
        (length > LINK_PROTOCOL_MAX_PAYLOAD) ||
        ((payload == 0) && (length != 0U))) {
        return false;
    }

    /* 逐字段写字节数组，不复制 C 结构体，避免大小端与对齐问题。 */
    frame[0] = LINK_PROTOCOL_SOF1;
    frame[1] = LINK_PROTOCOL_SOF2;
    frame[2] = type;
    frame[3] = sequence;
    frame[4] = flags;
    frame[5] = length;
    for (index = 0U; index < length; ++index) {
        frame[6U + index] = payload[index];
    }
    crc = LinkProtocol_Crc16(&frame[2], (uint8_t)(4U + length));
    frame[6U + length] = (uint8_t)(crc & 0xFFU);
    frame[7U + length] = (uint8_t)(crc >> 8U);

    return ctx->send(frame, (uint8_t)(LINK_PROTOCOL_FRAME_OVERHEAD + length));
}
