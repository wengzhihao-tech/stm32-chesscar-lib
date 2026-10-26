/**
 * @file vision_chess.c
 * @brief 棋子识别结果解析实现。
 */
#include "vision_chess.h"

#define VISION_X_SCALE_CM (2.0f) /* X 坐标量化步长 */

bool Vision_ParseChessFrame(const LinkProtocol_Frame *frame, Vision_Frame *out)
{
    uint8_t i;
    uint8_t count;

    if ((frame == 0) || (out == 0)) {
        return false;
    }

    out->valid = false;
    out->count = 0U;

    if (frame->type != LINK_MSG_CHESS_PIECES) {
        return false;
    }
    if (frame->length < 1U) {
        return false;
    }

    count = frame->payload[0];
    if (count > VISION_MAX_PIECES_PER_FRAME) {
        /*
         * 数量字段异常：可能是视觉板版本不匹配或数据损坏。
         * 这里截断而不是整帧丢弃，尽量保住能用的数据。
         */
        count = VISION_MAX_PIECES_PER_FRAME;
    }

    /* 长度自检：声明的棋子数必须与实际 payload 长度相符。 */
    if (frame->length < (uint8_t)(1U + count * 5U)) {
        return false;
    }

    for (i = 0U; i < count; ++i) {
        const uint8_t *p = &frame->payload[1U + i * 5U];
        uint8_t colorRaw = p[2];
        uint8_t levelRaw = p[3];

        out->pieces[i].x = (float)p[0] * VISION_X_SCALE_CM;
        out->pieces[i].y = (float)p[1];

        out->pieces[i].color = (colorRaw == 1U) ? CHESS_COLOR_RED :
                               (colorRaw == 2U) ? CHESS_COLOR_BLACK :
                                                  CHESS_COLOR_UNKNOWN;

        /* 等级越界时退化为未知，而不是产生一个非法枚举值。 */
        out->pieces[i].level = (levelRaw <= (uint8_t)CHESS_LEVEL_GENERAL)
                             ? (Chess_Level)levelRaw
                             : CHESS_LEVEL_UNKNOWN;

        out->pieces[i].confidence = (p[4] > 100U) ? 100U : p[4];
    }

    out->count = count;
    out->valid = (count > 0U);
    return out->valid;
}

uint8_t Vision_FeedToManager(const Vision_Frame *frame,
                             Chess_TargetManager *mgr,
                             uint8_t minConfidence,
                             uint32_t nowMs)
{
    uint8_t i;
    uint8_t accepted = 0U;

    if ((frame == 0) || (mgr == 0) || !frame->valid) {
        return 0U;
    }

    for (i = 0U; i < frame->count; ++i) {
        const Vision_PieceReport *r = &frame->pieces[i];

        /* 低置信度观测直接丢弃，避免追着误检跑。 */
        if (r->confidence < minConfidence) {
            continue;
        }

        /* 场地外的坐标必然是标定错误或误检。 */
        if ((r->x < 0.0f) || (r->x > FIELD_LENGTH_CM) ||
            (r->y < 0.0f) || (r->y > FIELD_WIDTH_CM)) {
            continue;
        }

        if (Chess_Observe(mgr, r->x, r->y, r->color, r->level, nowMs) >= 0) {
            accepted++;
        }
    }

    return accepted;
}

uint8_t Vision_BuildChessPayload(const Vision_Frame *frame,
                                 uint8_t *payload, uint8_t maxLen)
{
    uint8_t i;
    uint8_t needed;

    if ((frame == 0) || (payload == 0) || !frame->valid) {
        return 0U;
    }

    needed = (uint8_t)(1U + frame->count * 5U);
    if (maxLen < needed) {
        return 0U;
    }

    payload[0] = frame->count;
    for (i = 0U; i < frame->count; ++i) {
        uint8_t *p = &payload[1U + i * 5U];
        float xq = frame->pieces[i].x / VISION_X_SCALE_CM;
        float yq = frame->pieces[i].y;

        p[0] = (uint8_t)ChessCar_ClampFloat(xq, 0.0f, 255.0f);
        p[1] = (uint8_t)ChessCar_ClampFloat(yq, 0.0f, 255.0f);
        p[2] = (frame->pieces[i].color == CHESS_COLOR_RED) ? 1U :
               (frame->pieces[i].color == CHESS_COLOR_BLACK) ? 2U : 0U;
        p[3] = (uint8_t)frame->pieces[i].level;
        p[4] = frame->pieces[i].confidence;
    }

    return needed;
}
