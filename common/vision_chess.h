/**
 * @file vision_chess.h
 * @brief 视觉板 → STM32 的棋子识别结果解析（扩展 link_protocol 的消息类型）。
 *
 * 架构分工（双主控）：
 *   视觉板（K210 / MaixCAM / OpenMV）：图像处理、棋子颜色与位置识别
 *   STM32：位姿估计、决策、运动控制、安全
 *
 * 为什么分开：图像处理在 STM32F4 上做不动，而运动控制需要硬实时，
 * 视觉板跑 Python 也保证不了 10ms 节拍。各做各擅长的事。
 *
 * ========== 视觉识别的现实能力边界（务必按此设计策略） ==========
 *
 * 可靠：**红黑颜色区分**。棋子红黑对比强烈，颜色阈值即可，成功率高。
 * 较可靠：**圆形轮廓定位**。棋子是规则圆形，霍夫圆或色块质心都能定位，
 *         但距离越远、角度越斜，位置误差越大（典型 2~3cm）。
 * 不可靠：**汉字等级识别**（帅/将/车/马/炮/士/象/兵/卒）。
 *         棋子表面汉字在低分辨率、斜视角、运动模糊下极难稳定识别，
 *         低成本视觉模块上很难做到比赛可用的可靠性。
 *
 * **因此本库的策略是：等级允许为未知**。
 * Chess_LevelScore(CHESS_LEVEL_UNKNOWN) 返回保守中间值 4，
 * 车不会因为读不出汉字就放弃棋子——毕竟推进球门就能得分。
 * 若你们最终做出了可靠的汉字识别，直接在 payload 里填真实等级即可，
 * 决策层无需任何改动。
 *
 * 坐标约定：视觉板上报的是**场地绝对坐标 cm**，不是像素坐标。
 * 像素→场地坐标的标定在视觉板完成（俯视相机做一次四点透视变换即可），
 * 这样 STM32 侧不需要知道任何相机参数。
 */
#ifndef VISION_CHESS_H
#define VISION_CHESS_H

#include "chess_target.h"
#include "chesscar_common.h"
#include "link_protocol.h"

/** 单帧最多上报的棋子数，受协议 32 字节 payload 限制。 */
#define VISION_MAX_PIECES_PER_FRAME (6U)

/**
 * @brief LINK_MSG_LINE_RESULT(0x13) 之外新增的棋子上报消息。
 *
 * payload 布局（每枚棋子 5 字节，最多 6 枚 = 30 字节）：
 *   [0]      棋子数量 N (1~6)
 *   [1+5i+0] X 坐标，uint8，单位 2cm（0~150 → 0~300cm）
 *   [1+5i+1] Y 坐标，uint8，单位 1cm（0~200cm）
 *   [1+5i+2] 颜色：0 未知 / 1 红 / 2 黑
 *   [1+5i+3] 等级：见 Chess_Level 枚举值
 *   [1+5i+4] 置信度 0~100
 *
 * X 用 2cm 量化是为了用 1 字节覆盖 300cm 量程。
 * 2cm 的量化误差小于视觉本身的 2~3cm 定位误差，不是瓶颈。
 */
#define LINK_MSG_CHESS_PIECES (0x14U)

typedef struct {
    float x;
    float y;
    Chess_Color color;
    Chess_Level level;
    uint8_t confidence; /**< 0~100，低于阈值的观测会被丢弃。 */
} Vision_PieceReport;

typedef struct {
    Vision_PieceReport pieces[VISION_MAX_PIECES_PER_FRAME];
    uint8_t count;
    bool valid;
} Vision_Frame;

/**
 * @brief 解析一帧棋子上报。
 * @return true 表示解析成功且至少有一枚棋子。
 */
bool Vision_ParseChessFrame(const LinkProtocol_Frame *frame, Vision_Frame *out);

/**
 * @brief 把一帧视觉结果喂给棋子管理器。
 *
 * @param minConfidence 低于此置信度的观测直接丢弃，建议 50~60。
 * @return 实际接受的棋子数量。
 */
uint8_t Vision_FeedToManager(const Vision_Frame *frame,
                             Chess_TargetManager *mgr,
                             uint8_t minConfidence,
                             uint32_t nowMs);

/** 构造一帧棋子上报（视觉板侧或回环测试使用）。 */
uint8_t Vision_BuildChessPayload(const Vision_Frame *frame,
                                 uint8_t *payload, uint8_t maxLen);

#endif /* VISION_CHESS_H */
