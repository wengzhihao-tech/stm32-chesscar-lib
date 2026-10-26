/**
 * @file chess_target.h
 * @brief 棋子目标管理与"推哪一枚最划算"的决策（赛题的智能决策核心）。
 *
 * 赛题计分规则：把棋子**整体**推进己方球门区，按棋子等级 1-10 分累计。
 * 平局时依次比较进球总数、"帅/将"归属。
 *
 * 因此决策不是"推最近的"，而是要权衡三件事：
 *   1. 分值高低（帅/将 额外有平局抢分价值）
 *   2. 推送代价（车要先开到棋子后方，再把它推到球门，总路程越短越好）
 *   3. 风险（棋子离对方球门太近容易被对方反抢；贴墙的棋子难以对正）
 *
 * 本模块只做**排序与筛选**，不含任何硬件访问，可在 PC 上验证决策是否合理。
 *
 * ⚠ 等级识别的现实限制：
 *   区分红黑两色很容易（颜色阈值即可），但要读出棋子表面的汉字
 *   （帅/将/车/马/炮/士/象/兵/卒）从而确定 1-10 分等级，需要字符识别，
 *   在低成本视觉模块上可靠性有限。因此本模块允许等级"未知"，
 *   未知时按保守默认分值参与排序，不会因为识别不出等级就放弃该棋子。
 */
#ifndef CHESS_TARGET_H
#define CHESS_TARGET_H

#include "chesscar_common.h"
#include "field_config.h"
#include "pose_estimator.h"

#define CHESS_MAX_TRACKED_PIECES (32U)

/** 棋子颜色。颜色靠视觉的红/黑阈值判别，可靠性高。 */
typedef enum {
    CHESS_COLOR_UNKNOWN = 0,
    CHESS_COLOR_RED,
    CHESS_COLOR_BLACK
} Chess_Color;

/**
 * @brief 棋子等级。
 *
 * 分值为**本工程的假定值**：赛题只说"按棋子等级设置 1-10 分差异化分值"，
 * 并未公布每种棋子的具体分数。下面按象棋常规强弱排序给出估计，
 * **拿到主办方正式计分表后必须更新 Chess_LevelScore() 里的数值**。
 */
typedef enum {
    CHESS_LEVEL_UNKNOWN = 0, /**< 识别不出等级，按保守分值处理。 */
    CHESS_LEVEL_SOLDIER,     /**< 兵/卒 */
    CHESS_LEVEL_GUARD,       /**< 士/仕 */
    CHESS_LEVEL_ELEPHANT,    /**< 象/相 */
    CHESS_LEVEL_HORSE,       /**< 马 */
    CHESS_LEVEL_CANNON,      /**< 炮 */
    CHESS_LEVEL_CHARIOT,     /**< 车 */
    CHESS_LEVEL_GENERAL      /**< 帅/将，平局时决定归属，优先级最高 */
} Chess_Level;

typedef struct {
    bool valid;            /**< 该槽位是否在用。 */
    float x;               /**< 场地坐标，cm。 */
    float y;
    Chess_Color color;
    Chess_Level level;
    uint32_t lastSeenMs;   /**< 最近一次被观测到的时刻，用于老化剔除。 */
    uint8_t observeCount;  /**< 被连续观测到的次数，抗单帧误检。 */
    bool scored;           /**< 已推进球门，不再作为目标。 */
} Chess_Piece;

typedef struct {
    Chess_Piece pieces[CHESS_MAX_TRACKED_PIECES];
    uint8_t count;              /**< 当前有效棋子数。 */
    uint8_t ourSide;            /**< FIELD_SIDE_BLUE 或 FIELD_SIDE_RED。 */
    float goalX;                /**< 己方球门中心 X。 */
    float goalY;                /**< 己方球门中心 Y。 */
    uint32_t pieceTimeoutMs;    /**< 超过此时间未观测到则剔除。 */
    float mergeRadiusCm;        /**< 新观测与已有棋子距离小于此值则视为同一枚。 */
    uint8_t minObserveCount;    /**< 至少被观测这么多次才允许作为目标，抗误检。 */
    uint16_t ourScore;          /**< 本方累计得分（按假定分值估算）。 */
    uint8_t ourGoalCount;       /**< 本方进球总数，平局第一比较项。 */
    bool ourGeneralScored;      /**< 是否已拿到"帅/将"，平局第二比较项。 */
} Chess_TargetManager;

/** 返回某等级的假定分值（1-10）。等级未知时返回保守中间值。 */
uint8_t Chess_LevelScore(Chess_Level level);

/** 等级名称，用于 OLED 与串口调试输出。 */
const char *Chess_LevelName(Chess_Level level);

void Chess_InitManager(Chess_TargetManager *mgr, uint8_t ourSide);

/** 设置我方阵营并相应更新球门坐标。 */
void Chess_SetOurSide(Chess_TargetManager *mgr, uint8_t ourSide);

/**
 * @brief 加入或更新一次棋子观测。
 *
 * 同一枚棋子会被反复观测到，所以这里做就近合并：
 * 若新观测落在已有棋子的 mergeRadiusCm 内，则更新该棋子而不是新增，
 * 否则占用一个新槽位。这样视觉每帧都喂数据也不会把表撑爆。
 *
 * @return 该观测对应的棋子索引，满表时返回 -1。
 */
int8_t Chess_Observe(Chess_TargetManager *mgr, float x, float y,
                     Chess_Color color, Chess_Level level, uint32_t nowMs);

/** 剔除长时间未观测到的棋子（可能已被对方推走或原先是误检）。 */
void Chess_AgeOut(Chess_TargetManager *mgr, uint32_t nowMs);

/** 标记某棋子已得分，并累加比分、进球数与"帅/将"归属。 */
void Chess_MarkScored(Chess_TargetManager *mgr, uint8_t index);

/**
 * @brief 选出当前最值得推的棋子。
 *
 * 代价函数（越小越优）：
 *   cost = 接近距离 + 推送距离 × 推送权重 − 分值收益 × 分值权重 + 风险惩罚
 *
 * 其中"推送距离"指把棋子从当前位置推到己方球门的距离，
 * 它比"车到棋子的距离"更重要——因为推着棋子走远路既慢又容易推偏。
 *
 * @param pushDistanceWeight 推送距离权重，建议 1.5~2.5（推送比空载移动更费时）
 * @param scoreWeight        分值权重 cm/分，建议 8~15
 * @return 最优棋子索引，没有可选目标时返回 -1。
 */
int8_t Chess_SelectBestTarget(const Chess_TargetManager *mgr,
                              const Pose2D *carPose,
                              float pushDistanceWeight,
                              float scoreWeight);

/**
 * @brief 计算"推送起始点"：车应该先开到棋子的哪一侧才能把它推向球门。
 *
 * 这是推棋子的关键几何：车必须位于**棋子背向球门的一侧**，
 * 车头朝球门方向推。如果直接冲到棋子位置，只会把它撞偏甚至撞出场。
 *
 * @param standoffCm 在棋子后方留多少距离作为接近点（建议 12~18cm，
 *                   需大于车体前伸量，避免接近过程中提前碰到棋子）
 */
void Chess_ComputeApproachPoint(const Chess_TargetManager *mgr, uint8_t index,
                                float standoffCm, float *outX, float *outY);

/** 判断棋子是否已整体进入己方球门区（赛题要求"整体进入"）。 */
bool Chess_IsInOurGoal(const Chess_TargetManager *mgr, uint8_t index,
                       float goalDepthCm, float goalWidthCm);

const Chess_Piece *Chess_GetPiece(const Chess_TargetManager *mgr, uint8_t index);

#endif /* CHESS_TARGET_H */
