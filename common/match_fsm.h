/**
 * @file match_fsm.h
 * @brief 比赛流程状态机：从发车到反复「选目标→接近→推送→回场」的完整闭环。
 *
 * 赛题要求全程全自主，禁止场外遥控与人工干预，
 * 所以整场比赛的行为必须由车自己按状态机驱动，不能有任何人工介入点。
 *
 * 状态流转：
 *
 *   INIT ──(发车信号)──> SEARCH
 *     SEARCH  扫视场地寻找棋子（原地慢转或沿预设巡视点移动）
 *       │ 找到可选目标
 *       ↓
 *     APPROACH 开到棋子背向球门的一侧（推送起始点）
 *       │ 到位且已对准球门
 *       ↓
 *     PUSH     低速推着棋子朝球门走
 *       │ 棋子整体进门 → SCORED；推丢了 → SEARCH
 *       ↓
 *     SCORED   记分，退出球门区避免挡住自己的门
 *       │
 *       ↓
 *     SEARCH （循环，直到比赛结束或场上无棋子）
 *
 *   任意状态 ──(位姿失效/严重故障)──> RECOVER ──> SEARCH
 *   任意状态 ──(比赛结束)──> FINISHED
 *
 * 设计原则：每个状态都有超时。开放场地里任何动作都可能失败
 * （推丢棋子、被对方挡住、位姿漂移），没有超时就会卡死在某个状态里
 * 白白浪费比赛时间。
 */
#ifndef MATCH_FSM_H
#define MATCH_FSM_H

#include "chess_target.h"
#include "chesscar_common.h"
#include "navigator.h"
#include "pose_estimator.h"

typedef enum {
    MATCH_STATE_INIT = 0,
    MATCH_STATE_SEARCH,
    MATCH_STATE_APPROACH,
    MATCH_STATE_ALIGN,     /**< 到达推送起始点后，原地对准球门方向。 */
    MATCH_STATE_PUSH,
    MATCH_STATE_SCORED,
    MATCH_STATE_RETREAT,   /**< 退出球门区，避免挡住自己后续进球。 */
    MATCH_STATE_RECOVER,   /**< 位姿失效或卡死后的恢复动作。 */
    MATCH_STATE_FINISHED
} Match_State;

typedef struct {
    uint32_t matchDurationMs;    /**< 比赛总时长，赛题未给出，需按现场规则设置。 */
    uint32_t searchTimeoutMs;    /**< 搜索超时：太久找不到棋子就换巡视点。 */
    uint32_t approachTimeoutMs;  /**< 接近超时：可能被对方挡住或位姿漂了。 */
    uint32_t pushTimeoutMs;      /**< 推送超时：可能已把棋子推丢。 */
    uint32_t alignTimeoutMs;
    float approachStandoffCm;    /**< 推送起始点在棋子后方多远。 */
    float alignToleranceDeg;     /**< 对准球门的航向容差。 */
    float pushLostDistanceCm;    /**< 棋子偏离推送轴线超过此值视为推丢。 */
    float pushDistanceWeight;    /**< 目标选择：推送距离权重。 */
    float scoreWeight;           /**< 目标选择：分值权重。 */
    float retreatDistanceCm;     /**< 得分后退出球门的距离。 */
} Match_Config;

typedef struct {
    Match_Config config;
    Match_State state;
    uint32_t stateEnterMs;       /**< 进入当前状态的时刻，用于超时判断。 */
    uint32_t matchStartMs;
    int8_t currentTargetIndex;   /**< 当前锁定的棋子索引，-1 表示无目标。 */
    float approachX;
    float approachY;
    uint8_t searchWaypointIndex; /**< 当前巡视点序号。 */

    /* 诊断统计：赛后复盘用，能直接看出策略哪一环最弱。 */
    uint16_t pushAttempts;
    uint16_t pushSuccesses;
    uint16_t pushLostCount;
    uint16_t searchTimeouts;
    uint16_t recoverCount;
} Match_Fsm;

void Match_LoadDefaultConfig(Match_Config *config);
bool Match_Init(Match_Fsm *fsm, const Match_Config *config);

/** 发车。matchStartMs 为当前时刻。 */
void Match_Start(Match_Fsm *fsm, uint32_t nowMs);

/**
 * @brief 推进状态机一拍，建议与导航同周期（20ms）调用。
 *
 * 本函数会按当前状态设定 Navigator 的目标点，但**不直接控制电机**，
 * 电机由 Navigator → SpeedControl 链路驱动，职责分明。
 *
 * @param opponentDistCm 对方车距离，用于安全判断。
 */
void Match_Update(Match_Fsm *fsm, uint32_t nowMs,
                  Chess_TargetManager *mgr,
                  const PoseEstimator *pe,
                  Navigator *nav,
                  float opponentDistCm);

Match_State Match_GetState(const Match_Fsm *fsm);
const char *Match_StateName(Match_State state);

/** 获取当前巡视点坐标（搜索状态下车要去的地方）。 */
void Match_GetSearchWaypoint(const Match_Fsm *fsm, uint8_t ourSide,
                             float *outX, float *outY);

#endif /* MATCH_FSM_H */
