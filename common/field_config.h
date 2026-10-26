/**
 * @file field_config.h
 * @brief 第十五届光电设计竞赛 赛题1「智能车光电象棋对抗」场地与规则常量。
 *
 * 赛题原文要点（来自主办方赛题 PDF）：
 *   - 场地 3m × 2m 矩形，四周设 80mm 高挡板
 *   - 分区：中央对抗区、两侧球门区、发车区、棋子堆放区
 *   - 全场随机投放 32 枚象棋棋子（红黑各 16），按等级设 1-10 分差异化分值
 *   - 车辆发车尺寸 ≤ 250×200×180mm，质量 ≤ 1.5kg，车载电池供电
 *   - 可搭载 ≤1mW 的 II 类安全激光；允许推板/滚轮；禁止电磁铁吸附、遥控、破坏性装置
 *   - 推动棋子**整体**进入己方球门区得分
 *   - 恶意冲撞对方车辆、激光功率超标 → 扣分或直接判负
 *   - 平局依次比较进球总数、"帅/将"归属，仍相同则 60 秒金球加赛
 *
 * **重要：赛题图1 备注"尺寸后续由主办方根据场地要求进行修订"。**
 * 因此球门区与发车区的具体尺寸在下方以"待实测确认"标注，
 * 必须在拿到正式场地图或现场实测后更新，不要当作已确认数据使用。
 *
 * 坐标系定义（本工程统一）：
 *   原点 O 位于场地左下角内侧（挡板内表面交点）
 *   X 轴沿长边向右，0 ~ 300 cm
 *   Y 轴沿短边向上，0 ~ 200 cm
 *   航向角 theta：X 轴正方向为 0°，逆时针为正，范围 (-180°, 180°]
 *
 * 甲队（蓝）球门在 X≈0 一侧，乙队（红）球门在 X≈300 一侧。
 * 本车属于哪一方由 FIELD_OUR_SIDE 决定，必须在比赛前按抽签结果设置。
 */
#ifndef FIELD_CONFIG_H
#define FIELD_CONFIG_H

/* ===================== 场地尺寸（赛题明确给出） ===================== */
#define FIELD_LENGTH_CM                 (300.0f) /**< X 方向，3m。 */
#define FIELD_WIDTH_CM                  (200.0f) /**< Y 方向，2m。 */
#define FIELD_WALL_HEIGHT_MM            (80.0f)  /**< 四周挡板高度。 */

/* ===================== 车辆限制（赛题明确给出） ===================== */
#define CAR_MAX_LENGTH_MM               (250.0f)
#define CAR_MAX_WIDTH_MM                (200.0f)
#define CAR_MAX_HEIGHT_MM               (180.0f)
#define CAR_MAX_MASS_KG                 (1.5f)
#define CAR_MAX_LASER_POWER_MW          (1.0f)   /**< II 类安全激光上限，超标直接判负。 */

/* ===================== 球门区与发车区（**待实测确认**） ===================== */
/*
 * 赛题图1 只给了相对位置，未标注具体尺寸，且备注会被主办方修订。
 * 下面是按示意图比例估算的**占位值**，仅用于算法联调，
 * 正式比赛前必须用实际场地替换，否则进球判定位置会错。
 */
#define FIELD_GOAL_DEPTH_CM_ESTIMATED   (30.0f)  /**< 球门区沿 X 的深度（估算）。 */
#define FIELD_GOAL_WIDTH_CM_ESTIMATED   (60.0f)  /**< 球门区沿 Y 的宽度（估算）。 */
#define FIELD_GOAL_CENTER_Y_CM          (FIELD_WIDTH_CM * 0.5f)

/* 棋子堆放区位于场地中央（赛题图1）。尺寸同样为估算占位值。 */
#define FIELD_PILE_CENTER_X_CM          (FIELD_LENGTH_CM * 0.5f)
#define FIELD_PILE_CENTER_Y_CM          (FIELD_WIDTH_CM * 0.5f)
#define FIELD_PILE_SIZE_CM_ESTIMATED    (40.0f)

/* ===================== 我方阵营 ===================== */
#define FIELD_SIDE_BLUE                 (0) /**< 甲队，球门在 X≈0。 */
#define FIELD_SIDE_RED                  (1) /**< 乙队，球门在 X≈300。 */

/** 比赛前按抽签结果修改这一行，或在运行时用 Field_SetOurSide() 设置。 */
#ifndef FIELD_OUR_SIDE
#define FIELD_OUR_SIDE                  FIELD_SIDE_BLUE
#endif

/* ===================== 棋子物理参数 ===================== */
/*
 * 标准中国象棋棋子直径常见 30~35mm、厚 8~10mm。
 * **必须用比赛实际发放的棋子实测后修改**，推板高度与识别阈值都依赖它。
 */
#define CHESS_PIECE_DIAMETER_MM         (32.0f)
#define CHESS_PIECE_THICKNESS_MM        (9.0f)
#define CHESS_PIECE_TOTAL_COUNT         (32)
#define CHESS_PIECE_PER_COLOR           (16)

/* ===================== 安全边距 ===================== */
/*
 * 车体半径约 sqrt(12.5^2 + 10^2) ≈ 16cm（按最大尺寸估算的外接圆半径）。
 * 规划路径时与挡板保持余量，避免贴死挡板导致轮子打滑或卡住。
 */
#define CAR_BOUNDING_RADIUS_CM          (16.0f)
#define FIELD_WALL_SAFE_MARGIN_CM       (18.0f)

/* 禁止恶意冲撞：与对方车辆保持的最小距离，检测到更近就必须主动规避。 */
#define OPPONENT_AVOID_DISTANCE_CM      (35.0f)

#endif /* FIELD_CONFIG_H */
