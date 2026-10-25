/**
 * @file car_physical_config.h
 * @brief 光电象棋小车机械与电气参数的唯一软件配置入口。
 *
 * 轮径、轮距、减速比、编码器分辨率会直接传播到速度、距离、转角与
 * 横向偏差的全部计算结果。**任何参数误差都不要靠改算法来补偿**，
 * 一律回到这里修正实测值。
 *
 * 当前数值继承自本机 smartcar 工程的实测记录（MG310 + 48mm 轮，
 * 2026-07-22 实测确认），移到 STM32 平台后需重新核对下列三项：
 *   1. 轮胎安装后的实际外径（压缩量影响 1%~3% 的里程误差）
 *   2. 编码器每圈脉冲数 PPR 与减速比（换电机必须改）
 *   3. 起转死区 PWM（换驱动芯片、换电池、换车重都要重测）
 */
#ifndef CAR_PHYSICAL_CONFIG_H
#define CAR_PHYSICAL_CONFIG_H

/* ===================== 底盘机械参数 ===================== */
#define CAR_WHEEL_DIAMETER_CM                 (4.8f)   /**< 轮胎安装后实测外径 48mm。 */
#define CAR_WHEEL_TRACK_CM                    (12.0f)  /**< 左右轮中心距。 */
#define CAR_WHEELBASE_CM                      (12.0f)  /**< 前后轴距。 */
#define CAR_GEAR_RATIO                        (20.409f)/**< MG310 减速比。 */

#define CAR_WHEEL_CIRCUMFERENCE_CM            \
    (CAR_WHEEL_DIAMETER_CM * 3.1415926f)

/* ===================== 电机电气参数 ===================== */
#define CAR_MOTOR_NOMINAL_VOLTAGE_V           (7.4f)
#define CAR_MOTOR_NO_LOAD_RPM                 (500.0f)
#define CAR_MOTOR_RATED_RPM                   (400.0f)
#define CAR_MOTOR_RATED_CURRENT_A             (0.500f)
#define CAR_MOTOR_STALL_CURRENT_A             (2.0f)

#define CAR_THEORETICAL_NO_LOAD_CMPS          \
    (CAR_WHEEL_CIRCUMFERENCE_CM * CAR_MOTOR_NO_LOAD_RPM / 60.0f)
#define CAR_THEORETICAL_RATED_CMPS            \
    (CAR_WHEEL_CIRCUMFERENCE_CM * CAR_MOTOR_RATED_RPM / 60.0f)

/* ===================== 编码器参数 ===================== */
/* 当前为 13 PPR 霍尔编码器款，不是 500 PPR GMR 编码器款。 */
#define CAR_ENCODER_IS_HALL                   (1U)
#define CAR_MOTOR_ENCODER_PPR                 (13U)

/*
 * STM32 定时器编码器模式（Encoder Mode TI1 and TI2）自带四倍频，
 * 因此倍频系数固定为 4，不需要软件再乘。
 * 车轮每圈理论计数 = 13 × 4 × 20.409 ≈ 1061.3
 */
#define CAR_ENCODER_COUNT_MULTIPLIER          (4U)
#define CAR_ENCODER_COUNTS_PER_MOTOR_REV      \
    (CAR_MOTOR_ENCODER_PPR * CAR_ENCODER_COUNT_MULTIPLIER)
#define CAR_ENCODER_COUNTS_PER_WHEEL_REV      \
    ((float)CAR_ENCODER_COUNTS_PER_MOTOR_REV * CAR_GEAR_RATIO)

/* ===================== 光电/灰度探头几何 ===================== */
/*
 * 8 路探头从车头视角由左至右排列，OUT1 在最左。
 * ACTIVE_SPAN 按 OUT1 与 OUT8 探头中心之间的有效跨度处理，
 * 若 8.5cm 实际是 PCB 外形宽度，必须重新测两端探头中心距后修改。
 * 实装离地高度 1cm；若通道闪烁或黑白区分不稳定，优先降到 0.6~0.8cm。
 */
#define CAR_LINE_SENSOR_COUNT                 (8U)
#define CAR_LINE_SENSOR_ACTIVE_SPAN_CM        (8.5f)
#define CAR_LINE_SENSOR_PITCH_CM              \
    (CAR_LINE_SENSOR_ACTIVE_SPAN_CM / (float)(CAR_LINE_SENSOR_COUNT - 1U))
#define CAR_LINE_SENSOR_HEIGHT_CM             (1.0f)

/* ===================== 控制周期与 PWM ===================== */
/*
 * 速度环固定 10ms 一拍（基础定时器中断置标志，主循环执行）。
 * 循迹外环 20ms 一拍：外环慢于内环是串级 PID 的基本要求。
 */
#define CAR_SPEED_LOOP_PERIOD_MS              (10U)
#define CAR_LINE_LOOP_PERIOD_MS               (20U)

/*
 * TB6612FNG 的 PWM 开关频率上限为 100kHz，取 20kHz 既超出人耳可闻范围、
 * 又给驱动留足裕量。STM32F103 @72MHz：PSC=0、ARR=3599 → 72MHz/3600 = 20kHz。
 */
#define CAR_PWM_FREQUENCY_HZ                  (20000UL)
#define CAR_PWM_FULL_SCALE                    (3600)   /**< 等于 ARR+1，占空比分辨率。 */
#define CAR_PWM_OUTPUT_LIMIT                  (3400)   /**< 留约 5% 余量，不跑满 100%。 */

/*
 * 实车台架结论（MSPM0 平台实测，迁移到 STM32 后需复测）：
 * PWM 约 5.5% 占空比不能可靠起转，约 9.7% 附近可启动。
 * 因此必须补偿齿轮箱、轮胎与车重形成的静摩擦死区，
 * 取略低于实测起转点的值，让闭环仍有向下修正空间。
 */
#define CAR_MOTOR_STARTUP_DEADZONE_PWM        (320)

/* ===================== 调试建议速度 ===================== */
/* 首次循迹建议值，不代表电机机械极限；闭环稳定后再按赛道逐步提高。 */
#define CAR_RECOMMENDED_CRUISE_CMPS           (10.0f)
#define CAR_RECOMMENDED_MAX_CMPS              (30.0f)

/*
 * 安全提示：霍尔编码器 A/B 必须与 STM32 共地并使用 3.3V 电平。
 * 不要把上拉到 5V 的编码器输出直接接入 STM32 GPIO；
 * TB6612FNG 逻辑电源 VCC 支持 2.7~5.5V，控制侧可直接对接 3.3V。
 */

#endif /* CAR_PHYSICAL_CONFIG_H */
