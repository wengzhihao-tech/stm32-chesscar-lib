/**
 * @file speed_control.h
 * @brief 双轮差速底盘的编码器速度闭环（含前馈、死区补偿与安全自检）。
 *
 * 设计原则：本模块**不碰任何寄存器**，通过回调拿编码器累计计数、通过回调
 * 输出左右 PWM，因此同一份代码可以跑在 STM32、MSPM0 或 PC 测试桩上。
 *
 * 调用约定：
 *   1. SpeedControl_Init 传入机械参数与回调；
 *   2. 以固定周期（CAR_SPEED_LOOP_PERIOD_MS，建议 10ms）调用 SpeedControl_Update；
 *   3. 上层只调 SpeedControl_SetTargetCmps 设定目标速度。
 *
 * 编码器回调必须返回**累计计数**（int32_t，允许溢出回绕后仍单调），
 * 本模块自己做周期差分，不要在回调里清零计数器。
 */
#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "chesscar_common.h"
#include "encoder_math.h"
#include "pid_controller.h"

/** 返回编码器累计计数（不要在回调内清零）。 */
typedef int32_t (*SpeedControl_ReadEncoderCallback)(void);

/** 输出左右轮 PWM，范围 [-outputLimit, +outputLimit]，正值表示车辆前进。 */
typedef void (*SpeedControl_SetOutputCallback)(int16_t leftPwm, int16_t rightPwm);

typedef enum {
    SPEED_CONTROL_FAULT_NONE = 0,
    SPEED_CONTROL_FAULT_ENCODER_DIRECTION, /**< 目标与反馈方向持续相反：接线反了或电机堵转。 */
    SPEED_CONTROL_FAULT_OVERSPEED          /**< 反馈速度超过物理上限：编码器误计数或打滑。 */
} SpeedControl_Fault;

typedef struct {
    Encoder_Config encoder;

    float leftKp, leftKi, leftKd;
    float rightKp, rightKi, rightKd;
    float integralLimit;
    int16_t outputLimit;

    /** 速度前馈，单位 PWM/(cm/s)。标定方法见 docs/TUNING_GUIDE.md，0 表示关闭。 */
    float leftFeedforwardPwmPerCmps;
    float rightFeedforwardPwmPerCmps;

    /** 静摩擦死区补偿 PWM，0 表示关闭。 */
    int16_t leftDeadzonePwm;
    int16_t rightDeadzonePwm;

    /** 反馈低通滤波系数 (0,1]，1 表示不滤波。低速量化抖动大时取 0.3~0.6。 */
    float feedbackFilterK;

    /* ---- 安全自检阈值：连续若干拍确认后才锁定输出，避免误触发 ---- */
    float directionFaultMinTargetCmps;   /**< 目标速度低于此值不做方向判定。 */
    float directionFaultMinFeedbackCmps; /**< 反馈速度低于此值不做方向判定。 */
    float overspeedLimitCmps;            /**< 超速阈值，建议取理论空载速度的 1.2 倍。 */
    uint8_t directionFaultConfirmCycles;
    uint8_t overspeedConfirmCycles;

    /** 编码器方向符号：架空正转测试后若计数变负，把对应符号改为 -1。 */
    int8_t leftEncoderSign;
    int8_t rightEncoderSign;
    /** 电机方向符号：架空给正 PWM 若反转，把对应符号改为 -1。 */
    int8_t leftMotorSign;
    int8_t rightMotorSign;

    SpeedControl_ReadEncoderCallback readLeftEncoder;
    SpeedControl_ReadEncoderCallback readRightEncoder;
    SpeedControl_SetOutputCallback setOutput;
} SpeedControl_Config;

typedef struct {
    SpeedControl_Config config;
    PID_Controller leftPid;
    PID_Controller rightPid;

    int32_t lastLeftCount;
    int32_t lastRightCount;
    int32_t leftDelta;
    int32_t rightDelta;

    float targetLeftCmps;
    float targetRightCmps;
    float leftSpeedCmps;
    float rightSpeedCmps;
    float carSpeedCmps;
    bool feedbackInitialized; /**< 首拍旁路低通，避免从 0 起步产生额外滞后。 */

    float leftDistanceCm;
    float rightDistanceCm;
    float carDistanceCm;

    int16_t leftOutput;
    int16_t rightOutput;

    bool enabled;
    SpeedControl_Fault fault;
    uint8_t leftDirectionFaultCount;
    uint8_t rightDirectionFaultCount;
    /*
     * 换向宽限：原地转弯时一侧车轮反转，转弯结束立刻要求前进时，
     * 滤波后的反馈可能短时间仍为反向。此计数只抑制过渡期的方向判定，
     * 不会关闭正常运行中的方向故障检测。
     */
    uint8_t leftDirectionGraceCount;
    uint8_t rightDirectionGraceCount;
    uint8_t overspeedFaultCount;
} SpeedControl_Controller;

/** 填入一组适用于 MG310 + TB6612 + 13PPR 霍尔编码器的保守默认值。 */
void SpeedControl_LoadDefaultConfig(SpeedControl_Config *config);

bool SpeedControl_Init(SpeedControl_Controller *c, const SpeedControl_Config *config);
void SpeedControl_Reset(SpeedControl_Controller *c);
void SpeedControl_Enable(SpeedControl_Controller *c, bool enabled);

/** 设定左右目标速度 cm/s。目标反向时自动给出换向宽限。 */
void SpeedControl_SetTargetCmps(SpeedControl_Controller *c, float leftCmps, float rightCmps);

/** 差速接口：linear 为前进速度 cm/s，angular 为左右速度差 cm/s。 */
void SpeedControl_SetDifferential(SpeedControl_Controller *c, float linearCmps, float angularCmps);

bool SpeedControl_SetPidGains(SpeedControl_Controller *c,
                              float leftKp, float leftKi, float leftKd,
                              float rightKp, float rightKi, float rightKd);
bool SpeedControl_SetCompensation(SpeedControl_Controller *c,
                                  float leftFeedforward, float rightFeedforward,
                                  int16_t leftDeadzone, int16_t rightDeadzone);

/** 必须以固定周期调用；周期必须等于 config.encoder.samplePeriodMs。 */
void SpeedControl_Update(SpeedControl_Controller *c);

/** 立即零输出并清 PID 状态。任何异常、丢线、通信超时都应先调这个。 */
void SpeedControl_Stop(SpeedControl_Controller *c);

void SpeedControl_ClearFault(SpeedControl_Controller *c);
bool SpeedControl_HasFault(const SpeedControl_Controller *c);
SpeedControl_Fault SpeedControl_GetFault(const SpeedControl_Controller *c);
const char *SpeedControl_FaultName(SpeedControl_Fault fault);

float SpeedControl_GetLeftSpeedCmps(const SpeedControl_Controller *c);
float SpeedControl_GetRightSpeedCmps(const SpeedControl_Controller *c);
float SpeedControl_GetCarSpeedCmps(const SpeedControl_Controller *c);
float SpeedControl_GetCarDistanceCm(const SpeedControl_Controller *c);
int16_t SpeedControl_GetLeftOutput(const SpeedControl_Controller *c);
int16_t SpeedControl_GetRightOutput(const SpeedControl_Controller *c);

#endif /* SPEED_CONTROL_H */
