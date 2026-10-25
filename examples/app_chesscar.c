/**
 * @file app_chesscar.c
 * @brief 主应用实现：节拍调度、模式机、串口指令处理与遥测上报。
 */
#include "app_chesscar.h"

#include "bsp_config.h"
#include "car_physical_config.h"
#include "encoder_tim.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "motor_tb6612.h"
#include "oled_ssd1306.h"
#include "speed_control.h"
#include "uart_link.h"

/* ===================== 节拍标志 ===================== */
static volatile uint32_t g_tickMs;
static volatile bool g_flag10ms;
static volatile bool g_flag20ms;
static volatile bool g_flag100ms;

/* ===================== 控制器实例 ===================== */
static SpeedControl_Controller g_speed;
static LineSensor_State g_lineSensor;
static LineFollow_State g_lineFollow;
static App_CarMode g_mode = CAR_MODE_IDLE;

#if BSP_LINE_SENSOR_USE_ADC
static uint16_t g_lineAdc[BSP_LINE_SENSOR_CHANNELS];
#endif

/* ===================== 速度环回调 ===================== */
static int32_t App_ReadLeftEncoder(void)  { return Encoder_GetLeftCount(); }
static int32_t App_ReadRightEncoder(void) { return Encoder_GetRightCount(); }
static void App_SetMotorOutput(int16_t left, int16_t right)
{
    Motor_SetSides(left, right);
}

/* ===================== 探头采样 ===================== */
static void App_SampleLineSensor(void)
{
#if BSP_LINE_SENSOR_USE_ADC
    /*
     * ADC 已配置为扫描 + DMA 循环模式，g_lineAdc 由 DMA 自动刷新，
     * 这里直接取用。若没用 DMA，就改成逐通道 HAL_ADC_PollForConversion，
     * 但注意 8 通道轮询会占用约 100us。
     */
    LineSensor_UpdateAnalog(&g_lineSensor, g_lineAdc);
#else
    uint8_t mask = 0U;

    /* DO 版本：读到黑线为高电平则置 1。极性相反时整体取反即可。 */
    if (HAL_GPIO_ReadPin(BSP_LINE_DO0_PORT, BSP_LINE_DO0_PIN)) mask |= 0x01U;
    if (HAL_GPIO_ReadPin(BSP_LINE_DO1_PORT, BSP_LINE_DO1_PIN)) mask |= 0x02U;
    if (HAL_GPIO_ReadPin(BSP_LINE_DO2_PORT, BSP_LINE_DO2_PIN)) mask |= 0x04U;
    if (HAL_GPIO_ReadPin(BSP_LINE_DO3_PORT, BSP_LINE_DO3_PIN)) mask |= 0x08U;
    if (HAL_GPIO_ReadPin(BSP_LINE_DO4_PORT, BSP_LINE_DO4_PIN)) mask |= 0x10U;
    if (HAL_GPIO_ReadPin(BSP_LINE_DO5_PORT, BSP_LINE_DO5_PIN)) mask |= 0x20U;
    if (HAL_GPIO_ReadPin(BSP_LINE_DO6_PORT, BSP_LINE_DO6_PIN)) mask |= 0x40U;
    if (HAL_GPIO_ReadPin(BSP_LINE_DO7_PORT, BSP_LINE_DO7_PIN)) mask |= 0x80U;
    LineSensor_UpdateMask(&g_lineSensor, mask);
#endif
}

/* ===================== 串口帧处理 ===================== */
static void App_OnFrame(const LinkProtocol_Frame *frame)
{
    if (frame == 0) {
        return;
    }

    switch (frame->type) {
        case LINK_MSG_TASK_COMMAND:
            /* payload[0] = 目标模式。 */
            if (frame->length >= 1U) {
                App_SetMode((App_CarMode)frame->payload[0]);
            }
            break;

        case LINK_MSG_PID_TUNE:
            /*
             * payload: [0]=环编号(0速度/1循迹) [1..12]=三个 float（小端）
             * 在线调参非常实用，但**务必在车停住时改**，
             * 否则新增益叠加旧积分会窜车。PID_SetGains 内部已清积分。
             */
            if ((frame->length >= 13U) && (frame->payload[0] == 0U)) {
                float kp, ki, kd;
                memcpy(&kp, &frame->payload[1], 4U);
                memcpy(&ki, &frame->payload[5], 4U);
                memcpy(&kd, &frame->payload[9], 4U);
                SpeedControl_SetPidGains(&g_speed, kp, ki, kd, kp, ki, kd);
            }
            break;

        case LINK_MSG_CHESS_MOVE:
            /*
             * 象棋走子指令：payload[0]=起点格号, payload[1]=终点格号。
             * 这里只做占位：实际路径规划应由上位机或视觉板完成，
             * STM32 侧只负责按收到的分段距离/转角执行闭环运动。
             */
            break;

        default:
            break;
    }
}

/* ===================== 遥测上报 ===================== */
static void App_ReportStatus(void)
{
    uint8_t payload[16];
    int16_t leftSpeed = (int16_t)(SpeedControl_GetLeftSpeedCmps(&g_speed) * 10.0f);
    int16_t rightSpeed = (int16_t)(SpeedControl_GetRightSpeedCmps(&g_speed) * 10.0f);
    int16_t error = (int16_t)(LineSensor_GetError(&g_lineSensor) * 100.0f);
    int32_t distance = (int32_t)(SpeedControl_GetCarDistanceCm(&g_speed) * 10.0f);

    /* 全部按固定宽度整数定标后再发，避免浮点在不同上位机解析不一致。 */
    payload[0] = (uint8_t)g_mode;
    payload[1] = (uint8_t)SpeedControl_GetFault(&g_speed);
    payload[2] = LineSensor_GetMask(&g_lineSensor);
    payload[3] = 0U; /* 保留 */
    memcpy(&payload[4], &leftSpeed, 2U);
    memcpy(&payload[6], &rightSpeed, 2U);
    memcpy(&payload[8], &error, 2U);
    memcpy(&payload[10], &distance, 4U);

    (void)UartLink_SendFrame(LINK_MSG_CAR_STATUS, 0U, payload, 14U);
}

/* ===================== OLED 刷新 ===================== */
static void App_RefreshDisplay(void)
{
    /*
     * 整屏刷新约 25ms，所以 100ms 一次已经占用 25% 的主循环时间。
     * 这里只更新 4 行文本，且不在高速行驶时刷新更多内容。
     */
    OLED_ShowString(1U, 1U, "M:");
    OLED_ShowNum(1U, 3U, (uint32_t)g_mode, 1U);
    OLED_ShowString(1U, 5U, "F:");
    OLED_ShowString(1U, 7U, SpeedControl_FaultName(SpeedControl_GetFault(&g_speed)));

    OLED_ShowString(2U, 1U, "MSK:");
    OLED_ShowBinNum(2U, 5U, LineSensor_GetMask(&g_lineSensor), 8U);

    OLED_ShowString(3U, 1U, "E:");
    OLED_ShowSignedNum(3U, 3U, (int32_t)(LineSensor_GetError(&g_lineSensor) * 100.0f), 4U);

    OLED_ShowString(4U, 1U, "V:");
    OLED_ShowSignedNum(4U, 3U, (int32_t)(SpeedControl_GetLeftSpeedCmps(&g_speed) * 10.0f), 3U);
    OLED_ShowString(4U, 9U, "/");
    OLED_ShowSignedNum(4U, 10U, (int32_t)(SpeedControl_GetRightSpeedCmps(&g_speed) * 10.0f), 3U);
}

/* ===================== 初始化 ===================== */
void App_Init(void)
{
    SpeedControl_Config speedConfig;
    LineSensor_Config sensorConfig;
    LineFollow_Config followConfig;

    Motor_Init();
    Encoder_Init();
    OLED_Init();
    UartLink_Init(App_OnFrame);

#if BSP_LINE_SENSOR_USE_ADC
    HAL_ADC_Start_DMA(&BSP_LINE_SENSOR_ADC, (uint32_t *)g_lineAdc,
                      BSP_LINE_SENSOR_CHANNELS);
#endif

    SpeedControl_LoadDefaultConfig(&speedConfig);
    speedConfig.readLeftEncoder = App_ReadLeftEncoder;
    speedConfig.readRightEncoder = App_ReadRightEncoder;
    speedConfig.setOutput = App_SetMotorOutput;
    (void)SpeedControl_Init(&g_speed, &speedConfig);

    LineSensor_LoadDefaultConfig(&sensorConfig);
    (void)LineSensor_Init(&g_lineSensor, &sensorConfig);

    LineFollow_LoadDefaultConfig(&followConfig);
    (void)LineFollow_Init(&g_lineFollow, &followConfig);

    /* 启动控制节拍定时器。 */
    HAL_TIM_Base_Start_IT(&htim4);

    App_SetMode(CAR_MODE_IDLE);
}

void App_SetMode(App_CarMode mode)
{
    if (mode == g_mode) {
        return;
    }

    /* 任何模式切换都先停车清状态，禁止带着旧积分进入新模式。 */
    SpeedControl_Stop(&g_speed);
    LineFollow_Reset(&g_lineFollow);

    switch (mode) {
        case CAR_MODE_LINE_FOLLOW:
        case CAR_MODE_MANUAL:
            SpeedControl_Enable(&g_speed, true);
            break;

        case CAR_MODE_IDLE:
        case CAR_MODE_BENCH_TEST:
        case CAR_MODE_FAULT:
        default:
            SpeedControl_Enable(&g_speed, false);
            Motor_Stop();
            break;
    }

    g_mode = mode;
}

App_CarMode App_GetMode(void) { return g_mode; }

/* ===================== 1ms 中断：只置标志 ===================== */
void App_OnTick1ms(void)
{
    g_tickMs++;
    if ((g_tickMs % CAR_SPEED_LOOP_PERIOD_MS) == 0U) {
        g_flag10ms = true;
    }
    if ((g_tickMs % CAR_LINE_LOOP_PERIOD_MS) == 0U) {
        g_flag20ms = true;
    }
    if ((g_tickMs % 100U) == 0U) {
        g_flag100ms = true;
    }
}

/* ===================== 主循环 ===================== */
void App_Loop(void)
{
    /* 串口解析优先：指令延迟会直接表现为「车不听话」。 */
    UartLink_Poll();

    /* ---------- 20ms：循迹外环 ---------- */
    if (g_flag20ms) {
        g_flag20ms = false;

        if (g_mode == CAR_MODE_LINE_FOLLOW) {
            LineFollow_Update(&g_lineFollow, &g_lineSensor);
            if (LineFollow_IsStopRequested(&g_lineFollow)) {
                SpeedControl_SetTargetCmps(&g_speed, 0.0f, 0.0f);
            } else {
                SpeedControl_SetTargetCmps(&g_speed,
                                           LineFollow_GetTargetLeftCmps(&g_lineFollow),
                                           LineFollow_GetTargetRightCmps(&g_lineFollow));
            }
        }
    }

    /* ---------- 10ms：探头采样 + 速度内环 ---------- */
    if (g_flag10ms) {
        g_flag10ms = false;

        App_SampleLineSensor();
        SpeedControl_Update(&g_speed);

        /* 速度环报故障就强制进入故障模式，必须人工清除后才能继续。 */
        if (SpeedControl_HasFault(&g_speed) && (g_mode != CAR_MODE_FAULT)) {
            App_SetMode(CAR_MODE_FAULT);
        }
    }

    /* ---------- 100ms：显示与遥测 ---------- */
    if (g_flag100ms) {
        g_flag100ms = false;
        App_RefreshDisplay();
        App_ReportStatus();
    }
}
