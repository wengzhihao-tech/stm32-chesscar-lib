/**
 * @file oled_ssd1306.c
 * @brief SSD1306 I2C 文本驱动实现。
 */
#include "oled_ssd1306.h"

#include "oled_font.h"

#define OLED_CONTROL_COMMAND  (0x00U)
#define OLED_CONTROL_DATA     (0x40U)

static uint16_t g_errorCount;

/*
 * 显示失败绝不能影响控制与安全停车，所以所有 I2C 操作都忽略返回值，
 * 只累计错误计数供诊断使用。I2C 地址需左移 1 位变成 HAL 的 8 位格式。
 */
static void OLED_Write(uint8_t control, uint8_t value)
{
    uint8_t payload[2];

    payload[0] = control;
    payload[1] = value;
    if (HAL_I2C_Master_Transmit(&BSP_OLED_I2C,
                                (uint16_t)(BSP_OLED_I2C_ADDR_7BIT << 1),
                                payload, 2U,
                                BSP_OLED_I2C_TIMEOUT_MS) != HAL_OK) {
        g_errorCount++;
    }
}

static void OLED_WriteCommand(uint8_t command)
{
    OLED_Write(OLED_CONTROL_COMMAND, command);
}

static void OLED_WriteData(uint8_t data)
{
    OLED_Write(OLED_CONTROL_DATA, data);
}

static uint32_t OLED_Pow(uint32_t base, uint32_t exponent)
{
    uint32_t result = 1U;

    while (exponent-- != 0U) {
        result *= base;
    }
    return result;
}

static void OLED_SetCursor(uint8_t page, uint8_t column)
{
    OLED_WriteCommand((uint8_t)(0xB0U | page));                      /* 页地址 */
    OLED_WriteCommand((uint8_t)(0x10U | ((column & 0xF0U) >> 4U)));  /* 列高 4 位 */
    OLED_WriteCommand((uint8_t)(0x00U | (column & 0x0FU)));          /* 列低 4 位 */
}

void OLED_Clear(void)
{
    uint8_t page;
    uint8_t column;

    for (page = 0U; page < 8U; page++) {
        OLED_SetCursor(page, 0U);
        for (column = 0U; column < 128U; column++) {
            OLED_WriteData(0x00U);
        }
    }
}

void OLED_ShowChar(uint8_t line, uint8_t column, char ch)
{
    uint8_t i;
    uint8_t fontIndex;

    if ((line < 1U) || (line > 4U) || (column < 1U) || (column > 16U)) {
        return;
    }

    /* 字库只覆盖可打印 ASCII，越界字符统一显示空格而不是乱码。 */
    if ((ch < ' ') || (ch > '~')) {
        ch = ' ';
    }
    fontIndex = (uint8_t)(ch - ' ');

    /* 8x16 字符占两个页：上半页与下半页。 */
    OLED_SetCursor((uint8_t)((line - 1U) * 2U), (uint8_t)((column - 1U) * 8U));
    for (i = 0U; i < 8U; i++) {
        OLED_WriteData(OLED_F8x16[fontIndex][i]);
    }

    OLED_SetCursor((uint8_t)((line - 1U) * 2U + 1U), (uint8_t)((column - 1U) * 8U));
    for (i = 0U; i < 8U; i++) {
        OLED_WriteData(OLED_F8x16[fontIndex][i + 8U]);
    }
}

void OLED_ShowString(uint8_t line, uint8_t column, const char *string)
{
    while ((string != 0) && (*string != '\0') && (column <= 16U)) {
        OLED_ShowChar(line, column, *string);
        string++;
        column++;
    }
}

void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        OLED_ShowChar(line, (uint8_t)(column + i),
            (char)((number / OLED_Pow(10U, (uint32_t)(length - i - 1U)) % 10U) + '0'));
    }
}

void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length)
{
    uint8_t i;
    uint32_t absNumber;

    if (number >= 0) {
        OLED_ShowChar(line, column, '+');
        absNumber = (uint32_t)number;
    } else {
        OLED_ShowChar(line, column, '-');
        absNumber = (uint32_t)(-number);
    }

    for (i = 0U; i < length; i++) {
        OLED_ShowChar(line, (uint8_t)(column + i + 1U),
            (char)((absNumber / OLED_Pow(10U, (uint32_t)(length - i - 1U)) % 10U) + '0'));
    }
}

void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t i;
    uint8_t digit;

    for (i = 0U; i < length; i++) {
        digit = (uint8_t)(number / OLED_Pow(16U, (uint32_t)(length - i - 1U)) % 16U);
        if (digit < 10U) {
            OLED_ShowChar(line, (uint8_t)(column + i), (char)(digit + '0'));
        } else {
            OLED_ShowChar(line, (uint8_t)(column + i), (char)(digit - 10U + 'A'));
        }
    }
}

void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t i;

    for (i = 0U; i < length; i++) {
        OLED_ShowChar(line, (uint8_t)(column + i),
            (char)((number / OLED_Pow(2U, (uint32_t)(length - i - 1U)) % 2U) + '0'));
    }
}

void OLED_ShowFloat(uint8_t line, uint8_t column, float number,
                    uint8_t intLength, uint8_t fracLength)
{
    int32_t intPart;
    uint32_t fracPart;
    float fracValue;
    uint8_t i;

    intPart = (int32_t)number;
    fracValue = number - (float)intPart;
    if (fracValue < 0.0f) {
        fracValue = -fracValue;
    }
    for (i = 0U; i < fracLength; i++) {
        fracValue *= 10.0f;
    }
    fracPart = (uint32_t)fracValue;

    OLED_ShowSignedNum(line, column, intPart, intLength);
    OLED_ShowChar(line, (uint8_t)(column + intLength + 1U), '.');
    OLED_ShowNum(line, (uint8_t)(column + intLength + 2U), fracPart, fracLength);
}

void OLED_Init(void)
{
    g_errorCount = 0U;

    /* 上电后等电源与内部复位稳定，HAL_Delay 在初始化阶段可用。 */
    HAL_Delay(100U);

    OLED_WriteCommand(0xAEU);                        /* 关显示，开始配置 */
    OLED_WriteCommand(0xD5U); OLED_WriteCommand(0x80U); /* 显示时钟分频 */
    OLED_WriteCommand(0xA8U); OLED_WriteCommand(0x3FU); /* 复用率 1/64 */
    OLED_WriteCommand(0xD3U); OLED_WriteCommand(0x00U); /* 显示偏移 0 */
    OLED_WriteCommand(0x40U);                        /* 起始行 0 */
    OLED_WriteCommand(0xA1U);                        /* 列扫描方向（左右反了改 0xA0） */
    OLED_WriteCommand(0xC8U);                        /* COM 扫描方向（上下反了改 0xC0） */
    OLED_WriteCommand(0xDAU); OLED_WriteCommand(0x12U); /* COM 引脚硬件配置 */
    OLED_WriteCommand(0x81U); OLED_WriteCommand(0xCFU); /* 对比度 */
    OLED_WriteCommand(0xD9U); OLED_WriteCommand(0xF1U); /* 预充电周期 */
    OLED_WriteCommand(0xDBU); OLED_WriteCommand(0x30U); /* VCOMH 电压倍率 */
    OLED_WriteCommand(0xA4U);                        /* 显示内容来自显存 */
    OLED_WriteCommand(0xA6U);                        /* 正常显示不反色 */
    OLED_WriteCommand(0x8DU); OLED_WriteCommand(0x14U); /* 打开电荷泵 */
    OLED_WriteCommand(0xAFU);                        /* 打开显示 */

    OLED_Clear();
}

uint16_t OLED_GetErrorCount(void)
{
    return g_errorCount;
}
