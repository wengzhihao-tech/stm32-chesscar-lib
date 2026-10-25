/**
 * @file oled_ssd1306.h
 * @brief SSD1306 OLED（128x64，I2C）文本显示驱动（STM32 HAL）。
 *
 * 定位：**调试显示**，不是 UI 框架。只提供 4 行 × 16 列 8x16 字符输出，
 * 够用于现场看速度、偏差、掩码和故障码。
 *
 * 性能警告：
 *   整屏刷新 = 8 页 × 128 列 = 1024 字节 I2C 传输，400kHz 下约 25ms。
 *   **绝不能在控制中断里刷屏**，也不要每个 10ms 周期都刷。
 *   本模块建议 100ms 刷一次，且只在电机不处于高速状态时刷。
 *
 * 需要更丰富的图形（位图、多字号、中文）时，直接用成熟开源库：
 *   https://github.com/afiskon/stm32-ssd1306  （约 1.1k star，MIT，支持 I2C/SPI）
 */
#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include "bsp_config.h"
#include <stdint.h>

/** 初始化并清屏。I2C 失败时静默返回，不阻塞主流程。 */
void OLED_Init(void);

void OLED_Clear(void);

/** line 取 1~4，column 取 1~16。 */
void OLED_ShowChar(uint8_t line, uint8_t column, char ch);
void OLED_ShowString(uint8_t line, uint8_t column, const char *string);
void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length);
void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
void OLED_ShowFloat(uint8_t line, uint8_t column, float number,
                    uint8_t intLength, uint8_t fracLength);

/** 诊断：I2C 连续失败次数。非零说明接线、地址或上拉电阻有问题。 */
uint16_t OLED_GetErrorCount(void);

#endif /* OLED_SSD1306_H */
