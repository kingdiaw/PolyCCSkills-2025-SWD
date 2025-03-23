#ifndef HT16K33_H
#define HT16K33_H

#include "stm32l0xx_hal.h"
#include <stdbool.h>

#define HT16K33_ADDRESS 0x70 // I2C address of HT16K33
#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ 1
#define HT16K33_BLINK_1HZ 2
#define HT16K33_BLINK_HALFHZ 3
#define HT16K33_CMD_BRIGHTNESS 0xE0



typedef struct {
    I2C_HandleTypeDef *hi2c; // I2C handle
    uint16_t displayBuffer[8]; // 8x16 LED matrix buffer
} HT16K33_HandleTypeDef;

void HT16K33_Init(HT16K33_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c);
void HT16K33_SetDisplayState(HT16K33_HandleTypeDef *hdev, bool state);
void HT16K33_SetBrightness(HT16K33_HandleTypeDef *hdev, uint8_t brightness);
void HT16K33_BlinkRate(HT16K33_HandleTypeDef *hdev, uint8_t rate);
void HT16K33_Clear(HT16K33_HandleTypeDef *hdev);
void HT16K33_WriteDisplay(HT16K33_HandleTypeDef *hdev);
void HT16K33_DrawPixel(HT16K33_HandleTypeDef *hdev, uint8_t x, uint8_t y, bool state);
//void DisplayBitmapWithRotate(HT16K33_HandleTypeDef *hdev, const uint8_t *bitmap, int8_t rotateDirection);
void HT16K33_DisplayBitmapWithRotate(HT16K33_HandleTypeDef *hdev, const uint8_t *bitmap, int8_t rotateDirection);

#endif // HT16K33_H
