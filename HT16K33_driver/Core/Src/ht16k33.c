#include "ht16k33.h"

void HT16K33_Init(HT16K33_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c) {
    hdev->hi2c = hi2c;

    // Turn on oscillator
    uint8_t buffer[1] = {0x21};
    HAL_I2C_Master_Transmit(hdev->hi2c, HT16K33_ADDRESS << 1, buffer, 1, HAL_MAX_DELAY);

    // Clear display buffer and update display
    HT16K33_Clear(hdev);
    HT16K33_WriteDisplay(hdev);

    // Set default settings
    HT16K33_BlinkRate(hdev, HT16K33_BLINK_OFF);
    HT16K33_SetBrightness(hdev, 15); // Max brightness
    HT16K33_SetDisplayState(hdev, true);
}

void HT16K33_SetDisplayState(HT16K33_HandleTypeDef *hdev, bool state) {
    uint8_t buffer;
    if (state)
        buffer = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON;
    else
        buffer = HT16K33_BLINK_CMD;
    HAL_I2C_Master_Transmit(hdev->hi2c, HT16K33_ADDRESS << 1, &buffer, 1, HAL_MAX_DELAY);
}

void HT16K33_SetBrightness(HT16K33_HandleTypeDef *hdev, uint8_t brightness) {
    if (brightness > 15) brightness = 15;
    uint8_t buffer = HT16K33_CMD_BRIGHTNESS | brightness;
    HAL_I2C_Master_Transmit(hdev->hi2c, HT16K33_ADDRESS << 1, &buffer, 1, HAL_MAX_DELAY);
}

void HT16K33_BlinkRate(HT16K33_HandleTypeDef *hdev, uint8_t rate) {
    if (rate > 3) rate = 0;
    uint8_t buffer = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (rate << 1);
    HAL_I2C_Master_Transmit(hdev->hi2c, HT16K33_ADDRESS << 1, &buffer, 1, HAL_MAX_DELAY);
}

void HT16K33_Clear(HT16K33_HandleTypeDef *hdev) {
    for (uint8_t i = 0; i < 8; i++) {
        hdev->displayBuffer[i] = 0;
    }
}

void HT16K33_WriteDisplay(HT16K33_HandleTypeDef *hdev) {
    uint8_t buffer[17];
    buffer[0] = 0x00; // Start at address $00

    for (uint8_t i = 0; i < 8; i++) {
        buffer[1 + 2 * i] = hdev->displayBuffer[i] & 0xFF; // Lower byte
        buffer[2 + 2 * i] = hdev->displayBuffer[i] >> 8;  // Upper byte
    }

    HAL_I2C_Master_Transmit(hdev->hi2c, HT16K33_ADDRESS << 1, buffer, 17, HAL_MAX_DELAY);
}

void HT16K33_DrawPixel(HT16K33_HandleTypeDef *hdev, uint8_t x, uint8_t y, bool state) {
    if (x >= 16 || y >= 8) return; // Out of bounds

    if (state)
        hdev->displayBuffer[y] |= (1 << x);
    else
        hdev->displayBuffer[y] &= ~(1 << x);
}

