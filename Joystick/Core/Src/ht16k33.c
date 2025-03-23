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

void HT16K33_DisplayBitmapWithRotate(HT16K33_HandleTypeDef *hdev, const uint8_t *bitmap, int8_t rotateDirection) {
    static uint8_t rotateOffset = 0; // Tracks the current rotation position

    // Update the rotate offset based on the direction
    if (rotateDirection == 1) {
        rotateOffset = (rotateOffset + 1) % 16; // Rotate right
    } else if (rotateDirection == -1) {
        rotateOffset = (rotateOffset - 1 + 16) % 16; // Rotate left
    }
    // If rotateDirection is 0, rotateOffset remains unchanged (static display)

    // Convert the 8x8 bitmap into the 8x16 display buffer with rotation
    for (uint8_t y = 0; y < 8; y++) {
        // Center the 8x8 bitmap by shifting it left by 4 bits
        uint16_t rowData = (uint16_t)bitmap[y] << 4;

        // Apply the rotation by shifting the row data based on rotateOffset
        if (rotateDirection != 0) {
            // Rotate the row data by rotateOffset bits
            rowData = (rowData << rotateOffset) | (rowData >> (16 - rotateOffset));
        }

        // Store the rotated row data in the display buffer
        hdev->displayBuffer[y] = rowData;
    }

    HT16K33_WriteDisplay(hdev); // Update the display
}
