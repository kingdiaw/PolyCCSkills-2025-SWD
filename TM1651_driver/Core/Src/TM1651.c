#include "TM1651.h"

// Delay macros
#define SYSTICK_LOAD (SystemCoreClock / 1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define DELAY_US(us) \
    do { \
        uint32_t start = SysTick->VAL; \
        uint32_t ticks = (us * SYSTICK_LOAD) - SYSTICK_DELAY_CALIB; \
        while ((start - SysTick->VAL) < ticks); \
    } while (0)

// Level table for display
static int8_t LevelTab[] = {0x00, 0x40, 0x60, 0x70, 0x78, 0x7C, 0x7E, 0x7F}; // Level 0~7

// Initialize TM1651
void TM1651_Init(TM1651* tm, GPIO_TypeDef* clkPort, uint16_t clkPin, GPIO_TypeDef* dataPort, uint16_t dataPin) {
    tm->ClkPort = clkPort;
    tm->ClkPin = clkPin;
    tm->DataPort = dataPort;
    tm->DataPin = dataPin;

    // Configure CLK and DIO pins as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = clkPin | dataPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(clkPort, &GPIO_InitStruct);

    // Set default brightness and clear display
    TM1651_Set(tm, BRIGHT_TYPICAL, ADDR_AUTO, STARTADDR);
    TM1651_ClearDisplay(tm);
}

// Write a byte to TM1651
void TM1651_WriteByte(TM1651* tm, int8_t wr_data) {
    uint8_t data = wr_data;

    // 8 Data Bits
    for (uint8_t i = 0; i < 8; i++) {
        // CLK low
        HAL_GPIO_WritePin(tm->ClkPort, tm->ClkPin, GPIO_PIN_RESET);
        TM1651_BitDelay();

        // Set data bit
        if (data & 0x01)
            HAL_GPIO_WritePin(tm->DataPort, tm->DataPin, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(tm->DataPort, tm->DataPin, GPIO_PIN_RESET);

        TM1651_BitDelay();

        // CLK high
        HAL_GPIO_WritePin(tm->ClkPort, tm->ClkPin, GPIO_PIN_SET);
        TM1651_BitDelay();
        data = data >> 1;
    }

    // Wait for acknowledge
    // CLK low
    HAL_GPIO_WritePin(tm->ClkPort, tm->ClkPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(tm->DataPort, tm->DataPin, GPIO_PIN_SET); // Release data line
    TM1651_BitDelay();

    // CLK high
    HAL_GPIO_WritePin(tm->ClkPort, tm->ClkPin, GPIO_PIN_SET);
    TM1651_BitDelay();

    // Read acknowledge
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = tm->DataPin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(tm->DataPort, &GPIO_InitStruct);

    TM1651_BitDelay();
    uint8_t ack = HAL_GPIO_ReadPin(tm->DataPort, tm->DataPin);

    // Restore data pin to output
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(tm->DataPort, &GPIO_InitStruct);

    TM1651_BitDelay();
    HAL_GPIO_WritePin(tm->ClkPort, tm->ClkPin, GPIO_PIN_RESET);
    TM1651_BitDelay();
}

// Send start signal
void TM1651_Start(TM1651* tm) {
    HAL_GPIO_WritePin(tm->DataPort, tm->DataPin, GPIO_PIN_RESET);
    TM1651_BitDelay();
}

// Send stop signal
void TM1651_Stop(TM1651* tm) {
    HAL_GPIO_WritePin(tm->DataPort, tm->DataPin, GPIO_PIN_RESET);
    TM1651_BitDelay();
    HAL_GPIO_WritePin(tm->ClkPort, tm->ClkPin, GPIO_PIN_SET);
    TM1651_BitDelay();
    HAL_GPIO_WritePin(tm->DataPort, tm->DataPin, GPIO_PIN_SET);
    TM1651_BitDelay();
}

// Display level (0-7)
void TM1651_DisplayLevel(TM1651* tm, uint8_t level) {
    if (level > 7) return; // Level should be 0~7

    TM1651_Start(tm);
    TM1651_WriteByte(tm, ADDR_FIXED);
    TM1651_Stop(tm);

    TM1651_Start(tm);
    TM1651_WriteByte(tm, 0xC0);
    TM1651_WriteByte(tm, LevelTab[level]);
    TM1651_Stop(tm);

    TM1651_Start(tm);
    TM1651_WriteByte(tm, tm->Cmd_DispCtrl);
    TM1651_Stop(tm);
}

// Control frame display
void TM1651_Frame(TM1651* tm, bool frameFlag) {
    int8_t segData = (frameFlag == true) ? 0x40 : 0x00;

    TM1651_Start(tm);
    TM1651_WriteByte(tm, ADDR_AUTO);
    TM1651_Stop(tm);

    TM1651_Start(tm);
    TM1651_WriteByte(tm, 0xC1);
    for (uint8_t i = 0; i < 3; i++) {
        TM1651_WriteByte(tm, segData);
    }
    TM1651_Stop(tm);

    TM1651_Start(tm);
    TM1651_WriteByte(tm, tm->Cmd_DispCtrl);
    TM1651_Stop(tm);
}

// Clear display
void TM1651_ClearDisplay(TM1651* tm) {
    TM1651_DisplayLevel(tm, 0);
    TM1651_Frame(tm, FRAME_OFF);
}

// Set brightness and display mode
void TM1651_Set(TM1651* tm, uint8_t brightness, uint8_t setData, uint8_t setAddr) {
    tm->Cmd_SetData = setData;
    tm->Cmd_SetAddr = setAddr;
    tm->Cmd_DispCtrl = 0x88 + brightness; // Set brightness
}

// Bit delay (50us)
void TM1651_BitDelay(void) {
    DELAY_US(50);
}
