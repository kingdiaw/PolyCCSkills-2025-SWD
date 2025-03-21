#ifndef TM1651_H
#define TM1651_H

#include "stm32l0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

//************definitions for TM1651*********************
#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44

#define STARTADDR  0xC0

/**** definitions for the frame of the battery display *******/
#define FRAME_ON   1
#define FRAME_OFF  0

/**************definitions for brightness***********************/
#define BRIGHT_DARKEST 0
#define BRIGHT_TYPICAL 2
#define BRIGHTEST      7

// TM1651 structure
typedef struct {
    GPIO_TypeDef* ClkPort;
    uint16_t ClkPin;
    GPIO_TypeDef* DataPort;
    uint16_t DataPin;
    uint8_t Cmd_SetData;
    uint8_t Cmd_SetAddr;
    uint8_t Cmd_DispCtrl;
} TM1651;

// Function prototypes
void TM1651_Init(TM1651* tm, GPIO_TypeDef* clkPort, uint16_t clkPin, GPIO_TypeDef* dataPort, uint16_t dataPin);
void TM1651_WriteByte(TM1651* tm, int8_t wr_data);
void TM1651_Start(TM1651* tm);
void TM1651_Stop(TM1651* tm);
void TM1651_DisplayLevel(TM1651* tm, uint8_t level);
void TM1651_Frame(TM1651* tm, bool frameFlag);
void TM1651_ClearDisplay(TM1651* tm);
void TM1651_Set(TM1651* tm, uint8_t brightness, uint8_t setData, uint8_t setAddr);
void TM1651_BitDelay(void);

#endif
