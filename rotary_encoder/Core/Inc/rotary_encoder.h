#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "stm32l0xx_hal.h"

typedef enum {
	ROTARY_NONE = 0, ROTARY_CW,   // Clockwise rotation
	ROTARY_CCW   // Counter-clockwise rotation
} RotaryDirection_t;

typedef struct {
	GPIO_TypeDef *A_Port;
	uint16_t A_Pin;

	GPIO_TypeDef *B_Port;
	uint16_t B_Pin;

	GPIO_TypeDef *SW_Port;
	uint16_t SW_Pin;

	volatile int32_t counter; // Encoder position
	volatile RotaryDirection_t direction;
} RotaryEncoder_t;

void RotaryEncoder_Init(RotaryEncoder_t *encoder, GPIO_TypeDef *A_Port,
		uint16_t A_Pin, GPIO_TypeDef *B_Port, uint16_t B_Pin,
		GPIO_TypeDef *SW_Port, uint16_t SW_Pin);

void RotaryEncoder_Update(RotaryEncoder_t *encoder);
uint8_t RotaryEncoder_ReadSwitch(RotaryEncoder_t *encoder);

#endif // ws2812_NeoPixel_H
