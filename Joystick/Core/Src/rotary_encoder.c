#include "rotary_encoder.h"

static RotaryEncoder_t *encoder_instance = NULL;

void RotaryEncoder_Init(RotaryEncoder_t *encoder, GPIO_TypeDef *A_Port,
		uint16_t A_Pin, GPIO_TypeDef *B_Port, uint16_t B_Pin,
		GPIO_TypeDef *SW_Port, uint16_t SW_Pin) {
	encoder->A_Port = A_Port;
	encoder->A_Pin = A_Pin;

	encoder->B_Port = B_Port;
	encoder->B_Pin = B_Pin;

	encoder->SW_Port = SW_Port;
	encoder->SW_Pin = SW_Pin;

	encoder->counter = 0;
	encoder->direction = ROTARY_NONE;

	encoder_instance = encoder;
}

// Function to handle encoder movement
void RotaryEncoder_Update(RotaryEncoder_t *encoder) {
	uint8_t A_State = HAL_GPIO_ReadPin(encoder->A_Port, encoder->A_Pin);
	uint8_t B_State = HAL_GPIO_ReadPin(encoder->B_Port, encoder->B_Pin);

	if (A_State) {
		if (B_State) {
			encoder->direction = ROTARY_CCW;
			encoder->counter--;
		} else {
			encoder->direction = ROTARY_CW;
			encoder->counter++;
		}
	}
}

// Read switch state
uint8_t RotaryEncoder_ReadSwitch(RotaryEncoder_t *encoder) {
	return HAL_GPIO_ReadPin(encoder->SW_Port, encoder->SW_Pin);
}

// EXTI Callback (Must be placed inside stm32l0xx_it.c)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (encoder_instance == NULL)
		return;

	if (GPIO_Pin == encoder_instance->A_Pin) {
		RotaryEncoder_Update(encoder_instance);
	}
}
