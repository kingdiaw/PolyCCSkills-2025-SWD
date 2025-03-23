# Rotary Encoder Driver for STM32L053

This repository contains a rotary encoder driver for the STM32L053 microcontroller. The driver uses interrupts to detect changes in the rotary encoder's position and provides an interface to read the encoder's position, direction, and button state.

## Features

- **Rotary Encoder Support**: Detects clockwise (CW) and counter-clockwise (CCW) rotations.
- **Button Support**: Detects button presses on the rotary encoder.
- **Interrupt-Based**: Uses interrupts for efficient detection of encoder movements.
- **Debouncing**: Simple debouncing mechanism for the button.

## Hardware Setup

- **Rotary Encoder Signal A**: Connected to `PA10` on the STM32L053.
- **Rotary Encoder Signal B**: Connected to `PB3` on the STM32L053.
- **Rotary Encoder Button (SW)**: Connected to `PB5` on the STM32L053.

## Software Requirements

- STM32CubeMX for initializing the STM32 peripherals.
- STM32L0xx HAL library.
- A toolchain for ARM Cortex-M (e.g., GCC, Keil, IAR).

## Usage

### Initialization

1. **Include the Header File**: Include the `rotary_encoder.h` file in your main application.

    ```c
    #include "rotary_encoder.h"
    ```

2. **Initialize the Rotary Encoder**: Call the `RotaryEncoder_Init` function to initialize the encoder.

    ```c
    RotaryEncoder_t encoder;
    RotaryEncoder_Init(&encoder, GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_3, GPIOB, GPIO_PIN_5);
    ```

### Reading Encoder Position and Direction

- The encoder's position is stored in the `counter` field of the `RotaryEncoder_t` structure.
- The encoder's direction is stored in the `direction` field of the `RotaryEncoder_t` structure.

    ```c
    int32_t position = encoder.counter;
    RotaryDirection_t direction = encoder.direction;
    ```

### Reading the Button State

- Use the `RotaryEncoder_ReadSwitch` function to read the state of the button.

    ```c
    uint8_t button_state = RotaryEncoder_ReadSwitch(&encoder);
    ```

### Example Code

Here is an example of how to use the rotary encoder driver in your main application:

```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    RotaryEncoder_t encoder;
    RotaryEncoder_Init(&encoder, GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_3, GPIOB, GPIO_PIN_5);

    int8_t pos = 0;
    uint8_t sw_state_new, sw_state_old = GPIO_PIN_SET;

    while (1) {
        sw_state_new = RotaryEncoder_ReadSwitch(&encoder);
        if (sw_state_old == GPIO_PIN_SET && sw_state_new == GPIO_PIN_RESET) {
            printf("Button Pressed!\n");
        }
        sw_state_old = sw_state_new;

        if (encoder.direction != ROTARY_NONE) {
            printf("Encoder Direction: %s\n", encoder.direction == ROTARY_CW ? "CW" : "CCW");
            if (encoder.direction == ROTARY_CW) {
                if (++pos > 5) pos = 5;
            } else {
                if (--pos < 0) pos = 0;
            }
            encoder.direction = ROTARY_NONE;
            printf("Encoder Pos: %d\n", pos);
        }
        HAL_Delay(50); // Debouncing
    }
}
```

## Configuration

### GPIO Configuration

- **Signal A (PA10)**: Configured as an input with interrupt on rising/falling edge.
- **Signal B (PB3)**: Configured as an input.
- **Button (PB5)**: Configured as an input with pull-up resistor.

### Interrupt Configuration

- The EXTI interrupt for `PA10` must be enabled in the STM32CubeMX configuration.
- The `HAL_GPIO_EXTI_Callback` function must be placed in the `stm32l0xx_it.c` file to handle the interrupt.

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (encoder_instance == NULL) return;

    if (GPIO_Pin == encoder_instance->A_Pin) {
        RotaryEncoder_Update(encoder_instance);
    }
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

## Acknowledgments

- STMicroelectronics for the STM32L0xx HAL library.
- The open-source community for inspiration and support.

---

This README provides a basic overview of the rotary encoder driver and how to use it in your STM32L053 project. For more detailed information, refer to the source code and comments within the files.
