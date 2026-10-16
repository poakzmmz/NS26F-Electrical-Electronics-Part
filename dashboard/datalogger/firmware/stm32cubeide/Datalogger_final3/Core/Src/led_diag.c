#include "app_internal.h"

void LED_Set(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
  HAL_GPIO_WritePin(port, pin, state);
}

void LED_BlinkCode(uint8_t code) { LED_BlinkCodeDetail(code, 0U); }

void LED_BlinkCodeDetail(uint8_t code, uint8_t detail) {
  if (code == 0U) {
    code = 1U;
  }

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);

  while (1) {
    for (uint8_t i = 0U; i < code; i++) {
      HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
      HAL_Delay(LED_BLINK_ON_MS);
      HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
      HAL_Delay(LED_BLINK_OFF_MS);
    }

    if (detail != 0U) {
      HAL_Delay(LED_BLINK_PAUSE_MS);
      for (uint8_t i = 0U; i < detail; i++) {
        HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
        HAL_Delay(LED_BLINK_ON_MS);
        HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
        HAL_Delay(LED_BLINK_OFF_MS);
      }
    }

    HAL_Delay(LED_BLINK_PAUSE_MS);
  }
}
