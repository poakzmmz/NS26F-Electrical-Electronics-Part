#include "app_internal.h"

void App_Init(void)
{
  USER_CAN1_Apply500Kbps();
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);

  for (uint8_t i = 0U; i < 3U; i++) {
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_SET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
    HAL_Delay(120U);
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
    HAL_Delay(120U);
  }

  SD_WaitForCardInserted();
  RPMOutputs_Init();
  GNSSLoggerStart();

  if (ADC_LoggerConfigureHardware() == 0U) {
    LED_BlinkCode(SD_LED_ERR_ADC_CONFIG);
  }

  g_emu_can_enabled = EMU_CANLoggerConfigureHardware();
  g_sd_last_fresult = SD_TelemetryLoggerInit();
  TelemetryLoggerStart();

  if (g_emu_can_enabled != 0U) {
    EMU_CANLoggerStart();
  }

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
}

void App_Process(void)
{
  g_sd_last_fresult = SD_TelemetryLoggerProcess();
  RPMOutputs_Process();
}
