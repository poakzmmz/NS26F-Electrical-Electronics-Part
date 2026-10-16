#include "app_internal.h"

uint8_t ADC_LoggerConfigureHardware(void) {
  static const uint32_t adc_channels[ADC_DMA_CHANNEL_COUNT] = {
      ADC_CHANNEL_10, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15};
  ADC_ChannelConfTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  if (HAL_ADC_DeInit(&hadc1) != HAL_OK) {
    return 0U;
  }

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = ADC_DMA_CHANNEL_COUNT;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    return 0U;
  }

  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  for (uint32_t i = 0U; i < ADC_DMA_CHANNEL_COUNT; i++) {
    sConfig.Channel = adc_channels[i];
    sConfig.Rank = (uint32_t)(i + 1U);
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
      return 0U;
    }
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    return 0U;
  }

  return 1U;
}

void ADC_QueuePushFromIsr(const uint16_t *raw) {
  ADC_LogSample_t sample;
  uint16_t next_head = (uint16_t)((g_adc_rx_head + 1U) % ADC_LOG_QUEUE_LEN);

  if (next_head == g_adc_rx_tail) {
    g_adc_rx_overflow_count++;
    return;
  }

  sample.timestamp_ms = HAL_GetTick();
  memset(sample.raw, 0, sizeof(sample.raw));
  sample.raw[0] = raw[0];
  sample.raw[1] = g_emu_potentiometer4_raw;
  sample.raw[2] = g_emu_potentiometer5_raw;
  sample.raw[3] = raw[1];
  sample.raw[4] = raw[2];
  sample.raw[5] = raw[3];
  sample.raw[6] = g_emu_adc6_raw;

  g_adc_rx_queue[g_adc_rx_head] = sample;
  g_adc_rx_head = next_head;
}
