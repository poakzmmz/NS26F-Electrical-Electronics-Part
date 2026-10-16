#include "app_internal.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance != CAN1) {
    return;
  }

  EMU_CANDrainRxFifo();
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance != CAN1) {
    return;
  }

  EMU_CANDrainRxFifo();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN1) {
    g_emu_can_hal_error_count++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {

    GNSS_QueuePushFromIsr(g_gnss_rx_byte);
    if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_gnss_rx_byte, 1U) !=
        HAL_OK) {
      g_gnss_uart_error_count++;
    }
  }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART4) {
    g_race_nextion_uart_error_callback_count++;
    g_race_nextion_uart_last_error = huart->ErrorCode;
    g_race_nextion_uart_last_state = HAL_UART_GetState(huart);
  }

  if (huart->Instance == USART1) {

    g_gnss_uart_error_count++;
    if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_gnss_rx_byte, 1U) !=
        HAL_OK) {
      g_gnss_uart_error_count++;
    }
  }

  if (huart->Instance == USART6) {

    g_imu_rx_error_count++;
    if ((huart->ErrorCode & HAL_UART_ERROR_PE) != 0U) {
      g_imu_uart_parity_error_count++;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_NE) != 0U) {
      g_imu_uart_noise_error_count++;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_FE) != 0U) {
      g_imu_uart_framing_error_count++;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U) {
      g_imu_uart_overrun_error_count++;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U) {
      g_imu_uart_dma_error_count++;
    }
    g_imu_rearm_pending = 1U;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != ADC1) {
    return;
  }

  if (g_adc_legacy_queue_enabled != 0U) {
    ADC_QueuePushFromIsr((const uint16_t *)g_adc_dma_buffer);
  }
  g_adc_last_timestamp_us = Telemetry_GetTimestampUs();
  g_adc_sample_count++;
  if (g_telemetry_queue_enabled != 0U) {
    Telemetry_SnapshotPushFromIsr();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    g_tim1_overflow_count++;
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM8) {
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0U);
    g_ws2812_dma_done_count++;
    g_ws2812_dma_ready = 1U;
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  Wheel_LogEvent_t event;
  Wheel4_LogEvent_t wheel4_event;
  uint8_t channel;
  uint16_t capture;
  uint64_t timestamp_us;
  uint32_t delta_us;
  uint32_t raw_centi_kmh;

  if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)) {
    g_pb10_edge_count++;
  }

  channel = Wheel4_ChannelIndex(htim);
  if (channel >= WHEEL_CHANNEL_COUNT) {
    return;
  }

  g_wheel4_pulse_count[channel]++;

  capture =
      (uint16_t)HAL_TIM_ReadCapturedValue(htim, Wheel4_HALChannel(channel));
  if (htim->Instance == TIM1) {
    uint32_t overflow_count = g_tim1_overflow_count;

    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (capture < 32768U)) {
      overflow_count++;
    }

    timestamp_us =
        ((uint64_t)overflow_count * TIM1_TICKS_PER_OVERFLOW) + capture;
  } else {
    timestamp_us = Telemetry_GetTimestampUs();
  }

  delta_us =
      (g_wheel4_last_timestamp_us[channel] == 0ULL)
          ? 0U
          : (uint32_t)(timestamp_us - g_wheel4_last_timestamp_us[channel]);

  g_wheel4_last_timestamp_us[channel] = timestamp_us;
  g_wheel4_last_delta_us[channel] = delta_us;

  wheel4_event.timestamp_us = timestamp_us;
  wheel4_event.channel = channel;
  wheel4_event.delta_us = delta_us;
  wheel4_event.speed_centi_kmh = g_wheel4_filtered_centi_kmh[channel];

  if (delta_us >= SPEED_MIN_VALID_DELTA_US) {
    raw_centi_kmh = SPEED_CENTI_KMH_SCALE / delta_us;
    if (g_wheel4_filter_valid[channel] == 0U) {
      g_wheel4_filtered_centi_kmh[channel] = raw_centi_kmh;
      g_wheel4_filter_valid[channel] = 1U;
    } else {
      g_wheel4_filtered_centi_kmh[channel] =
          ((g_wheel4_filtered_centi_kmh[channel] *
            (SPEED_FILTER_ALPHA_DEN - SPEED_FILTER_ALPHA_NUM)) +
           (raw_centi_kmh * SPEED_FILTER_ALPHA_NUM)) /
          SPEED_FILTER_ALPHA_DEN;
    }

    wheel4_event.speed_centi_kmh = g_wheel4_filtered_centi_kmh[channel];
  }

  if (g_telemetry_queue_enabled != 0U) {
    g_telemetry_latest_wheel[channel].valid = 1U;
    g_telemetry_latest_wheel[channel].timestamp_us = timestamp_us;
    g_telemetry_latest_wheel[channel].delta_us = delta_us;
    g_telemetry_latest_wheel[channel].speed_centi_kmh =
        wheel4_event.speed_centi_kmh;
    Wheel4_QueuePushFromIsr(&wheel4_event);
  }

  if ((channel != 0U) || (g_wheel_legacy_queue_enabled == 0U)) {
    return;
  }

  event.timestamp_us = timestamp_us;
  event.delta_us = (g_wheel_last_timestamp_us == 0ULL)
                       ? 0U
                       : (uint32_t)(timestamp_us - g_wheel_last_timestamp_us);
  g_wheel_last_timestamp_us = timestamp_us;

  Wheel_QueuePushFromIsr(&event);
}
