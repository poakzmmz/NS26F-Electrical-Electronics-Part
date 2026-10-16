#include "app_internal.h"

void TelemetryLoggerStart(void) {
  memset((void *)g_wheel4_last_timestamp_us, 0,
         sizeof(g_wheel4_last_timestamp_us));
  memset((void *)g_wheel4_last_delta_us, 0, sizeof(g_wheel4_last_delta_us));
  memset((void *)g_wheel4_pulse_count, 0, sizeof(g_wheel4_pulse_count));
  memset((void *)g_wheel4_filtered_centi_kmh, 0,
         sizeof(g_wheel4_filtered_centi_kmh));
  memset((void *)g_wheel4_filter_valid, 0, sizeof(g_wheel4_filter_valid));
  memset((void *)g_adc_dma_buffer, 0, sizeof(g_adc_dma_buffer));

  g_tim1_overflow_count = 0U;
  g_wheel4_rx_head = 0U;
  g_wheel4_rx_tail = 0U;
  g_wheel4_rx_overflow_count = 0U;
  g_adc6_rx_overflow_count = 0U;
  g_wheel4_log_count = 0U;
  g_adc6_log_count = 0U;
  g_telemetry_log_count = 0U;
  g_telemetry_snapshot_head = 0U;
  g_telemetry_snapshot_tail = 0U;
  g_telemetry_snapshot_max_depth = 0U;
  g_telemetry_snapshot_overflow_count = 0U;
  g_telemetry_missed_snapshot_count = 0U;
  g_sd_last_write_duration_ms = 0U;
  g_sd_max_write_duration_ms = 0U;
  g_adc_sample_count = 0U;
  g_adc_last_timestamp_us = 0ULL;
  memset(&g_imu_latest, 0, sizeof(g_imu_latest));
  memset(g_imu_history, 0, sizeof(g_imu_history));
  g_imu_history_head = 0U;
  g_imu_history_count = 0U;
  g_imu_parse_batch_max = 0U;
  g_imu_last_rx_timestamp_us = 0ULL;
  g_imu_timestamp_error_us = 0;
  g_imu_estimated_missing_count = 0U;
  g_imu_resync_count = 0U;
  g_imu_timeout_count = 0U;
  g_imu_recovery_count = 0U;
  g_imu_link_valid = 0U;
  memset(g_imu_rx_dma_buffer, 0, sizeof(g_imu_rx_dma_buffer));
  g_imu_rx_read_pos = 0U;
  g_imu_packet_len = 0U;
  g_imu_packet_count = 0U;
  g_imu_checksum_error_count = 0U;
  g_imu_rx_error_count = 0U;
  g_imu_rx_byte_count = 0U;
  g_imu_uart_parity_error_count = 0U;
  g_imu_uart_noise_error_count = 0U;
  g_imu_uart_framing_error_count = 0U;
  g_imu_uart_overrun_error_count = 0U;
  g_imu_uart_dma_error_count = 0U;
  g_imu_rearm_pending = 0U;
  g_wheel_legacy_queue_enabled = 0U;
  g_adc_legacy_queue_enabled = 0U;
  g_telemetry_queue_enabled = 1U;
  g_nextion_latest_rpm = 0U;
  g_nextion_rpm_valid = 0U;
  g_nextion_last_tx_tick = HAL_GetTick();

  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

  g_tim1_timebase_ready = 0U;
  if (HAL_TIM_Base_Start_IT(&htim1) == HAL_OK) {
    g_tim1_timebase_ready = 1U;
  } else {
    g_wheel4_rx_overflow_count++;
  }

  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    g_wheel4_rx_overflow_count++;
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buffer,
                        ADC_DMA_CHANNEL_COUNT) != HAL_OK) {
    g_adc6_rx_overflow_count++;
  }

  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  if (HAL_TIM_Base_Start(&htim3) != HAL_OK) {
    g_adc6_rx_overflow_count++;
  }

  IMU_StartReceiver();

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
    g_wheel4_rx_overflow_count++;
  }

  for (uint8_t channel = 1U; channel < WHEEL_CHANNEL_COUNT; channel++) {
    if (HAL_TIM_IC_Start_IT(Wheel4_HALTimer(channel),
                            Wheel4_HALChannel(channel)) != HAL_OK) {
      g_wheel4_rx_overflow_count++;
    }
  }

  g_telemetry_next_snapshot_us =
      Telemetry_GetTimestampUs() + TELEMETRY_SNAPSHOT_INTERVAL_US;
}

uint64_t Telemetry_GetTimestampUs(void) {
  uint32_t overflow_count;
  uint32_t counter;
  uint32_t primask;

  if (g_tim1_timebase_ready == 0U) {
    return ((uint64_t)HAL_GetTick()) * 1000ULL;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  overflow_count = g_tim1_overflow_count;
  counter = __HAL_TIM_GET_COUNTER(&htim1);
  if ((__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) &&
      (counter < 32768U)) {
    overflow_count++;
  }
  if (primask == 0U) {
    __enable_irq();
  }

  return ((uint64_t)overflow_count * TIM1_TICKS_PER_OVERFLOW) + counter;
}

void Telemetry_FormatU64(uint64_t value, char *output,
                                size_t output_size) {
  char reversed[21];
  size_t length = 0U;

  if (output_size == 0U) {
    return;
  }

  do {
    reversed[length++] = (char)('0' + (value % 10ULL));
    value /= 10ULL;
  } while ((value != 0ULL) && (length < sizeof(reversed)));

  if (length >= output_size) {
    output[0] = '\0';
    return;
  }

  for (size_t i = 0U; i < length; i++) {
    output[i] = reversed[length - 1U - i];
  }
  output[length] = '\0';
}

void Telemetry_SnapshotPushFromIsr(void) {
  uint16_t next_head = (uint16_t)((g_telemetry_snapshot_head + 1U) %
                                  TELEMETRY_SNAPSHOT_QUEUE_LEN);
  Telemetry_Snapshot_t *snapshot;

  if (next_head == g_telemetry_snapshot_tail) {
    g_telemetry_snapshot_overflow_count++;
    g_telemetry_missed_snapshot_count++;
    return;
  }

  snapshot = &g_telemetry_snapshot_queue[g_telemetry_snapshot_head];
  snapshot->timestamp_us = g_adc_last_timestamp_us;
  snapshot->adc_timestamp_us = g_adc_last_timestamp_us;
  snapshot->adc_sample_count = g_adc_sample_count;
  memcpy(snapshot->can, g_telemetry_latest_can, sizeof(snapshot->can));
  memcpy(snapshot->wheel, g_telemetry_latest_wheel, sizeof(snapshot->wheel));
  snapshot->imu = g_imu_latest;
  snapshot->adc_raw[0] = g_adc_dma_buffer[0];
  snapshot->adc_raw[1] = g_emu_potentiometer4_raw;
  snapshot->adc_raw[2] = g_emu_potentiometer5_raw;
  snapshot->adc_raw[3] = g_adc_dma_buffer[1];
  snapshot->adc_raw[4] = g_adc_dma_buffer[2];
  snapshot->adc_raw[5] = g_adc_dma_buffer[3];
  snapshot->adc_raw[6] = g_emu_adc6_raw;
  for (uint8_t i = 0U; i < 2U; i++) {
    uint8_t channel = (uint8_t)(i + 2U);
    snapshot->rear_pulse_count[i] = g_wheel4_pulse_count[channel];
    snapshot->rear_delta_us[i] = g_wheel4_last_delta_us[channel];
    snapshot->rear_timestamp_us[i] = g_wheel4_last_timestamp_us[channel];
  }
  g_telemetry_snapshot_head = next_head;
  {
    uint16_t depth =
        (uint16_t)((g_telemetry_snapshot_head + TELEMETRY_SNAPSHOT_QUEUE_LEN -
                    g_telemetry_snapshot_tail) %
                   TELEMETRY_SNAPSHOT_QUEUE_LEN);
    if (depth > g_telemetry_snapshot_max_depth) {
      g_telemetry_snapshot_max_depth = depth;
    }
  }
}

uint8_t Telemetry_SnapshotPop(Telemetry_Snapshot_t *snapshot) {
  uint8_t available = 0U;
  __disable_irq();
  if (g_telemetry_snapshot_tail != g_telemetry_snapshot_head) {
    *snapshot = g_telemetry_snapshot_queue[g_telemetry_snapshot_tail];
    g_telemetry_snapshot_tail =
        (uint16_t)((g_telemetry_snapshot_tail + 1U) %
                   TELEMETRY_SNAPSHOT_QUEUE_LEN);
    available = 1U;
  }
  __enable_irq();
  return available;
}

uint8_t Telemetry_SnapshotPeek(Telemetry_Snapshot_t *snapshot) {
  uint8_t available = 0U;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (g_telemetry_snapshot_tail != g_telemetry_snapshot_head) {
    *snapshot = g_telemetry_snapshot_queue[g_telemetry_snapshot_tail];
    available = 1U;
  }
  if (primask == 0U) {
    __enable_irq();
  }

  return available;
}
