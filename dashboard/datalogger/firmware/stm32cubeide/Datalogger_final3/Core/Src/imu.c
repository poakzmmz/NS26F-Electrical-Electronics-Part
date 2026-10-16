#include "app_internal.h"

int16_t IMU_ReadS16BE(const uint8_t *data) {
  return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

void IMU_StartReceiver(void) {
  if (HAL_UART_Receive_DMA(&huart6, g_imu_rx_dma_buffer,
                           IMU_RX_DMA_BUFFER_SIZE) != HAL_OK) {
    g_imu_rx_error_count++;
    g_imu_rearm_pending = 1U;
  } else {
    g_imu_rearm_pending = 0U;
  }
}

void IMU_ParseByte(uint8_t byte) {
  if (g_imu_packet_len == 0U) {
    if (byte == 0x55U) {
      g_imu_packet_buffer[0] = byte;
      g_imu_packet_len = 1U;
    }
    return;
  }

  if (g_imu_packet_len == 1U) {
    if (byte == 0x55U) {
      g_imu_packet_buffer[1] = byte;
      g_imu_packet_len = 2U;
    } else {
      g_imu_packet_len = 0U;
    }
    return;
  }

  g_imu_packet_buffer[g_imu_packet_len++] = byte;
  if (g_imu_packet_len < IMU_PACKET_SIZE) {
    return;
  }

  uint16_t checksum = 0U;
  for (uint8_t i = 0U; i < (IMU_PACKET_SIZE - 2U); i++) {
    checksum = (uint16_t)(checksum + g_imu_packet_buffer[i]);
  }
  uint16_t received_checksum =
      ((uint16_t)g_imu_packet_buffer[IMU_PACKET_SIZE - 2U] << 8) |
      (uint16_t)g_imu_packet_buffer[IMU_PACKET_SIZE - 1U];

  if (checksum == received_checksum) {

    for (uint8_t axis = 0U; axis < 3U; axis++) {
      g_imu_latest.euler_centi_deg[axis] =
          IMU_ReadS16BE(&g_imu_packet_buffer[4U + (axis * 2U)]);
      g_imu_latest.gyro_deci_dps[axis] =
          IMU_ReadS16BE(&g_imu_packet_buffer[10U + (axis * 2U)]);
      g_imu_latest.accel_milli_g[axis] =
          IMU_ReadS16BE(&g_imu_packet_buffer[16U + (axis * 2U)]);
    }
    g_imu_latest.battery_pct =
        (uint16_t)IMU_ReadS16BE(&g_imu_packet_buffer[22U]);
    g_imu_latest.timestamp_us = Telemetry_GetTimestampUs();
    g_imu_latest.valid = 1U;
    g_imu_history[g_imu_history_head] = g_imu_latest;
    g_imu_history_head =
        (uint16_t)((g_imu_history_head + 1U) % IMU_HISTORY_QUEUE_LEN);
    if (g_imu_history_count < IMU_HISTORY_QUEUE_LEN) {
      g_imu_history_count++;
    }
    g_imu_packet_count++;
  } else {
    g_imu_checksum_error_count++;
  }

  g_imu_packet_len = 0U;
}

void IMU_ProcessReceiver(void) {
  uint16_t history_start_head = g_imu_history_head;
  uint32_t packet_count_before = g_imu_packet_count;
  uint64_t previous_packet_timestamp = 0ULL;
  uint64_t process_timestamp_us;
  if (g_imu_history_count > 0U) {
    uint16_t previous_index =
        (uint16_t)((g_imu_history_head + IMU_HISTORY_QUEUE_LEN - 1U) %
                   IMU_HISTORY_QUEUE_LEN);
    previous_packet_timestamp = g_imu_history[previous_index].timestamp_us;
  }
  if (g_imu_rearm_pending != 0U) {
    (void)HAL_UART_DMAStop(&huart6);
    g_imu_rx_read_pos = 0U;
    g_imu_packet_len = 0U;
    memset(g_imu_rx_dma_buffer, 0, sizeof(g_imu_rx_dma_buffer));
    IMU_StartReceiver();
    if (g_imu_rearm_pending != 0U) {
      return;
    }
  }

  uint16_t write_pos =
      (uint16_t)((IMU_RX_DMA_BUFFER_SIZE -
                  __HAL_DMA_GET_COUNTER(&hdma_usart6_rx)) %
                 IMU_RX_DMA_BUFFER_SIZE);

  while (g_imu_rx_read_pos != write_pos) {
    IMU_ParseByte(g_imu_rx_dma_buffer[g_imu_rx_read_pos]);
    g_imu_rx_byte_count++;
    g_imu_rx_read_pos =
        (uint16_t)((g_imu_rx_read_pos + 1U) % IMU_RX_DMA_BUFFER_SIZE);
  }

  process_timestamp_us = Telemetry_GetTimestampUs();
  {
    uint32_t parsed_count = g_imu_packet_count - packet_count_before;
    if (parsed_count > 0U) {
      uint64_t newest_rx_timestamp_us = process_timestamp_us;
      uint64_t batch_span_us;
      uint64_t first_timestamp_us;
      uint8_t force_resync = 0U;
      uint8_t link_was_timed_out =
          ((g_imu_last_rx_timestamp_us != 0ULL) &&
           (g_imu_link_valid == 0U))
              ? 1U
              : 0U;

      if (parsed_count > IMU_HISTORY_QUEUE_LEN) {
        parsed_count = IMU_HISTORY_QUEUE_LEN;
      }
      if (parsed_count > g_imu_parse_batch_max) {
        g_imu_parse_batch_max = (uint16_t)parsed_count;
      }

      batch_span_us =
          ((uint64_t)(parsed_count - 1U) * IMU_PACKET_INTERVAL_US);
      if (previous_packet_timestamp != 0ULL) {
        uint64_t predicted_newest_timestamp_us =
            previous_packet_timestamp +
            ((uint64_t)parsed_count * IMU_PACKET_INTERVAL_US);
        int64_t phase_error_us =
            (int64_t)newest_rx_timestamp_us -
            (int64_t)predicted_newest_timestamp_us;
        int64_t abs_phase_error_us =
            (phase_error_us < 0LL) ? -phase_error_us : phase_error_us;

        if (phase_error_us > (int64_t)INT32_MAX) {
          g_imu_timestamp_error_us = INT32_MAX;
        } else if (phase_error_us < (int64_t)INT32_MIN) {
          g_imu_timestamp_error_us = INT32_MIN;
        } else {
          g_imu_timestamp_error_us = (int32_t)phase_error_us;
        }

        if ((abs_phase_error_us >= IMU_TIMESTAMP_RESYNC_THRESHOLD_US) ||
            (link_was_timed_out != 0U)) {
          force_resync = 1U;
          if (phase_error_us > (int64_t)(IMU_PACKET_INTERVAL_US +
                                         (IMU_PACKET_INTERVAL_US / 2ULL))) {
            uint64_t estimated_missing =
                (uint64_t)((phase_error_us +
                            (int64_t)(IMU_PACKET_INTERVAL_US / 2ULL)) /
                           (int64_t)IMU_PACKET_INTERVAL_US);
            if (estimated_missing > UINT32_MAX) {
              estimated_missing = UINT32_MAX;
            }
            g_imu_estimated_missing_count += (uint32_t)estimated_missing;
          }
        }

        if (force_resync != 0U) {
          first_timestamp_us =
              (newest_rx_timestamp_us >= batch_span_us)
                  ? (newest_rx_timestamp_us - batch_span_us)
                  : 0ULL;
          g_imu_resync_count++;
        } else {
          int64_t correction_us =
              phase_error_us / IMU_TIMESTAMP_CORRECTION_DIV;
          uint64_t predicted_first_timestamp_us =
              previous_packet_timestamp + IMU_PACKET_INTERVAL_US;

          if (correction_us > IMU_TIMESTAMP_CORRECTION_MAX_US) {
            correction_us = IMU_TIMESTAMP_CORRECTION_MAX_US;
          } else if (correction_us < -IMU_TIMESTAMP_CORRECTION_MAX_US) {
            correction_us = -IMU_TIMESTAMP_CORRECTION_MAX_US;
          }

          if ((correction_us < 0LL) &&
              (predicted_first_timestamp_us <
               (uint64_t)(-correction_us))) {
            first_timestamp_us = 0ULL;
          } else {
            first_timestamp_us =
                (uint64_t)((int64_t)predicted_first_timestamp_us +
                           correction_us);
          }
        }
      } else {
        first_timestamp_us = (newest_rx_timestamp_us >= batch_span_us)
                                 ? (newest_rx_timestamp_us - batch_span_us)
                                 : 0ULL;
        g_imu_timestamp_error_us = 0;
      }

      for (uint32_t i = 0U; i < parsed_count; i++) {
        uint16_t index = (uint16_t)((history_start_head + i) %
                                    IMU_HISTORY_QUEUE_LEN);
        g_imu_history[index].timestamp_us = first_timestamp_us +
            ((uint64_t)i * IMU_PACKET_INTERVAL_US);
      }
      {
        uint16_t newest_index =
            (uint16_t)((g_imu_history_head + IMU_HISTORY_QUEUE_LEN - 1U) %
                       IMU_HISTORY_QUEUE_LEN);
        g_imu_latest = g_imu_history[newest_index];
      }

      if (link_was_timed_out != 0U) {
        g_imu_recovery_count++;
      }
      g_imu_last_rx_timestamp_us = newest_rx_timestamp_us;
      g_imu_link_valid = 1U;
    }
  }

  if ((g_imu_last_rx_timestamp_us != 0ULL) &&
      (g_imu_link_valid != 0U) &&
      ((process_timestamp_us - g_imu_last_rx_timestamp_us) >
       IMU_DATA_TIMEOUT_US)) {
    g_imu_link_valid = 0U;
    g_imu_timeout_count++;
  }
}

uint8_t IMU_HistoryGetAt(uint64_t timestamp_us,
                                Telemetry_IMULatest_t *sample) {
  uint8_t found = 0U;
  uint64_t best_timestamp = 0ULL;
  uint16_t oldest =
      (uint16_t)((g_imu_history_head + IMU_HISTORY_QUEUE_LEN -
                  g_imu_history_count) %
                 IMU_HISTORY_QUEUE_LEN);

  for (uint16_t i = 0U; i < g_imu_history_count; i++) {
    uint16_t index = (uint16_t)((oldest + i) % IMU_HISTORY_QUEUE_LEN);
    const Telemetry_IMULatest_t *candidate = &g_imu_history[index];
    if ((candidate->valid != 0U) &&
        (candidate->timestamp_us <= timestamp_us) &&
        (candidate->timestamp_us >= best_timestamp)) {
      *sample = *candidate;
      best_timestamp = candidate->timestamp_us;
      found = 1U;
    }
  }
  return found;
}
