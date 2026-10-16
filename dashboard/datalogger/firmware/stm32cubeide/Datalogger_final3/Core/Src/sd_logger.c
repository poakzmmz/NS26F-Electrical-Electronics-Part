#include "app_internal.h"

void SD_RecordFault(SD_FaultStage_t stage, FRESULT result,
                           uint32_t detail) {
  g_sd_fault_stage = stage;
  g_sd_fault_fresult = result;
  g_sd_fault_detail = detail;
  g_sd_fault_tick = HAL_GetTick();
  g_sd_fault_buffer_len = g_sd_log_buffer_len;
  g_sd_fault_unsynced_count = g_sd_log_unsynced_count;
  g_sd_fault_telemetry_count = g_telemetry_log_count;

  SD_WriteFaultLog();
}

void SD_WriteFaultLog(void) {
  static uint8_t fault_log_busy = 0U;
  FIL fault_file;
  UINT bytes_done = 0U;
  char line[768];
  int line_len;
  FRESULT res;

  if (fault_log_busy != 0U) {
    return;
  }

  fault_log_busy = 1U;
  line_len = snprintf(
      line, sizeof(line),
      "tick=%lu,stage=%lu,%s,fresult=%u,detail=%lu,"
      "buf_len=%u,unsynced=%lu,telemetry=%lu,"
      "last_write_tick=%lu,last_sync_tick=%lu,"
      "hal_error=0x%08lX,hal_stage=%lu,"
      "writer_bytes=%lu,write_calls=%lu,last_write=%lu,write_fail=%lu,"
      "sync_ok=%lu,sync_fail=%lu,last_sync_res=%u,flush_reason=%lu,"
      "ll_op=%lu,ll_rd=%lu,ll_wr=%lu,ll_retry=%lu,ll_rd_fail=%lu,"
      "ll_wr_fail=%lu,ll_ready_to=%lu,ll_sector=%lu,ll_blocks=%lu,"
      "ll_buf=0x%08lX,ll_hal=%lu,ll_err=0x%08lX,ll_state=%lu,"
      "ll_card=%lu,ll_wait=%lu,ll_tick=%lu,"
      "gnss_rx=%lu,gnss_drain=%lu,gnss_ovf=%lu,gnss_uart=%lu,"
      "can_rx=%lu,can_ovf=%lu,can_hal=%lu,wheel_ovf=%lu,adc_ovf=%lu,"
      "active_path=%s\r\n",
      (unsigned long)g_sd_fault_tick, (unsigned long)g_sd_fault_stage,
      SD_FaultStageName(g_sd_fault_stage), (unsigned int)g_sd_fault_fresult,
      (unsigned long)g_sd_fault_detail, (unsigned int)g_sd_fault_buffer_len,
      (unsigned long)g_sd_fault_unsynced_count,
      (unsigned long)g_sd_fault_telemetry_count,
      (unsigned long)g_sd_log_last_write_tick,
      (unsigned long)g_sd_log_last_sync_tick, (unsigned long)hsd.ErrorCode,
      (unsigned long)g_sd_hal_stage, (unsigned long)g_sd_total_bytes_written,
      (unsigned long)g_sd_write_call_count, (unsigned long)g_sd_last_write_size,
      (unsigned long)g_sd_write_fail_count, (unsigned long)g_sd_sync_ok_count,
      (unsigned long)g_sd_sync_fail_count, (unsigned int)g_sd_last_sync_fresult,
      (unsigned long)g_sd_last_flush_reason, (unsigned long)g_sd_ll_last_op,
      (unsigned long)g_sd_ll_read_call_count,
      (unsigned long)g_sd_ll_write_call_count,
      (unsigned long)g_sd_ll_retry_count,
      (unsigned long)g_sd_ll_read_fail_count,
      (unsigned long)g_sd_ll_write_fail_count,
      (unsigned long)g_sd_ll_ready_timeout_count,
      (unsigned long)g_sd_ll_last_sector, (unsigned long)g_sd_ll_last_blocks,
      (unsigned long)g_sd_ll_last_buffer_addr,
      (unsigned long)g_sd_ll_last_hal_status,
      (unsigned long)g_sd_ll_last_hal_error,
      (unsigned long)g_sd_ll_last_hal_state,
      (unsigned long)g_sd_ll_last_card_state,
      (unsigned long)g_sd_ll_last_wait_ms, (unsigned long)g_sd_ll_last_tick,
      (unsigned long)g_gnss_rx_count, (unsigned long)g_gnss_drain_byte_count,
      (unsigned long)g_gnss_rx_overflow_count,
      (unsigned long)g_gnss_uart_error_count, (unsigned long)g_emu_can_rx_count,
      (unsigned long)g_emu_can_rx_overflow_count,
      (unsigned long)g_emu_can_hal_error_count,
      (unsigned long)g_wheel4_rx_overflow_count,
      (unsigned long)g_adc6_rx_overflow_count, g_sd_log_path);

  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line))) {
    g_sd_fault_log_fresult = FR_INT_ERR;
    fault_log_busy = 0U;
    return;
  }

  res = f_open(&fault_file, SD_FAULT_LOG_FILE, FA_OPEN_APPEND | FA_WRITE);
  if (res == FR_OK) {
    res = f_write(&fault_file, line, (UINT)line_len, &bytes_done);
    if ((res == FR_OK) && (bytes_done != (UINT)line_len)) {
      res = FR_DISK_ERR;
    }

    if (res == FR_OK) {
      res = f_close(&fault_file);
    } else {
      (void)f_close(&fault_file);
    }
  }

  g_sd_fault_log_fresult = res;
  fault_log_busy = 0U;
}

const char *SD_FaultStageName(SD_FaultStage_t stage) {
  switch (stage) {
  case SD_FAULT_NO_CARD:
    return "NO_CARD";
  case SD_FAULT_MOUNT:
    return "MOUNT";
  case SD_FAULT_OPEN_TELEMETRY:
    return "OPEN_TELEMETRY";
  case SD_FAULT_HEADER_WRITE:
    return "HEADER_WRITE";
  case SD_FAULT_HEADER_SHORT_WRITE:
    return "HEADER_SHORT_WRITE";
  case SD_FAULT_HEADER_SYNC:
    return "HEADER_SYNC";
  case SD_FAULT_TELEMETRY_DRAIN:
    return "TELEMETRY_DRAIN";
  case SD_FAULT_SNAPSHOT_FORMAT:
    return "SNAPSHOT_FORMAT";
  case SD_FAULT_SNAPSHOT_FLUSH:
    return "SNAPSHOT_FLUSH";
  case SD_FAULT_SNAPSHOT_BUFFER_FULL:
    return "SNAPSHOT_BUFFER_FULL";
  case SD_FAULT_IDLE_FLUSH:
    return "IDLE_FLUSH";
  case SD_FAULT_INTERVAL_SYNC:
    return "INTERVAL_SYNC";
  case SD_FAULT_FLUSH_NO_ACTIVE_FILE:
    return "FLUSH_NO_ACTIVE_FILE";
  case SD_FAULT_FLUSH_WRITE:
    return "FLUSH_WRITE";
  case SD_FAULT_FLUSH_SHORT_WRITE:
    return "FLUSH_SHORT_WRITE";
  case SD_FAULT_FLUSH_SYNC:
    return "FLUSH_SYNC";
  case SD_FAULT_NONE:
  default:
    return "NONE";
  }
}

void SD_RecoverAfterIoError(void) {
  HAL_SD_Abort(&hsd);
  (void)HAL_SD_DeInit(&hsd);
  HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
  (void)HAL_SD_Init(&hsd);

  if (SD_BRINGUP_USE_4BIT_BUS != 0U) {
    (void)HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
  }

  HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
}

uint8_t SD_WaitForTransferReady(uint32_t timeout_ms) {
  uint32_t start_tick = HAL_GetTick();
  HAL_SD_CardStateTypeDef card_state;

  do {
    card_state = HAL_SD_GetCardState(&hsd);
    g_sd_ll_last_card_state = card_state;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
    g_sd_ll_last_hal_error = hsd.ErrorCode;

    if (card_state == HAL_SD_CARD_TRANSFER) {
      g_sd_ll_last_wait_ms = HAL_GetTick() - start_tick;
      return MSD_OK;
    }
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  g_sd_ll_ready_timeout_count++;
  g_sd_ll_last_wait_ms = HAL_GetTick() - start_tick;
  g_sd_hal_error_code = hsd.ErrorCode;
  return MSD_ERROR;
}

void SD_WaitForCardInserted(void) {
  uint32_t last_print = 0U;
  while (SD_IsCardInserted() == GPIO_PIN_RESET) {
    uint32_t now = HAL_GetTick();
    if ((now - last_print) >= 1000U) {
      const char msg[] = "Waiting for SD card...\r\n";
      (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                              (uint16_t)(sizeof(msg) - 1U), 50U);
      last_print = now;
    }
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);
    HAL_Delay(TELEMETRY_SD_MISSING_BLINK_MS);
  }

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
}

FRESULT SD_TelemetryLoggerInit(void) {
  FRESULT res;
  UINT bytes_done = 0U;
  static const char header[] =
      "gps_time,timestamp_us,seq,"
      "front_brake_pressure_raw,rl_potentiometer_raw,"
      "rr_potentiometer_raw,fr_potentiometer_raw,steering_angle_raw,"
      "fl_potentiometer_raw,rear_brake_pressure_raw,"
      "can600_data,can601_data,can602_data,can603_data,"
      "can604_data,can605_data,can606_data,can607_data,"
      "wheel1_speed_centi_kmh,wheel2_speed_centi_kmh,"
      "rr_wheel_speed_centi_kmh,rl_wheel_speed_centi_kmh,"
      "rr_pulse_count,rl_pulse_count,rr_delta_us,rl_delta_us,"
      "rr_age_us,rl_age_us,"
      "imu_gyro_x_deci_dps,imu_gyro_y_deci_dps,imu_gyro_z_deci_dps,"
      "imu_accel_x_milli_g,imu_accel_y_milli_g,imu_accel_z_milli_g,"
      "imu_battery_pct,imu_age_us,"
      "imu_rx_byte_count,imu_packet_count,imu_checksum_error_count,"
      "imu_uart_error_count,imu_uart_parity_error_count,"
      "imu_uart_noise_error_count,imu_uart_framing_error_count,"
      "imu_uart_overrun_error_count,imu_uart_dma_error_count,"
      "gps_rx_byte_count,gps_0xB5_count,gps_dollar_count,gps_rx_overflow_count,"
      "gps_lat,gps_lon,gps_speed_kmh,gps_sat,gps_qual,gps_heading,"
      "logger_timestamp_us,"
      "can600_age_us,can601_age_us,can602_age_us,can603_age_us,"
      "can604_age_us,can605_age_us,can606_age_us,can607_age_us,"
      "can_valid_mask,can_rx_count,can_rx_overflow_count,can_hal_error_count,"
      "adc_sample_timestamp_us,adc_age_us,adc_sample_count,"
      "gps_fix_age_us,gps_fix_update_count,gps_uart_error_count,"
      "sd_write_call_count,sd_write_fail_count,sd_sync_ok_count,"
      "sd_sync_fail_count,sd_ll_retry_count,sd_ll_write_fail_count,"
      "sd_ll_ready_timeout_count,sd_max_buffer_len,sd_unsynced_count,"
      "telemetry_missed_snapshot_count,wheel_rx_overflow_count,"
      "imu_roll_centi_deg,imu_pitch_centi_deg,imu_yaw_centi_deg,"
      "imu_sample_timestamp_us,telemetry_queue_depth,"
      "telemetry_queue_max_depth,telemetry_queue_overflow_count,"
      "sd_last_write_duration_ms,sd_max_write_duration_ms,"
      "imu_dma_backlog_bytes,gps_rx_queue_depth,"
      "imu_row_valid,imu_link_valid,imu_rx_age_us,"
      "imu_timestamp_error_us,imu_estimated_missing_count,"
      "imu_resync_count,imu_timeout_count,imu_recovery_count,"
      "nextion_last_rpm_sent,nextion_rpm_tx_count,"
      "nextion_last_tps_sent,nextion_tps_tx_count,"
      "nextion_last_gear_sent,nextion_gear_tx_count,"
      "nextion_last_water_sent,nextion_water_tx_count,"
      "nextion_last_speed_sent,nextion_speed_tx_count,"
      "nextion_tx_fail_count,nextion_tx_busy_count,imu_parse_batch_max,"
      "imu_history_count\r\n";

  g_sd_detect_state = SD_IsCardInserted();
  if (g_sd_detect_state == GPIO_PIN_RESET) {
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
    g_sd_bringup_status = SD_LED_ERR_FATFS;
    SD_RecordFault(SD_FAULT_NO_CARD, FR_NOT_READY, 0U);
    return FR_NOT_READY;
  }

  res = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);
  if (res != FR_OK) {
    g_sd_bringup_status = SD_LED_ERR_MOUNT;
    SD_RecordFault(SD_FAULT_MOUNT, res, 0U);
    return res;
  }

  res = SD_OpenNextTelemetryLogFile();
  if (res != FR_OK) {
    (void)f_mount(NULL, (TCHAR const *)SDPath, 0U);
    g_sd_bringup_status = SD_LED_ERR_OPEN_EMU;
    SD_RecordFault(SD_FAULT_OPEN_TELEMETRY, res, 0U);
    return res;
  }

  res = f_write(&g_emu_can_file, header, (UINT)(sizeof(header) - 1U),
                &bytes_done);
  if (res != FR_OK) {
    g_sd_bringup_status = SD_LED_ERR_WRITE_HEADER;
    SD_RecordFault(SD_FAULT_HEADER_WRITE, res, bytes_done);
    return res;
  }

  if (bytes_done != (sizeof(header) - 1U)) {
    g_sd_bringup_status = SD_LED_ERR_WRITE_HEADER;
    SD_RecordFault(SD_FAULT_HEADER_SHORT_WRITE, FR_DISK_ERR, bytes_done);
    return FR_DISK_ERR;
  }

  res = f_sync(&g_emu_can_file);
  if (res != FR_OK) {
    g_sd_bringup_status = SD_LED_ERR_SYNC;
    SD_RecordFault(SD_FAULT_HEADER_SYNC, res, 0U);
    return res;
  }

  g_sd_active_file = &g_emu_can_file;
  g_sd_log_buffer_len = 0U;
  g_sd_log_last_write_tick = HAL_GetTick();
  g_sd_log_last_sync_tick = g_sd_log_last_write_tick;
  g_sd_log_unsynced_count = 0U;
  g_telemetry_snapshot_seq = 0U;
  g_telemetry_missed_snapshot_count = 0U;
  g_telemetry_next_snapshot_us =
      Telemetry_GetTimestampUs() + TELEMETRY_SNAPSHOT_INTERVAL_US;
  memset(&g_telemetry_latest_can, 0, sizeof(g_telemetry_latest_can));
  memset(g_telemetry_latest_wheel, 0, sizeof(g_telemetry_latest_wheel));
  memset(g_telemetry_latest_adc_raw, 0, sizeof(g_telemetry_latest_adc_raw));
  g_telemetry_adc_seen = 0U;
  g_sd_bringup_status = 0U;
  return FR_OK;
}

FRESULT SD_OpenNextTelemetryLogFile(void) {
  FRESULT res;

  for (uint16_t i = 0U; i <= SD_LOG_FILE_MAX_INDEX; i++) {
    (void)snprintf(g_sd_log_path, sizeof(g_sd_log_path),
                   "0:/Telemetry_%03u.csv", i);
    res = f_open(&g_emu_can_file, g_sd_log_path, FA_CREATE_NEW | FA_WRITE);
    if (res == FR_OK) {
      return FR_OK;
    }

    if (res != FR_EXIST) {
      return res;
    }
  }

  return FR_EXIST;
}

FRESULT SD_TelemetryLoggerProcess(void) {
  FRESULT res = FR_OK;
  uint32_t now;
  Telemetry_Snapshot_t snapshot;

  Nextion_UARTPump();
  RaceNextionProcess();
  Nextion_UARTPump();

  IMU_ProcessReceiver();

  if (g_emu_can_rx_irq_enabled == 0U) {
    EMU_CANDrainRxFifo();
  }

  res = SD_TelemetryLoggerDrainInputs();
  if (res != FR_OK) {
    SD_RecordFault(SD_FAULT_TELEMETRY_DRAIN, res, 0U);
    return res;
  }

  GNSS_ProcessParser();

  uint8_t loop_count = 0U;
  while (loop_count < 32U) {
    Telemetry_IMULatest_t imu_at_snapshot;

    IMU_ProcessReceiver();
    if (Telemetry_SnapshotPeek(&snapshot) == 0U) {
      break;
    }

    if (((g_imu_latest.valid == 0U) ||
         (g_imu_latest.timestamp_us < snapshot.timestamp_us)) &&
        ((Telemetry_GetTimestampUs() - snapshot.timestamp_us) <
         IMU_DATA_TIMEOUT_US)) {
      break;
    }

    if (Telemetry_SnapshotPop(&snapshot) == 0U) {
      break;
    }
    if (IMU_HistoryGetAt(snapshot.timestamp_us, &imu_at_snapshot) != 0U) {
      snapshot.imu = imu_at_snapshot;
    }
    res = SD_TelemetryLoggerAppendSnapshot(&snapshot);
    if (res != FR_OK) {
      break;
    }
    Nextion_UARTPump();
    loop_count++;
  }

  Nextion_UARTPump();

  now = HAL_GetTick();
  if ((g_sd_log_buffer_len > 0U) &&
      ((now - g_sd_log_last_write_tick) >= SD_LOG_FLUSH_IDLE_MS)) {
    return SD_PulseLoggerFlush(0U);
  }

  if ((g_sd_log_unsynced_count > 0U) &&
      ((now - g_sd_log_last_sync_tick) >= SD_LOG_SYNC_INTERVAL_MS)) {
    return SD_PulseLoggerFlush(1U);
  }

  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin,
          ((g_emu_can_rx_overflow_count == 0U) &&
           (g_emu_can_hal_error_count == 0U) &&
           (g_wheel4_rx_overflow_count == 0U) &&
           (g_adc6_rx_overflow_count == 0U))
              ? GPIO_PIN_SET
              : GPIO_PIN_RESET);
  return FR_OK;
}

FRESULT SD_TelemetryLoggerDrainInputs(void) {
  EMU_CAN_Frame_t can_frame;
  Wheel4_LogEvent_t wheel_event;

  while (EMU_CANQueuePop(&can_frame) != 0U) {
    if ((can_frame.std_id >= EMU_CAN_BASE_ID) &&
        (can_frame.std_id < (EMU_CAN_BASE_ID + EMU_CAN_FRAME_COUNT))) {
      uint8_t can_index = (uint8_t)(can_frame.std_id - EMU_CAN_BASE_ID);
      g_telemetry_latest_can[can_index].valid = 1U;
      g_telemetry_latest_can[can_index].timestamp_us = can_frame.timestamp_us;
      g_telemetry_latest_can[can_index].std_id = can_frame.std_id;
      g_telemetry_latest_can[can_index].dlc = can_frame.dlc;
      memcpy(g_telemetry_latest_can[can_index].data, can_frame.data,
             sizeof(g_telemetry_latest_can[can_index].data));

      if ((can_frame.std_id == (EMU_CAN_BASE_ID + 1U)) &&
          (can_frame.dlc >= 8U)) {
        g_telemetry_latest_adc_raw[1] = g_emu_potentiometer4_raw;
      } else if ((can_frame.std_id == (EMU_CAN_BASE_ID + 6U)) &&
                 (can_frame.dlc >= 2U)) {
        g_telemetry_latest_adc_raw[2] = g_emu_potentiometer5_raw;
        if (can_frame.dlc >= 4U) {
          g_telemetry_latest_adc_raw[6] = g_emu_adc6_raw;
        }
      }
    }

    RaceLoggerUpdateLatestFromCAN(can_frame.std_id, can_frame.dlc,
                                  can_frame.data);

    if ((can_frame.std_id == NEXTION_RPM_CAN_ID) && (can_frame.dlc >= 2U)) {
      g_nextion_latest_rpm = (uint16_t)((uint16_t)can_frame.data[0] |
                                        ((uint16_t)can_frame.data[1] << 8));
      g_nextion_rpm_valid = 1U;
    }
  }

  while (Wheel4_QueuePop(&wheel_event) != 0U) {
    if (wheel_event.channel < WHEEL_CHANNEL_COUNT) {
      g_telemetry_latest_wheel[wheel_event.channel].valid = 1U;
      g_telemetry_latest_wheel[wheel_event.channel].timestamp_us =
          wheel_event.timestamp_us;
      g_telemetry_latest_wheel[wheel_event.channel].delta_us =
          wheel_event.delta_us;
      g_telemetry_latest_wheel[wheel_event.channel].speed_centi_kmh =
          wheel_event.speed_centi_kmh;
    }
  }

  g_telemetry_latest_adc_raw[0] = g_adc_dma_buffer[0];
  g_telemetry_latest_adc_raw[3] = g_adc_dma_buffer[1];
  g_telemetry_latest_adc_raw[4] = g_adc_dma_buffer[2];
  g_telemetry_latest_adc_raw[5] = g_adc_dma_buffer[3];
  g_telemetry_adc_seen = 1U;

  return FR_OK;
}

FRESULT SD_TelemetryLoggerAppendSnapshot(
    const Telemetry_Snapshot_t *snapshot) {
  FRESULT res;
  uint64_t timestamp_us = snapshot->timestamp_us;
  const Telemetry_CANLatest_t *snapshot_can = snapshot->can;
  const Telemetry_WheelLatest_t *snapshot_wheel = snapshot->wheel;
  static char line[1536];
  int line_len;
  size_t line_used = 0U;
  uint8_t can_valid[EMU_CAN_FRAME_COUNT] = {0};
  uint64_t can_age_us[EMU_CAN_FRAME_COUNT] = {0};
  uint32_t can_valid_mask = 0U;
  uint16_t adc_raw[ADC_CHANNEL_COUNT];
  uint64_t adc_sample_timestamp_us = 0ULL;
  uint64_t adc_age_us = 0ULL;
  uint32_t adc_sample_count = 0U;
  uint32_t wheel_speed_centi_kmh[WHEEL_CHANNEL_COUNT];
  uint32_t rear_pulse_count[2];
  uint32_t rear_delta_us[2];
  uint64_t rear_timestamp_us[2];
  uint64_t rear_age_us[2];
  int16_t imu_gyro_deci_dps[3] = {0};
  int16_t imu_accel_milli_g[3] = {0};
  int16_t imu_euler_centi_deg[3] = {0};
  uint16_t imu_battery_pct = 0U;
  uint64_t imu_age_us = 0ULL;
  uint64_t imu_sample_timestamp_us = 0ULL;
  uint64_t imu_last_rx_timestamp_us = 0ULL;
  uint64_t imu_rx_age_us = 0ULL;
  int32_t imu_timestamp_error_us = 0;
  uint32_t imu_estimated_missing_count = 0U;
  uint32_t imu_resync_count = 0U;
  uint32_t imu_timeout_count = 0U;
  uint32_t imu_recovery_count = 0U;
  uint8_t imu_row_valid = 0U;
  uint8_t imu_link_valid = 0U;
  uint64_t gps_fix_age_us = 0ULL;
  char logger_timestamp_text[21];
  char can_age_text[EMU_CAN_FRAME_COUNT][21];
  char adc_sample_timestamp_text[21];
  char adc_age_text[21];
  char gps_fix_age_text[21];
  char imu_sample_timestamp_text[21];
  uint32_t telemetry_queue_depth;
  uint32_t imu_dma_backlog_bytes;
  uint32_t gps_rx_queue_depth;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < 2U; i++) {
    rear_pulse_count[i] = snapshot->rear_pulse_count[i];
    rear_delta_us[i] = snapshot->rear_delta_us[i];
    rear_timestamp_us[i] = snapshot->rear_timestamp_us[i];
  }
  imu_last_rx_timestamp_us = g_imu_last_rx_timestamp_us;
  imu_timestamp_error_us = g_imu_timestamp_error_us;
  imu_estimated_missing_count = g_imu_estimated_missing_count;
  imu_resync_count = g_imu_resync_count;
  imu_timeout_count = g_imu_timeout_count;
  imu_recovery_count = g_imu_recovery_count;
  imu_link_valid = g_imu_link_valid;

  if (snapshot->imu.valid != 0U) {
    if (timestamp_us >= snapshot->imu.timestamp_us) {
      imu_age_us = timestamp_us - snapshot->imu.timestamp_us;
      if (imu_age_us <= IMU_DATA_TIMEOUT_US) {
        imu_row_valid = 1U;
      }
    } else if ((snapshot->imu.timestamp_us - timestamp_us) <=
               TELEMETRY_CAN_CATCHUP_TOLERANCE_US) {
      imu_row_valid = 1U;
    }

    if (imu_row_valid != 0U) {
      for (uint8_t axis = 0U; axis < 3U; axis++) {
        imu_euler_centi_deg[axis] = snapshot->imu.euler_centi_deg[axis];
        imu_gyro_deci_dps[axis] = snapshot->imu.gyro_deci_dps[axis];
        imu_accel_milli_g[axis] = snapshot->imu.accel_milli_g[axis];
      }
      imu_battery_pct = snapshot->imu.battery_pct;
      imu_sample_timestamp_us = snapshot->imu.timestamp_us;
    }
  }
  adc_sample_timestamp_us = snapshot->adc_timestamp_us;
  adc_sample_count = snapshot->adc_sample_count;
  telemetry_queue_depth =
      (uint32_t)((g_telemetry_snapshot_head + TELEMETRY_SNAPSHOT_QUEUE_LEN -
                  g_telemetry_snapshot_tail) %
                 TELEMETRY_SNAPSHOT_QUEUE_LEN);
  gps_rx_queue_depth =
      (uint32_t)((g_gnss_rx_head + GNSS_RX_QUEUE_LEN - g_gnss_rx_tail) %
                 GNSS_RX_QUEUE_LEN);
  {
    uint16_t imu_write_pos = (uint16_t)(IMU_RX_DMA_BUFFER_SIZE -
        __HAL_DMA_GET_COUNTER(huart6.hdmarx));
    if (imu_write_pos >= IMU_RX_DMA_BUFFER_SIZE) {
      imu_write_pos = 0U;
    }
    imu_dma_backlog_bytes =
        (uint32_t)((imu_write_pos + IMU_RX_DMA_BUFFER_SIZE -
                    g_imu_rx_read_pos) % IMU_RX_DMA_BUFFER_SIZE);
  }
  if (primask == 0U) {
    __enable_irq();
  }

  {
    uint64_t diagnostic_now_us = Telemetry_GetTimestampUs();
    if ((imu_last_rx_timestamp_us != 0ULL) &&
        (diagnostic_now_us >= imu_last_rx_timestamp_us)) {
      imu_rx_age_us = diagnostic_now_us - imu_last_rx_timestamp_us;
    }
  }

  for (uint8_t i = 0U; i < ADC_CHANNEL_COUNT; i++) {
    adc_raw[i] = snapshot->adc_raw[i];
  }
  if ((adc_sample_timestamp_us != 0ULL) &&
      (timestamp_us >= adc_sample_timestamp_us)) {
    adc_age_us = timestamp_us - adc_sample_timestamp_us;
  }

  if ((g_gnss_last_fix_timestamp_us != 0ULL) &&
      (timestamp_us >= g_gnss_last_fix_timestamp_us)) {
    gps_fix_age_us = timestamp_us - g_gnss_last_fix_timestamp_us;
  }

  for (uint8_t i = 0U; i < EMU_CAN_FRAME_COUNT; i++) {
    if (snapshot_can[i].valid != 0U) {
      uint64_t can_timestamp_us = snapshot_can[i].timestamp_us;

      if (timestamp_us >= can_timestamp_us) {
        uint64_t age_us = timestamp_us - can_timestamp_us;
        can_age_us[i] = age_us;
        if (age_us <= TELEMETRY_CAN_TIMEOUT_US) {
          can_valid[i] = 1U;
        }
      } else {

        uint64_t lookahead_us = can_timestamp_us - timestamp_us;
        if (lookahead_us <= TELEMETRY_CAN_CATCHUP_TOLERANCE_US) {
          can_valid[i] = 1U;
          can_age_us[i] = 0ULL;
        }
      }
      if (can_valid[i] != 0U) {
        can_valid_mask |= (uint32_t)(1UL << i);
      }
    }
  }

  for (uint8_t i = 0U; i < WHEEL_CHANNEL_COUNT; i++) {
    wheel_speed_centi_kmh[i] = 0U;

    if (snapshot_wheel[i].valid != 0U) {
      uint64_t wheel_timestamp_us =
          snapshot_wheel[i].timestamp_us;
      uint8_t wheel_valid = 0U;

      if (timestamp_us >= wheel_timestamp_us) {
        uint64_t age_us = timestamp_us - wheel_timestamp_us;
        if (age_us <= TELEMETRY_WHEEL_TIMEOUT_US) {
          wheel_valid = 1U;
        }
      } else {

        uint64_t lookahead_us = wheel_timestamp_us - timestamp_us;
        if (lookahead_us <= TELEMETRY_WHEEL_CATCHUP_TOLERANCE_US) {
          wheel_valid = 1U;
        }
      }

      if (wheel_valid != 0U) {
        wheel_speed_centi_kmh[i] =
            snapshot_wheel[i].speed_centi_kmh;
      }
    }
  }

  for (uint8_t i = 0U; i < 2U; i++) {
    rear_age_us[i] =
        ((rear_timestamp_us[i] != 0ULL) &&
         (timestamp_us >= rear_timestamp_us[i]))
            ? (timestamp_us - rear_timestamp_us[i])
            : 0ULL;
  }

  Telemetry_FormatU64(timestamp_us, logger_timestamp_text,
                      sizeof(logger_timestamp_text));
  for (uint8_t i = 0U; i < EMU_CAN_FRAME_COUNT; i++) {
    Telemetry_FormatU64(can_age_us[i], can_age_text[i],
                        sizeof(can_age_text[i]));
  }
  Telemetry_FormatU64(adc_sample_timestamp_us, adc_sample_timestamp_text,
                      sizeof(adc_sample_timestamp_text));
  Telemetry_FormatU64(adc_age_us, adc_age_text, sizeof(adc_age_text));
  Telemetry_FormatU64(gps_fix_age_us, gps_fix_age_text,
                      sizeof(gps_fix_age_text));
  Telemetry_FormatU64(imu_sample_timestamp_us, imu_sample_timestamp_text,
                      sizeof(imu_sample_timestamp_text));

  line_len = snprintf(
      line, sizeof(line), "%s,%lu.%02lu,%lu,%u,%u,%u,%u,%u,%u,%u,",
      (const char *)g_gnss_parsed.time,
      (unsigned long)(timestamp_us / 1000000ULL),
      (unsigned long)((timestamp_us % 1000000ULL) / 10000ULL),
      (unsigned long)g_telemetry_snapshot_seq, (unsigned int)adc_raw[0],
      (unsigned int)adc_raw[1], (unsigned int)adc_raw[2],
      (unsigned int)adc_raw[3], (unsigned int)adc_raw[4],
      (unsigned int)adc_raw[5], (unsigned int)adc_raw[6]);
  if ((line_len <= 0) || ((size_t)line_len >= sizeof(line))) {
    SD_RecordFault(SD_FAULT_SNAPSHOT_FORMAT, FR_INT_ERR,
                   (uint32_t)sizeof(line));
    return FR_INT_ERR;
  }
  line_used = (size_t)line_len;

  for (uint8_t i = 0U; i < EMU_CAN_FRAME_COUNT; i++) {
    const Telemetry_CANLatest_t *can = &snapshot_can[i];
    line_len = snprintf(
        &line[line_used], sizeof(line) - line_used,
        "%02X%02X%02X%02X%02X%02X%02X%02X,",
        (unsigned int)(can_valid[i] ? can->data[0] : 0U),
        (unsigned int)(can_valid[i] ? can->data[1] : 0U),
        (unsigned int)(can_valid[i] ? can->data[2] : 0U),
        (unsigned int)(can_valid[i] ? can->data[3] : 0U),
        (unsigned int)(can_valid[i] ? can->data[4] : 0U),
        (unsigned int)(can_valid[i] ? can->data[5] : 0U),
        (unsigned int)(can_valid[i] ? can->data[6] : 0U),
        (unsigned int)(can_valid[i] ? can->data[7] : 0U));
    if ((line_len <= 0) || ((size_t)line_len >= (sizeof(line) - line_used))) {
      SD_RecordFault(SD_FAULT_SNAPSHOT_FORMAT, FR_INT_ERR,
                     (uint32_t)sizeof(line));
      return FR_INT_ERR;
    }
    line_used += (size_t)line_len;
  }

  line_len = snprintf(
      &line[line_used], sizeof(line) - line_used,
      "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"
      "%d,%d,%d,%d,%d,%d,%u,%lu,"
      "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"
      "%lu,%lu,%lu,%lu,"
      "%s,%s,%s,%s,%s,%s",
      (unsigned long)wheel_speed_centi_kmh[0],
      (unsigned long)wheel_speed_centi_kmh[1],
      (unsigned long)wheel_speed_centi_kmh[2],
      (unsigned long)wheel_speed_centi_kmh[3],
      (unsigned long)rear_pulse_count[0],
      (unsigned long)rear_pulse_count[1],
      (unsigned long)rear_delta_us[0], (unsigned long)rear_delta_us[1],
      (unsigned long)rear_age_us[0], (unsigned long)rear_age_us[1],
      (int)imu_gyro_deci_dps[0], (int)imu_gyro_deci_dps[1],
      (int)imu_gyro_deci_dps[2], (int)imu_accel_milli_g[0],
      (int)imu_accel_milli_g[1], (int)imu_accel_milli_g[2],
      (unsigned int)imu_battery_pct, (unsigned long)imu_age_us,
      (unsigned long)g_imu_rx_byte_count, (unsigned long)g_imu_packet_count,
      (unsigned long)g_imu_checksum_error_count,
      (unsigned long)g_imu_rx_error_count,
      (unsigned long)g_imu_uart_parity_error_count,
      (unsigned long)g_imu_uart_noise_error_count,
      (unsigned long)g_imu_uart_framing_error_count,
      (unsigned long)g_imu_uart_overrun_error_count,
      (unsigned long)g_imu_uart_dma_error_count,
      (unsigned long)g_gnss_rx_byte_count, (unsigned long)g_gnss_0xB5_count,
      (unsigned long)g_gnss_dollar_count,
      (unsigned long)g_gnss_rx_overflow_count, (const char *)g_gnss_parsed.lat,
      (const char *)g_gnss_parsed.lon, (const char *)g_gnss_parsed.speed,
      (const char *)g_gnss_parsed.sat_count,
      (const char *)g_gnss_parsed.fix_qual,
      (const char *)g_gnss_parsed.heading);

  if ((line_len <= 0) || ((size_t)line_len >= (sizeof(line) - line_used))) {
    SD_RecordFault(SD_FAULT_SNAPSHOT_FORMAT, FR_INT_ERR,
                   (uint32_t)sizeof(line));
    return FR_INT_ERR;
  }
  line_used += (size_t)line_len;

  line_len = snprintf(
      &line[line_used], sizeof(line) - line_used,
      ",%s,%s,%s,%s,%s,%s,%s,%s,%s,"
      "%lu,%lu,%lu,%lu,%s,%s,%lu,%s,%lu,%lu,"
      "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"
      "%d,%d,%d,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"
      "%lu,%lu,%lu,%ld,%lu,%lu,%lu,%lu,"
      "%lu,%lu,%lu,%lu,%lu,%lu,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
      logger_timestamp_text, can_age_text[0], can_age_text[1],
      can_age_text[2], can_age_text[3], can_age_text[4], can_age_text[5],
      can_age_text[6], can_age_text[7], (unsigned long)can_valid_mask,
      (unsigned long)g_emu_can_rx_count,
      (unsigned long)g_emu_can_rx_overflow_count,
      (unsigned long)g_emu_can_hal_error_count,
      adc_sample_timestamp_text, adc_age_text, (unsigned long)adc_sample_count,
      gps_fix_age_text,
      (unsigned long)g_gnss_fix_update_count,
      (unsigned long)g_gnss_uart_error_count,
      (unsigned long)g_sd_write_call_count,
      (unsigned long)g_sd_write_fail_count,
      (unsigned long)g_sd_sync_ok_count,
      (unsigned long)g_sd_sync_fail_count,
      (unsigned long)g_sd_ll_retry_count,
      (unsigned long)g_sd_ll_write_fail_count,
      (unsigned long)g_sd_ll_ready_timeout_count,
      (unsigned long)g_sd_max_buffer_len,
      (unsigned long)g_sd_log_unsynced_count,
      (unsigned long)g_telemetry_missed_snapshot_count,
      (unsigned long)g_wheel4_rx_overflow_count,
      (int)imu_euler_centi_deg[0], (int)imu_euler_centi_deg[1],
      (int)imu_euler_centi_deg[2],
      imu_sample_timestamp_text, (unsigned long)telemetry_queue_depth,
      (unsigned long)g_telemetry_snapshot_max_depth,
      (unsigned long)g_telemetry_snapshot_overflow_count,
      (unsigned long)g_sd_last_write_duration_ms,
      (unsigned long)g_sd_max_write_duration_ms,
      (unsigned long)imu_dma_backlog_bytes,
      (unsigned long)gps_rx_queue_depth,
      (unsigned long)imu_row_valid,
      (unsigned long)imu_link_valid,
      (unsigned long)imu_rx_age_us,
      (long)imu_timestamp_error_us,
      (unsigned long)imu_estimated_missing_count,
      (unsigned long)imu_resync_count,
      (unsigned long)imu_timeout_count,
      (unsigned long)imu_recovery_count,
      (unsigned long)g_race_nextion_last_rpm_sent,
      (unsigned long)g_race_nextion_rpm_tx_count,
      (unsigned long)g_race_nextion_last_tps_sent,
      (unsigned long)g_race_nextion_tps_tx_count,
      (unsigned long)g_race_nextion_last_gear_sent,
      (unsigned long)g_race_nextion_gear_tx_count,
      (int)g_race_nextion_last_water_sent,
      (unsigned long)g_race_nextion_water_tx_count,
      (unsigned long)g_race_nextion_last_speed_sent,
      (unsigned long)g_race_nextion_speed_tx_count,
      (unsigned long)g_race_nextion_tx_fail_count,
      (unsigned long)g_race_nextion_tx_busy_count,
      (unsigned long)g_imu_parse_batch_max,
      (unsigned long)g_imu_history_count);

  if ((line_len <= 0) || ((size_t)line_len >= (sizeof(line) - line_used))) {
    SD_RecordFault(SD_FAULT_SNAPSHOT_FORMAT, FR_INT_ERR,
                   (uint32_t)sizeof(line));
    return FR_INT_ERR;
  }
  line_used += (size_t)line_len;
  line_len = (int)line_used;

#if (TELEMETRY_UART_OUTPUT_ENABLE != 0U)

  (void)HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)line_len, 50U);
#endif

  if (((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len)) &&
      (g_sd_log_buffer_len > 0U)) {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK) {
      return res;
    }
  }

  if ((UINT)line_len > (SD_LOG_BUFFER_SIZE - g_sd_log_buffer_len)) {
    SD_RecordFault(SD_FAULT_SNAPSHOT_BUFFER_FULL, FR_INT_ERR,
                   (uint32_t)line_len);
    return FR_INT_ERR;
  }

  memcpy(&g_sd_log_buffer[g_sd_log_buffer_len], line, (size_t)line_len);
  g_sd_log_buffer_len += (UINT)line_len;
  if (g_sd_log_buffer_len > g_sd_max_buffer_len) {
    g_sd_max_buffer_len = g_sd_log_buffer_len;
  }
  g_telemetry_snapshot_seq++;
  g_telemetry_log_count++;
  g_sd_log_unsynced_count++;
  HAL_GPIO_TogglePin(USER_LED_0_GPIO_Port, USER_LED_0_Pin);

#if (SD_LOG_DIAG_FORCE_SYNC != 0U)
  return SD_PulseLoggerFlush(1U);
#endif

  if (g_sd_log_buffer_len >= (SD_LOG_BUFFER_SIZE - (UINT)sizeof(line))) {
    res = SD_PulseLoggerFlush(0U);
    if (res != FR_OK) {
      return res;
    }
    return res;
  }

  return FR_OK;
}

uint8_t BSP_SD_Init(void) {
  g_sd_hal_stage = 0U;
  g_sd_hal_error_code = HAL_SD_ERROR_NONE;

  if (HAL_SD_Init(&hsd) != HAL_OK) {
    g_sd_hal_stage = SD_LED_ERR_HAL_INIT;
    g_sd_hal_error_code = hsd.ErrorCode;
    return MSD_ERROR;
  }

  if ((SD_BRINGUP_USE_4BIT_BUS != 0U) &&
      (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)) {
    g_sd_hal_stage = SD_LED_ERR_WIDE_BUS;
    g_sd_hal_error_code = hsd.ErrorCode;
    return MSD_ERROR;
  }

  return MSD_OK;
}

uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr,
                          uint32_t NumOfBlocks, uint32_t Timeout) {
  HAL_StatusTypeDef hal_status = HAL_ERROR;

  g_sd_ll_last_op = SD_LOWLEVEL_OP_READ;
  g_sd_ll_read_call_count++;
  g_sd_ll_last_sector = ReadAddr;
  g_sd_ll_last_blocks = NumOfBlocks;
  g_sd_ll_last_buffer_addr = (uint32_t)pData;
  g_sd_ll_last_tick = HAL_GetTick();

  if ((pData == NULL) || (NumOfBlocks == 0U) ||
      (SD_IsCardInserted() == GPIO_PIN_RESET)) {
    g_sd_ll_read_fail_count++;
    g_sd_ll_last_hal_status = HAL_ERROR;
    g_sd_ll_last_hal_error = hsd.ErrorCode;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
    g_sd_hal_error_code = hsd.ErrorCode;
    return MSD_ERROR;
  }

  for (uint8_t attempt = 0U; attempt < SD_LOWLEVEL_IO_RETRY_COUNT; attempt++) {
    if (attempt != 0U) {
      g_sd_ll_retry_count++;
      HAL_Delay(SD_LOWLEVEL_IO_RETRY_DELAY_MS);
    }

    if (SD_WaitForTransferReady(SD_LOWLEVEL_READY_TIMEOUT_MS) != MSD_OK) {
      hal_status = HAL_TIMEOUT;
      (void)HAL_SD_Abort(&hsd);
      continue;
    }

    hal_status = HAL_SD_ReadBlocks(&hsd, (uint8_t *)pData, ReadAddr,
                                   NumOfBlocks, Timeout);
    g_sd_ll_last_hal_status = hal_status;
    g_sd_ll_last_hal_error = hsd.ErrorCode;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);

    if (hal_status == HAL_OK) {
      if (SD_WaitForTransferReady(SD_LOWLEVEL_READY_TIMEOUT_MS) == MSD_OK) {
        g_sd_ll_last_hal_status = HAL_OK;
        g_sd_ll_last_hal_error = hsd.ErrorCode;
        g_sd_hal_error_code = hsd.ErrorCode;
        return MSD_OK;
      }

      hal_status = HAL_TIMEOUT;
      g_sd_ll_last_hal_status = hal_status;
    }

    g_sd_hal_error_code = hsd.ErrorCode;
    (void)HAL_SD_Abort(&hsd);
  }

  g_sd_ll_read_fail_count++;
  g_sd_ll_last_hal_status = hal_status;
  g_sd_ll_last_hal_error = hsd.ErrorCode;
  g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
  g_sd_hal_error_code = hsd.ErrorCode;
  return MSD_ERROR;
}

uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr,
                           uint32_t NumOfBlocks, uint32_t Timeout) {
  HAL_StatusTypeDef hal_status = HAL_ERROR;

  g_sd_ll_last_op = SD_LOWLEVEL_OP_WRITE;
  g_sd_ll_write_call_count++;
  g_sd_ll_last_sector = WriteAddr;
  g_sd_ll_last_blocks = NumOfBlocks;
  g_sd_ll_last_buffer_addr = (uint32_t)pData;
  g_sd_ll_last_tick = HAL_GetTick();

  if ((pData == NULL) || (NumOfBlocks == 0U) ||
      (SD_IsCardInserted() == GPIO_PIN_RESET)) {
    g_sd_ll_write_fail_count++;
    g_sd_ll_last_hal_status = HAL_ERROR;
    g_sd_ll_last_hal_error = hsd.ErrorCode;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
    g_sd_hal_stage = SD_LED_ERR_WRITE_LOG;
    g_sd_hal_error_code = hsd.ErrorCode;
    return MSD_ERROR;
  }

  for (uint8_t attempt = 0U; attempt < SD_LOWLEVEL_IO_RETRY_COUNT; attempt++) {
    if (attempt != 0U) {
      g_sd_ll_retry_count++;
      HAL_Delay(SD_LOWLEVEL_IO_RETRY_DELAY_MS);
    }

    if (SD_WaitForTransferReady(SD_LOWLEVEL_READY_TIMEOUT_MS) != MSD_OK) {
      hal_status = HAL_TIMEOUT;
      (void)HAL_SD_Abort(&hsd);
      continue;
    }

    hal_status = HAL_SD_WriteBlocks(&hsd, (uint8_t *)pData, WriteAddr,
                                    NumOfBlocks, Timeout);
    g_sd_ll_last_hal_status = hal_status;
    g_sd_ll_last_hal_error = hsd.ErrorCode;
    g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);

    if (hal_status == HAL_OK) {
      if (SD_WaitForTransferReady(SD_LOWLEVEL_READY_TIMEOUT_MS) == MSD_OK) {
        g_sd_ll_last_hal_status = HAL_OK;
        g_sd_ll_last_hal_error = hsd.ErrorCode;
        g_sd_hal_error_code = hsd.ErrorCode;
        return MSD_OK;
      }

      hal_status = HAL_TIMEOUT;
      g_sd_ll_last_hal_status = hal_status;
    }

    g_sd_hal_stage = SD_LED_ERR_WRITE_LOG;
    g_sd_hal_error_code = hsd.ErrorCode;
    (void)HAL_SD_Abort(&hsd);
  }

  g_sd_ll_write_fail_count++;
  g_sd_ll_last_hal_status = hal_status;
  g_sd_ll_last_hal_error = hsd.ErrorCode;
  g_sd_ll_last_hal_state = HAL_SD_GetState(&hsd);
  g_sd_hal_stage = SD_LED_ERR_WRITE_LOG;
  g_sd_hal_error_code = hsd.ErrorCode;
  return MSD_ERROR;
}

GPIO_PinState SD_IsCardInserted(void) {
  return (HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin) ==
          GPIO_PIN_RESET)
             ? GPIO_PIN_SET
             : GPIO_PIN_RESET;
}

FRESULT SD_LogWriterFlush(uint8_t force_sync,
                                 SD_LogFlushReason_t reason) {
  FRESULT res = FR_OK;
  UINT bytes_done = 0U;
  UINT bytes_to_write;

  g_sd_last_flush_reason = reason;

  if (g_sd_log_buffer_len > 0U) {
    uint32_t write_start_tick = HAL_GetTick();
    if (g_sd_active_file == NULL) {
      SD_RecordFault(SD_FAULT_FLUSH_NO_ACTIVE_FILE, FR_INVALID_OBJECT, 0U);
      return FR_INVALID_OBJECT;
    }

    bytes_to_write = g_sd_log_buffer_len;
    for (uint8_t attempt = 0U; attempt < SD_LOG_IO_RETRY_COUNT; attempt++) {
      bytes_done = 0U;
      res = f_write(g_sd_active_file, g_sd_log_buffer, bytes_to_write,
                    &bytes_done);
      if ((res == FR_OK) && (bytes_done == bytes_to_write)) {
        break;
      }

      if (bytes_done != 0U) {
        break;
      }

      SD_RecoverAfterIoError();
      HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
    }

    if (res != FR_OK) {
      g_sd_write_fail_count++;
      SD_RecordFault(SD_FAULT_FLUSH_WRITE, res, bytes_done);
      return res;
    }

    if (bytes_done != bytes_to_write) {
      g_sd_write_fail_count++;
      SD_RecordFault(SD_FAULT_FLUSH_SHORT_WRITE, FR_DISK_ERR, bytes_done);
      return FR_DISK_ERR;
    }

    g_sd_last_write_duration_ms = HAL_GetTick() - write_start_tick;
    if (g_sd_last_write_duration_ms > g_sd_max_write_duration_ms) {
      g_sd_max_write_duration_ms = g_sd_last_write_duration_ms;
    }

    g_sd_write_call_count++;
    g_sd_last_write_size = bytes_done;
    g_sd_total_bytes_written += bytes_done;
    g_sd_log_buffer_len = 0U;
    g_sd_log_last_write_tick = HAL_GetTick();
  }

  if ((force_sync != 0U) && (g_sd_log_unsynced_count > 0U)) {
    if (g_sd_active_file == NULL) {
      SD_RecordFault(SD_FAULT_FLUSH_NO_ACTIVE_FILE, FR_INVALID_OBJECT, 1U);
      return FR_INVALID_OBJECT;
    }

    for (uint8_t attempt = 0U; attempt < SD_LOG_IO_RETRY_COUNT; attempt++) {
      res = f_sync(g_sd_active_file);
      if (res == FR_OK) {
        break;
      }

      HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
    }

    if (res == FR_OK) {
      g_sd_log_unsynced_count = 0U;
      g_sd_log_last_sync_tick = HAL_GetTick();
      g_sd_last_sync_fresult = FR_OK;
      g_sd_sync_ok_count++;
    } else {
      g_sd_sync_fail_count++;
      g_sd_last_sync_fresult = res;
#if (SD_LOG_SYNC_FAIL_FATAL != 0U)
      SD_RecordFault(SD_FAULT_FLUSH_SYNC, res, g_sd_log_unsynced_count);
#else
      g_sd_fault_stage = SD_FAULT_FLUSH_SYNC;
      g_sd_fault_fresult = res;
      g_sd_fault_detail = g_sd_log_unsynced_count;
      g_sd_fault_tick = HAL_GetTick();
      g_sd_fault_buffer_len = g_sd_log_buffer_len;
      g_sd_fault_unsynced_count = g_sd_log_unsynced_count;
      g_sd_fault_telemetry_count = g_telemetry_log_count;
      g_sd_log_last_sync_tick = HAL_GetTick();
      res = FR_OK;
#endif
    }
  }

  return res;
}

FRESULT SD_PulseLoggerFlush(uint8_t force_sync) {
  return SD_LogWriterFlush(force_sync, (force_sync != 0U)
                                           ? SD_LOG_FLUSH_REASON_SYNC
                                           : SD_LOG_FLUSH_REASON_LEGACY);
}
