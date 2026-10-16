#include "app_internal.h"

void RaceLoggerUpdateLatestFromCAN(uint32_t std_id, uint8_t dlc,
                                          const uint8_t *data) {

  g_race_raw_can_seen_count++;
  g_race_can_seen_count++;
  g_race_last_can_rx_tick = HAL_GetTick();
  g_race_last_raw_can_rx_tick = g_race_last_can_rx_tick;

  if ((std_id == (EMU_CAN_BASE_ID + 0U)) && (dlc >= 8U)) {
    uint16_t rpm = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    uint16_t tps_x10 = (uint16_t)data[2] * 5U;

    if ((g_race_can_latest.rpm_valid == 0U) || (g_race_can_latest.rpm != rpm)) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_RPM;
    }
    if ((g_race_can_latest.tps_valid == 0U) ||
        (g_race_can_latest.tps_x10 != tps_x10)) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_TPS;
    }
    g_race_can_latest.rpm = rpm;
    g_race_can_latest.tps_x10 = tps_x10;
    g_race_can_latest.rpm_valid = 1U;
    g_race_can_latest.tps_valid = 1U;
    RPMOutputs_SetRPM(rpm);
  } else if ((std_id == (EMU_CAN_BASE_ID + 2U)) && (dlc >= 8U)) {
    uint16_t speed_kmh =
        (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    int16_t oil_c = (int16_t)data[3];
    int16_t water_c = (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));

    if ((g_race_can_latest.water_valid == 0U) ||
        (g_race_can_latest.water_c != water_c)) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_WATER;
    }
    if ((g_race_can_latest.speed_valid == 0U) ||
        (g_race_can_latest.speed_kmh != speed_kmh)) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_SPEED;
    }
    if ((g_race_can_latest.oil_valid == 0U) ||
        (g_race_can_latest.oil_c != oil_c)) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_OIL;
    }
    g_race_can_latest.speed_kmh = speed_kmh;
    g_race_can_latest.oil_c = oil_c;
    g_race_can_latest.water_c = water_c;
    g_race_can_latest.speed_valid = 1U;
    g_race_can_latest.oil_valid = 1U;
    g_race_can_latest.water_valid = 1U;
  } else if ((std_id == (EMU_CAN_BASE_ID + 4U)) && (dlc >= 8U)) {
    uint8_t gear = data[0];
    uint32_t battery_mV =
        (uint32_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8)) * 27U;

    if ((g_race_can_latest.gear_valid == 0U) ||
        (g_race_can_latest.gear != gear)) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_GEAR;
    }
    if ((g_race_can_latest.battery_valid == 0U) ||
        (g_race_can_latest.battery_mV != battery_mV)) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_BATTERY;
    }

    g_race_can_latest.gear = gear;
    g_race_can_latest.battery_mV = battery_mV;
    g_race_can_latest.gear_valid = 1U;
    g_race_can_latest.battery_valid = 1U;
  } else if ((std_id == (EMU_CAN_BASE_ID + 7U)) && (dlc >= 8U)) {
    uint16_t fuel_used_x100 =
        (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));

    if ((g_race_can_latest.fuel_valid == 0U) ||
        (g_race_can_latest.fuel_used_x100 != fuel_used_x100)) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_FUEL;
    }
    g_race_can_latest.fuel_used_x100 = fuel_used_x100;
    g_race_can_latest.fuel_valid = 1U;
  }
}

void RaceNextionProcess(void) {
  uint32_t now = HAL_GetTick();
  RaceCANLatest_t latest;
  uint8_t can_signal_lost;
  uint8_t logger_ok;
  uint8_t too_hot;
  uint8_t dirty_mask;
  uint8_t sent = 0U;

  Nextion_UARTPump();

  if ((int32_t)(now - g_race_nextion_next_tx_tick) < 0) {
    return;
  }

  if (g_nextion_logok_visibility == NEXTION_VIS_UNKNOWN) {
    if (Nextion_SendVisibility("t_logok", 0U) != 0U) {
      g_nextion_logok_visibility = NEXTION_VIS_HIDDEN;
      sent = 1U;
    }
    goto transmit_done;
  }
  if (g_nextion_logoff_visibility == NEXTION_VIS_UNKNOWN) {
    if (Nextion_SendVisibility("t_logoff", 0U) != 0U) {
      g_nextion_logoff_visibility = NEXTION_VIS_HIDDEN;
      sent = 1U;
    }
    goto transmit_done;
  }
  if (g_nextion_toohot_visibility == NEXTION_VIS_UNKNOWN) {
    if (Nextion_SendVisibility("t_toohot", 0U) != 0U) {
      g_nextion_toohot_visibility = NEXTION_VIS_HIDDEN;
      sent = 1U;
    }
    goto transmit_done;
  }

  logger_ok = ((g_sd_active_file != NULL) &&
               ((now - g_sd_log_last_write_tick) < RACE_LOG_WRITE_TIMEOUT_MS))
                  ? 1U
                  : 0U;

  if (logger_ok != 0U) {
    if (g_nextion_logoff_visibility != NEXTION_VIS_HIDDEN) {
      if (Nextion_SendVisibility("t_logoff", 0U) != 0U) {
        g_nextion_logoff_visibility = NEXTION_VIS_HIDDEN;
        sent = 1U;
      }
      goto transmit_done;
    }
    if (g_nextion_logok_visibility != NEXTION_VIS_VISIBLE) {
      if (Nextion_SendVisibility("t_logok", 1U) != 0U) {
        g_nextion_logok_visibility = NEXTION_VIS_VISIBLE;
        sent = 1U;
      }
      goto transmit_done;
    }
  } else {
    if (g_nextion_logok_visibility != NEXTION_VIS_HIDDEN) {
      if (Nextion_SendVisibility("t_logok", 0U) != 0U) {
        g_nextion_logok_visibility = NEXTION_VIS_HIDDEN;
        sent = 1U;
      }
      goto transmit_done;
    }
    if (g_nextion_logoff_visibility != NEXTION_VIS_VISIBLE) {
      if (Nextion_SendVisibility("t_logoff", 1U) != 0U) {
        g_nextion_logoff_visibility = NEXTION_VIS_VISIBLE;
        sent = 1U;
      }
      goto transmit_done;
    }
  }

  too_hot = ((g_race_can_latest.water_valid != 0U) &&
             (g_race_can_latest.water_c >= RACE_CLT_TOO_HOT_C))
                ? 1U
                : 0U;
  if ((too_hot != 0U) &&
      (g_nextion_toohot_visibility != NEXTION_VIS_VISIBLE)) {
    if (Nextion_SendVisibility("t_toohot", 1U) != 0U) {
      g_nextion_toohot_visibility = NEXTION_VIS_VISIBLE;
      sent = 1U;
    }
    goto transmit_done;
  }
  if ((too_hot == 0U) &&
      (g_nextion_toohot_visibility != NEXTION_VIS_HIDDEN)) {
    if (Nextion_SendVisibility("t_toohot", 0U) != 0U) {
      g_nextion_toohot_visibility = NEXTION_VIS_HIDDEN;
      sent = 1U;
    }
    goto transmit_done;
  }

  can_signal_lost =
      ((g_race_can_seen_count == 0U) ||
       ((now - g_race_last_can_rx_tick) >= RACE_CAN_SIGNAL_TIMEOUT_MS))
          ? 1U
          : 0U;

  if (can_signal_lost != 0U) {
    g_race_can_signal_lost = 1U;
    if ((now - g_race_nextion_last_no_sig_tick) >=
        RACE_NEXTION_NO_SIG_REPEAT_MS) {
      if (Nextion_SendTextCommand("t_rpm", "NO SIG") != 0U) {
        g_race_nextion_last_no_sig_tick = now;
        g_race_nextion_last_rpm_tick = now;
        sent = 1U;
      }
    }

    goto transmit_done;
  }

  if (g_race_can_signal_lost != 0U) {
    __disable_irq();
    g_race_can_signal_lost = 0U;
    if (g_race_can_latest.rpm_valid != 0U) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_RPM;
    }
    if (g_race_can_latest.gear_valid != 0U) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_GEAR;
    }
    if (g_race_can_latest.water_valid != 0U) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_WATER;
    }
    if (g_race_can_latest.tps_valid != 0U) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_TPS;
    }
    if (g_race_can_latest.speed_valid != 0U) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_SPEED;
    }
    if (g_race_can_latest.oil_valid != 0U) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_OIL;
    }
    if (g_race_can_latest.battery_valid != 0U) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_BATTERY;
    }
    if (g_race_can_latest.fuel_valid != 0U) {
      g_race_nextion_dirty_mask |= RACE_NEXTION_DIRTY_FUEL;
    }
    __enable_irq();
  }

  RaceNextionGetSnapshot(&latest, &dirty_mask);

  if (((dirty_mask & RACE_NEXTION_DIRTY_GEAR) != 0U) &&
      (latest.gear_valid != 0U)) {
    if (RaceNextionSendGear(latest.gear) != 0U) {
      g_race_nextion_last_gear_sent = latest.gear;
      g_race_nextion_gear_tx_count++;
      g_race_nextion_last_gear_tick = now;
      RaceNextionClearDirty(RACE_NEXTION_DIRTY_GEAR);
      sent = 1U;
    }
    goto transmit_done;
  }

  for (uint8_t attempt = 0U; attempt < RACE_NEXTION_FIELD_COUNT; attempt++) {
    uint8_t field = (uint8_t)((g_race_nextion_rr_index + attempt) %
                              RACE_NEXTION_FIELD_COUNT);
    uint8_t eligible = 0U;

    switch (field) {
    case RACE_NEXTION_FIELD_RPM:
      eligible = (latest.rpm_valid != 0U) &&
                 (((dirty_mask & RACE_NEXTION_DIRTY_RPM) != 0U) ||
                  ((now - g_race_nextion_last_rpm_tick) >=
                   RACE_NEXTION_RPM_UPDATE_MS));
      break;
    case RACE_NEXTION_FIELD_TPS:
      eligible = (latest.tps_valid != 0U) &&
                 (((dirty_mask & RACE_NEXTION_DIRTY_TPS) != 0U) ||
                  ((now - g_race_nextion_last_tps_tick) >=
                   RACE_NEXTION_TPS_UPDATE_MS));
      break;
    case RACE_NEXTION_FIELD_SPEED:
      eligible = (latest.speed_valid != 0U) &&
                 (((dirty_mask & RACE_NEXTION_DIRTY_SPEED) != 0U) ||
                  ((now - g_race_nextion_last_speed_tick) >=
                   RACE_NEXTION_SPEED_UPDATE_MS));
      break;
    case RACE_NEXTION_FIELD_WATER:
      eligible = (latest.water_valid != 0U) &&
                 (((dirty_mask & RACE_NEXTION_DIRTY_WATER) != 0U) ||
                  ((now - g_race_nextion_last_water_tick) >=
                   RACE_NEXTION_DATA_UPDATE_MS));
      break;
    case RACE_NEXTION_FIELD_GEAR:
      eligible = (latest.gear_valid != 0U) &&
                 ((now - g_race_nextion_last_gear_tick) >=
                  RACE_NEXTION_GEAR_UPDATE_MS);
      break;
    case RACE_NEXTION_FIELD_OIL:
      eligible = (latest.oil_valid != 0U) &&
                 (((dirty_mask & RACE_NEXTION_DIRTY_OIL) != 0U) ||
                  ((now - g_race_nextion_last_oil_tick) >=
                   RACE_NEXTION_DATA_UPDATE_MS));
      break;
    case RACE_NEXTION_FIELD_BATTERY:
      eligible = (latest.battery_valid != 0U) &&
                 (((dirty_mask & RACE_NEXTION_DIRTY_BATTERY) != 0U) ||
                  ((now - g_race_nextion_last_battery_tick) >=
                   RACE_NEXTION_DATA_UPDATE_MS));
      break;
    case RACE_NEXTION_FIELD_FUEL:
      eligible = (latest.fuel_valid != 0U) &&
                 (((dirty_mask & RACE_NEXTION_DIRTY_FUEL) != 0U) ||
                  ((now - g_race_nextion_last_fuel_tick) >=
                   RACE_NEXTION_DATA_UPDATE_MS));
      break;
    default:
      break;
    }

    if (eligible == 0U) {
      continue;
    }

    g_race_nextion_rr_index =
        (uint8_t)((field + 1U) % RACE_NEXTION_FIELD_COUNT);
    switch (field) {
    case RACE_NEXTION_FIELD_RPM:
      if (RaceNextionSendNumber("t_rpm", (int32_t)latest.rpm) != 0U) {
        g_race_nextion_last_rpm_sent = latest.rpm;
        g_race_nextion_rpm_tx_count++;
        g_race_nextion_last_rpm_tick = now;
        RaceNextionClearDirty(RACE_NEXTION_DIRTY_RPM);
        sent = 1U;
      }
      break;
    case RACE_NEXTION_FIELD_TPS: {
      char tps_text[16];
      (void)snprintf(tps_text, sizeof(tps_text), "%u.%u",
                     (unsigned int)(latest.tps_x10 / 10U),
                     (unsigned int)(latest.tps_x10 % 10U));
      if (Nextion_SendTextCommand("t_tps", tps_text) != 0U) {
        g_race_nextion_last_tps_sent = latest.tps_x10;
        g_race_nextion_tps_tx_count++;
        g_race_nextion_last_tps_tick = now;
        RaceNextionClearDirty(RACE_NEXTION_DIRTY_TPS);
        sent = 1U;
      }
      break;
    }
    case RACE_NEXTION_FIELD_SPEED:
      if (RaceNextionSendNumber("t_speed", (int32_t)latest.speed_kmh) != 0U) {
        g_race_nextion_last_speed_sent = latest.speed_kmh;
        g_race_nextion_speed_tx_count++;
        g_race_nextion_last_speed_tick = now;
        RaceNextionClearDirty(RACE_NEXTION_DIRTY_SPEED);
        sent = 1U;
      }
      break;
    case RACE_NEXTION_FIELD_WATER:
      if (RaceNextionSendNumber("t_clt", (int32_t)latest.water_c) != 0U) {
        g_race_nextion_last_water_sent = latest.water_c;
        g_race_nextion_water_tx_count++;
        g_race_nextion_last_water_tick = now;
        RaceNextionClearDirty(RACE_NEXTION_DIRTY_WATER);
        sent = 1U;
      }
      break;
    case RACE_NEXTION_FIELD_OIL:
      if (RaceNextionSendNumber("t_oil", (int32_t)latest.oil_c) != 0U) {
        g_race_nextion_last_oil_tick = now;
        RaceNextionClearDirty(RACE_NEXTION_DIRTY_OIL);
        sent = 1U;
      }
      break;
    case RACE_NEXTION_FIELD_BATTERY: {
      char battery_text[16];
      uint32_t battery_cV = (latest.battery_mV + 5U) / 10U;
      (void)snprintf(battery_text, sizeof(battery_text), "%lu.%02lu",
                     (unsigned long)(battery_cV / 100U),
                     (unsigned long)(battery_cV % 100U));
      if (Nextion_SendTextCommand("t_battery", battery_text) != 0U) {
        g_race_nextion_last_battery_tick = now;
        RaceNextionClearDirty(RACE_NEXTION_DIRTY_BATTERY);
        sent = 1U;
      }
      break;
    }
    case RACE_NEXTION_FIELD_FUEL: {
      char fuel_text[16];
      (void)snprintf(fuel_text, sizeof(fuel_text), "%u.%02u",
                     (unsigned int)(latest.fuel_used_x100 / 100U),
                     (unsigned int)(latest.fuel_used_x100 % 100U));
      if (Nextion_SendTextCommand("t_fuel", fuel_text) != 0U) {
        g_race_nextion_last_fuel_tick = now;
        RaceNextionClearDirty(RACE_NEXTION_DIRTY_FUEL);
        sent = 1U;
      }
      break;
    }
    case RACE_NEXTION_FIELD_GEAR:
      if (RaceNextionSendGear(latest.gear) != 0U) {
        g_race_nextion_last_gear_sent = latest.gear;
        g_race_nextion_gear_tx_count++;
        g_race_nextion_last_gear_tick = now;
        sent = 1U;
      }
      break;
    default:
      break;
    }
    break;
  }

transmit_done:
  if (sent != 0U) {
    g_race_nextion_next_tx_tick = HAL_GetTick() + RACE_NEXTION_TX_SPACING_MS;
  }
}

void RaceNextionGetSnapshot(RaceCANLatest_t *latest,
                                   uint8_t *dirty_mask) {
  __disable_irq();
  *latest = g_race_can_latest;
  *dirty_mask = g_race_nextion_dirty_mask;
  __enable_irq();
}

void RaceNextionClearDirty(uint8_t dirty_bit) {
  __disable_irq();
  g_race_nextion_dirty_mask &= (uint8_t)~dirty_bit;
  __enable_irq();
}

uint8_t RaceNextionSendNumber(const char *component, int32_t value) {
  char text[16];

  (void)snprintf(text, sizeof(text), "%ld", (long)value);
  return Nextion_SendTextCommand(component, text);
}

uint8_t RaceNextionSendGear(uint8_t gear) {
  if (gear == 0U) {
    return Nextion_SendTextCommand("t_gear", "N");
  }

  return RaceNextionSendNumber("t_gear", (int32_t)gear);
}

uint8_t Nextion_SendCommand(const char *command) {
  size_t command_len;

  command_len = strlen(command);
  if ((command_len == 0U) ||
      (command_len > (sizeof(g_race_nextion_tx_buffer) - 3U))) {
    g_nextion_hmi_tx_fail_count++;
    g_race_nextion_tx_fail_count++;
    return 0U;
  }

  g_race_nextion_tx_attempt_count++;
  Nextion_UARTPump();

  if (g_race_nextion_tx_active != 0U) {
    g_race_nextion_tx_busy_count++;
    return 0U;
  }

  __disable_irq();
  memcpy(g_race_nextion_tx_buffer, command, command_len);
  g_race_nextion_tx_buffer[command_len++] = 0xFFU;
  g_race_nextion_tx_buffer[command_len++] = 0xFFU;
  g_race_nextion_tx_buffer[command_len++] = 0xFFU;
  g_race_nextion_tx_len = (uint16_t)command_len;
  g_race_nextion_tx_pos = 0U;
  g_race_nextion_tx_start_tick = HAL_GetTick();
  g_race_nextion_tx_active = 1U;
  __enable_irq();

  g_nextion_hmi_tx_count++;
  g_race_nextion_tx_count++;
  g_race_last_nextion_tx_tick = HAL_GetTick();
  Nextion_UARTPump();
  return 1U;
}

uint8_t Nextion_SendTextCommand(const char *component,
                                       const char *text) {
  char command[64];
  int command_len;

  command_len = snprintf(command, sizeof(command), "%s.txt=\"%s\"",
                         component, text);
  if ((command_len <= 0) || ((size_t)command_len >= sizeof(command))) {
    g_nextion_hmi_tx_fail_count++;
    g_race_nextion_tx_fail_count++;
    return 0U;
  }

  return Nextion_SendCommand(command);
}

uint8_t Nextion_SendVisibility(const char *component, uint8_t visible) {
  char command[40];
  int command_len =
      snprintf(command, sizeof(command), "vis %s,%u", component,
               (unsigned int)((visible != 0U) ? 1U : 0U));

  if ((command_len <= 0) || ((size_t)command_len >= sizeof(command))) {
    g_nextion_hmi_tx_fail_count++;
    g_race_nextion_tx_fail_count++;
    return 0U;
  }

  return Nextion_SendCommand(command);
}

void Nextion_UARTPump(void) {
  uint32_t now;

  if (g_race_nextion_tx_active == 0U) {
    return;
  }

  now = HAL_GetTick();
  if ((now - g_race_nextion_tx_start_tick) >= RACE_NEXTION_RAW_TX_TIMEOUT_MS) {
    g_nextion_hmi_tx_fail_count++;
    g_race_nextion_tx_fail_count++;
    g_race_nextion_tx_active = 0U;
    g_race_nextion_tx_len = 0U;
    g_race_nextion_tx_pos = 0U;
    Nextion_RecoverUART4();
    return;
  }

  while ((g_race_nextion_tx_active != 0U) &&
         ((UART4->SR & USART_SR_TXE) != 0U)) {
    if (g_race_nextion_tx_pos < g_race_nextion_tx_len) {
      UART4->DR = g_race_nextion_tx_buffer[g_race_nextion_tx_pos++];
    } else {
      if ((UART4->SR & USART_SR_TC) != 0U) {
        g_race_nextion_tx_active = 0U;
        g_race_nextion_tx_len = 0U;
        g_race_nextion_tx_pos = 0U;
      }
      break;
    }
  }
}

void Nextion_RecoverUART4(void) {
  g_race_nextion_uart_last_error = huart4.ErrorCode;
  g_race_nextion_uart_last_state = HAL_UART_GetState(&huart4);
  g_race_nextion_uart_recover_count++;

  (void)HAL_UART_Abort(&huart4);
  (void)HAL_UART_DeInit(&huart4);
  MX_UART4_Init();
  g_nextion_logok_visibility = NEXTION_VIS_UNKNOWN;
  g_nextion_logoff_visibility = NEXTION_VIS_UNKNOWN;
  g_nextion_toohot_visibility = NEXTION_VIS_UNKNOWN;
}
