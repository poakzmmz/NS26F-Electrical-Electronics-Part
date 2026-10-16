#include "app_internal.h"

void GNSSLoggerStart(void) {

  g_gnss_rx_byte = 0U;
  g_gnss_rx_head = 0U;
  g_gnss_rx_tail = 0U;
  g_gnss_rx_count = 0U;
  g_gnss_rx_overflow_count = 0U;
  g_gnss_uart_error_count = 0U;
  g_gnss_rx_byte_count = 0U;
  g_gnss_dollar_count = 0U;
  g_gnss_0xB5_count = 0U;
  g_gnss_fix_update_count = 0U;
  g_gnss_last_fix_timestamp_us = 0ULL;

  (void)huart1.Instance->SR;
  (void)huart1.Instance->DR;
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void GNSS_QueuePushFromIsr(uint8_t byte) {
  uint16_t next_head = (uint16_t)((g_gnss_rx_head + 1U) % GNSS_RX_QUEUE_LEN);

  if (next_head == g_gnss_rx_tail) {
    g_gnss_rx_overflow_count++;
    return;
  }

  g_gnss_rx_queue[g_gnss_rx_head] = byte;
  g_gnss_rx_head = next_head;
  g_gnss_rx_count++;
}

uint8_t GNSS_QueuePop(uint8_t *byte) {
  uint8_t has_byte = 0U;

  __disable_irq();
  if (g_gnss_rx_tail != g_gnss_rx_head) {
    *byte = g_gnss_rx_queue[g_gnss_rx_tail];
    g_gnss_rx_tail = (uint16_t)((g_gnss_rx_tail + 1U) % GNSS_RX_QUEUE_LEN);
    has_byte = 1U;
  }
  __enable_irq();

  return has_byte;
}

void NMEA_GetField(const char *line, uint8_t field_idx, char *out_buf,
                          uint8_t max_len) {
  uint8_t current_field = 0U;
  uint16_t start_idx = 0U;
  uint16_t i = 0U;

  while (line[i] != '\0') {
    if (line[i] == ',') {
      if (current_field == field_idx) {
        uint16_t len = i - start_idx;
        if (len >= max_len) {
          len = (uint16_t)(max_len - 1U);
        }
        if (len > 0U) {
          memcpy(out_buf, &line[start_idx], len);
          out_buf[len] = '\0';
        } else {
          out_buf[0] = '\0';
        }
        return;
      }
      start_idx = (uint16_t)(i + 1U);
      current_field++;
    }
    i++;
  }

  if (current_field == field_idx) {
    uint16_t len = i - start_idx;
    if (len >= max_len) {
      len = (uint16_t)(max_len - 1U);
    }
    if (len > 0U) {
      memcpy(out_buf, &line[start_idx], len);
      out_buf[len] = '\0';
    } else {
      out_buf[0] = '\0';
    }
  } else {
    out_buf[0] = '\0';
  }
}

void GNSS_ParseLine(const char *line) {
  char field[24];

  if (g_gnss_parsed.heading[0] == '\0') {
    __disable_irq();
    strcpy(g_gnss_parsed.heading, "0.0");
    __enable_irq();
  }

  if ((strstr(line, "$GNGGA") != NULL) || (strstr(line, "$GPGGA") != NULL)) {
    NMEA_GetField(line, 1U, field, sizeof(field));
    if (field[0] != '\0' && strlen(field) >= 6) {
      int hour = (field[0] - '0') * 10 + (field[1] - '0');
      hour = (hour + 9) % 24;
      char formatted_time[16];
      formatted_time[0] = (char)('0' + (hour / 10));
      formatted_time[1] = (char)('0' + (hour % 10));
      formatted_time[2] = ':';
      formatted_time[3] = field[2];
      formatted_time[4] = field[3];
      formatted_time[5] = ':';
      formatted_time[6] = field[4];
      formatted_time[7] = field[5];

      strncpy(&formatted_time[8], &field[6], sizeof(formatted_time) - 8U - 1U);
      formatted_time[sizeof(formatted_time) - 1U] = '\0';

      __disable_irq();
      strncpy(g_gnss_parsed.time, formatted_time,
              sizeof(g_gnss_parsed.time) - 1);
      g_gnss_parsed.time[sizeof(g_gnss_parsed.time) - 1] = '\0';
      __enable_irq();
    }

    NMEA_GetField(line, 2U, field, sizeof(field));
    if (field[0] != '\0') {
      __disable_irq();
      strncpy(g_gnss_parsed.lat, field, sizeof(g_gnss_parsed.lat) - 1);
      g_gnss_parsed.lat[sizeof(g_gnss_parsed.lat) - 1] = '\0';
      __enable_irq();
    }

    NMEA_GetField(line, 4U, field, sizeof(field));
    if (field[0] != '\0') {
      __disable_irq();
      strncpy(g_gnss_parsed.lon, field, sizeof(g_gnss_parsed.lon) - 1);
      g_gnss_parsed.lon[sizeof(g_gnss_parsed.lon) - 1] = '\0';
      __enable_irq();
    }

    NMEA_GetField(line, 6U, field, sizeof(field));
    if (field[0] != '\0') {
      uint8_t fix_valid = (uint8_t)(strtoul(field, NULL, 10) > 0U);
      __disable_irq();
      strncpy(g_gnss_parsed.fix_qual, field,
              sizeof(g_gnss_parsed.fix_qual) - 1);
      g_gnss_parsed.fix_qual[sizeof(g_gnss_parsed.fix_qual) - 1] = '\0';
      __enable_irq();
      if (fix_valid != 0U) {
        g_gnss_last_fix_timestamp_us = Telemetry_GetTimestampUs();
        g_gnss_fix_update_count++;
      }
    }

    NMEA_GetField(line, 7U, field, sizeof(field));
    if (field[0] != '\0') {
      __disable_irq();
      strncpy(g_gnss_parsed.sat_count, field,
              sizeof(g_gnss_parsed.sat_count) - 1);
      g_gnss_parsed.sat_count[sizeof(g_gnss_parsed.sat_count) - 1] = '\0';
      __enable_irq();
    }
  } else if (strstr(line, "RMC") != NULL) {
    char date_field[16];
    char time_field[16];

    NMEA_GetField(line, 9U, date_field, sizeof(date_field));
    NMEA_GetField(line, 1U, time_field, sizeof(time_field));

    if (date_field[0] != '\0' && strlen(date_field) >= 6 &&
        time_field[0] != '\0' && strlen(time_field) >= 6) {
      int day = (date_field[0] - '0') * 10 + (date_field[1] - '0');
      int month = (date_field[2] - '0') * 10 + (date_field[3] - '0');
      int year = 2000 + (date_field[4] - '0') * 10 + (date_field[5] - '0');

      int hour = (time_field[0] - '0') * 10 + (time_field[1] - '0');
      int min = (time_field[2] - '0') * 10 + (time_field[3] - '0');
      int sec = (time_field[4] - '0') * 10 + (time_field[5] - '0');

      hour += 9;
      if (hour >= 24) {
        hour -= 24;
        day += 1;
        int days_in_month = 31;
        if (month == 4 || month == 6 || month == 9 || month == 11)
          days_in_month = 30;
        else if (month == 2)
          days_in_month = (year % 4 == 0) ? 29 : 28;

        if (day > days_in_month) {
          day = 1;
          month += 1;
          if (month > 12) {
            month = 1;
            year += 1;
          }
        }
      }

      g_gps_fat_time = ((uint32_t)(year - 1980) << 25) |
                       ((uint32_t)month << 21) | ((uint32_t)day << 16) |
                       ((uint32_t)hour << 11) | ((uint32_t)min << 5) |
                       ((uint32_t)sec / 2U);
    }

    NMEA_GetField(line, 7U, field, sizeof(field));
    if (field[0] != '\0') {
      float knots = strtof(field, NULL);
      float kmh = knots * 1.852f;
      int32_t speed_temp =
          (int32_t)(kmh * 10.0f + 0.5f);
      if (speed_temp < 0)
        speed_temp = 0;
      char speed_str[12];
      (void)snprintf(speed_str, sizeof(speed_str), "%ld.%ld",
                     (long)(speed_temp / 10), (long)(speed_temp % 10));

      __disable_irq();
      strncpy(g_gnss_parsed.speed, speed_str, sizeof(g_gnss_parsed.speed) - 1);
      g_gnss_parsed.speed[sizeof(g_gnss_parsed.speed) - 1] = '\0';
      __enable_irq();
    }

    NMEA_GetField(line, 3U, field, sizeof(field));
    if (field[0] != '\0') {
      __disable_irq();
      strncpy(g_gnss_parsed.lat, field, sizeof(g_gnss_parsed.lat) - 1);
      g_gnss_parsed.lat[sizeof(g_gnss_parsed.lat) - 1] = '\0';
      __enable_irq();
    }

    NMEA_GetField(line, 5U, field, sizeof(field));
    if (field[0] != '\0') {
      __disable_irq();
      strncpy(g_gnss_parsed.lon, field, sizeof(g_gnss_parsed.lon) - 1);
      g_gnss_parsed.lon[sizeof(g_gnss_parsed.lon) - 1] = '\0';
      __enable_irq();
    }
  } else if (strstr(line, "VTG") != NULL) {
    NMEA_GetField(line, 7U, field, sizeof(field));
    if (field[0] != '\0') {
      float speed_val = strtof(field, NULL);
      int32_t speed_temp = (int32_t)(speed_val * 10.0f + 0.5f);
      if (speed_temp < 0)
        speed_temp = 0;
      char speed_str[12];
      (void)snprintf(speed_str, sizeof(speed_str), "%ld.%ld",
                     (long)(speed_temp / 10), (long)(speed_temp % 10));

      __disable_irq();
      strncpy(g_gnss_parsed.speed, speed_str, sizeof(g_gnss_parsed.speed) - 1);
      g_gnss_parsed.speed[sizeof(g_gnss_parsed.speed) - 1] = '\0';
      __enable_irq();
    }
  } else if ((strstr(line, "HDT") != NULL) || (strstr(line, "THS") != NULL)) {
    char status_field[4] = {0};

    if (strstr(line, "THS") != NULL) {
      NMEA_GetField(line, 2U, status_field, sizeof(status_field));
    }

    if (status_field[0] != 'V') {
      NMEA_GetField(line, 1U, field, sizeof(field));
      if (field[0] != '\0') {
        __disable_irq();
        strncpy(g_gnss_parsed.heading, field,
                sizeof(g_gnss_parsed.heading) - 1);
        g_gnss_parsed.heading[sizeof(g_gnss_parsed.heading) - 1] = '\0';
        __enable_irq();
      }
    }
  }
}

void GNSS_ProcessParser(void) {
  uint8_t byte;

  static char line_buf[128];
  static uint16_t line_idx = 0U;
  static uint8_t gnss_in_ubx = 0U;

  typedef enum {
    UBX_STATE_SYNC1 = 0,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN_LSB,
    UBX_STATE_LEN_MSB,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CHKA,
    UBX_STATE_CHKB
  } UbxState_t;

  static UbxState_t ubx_state = UBX_STATE_SYNC1;
  static uint8_t ubx_class = 0;
  static uint8_t ubx_id = 0;
  static uint16_t ubx_len = 0;
  static uint16_t ubx_idx = 0;
  static uint8_t ubx_payload[128];
  static uint8_t ubx_chka = 0;
  static uint8_t ubx_chkb = 0;

  while (GNSS_QueuePop(&byte) != 0U) {
    g_gnss_rx_byte_count++;
    if (byte == '$') {
      g_gnss_dollar_count++;
    }
    if (byte == 0xB5) {
      g_gnss_0xB5_count++;
    }

    if (gnss_in_ubx == 0U) {
      if (byte == 0xB5) {
        gnss_in_ubx = 1U;
        ubx_state = UBX_STATE_SYNC2;
        line_idx = 0U;
      }
    }

    if (gnss_in_ubx != 0U) {
      switch (ubx_state) {
      case UBX_STATE_SYNC1:
        if (byte == 0xB5) {
          ubx_state = UBX_STATE_SYNC2;
        }
        break;

      case UBX_STATE_SYNC2:
        if (byte == 0x62) {
          ubx_state = UBX_STATE_CLASS;
        } else if (byte == 0xB5) {
          ubx_state = UBX_STATE_SYNC2;
        } else {
          ubx_state = UBX_STATE_SYNC1;
        }
        break;

      case UBX_STATE_CLASS:
        ubx_class = byte;
        ubx_chka = byte;
        ubx_chkb = byte;
        ubx_state = UBX_STATE_ID;
        break;

      case UBX_STATE_ID:
        ubx_id = byte;
        ubx_chka += byte;
        ubx_chkb += ubx_chka;
        ubx_state = UBX_STATE_LEN_LSB;
        break;

      case UBX_STATE_LEN_LSB:
        ubx_len = byte;
        ubx_chka += byte;
        ubx_chkb += ubx_chka;
        ubx_state = UBX_STATE_LEN_MSB;
        break;

      case UBX_STATE_LEN_MSB:
        ubx_len |= ((uint16_t)byte << 8);
        ubx_chka += byte;
        ubx_chkb += ubx_chka;
        if (ubx_len > sizeof(ubx_payload)) {
          ubx_state = UBX_STATE_SYNC1;
        } else {
          ubx_idx = 0U;
          if (ubx_len == 0U) {
            ubx_state = UBX_STATE_CHKA;
          } else {
            ubx_state = UBX_STATE_PAYLOAD;
          }
        }
        break;

      case UBX_STATE_PAYLOAD:
        ubx_payload[ubx_idx] = byte;
        ubx_idx++;
        ubx_chka += byte;
        ubx_chkb += ubx_chka;
        if (ubx_idx >= ubx_len) {
          ubx_state = UBX_STATE_CHKA;
        }
        break;

      case UBX_STATE_CHKA:
        if (byte == ubx_chka) {
          ubx_state = UBX_STATE_CHKB;
        } else {
          ubx_state = UBX_STATE_SYNC1;
        }
        break;

      case UBX_STATE_CHKB:
        if (byte == ubx_chkb) {
          if ((ubx_class == 0x01) && (ubx_id == 0x3C) && (ubx_len >= 64)) {
            int32_t heading_raw = (int32_t)ubx_payload[24] |
                                  ((int32_t)ubx_payload[25] << 8) |
                                  ((int32_t)ubx_payload[26] << 16) |
                                  ((int32_t)ubx_payload[27] << 24);

            if (heading_raw != 0) {
              float heading_deg = (float)heading_raw * 1e-5f;
              if (heading_deg < 0.0f) {
                heading_deg += 360.0f;
              }
              char heading_str[12];
              (void)snprintf(heading_str, sizeof(heading_str), "%.2f",
                             heading_deg);

              __disable_irq();
              strncpy(g_gnss_parsed.heading, heading_str,
                      sizeof(g_gnss_parsed.heading) - 1);
              g_gnss_parsed.heading[sizeof(g_gnss_parsed.heading) - 1] = '\0';
              __enable_irq();
            } else {
              __disable_irq();
              strcpy(g_gnss_parsed.heading, "0.0");
              __enable_irq();
            }
          }
        }
        ubx_state = UBX_STATE_SYNC1;
        break;

      default:
        ubx_state = UBX_STATE_SYNC1;
        break;
      }

      if (ubx_state == UBX_STATE_SYNC1) {
        gnss_in_ubx = 0U;
      }
    } else {
      if ((byte == '\n') || (byte == '\r')) {
        if (line_idx > 0U) {
          line_buf[line_idx] = '\0';
          GNSS_ParseLine(line_buf);
          line_idx = 0U;
        }
      } else {
        if (line_idx < (sizeof(line_buf) - 1U)) {
          line_buf[line_idx] = (char)byte;
          line_idx++;
        } else {
          line_idx = 0U;
        }
      }
    }
  }
}
