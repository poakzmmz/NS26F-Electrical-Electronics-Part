#include "app_internal.h"

void RPMOutputs_Init(void) {
  WS2812_InitTimings();
  g_ws2812_dma_ready = 1U;
  g_ws2812_last_update_tick = 0U;
  RPMOutputs_SetRPM(g_current_rpm);
  WS2812_ShowRPM(g_current_rpm, g_rpm_led_mode);
}

void RPMOutputs_Process(void) {
  uint32_t now = HAL_GetTick();

  if ((now - g_ws2812_last_update_tick) < WS2812_UPDATE_MS) {
    return;
  }

  g_ws2812_last_update_tick = now;
  WS2812_ShowRPM(g_current_rpm, g_rpm_led_mode);
}

void RPMOutputs_SetRPM(uint16_t rpm) {
  if (rpm > RPM_MAX) {
    rpm = RPM_MAX;
  }

  g_current_rpm = rpm;
  g_nextion_hmi_rpm = rpm;
  g_nextion_latest_rpm = rpm;
  g_nextion_rpm_valid = 1U;
}

void WS2812_InitTimings(void) {
  uint32_t timer_period = htim8.Init.Period + 1U;

  g_ws2812_pwm_0 =
      (uint16_t)(((timer_period * WS2812_T0H_NS) + (WS2812_PERIOD_NS / 2U)) /
                 WS2812_PERIOD_NS);
  g_ws2812_pwm_1 =
      (uint16_t)(((timer_period * WS2812_T1H_NS) + (WS2812_PERIOD_NS / 2U)) /
                 WS2812_PERIOD_NS);
}

uint8_t PercentToByte(uint8_t percent) {
  if (percent > 100U) {
    percent = 100U;
  }

  return (uint8_t)(((uint32_t)255U * percent) / 100U);
}

void WS2812_SetLedBits(uint32_t *index, uint8_t red, uint8_t green,
                              uint8_t blue) {
  uint8_t color[3] = {green, red, blue};

  for (uint32_t color_index = 0U; color_index < 3U; color_index++) {
    for (int8_t bit = 7; bit >= 0; bit--) {
      g_ws2812_pwm_buffer[*index] = ((color[color_index] >> bit) & 0x01U)
                                        ? g_ws2812_pwm_1
                                        : g_ws2812_pwm_0;
      (*index)++;
    }
  }
}

void WS2812_ShowRPM(uint16_t rpm, uint8_t mode) {
  (void)mode;
  WS2812_ShowRPMBar(rpm);
}

void WS2812_ShowRPMBar(uint16_t rpm) {
  uint32_t index = 0U;
  uint32_t now = HAL_GetTick();
  uint8_t led_colors[WS2812_LED_COUNT][3] = {{0U}};
  uint8_t lit_count;
  uint8_t green = PercentToByte(RPM_LED_BRIGHTNESS);
  uint8_t sky_blue_green = PercentToByte(25U);
  uint8_t sky_blue_blue = PercentToByte(100U);
  uint8_t yellow_red = PercentToByte(RPM_LED_BRIGHTNESS);
  uint8_t yellow_green = PercentToByte(RPM_LED_BRIGHTNESS);
  uint8_t orange_green = PercentToByte(RPM_LED_BRIGHTNESS / 2U);
  uint8_t red = PercentToByte(RPM_LED_BRIGHTNESS);
  uint8_t blink_on = 1U;
  uint8_t can_ok;

  if (g_ws2812_dma_ready == 0U) {
    g_ws2812_dma_busy_skip_count++;
    return;
  }

  if (rpm >= RPM_REDZONE_START) {
    blink_on = (uint8_t)(((now / RPM_REDZONE_BLINK_MS) % 2U) == 0U);

    for (uint8_t led_number = 1U; led_number <= RPM_USED_LED_COUNT;
         led_number++) {
      if (blink_on != 0U) {
        led_colors[led_number - 1U][0] = red;
      }
    }
  } else {

    lit_count = (uint8_t)(rpm / RPM_PER_LED);
    if (lit_count > RPM_USED_LED_COUNT) {
      lit_count = RPM_USED_LED_COUNT;
    }

    for (uint8_t step = 0U; step < lit_count; step++) {
      uint8_t led_number = (uint8_t)(10U - step);
      uint8_t led_index = (uint8_t)(led_number - 1U);

      if (led_number >= 8U) {
        led_colors[led_index][1] = green;
      } else if (led_number >= 5U) {
        led_colors[led_index][0] = yellow_red;
        led_colors[led_index][1] = yellow_green;
      } else {
        led_colors[led_index][0] = red;
      }
    }
  }

  can_ok = ((g_race_can_seen_count != 0U) &&
            ((now - g_race_last_can_rx_tick) < RACE_CAN_LED_TIMEOUT_MS))
               ? 1U
               : 0U;
  if (can_ok != 0U) {
    led_colors[11U][1] = sky_blue_green;
    led_colors[11U][2] = sky_blue_blue;
  } else {
    led_colors[11U][0] = red;
  }

  if (g_race_can_latest.water_c >= RACE_CLT_TOO_HOT_C) {
    led_colors[10U][0] = red;
  } else if (g_race_can_latest.water_c >= 70) {
    led_colors[10U][0] = red;
    led_colors[10U][1] = orange_green;
  } else {
    led_colors[10U][1] = sky_blue_green;
    led_colors[10U][2] = sky_blue_blue;
  }

  for (uint8_t led_index = 0U; led_index < WS2812_LED_COUNT; led_index++) {
    WS2812_SetLedBits(&index, led_colors[led_index][0],
                      led_colors[led_index][1], led_colors[led_index][2]);
  }

  WS2812_SendBuffer(index);
}

void WS2812_SendBuffer(uint32_t used_len) {
  uint32_t index = used_len;

  while (index < WS2812_BUFFER_LEN) {
    g_ws2812_pwm_buffer[index++] = 0U;
  }

  __disable_irq();
  if (g_ws2812_dma_ready == 0U) {
    __enable_irq();
    g_ws2812_dma_busy_skip_count++;
    return;
  }
  g_ws2812_dma_ready = 0U;
  __enable_irq();

  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COUNTER(&htim8, 0U);
  if (HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1,
                            (uint32_t *)g_ws2812_pwm_buffer,
                            WS2812_BUFFER_LEN) != HAL_OK) {
    g_ws2812_dma_ready = 1U;
    g_ws2812_dma_start_fail_count++;
  }
}
