#include "app_internal.h"

uint8_t Wheel4_ChannelIndex(const TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    return (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) ? 0U
                                                       : WHEEL_CHANNEL_COUNT;
  }

  if (htim->Instance == TIM2) {
    switch (htim->Channel) {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      return 1U;
    case HAL_TIM_ACTIVE_CHANNEL_2:
      return 2U;
    case HAL_TIM_ACTIVE_CHANNEL_3:
      return 3U;
    default:
      return WHEEL_CHANNEL_COUNT;
    }
  }

  return WHEEL_CHANNEL_COUNT;
}

uint32_t Wheel4_HALChannel(uint8_t channel) {
  static const uint32_t hal_channels[WHEEL_CHANNEL_COUNT] = {
      TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

  return hal_channels[channel];
}

TIM_HandleTypeDef *Wheel4_HALTimer(uint8_t channel) {
  return (channel == 0U) ? &htim1 : &htim2;
}

void Wheel4_QueuePushFromIsr(const Wheel4_LogEvent_t *event) {
  uint16_t next_head =
      (uint16_t)((g_wheel4_rx_head + 1U) % WHEEL4_RX_QUEUE_LEN);

  if (next_head == g_wheel4_rx_tail) {
    g_wheel4_rx_overflow_count++;
    return;
  }

  g_wheel4_rx_queue[g_wheel4_rx_head] = *event;
  g_wheel4_rx_head = next_head;
}

uint8_t Wheel4_QueuePop(Wheel4_LogEvent_t *event) {
  uint8_t has_event = 0U;

  __disable_irq();
  if (g_wheel4_rx_tail != g_wheel4_rx_head) {
    *event = g_wheel4_rx_queue[g_wheel4_rx_tail];
    g_wheel4_rx_tail =
        (uint16_t)((g_wheel4_rx_tail + 1U) % WHEEL4_RX_QUEUE_LEN);
    has_event = 1U;
  }
  __enable_irq();

  return has_event;
}

void Wheel_QueuePushFromIsr(const Wheel_LogEvent_t *event) {
  uint16_t next_head = (uint16_t)((g_wheel_rx_head + 1U) % WHEEL_RX_QUEUE_LEN);

  if (next_head == g_wheel_rx_tail) {
    g_wheel_rx_overflow_count++;
    return;
  }

  g_wheel_rx_queue[g_wheel_rx_head] = *event;
  g_wheel_rx_head = next_head;
}
