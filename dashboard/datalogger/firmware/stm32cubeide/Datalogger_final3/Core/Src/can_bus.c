#include "app_internal.h"

void USER_CAN1_Apply500Kbps(void) {
  if (HAL_CAN_DeInit(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
}

uint8_t EMU_CANLoggerConfigureHardware(void) {
  CAN_FilterTypeDef can_filter = {0};

  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = (uint16_t)(EMU_CAN_BASE_ID << 5);
  can_filter.FilterIdLow = 0x0000U;
  can_filter.FilterMaskIdHigh = (uint16_t)(EMU_CAN_ID_MASK << 5);
  can_filter.FilterMaskIdLow = 0x0000U;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  return (HAL_CAN_ConfigFilter(&hcan1, &can_filter) == HAL_OK) ? 1U : 0U;
}

void EMU_CANLoggerStart(void) {
  g_emu_can_rx_head = 0U;
  g_emu_can_rx_tail = 0U;
  g_emu_can_rx_count = 0U;
  g_emu_can_log_count = 0U;
  g_emu_can_rx_overflow_count = 0U;
  g_emu_can_hal_error_count = 0U;
  g_emu_can_rx_irq_enabled = 0U;

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    g_emu_can_enabled = 0U;
    return;
  }

  g_emu_can_rx_irq_enabled = 0U;
  if (HAL_CAN_ActivateNotification(
          &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL |
                      CAN_IT_RX_FIFO0_OVERRUN) == HAL_OK) {
    g_emu_can_rx_irq_enabled = 1U;
  }

  EMU_CANDrainRxFifo();
}

void EMU_CANDrainRxFifo(void) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, data) !=
        HAL_OK) {
      g_emu_can_hal_error_count++;
      return;
    }

    g_race_raw_can_seen_count++;
    g_race_last_raw_can_rx_tick = HAL_GetTick();

    if ((rx_header.IDE == CAN_ID_STD) && (rx_header.RTR == CAN_RTR_DATA) &&
        ((rx_header.StdId & EMU_CAN_ID_MASK) == EMU_CAN_BASE_ID)) {
      uint8_t can_index = (uint8_t)(rx_header.StdId - EMU_CAN_BASE_ID);
      g_telemetry_latest_can[can_index].valid = 1U;
      g_telemetry_latest_can[can_index].timestamp_us = Telemetry_GetTimestampUs();
      g_telemetry_latest_can[can_index].std_id = rx_header.StdId;
      g_telemetry_latest_can[can_index].dlc =
          (rx_header.DLC > 8U) ? 8U : rx_header.DLC;
      memcpy(g_telemetry_latest_can[can_index].data, data, 8U);
      g_race_can_seen_count++;
      g_race_last_can_rx_tick = HAL_GetTick();
      RaceLoggerUpdateLatestFromCAN(
          rx_header.StdId, (rx_header.DLC > 8U) ? 8U : rx_header.DLC, data);
      EMU_CANQueuePushFromIsr(&rx_header, data);
    }
  }
}

void EMU_CANQueuePushFromIsr(const CAN_RxHeaderTypeDef *rx_header,
                                    const uint8_t *data) {
  EMU_CAN_Frame_t frame;
  uint16_t next_head =
      (uint16_t)((g_emu_can_rx_head + 1U) % EMU_CAN_RX_QUEUE_LEN);
  uint8_t dlc = (rx_header->DLC > 8U) ? 8U : rx_header->DLC;

  if (next_head == g_emu_can_rx_tail) {
    g_emu_can_rx_overflow_count++;
    return;
  }

  frame.timestamp_us = Telemetry_GetTimestampUs();
  frame.timestamp_ms = (uint32_t)(frame.timestamp_us / 1000ULL);
  frame.std_id = rx_header->StdId;
  frame.rpm = g_race_can_latest.rpm;
  frame.water_c = g_race_can_latest.water_c;
  frame.oil_c = g_race_can_latest.oil_c;
  frame.can_speed_kmh = g_race_can_latest.speed_kmh;
  frame.gear = g_race_can_latest.gear;
  frame.battery_mV = g_race_can_latest.battery_mV;
  frame.rl_speed_kmh = g_wheel_speed_kmh;
  if ((rx_header->StdId == (EMU_CAN_BASE_ID + 1U)) && (dlc >= 8U)) {
    g_emu_potentiometer4_raw =
        (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
  } else if ((rx_header->StdId == (EMU_CAN_BASE_ID + 6U)) && (dlc >= 2U)) {
    g_emu_potentiometer5_raw =
        (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    if (dlc >= 4U) {
      g_emu_adc6_raw =
          (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
    }
  }

  memset(frame.adc_raw, 0, sizeof(frame.adc_raw));
  frame.adc_raw[0] = g_adc_dma_buffer[0];
  frame.adc_raw[1] = g_emu_potentiometer4_raw;
  frame.adc_raw[2] = g_emu_potentiometer5_raw;
  frame.adc_raw[3] = g_adc_dma_buffer[1];
  frame.adc_raw[4] = g_adc_dma_buffer[2];
  frame.adc_raw[5] = g_adc_dma_buffer[3];
  frame.adc_raw[6] = g_emu_adc6_raw;
  frame.dlc = dlc;
  memset(frame.data, 0, sizeof(frame.data));
  memcpy(frame.data, data, dlc);

  g_emu_can_rx_queue[g_emu_can_rx_head] = frame;
  g_emu_can_rx_head = next_head;
  g_emu_can_rx_count++;
}

uint8_t EMU_CANQueuePop(EMU_CAN_Frame_t *frame) {
  uint8_t has_frame = 0U;

  __disable_irq();
  if (g_emu_can_rx_tail != g_emu_can_rx_head) {
    *frame = g_emu_can_rx_queue[g_emu_can_rx_tail];
    g_emu_can_rx_tail =
        (uint16_t)((g_emu_can_rx_tail + 1U) % EMU_CAN_RX_QUEUE_LEN);
    has_frame = 1U;
  }
  __enable_irq();

  return has_frame;
}
