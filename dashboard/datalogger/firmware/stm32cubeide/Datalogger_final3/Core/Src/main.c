
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_internal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim8_ch1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
volatile FRESULT g_sd_last_fresult = FR_OK;
volatile SD_FaultStage_t g_sd_fault_stage = SD_FAULT_NONE;
volatile FRESULT g_sd_fault_fresult = FR_OK;
volatile FRESULT g_sd_fault_log_fresult = FR_OK;
volatile uint32_t g_sd_fault_detail = 0U;
volatile uint32_t g_sd_fault_tick = 0U;
volatile UINT g_sd_fault_buffer_len = 0U;
volatile uint32_t g_sd_fault_unsynced_count = 0U;
volatile uint32_t g_sd_fault_telemetry_count = 0U;
volatile uint32_t g_sd_write_fail_count = 0U;
volatile uint32_t g_sd_sync_fail_count = 0U;
volatile FRESULT g_sd_last_sync_fresult = FR_OK;
volatile uint32_t g_sd_write_call_count = 0U;
volatile uint32_t g_sd_sync_ok_count = 0U;
volatile uint32_t g_sd_last_write_size = 0U;
volatile uint32_t g_sd_max_buffer_len = 0U;
volatile uint32_t g_sd_last_write_duration_ms = 0U;
volatile uint32_t g_sd_max_write_duration_ms = 0U;
volatile uint64_t g_sd_total_bytes_written = 0ULL;
volatile SD_LogFlushReason_t g_sd_last_flush_reason =
    SD_LOG_FLUSH_REASON_NONE;
volatile uint8_t g_sd_bringup_status = 0U;
volatile GPIO_PinState g_sd_detect_state = GPIO_PIN_RESET;
volatile uint8_t g_sd_hal_stage = 0U;
volatile uint32_t g_sd_hal_error_code = HAL_SD_ERROR_NONE;
volatile uint32_t g_sd_ll_last_op = SD_LOWLEVEL_OP_NONE;
volatile uint32_t g_sd_ll_read_call_count = 0U;
volatile uint32_t g_sd_ll_write_call_count = 0U;
volatile uint32_t g_sd_ll_retry_count = 0U;
volatile uint32_t g_sd_ll_read_fail_count = 0U;
volatile uint32_t g_sd_ll_write_fail_count = 0U;
volatile uint32_t g_sd_ll_ready_timeout_count = 0U;
volatile uint32_t g_sd_ll_last_sector = 0U;
volatile uint32_t g_sd_ll_last_blocks = 0U;
volatile uint32_t g_sd_ll_last_buffer_addr = 0U;
volatile uint32_t g_sd_ll_last_hal_status = HAL_OK;
volatile uint32_t g_sd_ll_last_hal_error = HAL_SD_ERROR_NONE;
volatile uint32_t g_sd_ll_last_hal_state = HAL_SD_STATE_RESET;
volatile uint32_t g_sd_ll_last_card_state = 0U;
volatile uint32_t g_sd_ll_last_wait_ms = 0U;
volatile uint32_t g_sd_ll_last_tick = 0U;
FIL g_emu_can_file;
FIL *g_sd_active_file = NULL;
volatile uint32_t g_wheel_log_count = 0U;
volatile uint32_t g_adc_log_count = 0U;
volatile uint32_t g_emu_can_log_count = 0U;
volatile uint32_t g_emu_can_rx_count = 0U;
volatile uint32_t g_emu_can_rx_overflow_count = 0U;
volatile uint32_t g_emu_can_hal_error_count = 0U;
volatile uint8_t g_emu_can_enabled = 0U;
volatile uint32_t g_telemetry_log_count = 0U;
volatile uint32_t g_wheel4_log_count = 0U;
volatile uint32_t g_adc6_log_count = 0U;
volatile uint32_t g_wheel4_rx_overflow_count = 0U;
volatile uint32_t g_adc6_rx_overflow_count = 0U;
volatile uint8_t g_telemetry_queue_enabled = 0U;
volatile uint8_t g_wheel_legacy_queue_enabled = 0U;
volatile uint8_t g_adc_legacy_queue_enabled = 0U;
volatile uint32_t g_adc_rx_overflow_count = 0U;
volatile uint32_t g_adc_sample_count = 0U;
volatile uint64_t g_adc_last_timestamp_us = 0ULL;
volatile uint16_t g_adc_dma_buffer[ADC_DMA_CHANNEL_COUNT];
volatile uint16_t g_emu_potentiometer4_raw = 0U;
volatile uint16_t g_emu_potentiometer5_raw = 0U;
volatile uint16_t g_emu_adc6_raw = 0U;
volatile uint16_t g_adc_rx_head = 0U;
volatile uint16_t g_adc_rx_tail = 0U;
ADC_LogSample_t g_adc_rx_queue[ADC_LOG_QUEUE_LEN];
volatile uint16_t g_emu_can_rx_head = 0U;
volatile uint16_t g_emu_can_rx_tail = 0U;
EMU_CAN_Frame_t g_emu_can_rx_queue[EMU_CAN_RX_QUEUE_LEN];
volatile uint16_t g_wheel4_rx_head = 0U;
volatile uint16_t g_wheel4_rx_tail = 0U;
Wheel4_LogEvent_t g_wheel4_rx_queue[WHEEL4_RX_QUEUE_LEN] CCMRAM_BUFFER;
volatile uint64_t g_wheel4_last_timestamp_us[WHEEL_CHANNEL_COUNT];
volatile uint32_t g_wheel4_last_delta_us[WHEEL_CHANNEL_COUNT];
volatile uint32_t g_wheel4_pulse_count[WHEEL_CHANNEL_COUNT];
volatile uint32_t g_wheel4_filtered_centi_kmh[WHEEL_CHANNEL_COUNT];
volatile uint8_t g_wheel4_filter_valid[WHEEL_CHANNEL_COUNT];
volatile uint32_t g_wheel_rx_overflow_count = 0U;
volatile uint32_t g_tim1_overflow_count = 0U;
volatile uint8_t g_tim1_timebase_ready = 0U;
volatile uint64_t g_wheel_last_timestamp_us = 0ULL;
volatile uint16_t g_wheel_rx_head = 0U;
volatile uint16_t g_wheel_rx_tail = 0U;
Wheel_LogEvent_t g_wheel_rx_queue[WHEEL_RX_QUEUE_LEN];

volatile uint32_t g_pb10_edge_count = 0U;
uint16_t g_wheel_speed_kmh = 0U;

volatile uint8_t g_gnss_rx_byte = 0U;
volatile uint16_t g_gnss_rx_head = 0U;
volatile uint16_t g_gnss_rx_tail = 0U;
volatile uint32_t g_gnss_rx_count = 0U;
volatile uint32_t g_gnss_rx_overflow_count = 0U;
volatile uint32_t g_gnss_uart_error_count = 0U;
volatile uint32_t g_gnss_drain_call_count = 0U;
volatile uint64_t g_gnss_drain_byte_count = 0ULL;
volatile uint8_t g_gnss_rx_queue[GNSS_RX_QUEUE_LEN] CCMRAM_BUFFER;

volatile uint32_t g_gnss_rx_byte_count = 0U;
volatile uint32_t g_gnss_dollar_count = 0U;
volatile uint32_t g_gnss_0xB5_count = 0U;
volatile uint32_t g_gnss_fix_update_count = 0U;
volatile uint64_t g_gnss_last_fix_timestamp_us = 0ULL;

uint8_t g_imu_rx_dma_buffer[IMU_RX_DMA_BUFFER_SIZE];
uint16_t g_imu_rx_read_pos = 0U;
uint8_t g_imu_packet_buffer[IMU_PACKET_SIZE];
uint8_t g_imu_packet_len = 0U;
volatile uint32_t g_imu_packet_count = 0U;
volatile uint32_t g_imu_checksum_error_count = 0U;
volatile uint32_t g_imu_rx_error_count = 0U;
volatile uint32_t g_imu_rx_byte_count = 0U;
volatile uint32_t g_imu_uart_parity_error_count = 0U;
volatile uint32_t g_imu_uart_noise_error_count = 0U;
volatile uint32_t g_imu_uart_framing_error_count = 0U;
volatile uint32_t g_imu_uart_overrun_error_count = 0U;
volatile uint32_t g_imu_uart_dma_error_count = 0U;
volatile uint8_t g_imu_rearm_pending = 0U;
Telemetry_IMULatest_t g_imu_latest;
Telemetry_IMULatest_t g_imu_history[IMU_HISTORY_QUEUE_LEN];
uint16_t g_imu_history_head = 0U;
uint16_t g_imu_history_count = 0U;
volatile uint16_t g_imu_parse_batch_max = 0U;
volatile uint64_t g_imu_last_rx_timestamp_us = 0ULL;
volatile int32_t g_imu_timestamp_error_us = 0;
volatile uint32_t g_imu_estimated_missing_count = 0U;
volatile uint32_t g_imu_resync_count = 0U;
volatile uint32_t g_imu_timeout_count = 0U;
volatile uint32_t g_imu_recovery_count = 0U;
volatile uint8_t g_imu_link_valid = 0U;

char g_sd_log_buffer[SD_LOG_BUFFER_SIZE] SDIO_ALIGNED_BUFFER;
char g_sd_log_path[SD_LOG_PATH_LEN];
UINT g_sd_log_buffer_len = 0U;
uint32_t g_sd_log_last_write_tick = 0U;
uint32_t g_sd_log_last_sync_tick = 0U;
uint32_t g_sd_log_unsynced_count = 0U;
Telemetry_CANLatest_t g_telemetry_latest_can[EMU_CAN_FRAME_COUNT];
Telemetry_WheelLatest_t g_telemetry_latest_wheel[WHEEL_CHANNEL_COUNT];
uint16_t g_telemetry_latest_adc_raw[ADC_CHANNEL_COUNT];
uint8_t g_telemetry_adc_seen = 0U;
uint64_t g_telemetry_next_snapshot_us = 0ULL;
uint32_t g_telemetry_snapshot_seq = 0U;
volatile uint32_t g_telemetry_missed_snapshot_count = 0U;
volatile uint16_t g_telemetry_snapshot_head = 0U;
volatile uint16_t g_telemetry_snapshot_tail = 0U;
volatile uint16_t g_telemetry_snapshot_max_depth = 0U;
volatile uint32_t g_telemetry_snapshot_overflow_count = 0U;
Telemetry_Snapshot_t
    g_telemetry_snapshot_queue[TELEMETRY_SNAPSHOT_QUEUE_LEN];
GNSS_ParsedData_t g_gnss_parsed = {.time = "00:00:00.00",
                                          .lat = "0.0",
                                          .lon = "0.0",
                                          .speed = "0.0",
                                          .sat_count = "0",
                                          .fix_qual = "0",
                                          .heading = "0.0"};
volatile uint32_t g_gps_fat_time = 0U;
RaceCANLatest_t g_race_can_latest;
uint32_t g_race_nextion_last_rpm_tick = 0U;
uint32_t g_race_nextion_last_gear_tick = 0U;
uint32_t g_race_nextion_last_water_tick = 0U;
uint32_t g_race_nextion_last_tps_tick = 0U;
uint32_t g_race_nextion_last_speed_tick = 0U;
uint32_t g_race_nextion_last_oil_tick = 0U;
uint32_t g_race_nextion_last_battery_tick = 0U;
uint32_t g_race_nextion_last_fuel_tick = 0U;
volatile uint16_t g_race_nextion_last_rpm_sent = 0U;
volatile uint32_t g_race_nextion_rpm_tx_count = 0U;
volatile uint16_t g_race_nextion_last_tps_sent = 0U;
volatile uint32_t g_race_nextion_tps_tx_count = 0U;
volatile uint8_t g_race_nextion_last_gear_sent = 0U;
volatile uint32_t g_race_nextion_gear_tx_count = 0U;
volatile int16_t g_race_nextion_last_water_sent = 0;
volatile uint32_t g_race_nextion_water_tx_count = 0U;
volatile uint16_t g_race_nextion_last_speed_sent = 0U;
volatile uint32_t g_race_nextion_speed_tx_count = 0U;
uint8_t g_race_nextion_rr_index = RACE_NEXTION_FIELD_RPM;
uint32_t g_race_nextion_next_tx_tick = 0U;
uint32_t g_race_nextion_last_no_sig_tick = 0U;
volatile uint8_t g_race_nextion_dirty_mask = 0U;
uint8_t g_race_can_signal_lost = 1U;
volatile uint32_t g_race_raw_can_seen_count = 0U;
volatile uint32_t g_race_can_seen_count = 0U;
volatile uint32_t g_race_last_raw_can_rx_tick = 0U;
volatile uint32_t g_race_last_can_rx_tick = 0U;
uint32_t g_race_last_nextion_tx_tick = 0U;
volatile uint32_t g_race_log_count = 0U;
volatile uint32_t g_race_start_trigger_count = 0U;
volatile uint32_t g_race_nextion_tx_attempt_count = 0U;
volatile uint32_t g_race_nextion_tx_count = 0U;
volatile uint32_t g_race_nextion_tx_fail_count = 0U;
volatile uint32_t g_race_nextion_uart_recover_count = 0U;
volatile uint32_t g_race_nextion_uart_error_callback_count = 0U;
volatile uint32_t g_race_nextion_uart_last_error = 0U;
volatile uint32_t g_race_nextion_uart_last_state = 0U;
uint8_t g_race_nextion_tx_buffer[64];
volatile uint16_t g_race_nextion_tx_len = 0U;
volatile uint16_t g_race_nextion_tx_pos = 0U;
volatile uint8_t g_race_nextion_tx_active = 0U;
uint32_t g_race_nextion_tx_start_tick = 0U;
volatile uint32_t g_race_nextion_tx_busy_count = 0U;
volatile uint8_t g_emu_can_rx_irq_enabled = 0U;
uint8_t g_nextion_logok_visibility = NEXTION_VIS_UNKNOWN;
uint8_t g_nextion_logoff_visibility = NEXTION_VIS_UNKNOWN;
uint8_t g_nextion_toohot_visibility = NEXTION_VIS_UNKNOWN;
uint16_t g_nextion_latest_rpm = 0U;
uint8_t g_nextion_rpm_valid = 0U;
uint32_t g_nextion_last_tx_tick = 0U;
uint16_t g_nextion_hmi_rpm = 0U;
volatile uint32_t g_nextion_hmi_tx_count = 0U;
volatile uint32_t g_nextion_hmi_tx_fail_count = 0U;
volatile uint16_t g_current_rpm = 0U;
uint8_t g_rpm_led_mode = RPM_LED_MODE_DEFAULT;
uint32_t g_ws2812_last_update_tick = 0U;
uint16_t g_ws2812_pwm_buffer[WS2812_BUFFER_LEN];
volatile uint8_t g_ws2812_dma_ready = 1U;
volatile uint32_t g_ws2812_dma_done_count = 0U;
volatile uint32_t g_ws2812_dma_start_fail_count = 0U;
volatile uint32_t g_ws2812_dma_busy_skip_count = 0U;
uint16_t g_ws2812_pwm_0 = 0U;
uint16_t g_ws2812_pwm_1 = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM8_Init();

  /* USER CODE BEGIN 2 */
  USER_CAN1_Apply500Kbps();
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);

  for (uint8_t i = 0U; i < 3U; i++) {
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_SET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_SET);
    HAL_Delay(120U);
    LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
    LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);
    HAL_Delay(120U);
  }

  SD_WaitForCardInserted();

  RPMOutputs_Init();
  GNSSLoggerStart();

  if (ADC_LoggerConfigureHardware() == 0U) {
    LED_BlinkCode(SD_LED_ERR_ADC_CONFIG);
  }

  g_emu_can_enabled = EMU_CANLoggerConfigureHardware();

  g_sd_last_fresult = SD_TelemetryLoggerInit();
  TelemetryLoggerStart();

  if (g_emu_can_enabled != 0U) {
    EMU_CANLoggerStart();
  }

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    g_sd_last_fresult = SD_TelemetryLoggerProcess();
    RPMOutputs_Process();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
  static const uint32_t adc_channels[ADC_DMA_CHANNEL_COUNT] = {
      ADC_CHANNEL_10, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15};
  ADC_ChannelConfTypeDef sConfig = {0};

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
    Error_Handler();
  }

  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  for (uint32_t i = 0U; i < ADC_DMA_CHANNEL_COUNT; i++) {
    sConfig.Channel = adc_channels[i];
    sConfig.Rank = (uint32_t)(i + 1U);
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
      Error_Handler();
    }
  }
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void) {

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */
  /* The logger writes slowly, so prefer signal margin over bus speed. Hardware
   * flow control also lets SDIO pause the clock while the FIFO is tight, which
   * is useful with long wires or marginal cards.
   */
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd.Init.ClockDiv = 12;

  /* USER CODE END SDIO_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* 72 MHz APB1 timer clock / 72 = 1 MHz. An update every 10,000 ticks
   * provides a dedicated 100 Hz trigger for the four local ADC channels. */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 89;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration (USART6 RX) */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USER_LED_0_Pin | USER_LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_Detect_Pin */
  GPIO_InitStruct.Pin = SD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_LED_0_Pin USER_LED_1_Pin */
  GPIO_InitStruct.Pin = USER_LED_0_Pin | USER_LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_Detection_Pin */
  GPIO_InitStruct.Pin = VBUS_Detection_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_Detection_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  while (1) {
    HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
    HAL_Delay(LED_BLINK_ON_MS);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
