
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
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint64_t timestamp_us;
  uint32_t delta_us;
} Wheel_LogEvent_t;

typedef struct {
  uint32_t timestamp_ms;
  uint16_t raw[7];
} ADC_LogSample_t;

typedef struct {
  uint64_t timestamp_us;
  uint32_t timestamp_ms;
  uint32_t std_id;
  uint32_t battery_mV;
  uint16_t rpm;
  int16_t water_c;
  int16_t oil_c;
  uint16_t can_speed_kmh;
  uint16_t rl_speed_kmh;
  uint16_t adc_raw[7];
  uint8_t gear;
  uint8_t dlc;
  uint8_t data[8];
} EMU_CAN_Frame_t;

typedef struct {
  uint64_t timestamp_us;
  uint8_t channel;
  uint32_t delta_us;
  uint32_t speed_centi_kmh;
} Wheel4_LogEvent_t;

typedef struct {
  uint8_t valid;
  uint64_t timestamp_us;
  uint32_t std_id;
  uint8_t dlc;
  uint8_t data[8];
} Telemetry_CANLatest_t;

typedef struct {
  uint8_t valid;
  uint64_t timestamp_us;
  uint32_t delta_us;
  uint32_t speed_centi_kmh;
} Telemetry_WheelLatest_t;

typedef struct {
  uint8_t valid;
  uint64_t timestamp_us;
  int16_t euler_centi_deg[3];
  int16_t gyro_deci_dps[3];
  int16_t accel_milli_g[3];
  uint16_t battery_pct;
} Telemetry_IMULatest_t;

typedef struct {
  uint64_t timestamp_us;
  uint64_t adc_timestamp_us;
  uint32_t adc_sample_count;
  Telemetry_CANLatest_t can[8];
  Telemetry_WheelLatest_t wheel[4];
  Telemetry_IMULatest_t imu;
  uint16_t adc_raw[7];
  uint32_t rear_pulse_count[2];
  uint32_t rear_delta_us[2];
  uint64_t rear_timestamp_us[2];
} Telemetry_Snapshot_t;

typedef struct {
  char time[16];
  char lat[16];
  char lon[16];
  char speed[12];
  char sat_count[4];
  char fix_qual[4];
  char heading[12];
} GNSS_ParsedData_t;

typedef struct {
  uint8_t rpm_valid;
  uint8_t water_valid;
  uint8_t oil_valid;
  uint8_t speed_valid;
  uint8_t tps_valid;
  uint8_t gear_valid;
  uint8_t battery_valid;
  uint8_t fuel_valid;
  uint16_t rpm;
  int16_t water_c;
  int16_t oil_c;
  uint16_t speed_kmh;
  uint16_t tps_x10;
  uint8_t gear;
  uint32_t battery_mV;
  uint16_t fuel_used_x100;
} RaceCANLatest_t;

typedef enum {
  SD_FAULT_NONE = 0,
  SD_FAULT_NO_CARD = 1,
  SD_FAULT_MOUNT = 2,
  SD_FAULT_OPEN_TELEMETRY = 3,
  SD_FAULT_HEADER_WRITE = 4,
  SD_FAULT_HEADER_SHORT_WRITE = 5,
  SD_FAULT_HEADER_SYNC = 6,
  SD_FAULT_TELEMETRY_DRAIN = 7,
  SD_FAULT_SNAPSHOT_FORMAT = 8,
  SD_FAULT_SNAPSHOT_FLUSH = 9,
  SD_FAULT_SNAPSHOT_BUFFER_FULL = 10,
  SD_FAULT_IDLE_FLUSH = 11,
  SD_FAULT_INTERVAL_SYNC = 12,
  SD_FAULT_FLUSH_NO_ACTIVE_FILE = 13,
  SD_FAULT_FLUSH_WRITE = 14,
  SD_FAULT_FLUSH_SHORT_WRITE = 15,
  SD_FAULT_FLUSH_SYNC = 16
} SD_FaultStage_t;

typedef enum {
  SD_LOG_FLUSH_REASON_NONE = 0,
  SD_LOG_FLUSH_REASON_BUFFER_FULL = 1,
  SD_LOG_FLUSH_REASON_IDLE = 2,
  SD_LOG_FLUSH_REASON_SYNC = 3,
  SD_LOG_FLUSH_REASON_LEGACY = 4
} SD_LogFlushReason_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SD_BRINGUP_TEST_FILE "0:/DATALOG_TEST.TXT"
#define SD_BRINGUP_TEST_PAYLOAD                                                \
  "STM32F405 1-bit SDIO FATFS write/read test OK\r\nRemove card and check "    \
  "this file on your PC.\r\n"
#define SD_LED_ERR_HAL_INIT 6U
#define SD_LED_ERR_WIDE_BUS 7U
#define SD_LED_ERR_FATFS 8U
#define SD_LED_ERR_TIM 10U
#define SD_LED_ERR_MOUNT 11U
#define SD_LED_ERR_OPEN_WHEEL 13U
#define SD_LED_ERR_OPEN_ADC 13U
#define SD_LED_ERR_WRITE_HEADER 14U
#define SD_LED_ERR_SYNC 15U
#define SD_LED_ERR_WRITE_LOG 16U
#define SD_LED_ERR_ADC_CONFIG 17U
#define SD_LED_ERR_CAN_CONFIG 18U
#define SD_LED_ERR_OPEN_EMU 19U
#define SD_LED_ERR_GNSS_UART 20U
#define SD_FAULT_LOG_FILE "0:/FAULT.TXT"

/* USB Mass Storage is disabled for all logger modes.
 * The old USB MSC bring-up path exposes the same SD card to the host PC while
 * the firmware is also using FatFs. Even read-only host access can reinitialize
 * or read the card through BSP_SD at the same time as f_write()/f_sync(), which
 * can corrupt the active logger session or produce intermittent write errors.
 */
#define USB_MSC_ENABLE 0U

/* Integrated Logger Configuration.
 * Firmware runs a single unified runtime:
 *   - CAN (500Kbps) stream processing & Nextion HMI + Shift Lights update
 *   - ADC (6CH DMA) & Wheel Speed sampling
 *   - SD Card real-time CSV logging (loggedfile001.csv)
 *   - GPS (USART1 NMEA) continuous parsing & SD timestamping
 */
/* GNSS serial logging tuning.
 * USART1 is configured at 115200 8N1 by MX_USART1_UART_Init().
 * At 115200 baud the maximum payload rate is roughly 11.5 kB/s, so this 16 kB
 * queue gives the SD writer some time to recover from short FATFS write stalls.
 * The queue is placed in CCMRAM to avoid consuming the normal SRAM region.
 */
#define GNSS_RX_QUEUE_LEN 16384U
#define GNSS_RX_DRAIN_CHUNK_SIZE 128U
#define GNSS_LOG_FLUSH_IDLE_MS 5000U
#define GNSS_LOG_SYNC_INTERVAL_MS 60000U
#define SD_LOG_SYNC_FAIL_FATAL 0U
#define SD_BRINGUP_USE_4BIT_BUS 0U
#define SD_LOG_PATH_LEN 32U
#define SD_LOG_FILE_MAX_INDEX 999U
#define EMU_CAN_BASE_ID 0x600U
#define EMU_CAN_ID_MASK 0x7F8U
#define EMU_CAN_FRAME_COUNT 8U
#define EMU_CAN_RX_QUEUE_LEN 1024U
#define WHEEL_CHANNEL_COUNT 4U
#define WHEEL4_RX_QUEUE_LEN 1024U
#define ADC_CHANNEL_COUNT 7U
#define ADC_DMA_CHANNEL_COUNT 4U
#define ADC_LOG_QUEUE_LEN 256U
#define IMU_RX_DMA_BUFFER_SIZE 1024U
#define IMU_HISTORY_QUEUE_LEN 64U
#define IMU_PACKET_INTERVAL_US 20000ULL
#define IMU_PACKET_SIZE 26U
#define IMU_DATA_TIMEOUT_US 200000ULL
#define IMU_TIMESTAMP_RESYNC_THRESHOLD_US 100000LL
#define IMU_TIMESTAMP_CORRECTION_DIV 8LL
#define IMU_TIMESTAMP_CORRECTION_MAX_US 1000LL
#define WHEEL_RX_QUEUE_LEN 512U
#define TIM1_TICKS_PER_OVERFLOW 65536ULL
#define WHEEL_CIRCUMFERENCE_MM 1436ULL
#define WHEEL_TEETH_COUNT 24ULL
#define SPEED_CENTI_KMH_SCALE                                                  \
  ((uint32_t)((WHEEL_CIRCUMFERENCE_MM * 360000ULL) / WHEEL_TEETH_COUNT))
#define SPEED_MIN_VALID_DELTA_US 800U
#define SPEED_FILTER_ALPHA_NUM 1U
#define SPEED_FILTER_ALPHA_DEN 4U
#define SPEED_LOG_INTERVAL_MS 100U
#define SD_LOG_BUFFER_SIZE 16384U
#define SD_LOG_FLUSH_MARGIN 512U
#define SD_LOG_FLUSH_IDLE_MS 5000U
#define SD_LOG_SYNC_EVERY 256U
#define SD_LOG_SYNC_INTERVAL_MS 2000U
#define SD_LOG_IO_RETRY_COUNT 5U
#define SD_LOG_IO_RETRY_DELAY_MS 50U
#define SD_LOWLEVEL_IO_RETRY_COUNT 3U
#define SD_LOWLEVEL_IO_RETRY_DELAY_MS 20U
#define SD_LOWLEVEL_READY_TIMEOUT_MS 1000U
#define SD_LOWLEVEL_OP_NONE 0U
#define SD_LOWLEVEL_OP_READ 1U
#define SD_LOWLEVEL_OP_WRITE 2U
#define SD_LOG_DIAG_FORCE_SYNC 0U
#define TELEMETRY_SNAPSHOT_INTERVAL_US 10000ULL
#define TELEMETRY_SNAPSHOT_QUEUE_LEN 60U
#define TELEMETRY_UART_OUTPUT_ENABLE 0U
#define TELEMETRY_CAN_TIMEOUT_US 500000ULL
/*
 * A blocking SD/FAT operation can make the main loop catch up several 10 ms
 * snapshots at once.  CAN is drained before those snapshots are formatted,
 * so the newest frame can legitimately be a few milliseconds newer than the
 * scheduled snapshot being written.  Do not turn that healthy frame into an
 * all-zero payload merely because it is on the newer side of the snapshot.
 * Delays above 200 ms are already realigned in SD_TelemetryLoggerProcess().
 */
#define TELEMETRY_CAN_CATCHUP_TOLERANCE_US 200000ULL
#define TELEMETRY_WHEEL_TIMEOUT_US 500000ULL
#define TELEMETRY_WHEEL_CATCHUP_TOLERANCE_US 200000ULL
#define TELEMETRY_SD_MISSING_BLINK_MS 350U
#define RACE_LOG_RPM_START_THRESHOLD 1000U
#define RACE_LOG_START_DELAY_MS 2000U
#define RACE_NEXTION_RPM_UPDATE_MS 50U
#define RACE_NEXTION_TPS_UPDATE_MS 50U
#define RACE_NEXTION_GEAR_UPDATE_MS 250U
#define RACE_NEXTION_DATA_UPDATE_MS 500U
#define RACE_NEXTION_SPEED_UPDATE_MS 40U
#define RACE_NEXTION_UART_TIMEOUT_MS 30U
#define RACE_NEXTION_RAW_TX_TIMEOUT_MS 250U
#define RACE_NEXTION_TX_SPACING_MS 20U
#define RACE_NEXTION_TEST_RPM_MS 100U
#define RACE_NEXTION_TEST_TPS_MS 120U
#define RACE_NEXTION_TEST_GEAR_MS 300U
#define RACE_NEXTION_TEST_WATER_MS 600U
#define RACE_NEXTION_TEST_OIL_MS 800U
#define RACE_CAN_SIGNAL_TIMEOUT_MS 2500U
#define RACE_CAN_LED_TIMEOUT_MS 2000U
#define RACE_LOG_WRITE_TIMEOUT_MS 3000U
#define RACE_CLT_TOO_HOT_C 100
#define RACE_NEXTION_NO_SIG_REPEAT_MS 500U
#define RACE_CAN_PROCESS_BUDGET 64U
#define RACE_ACTIVITY_LED_PULSE_MS 120U
#define RACE_ACTIVITY_LED_MIN_GAP_MS 150U
#define RACE_MAIN_HEARTBEAT_MS 500U
#define RACE_LOG_FLUSH_IDLE_MS 1000U
#define RACE_LOG_SYNC_INTERVAL_MS 5000U
#define NEXTION_HMI_TEST_MODE 0U
#define NEXTION_HMI_RPM_INTERVAL_MS 50U
#define NEXTION_HMI_GEAR_INTERVAL_MS 250U
#define NEXTION_HMI_DATA_INTERVAL_MS 500U
#define TEST_RPM_SWEEP_HALF_MS 500U
#define NEXTION_HMI_UART_TIMEOUT_MS 10U
#define NEXTION_HMI_RPM_MAX 10000U
#define NEXTION_HMI_RPM_STEP 250U
#define NEXTION_HMI_SMALL_MAX 100U
#define NEXTION_HMI_SMALL_STEP 5U
#define NEXTION_RPM_TX_INTERVAL_MS 50U
#define NEXTION_RPM_CAN_ID (EMU_CAN_BASE_ID + 0U)
#define NEXTION_RPM_PROP "rpm.val"
#define WS2812_LED_COUNT 12U
#define WS2812_BITS_PER_LED 24U
#define WS2812_RESET_SLOTS 64U
#define WS2812_BUFFER_LEN                                                      \
  ((WS2812_LED_COUNT * WS2812_BITS_PER_LED) + WS2812_RESET_SLOTS)
#define WS2812_T0H_NS 350U
#define WS2812_T1H_NS 700U
#define WS2812_PERIOD_NS 1250U
#define WS2812_DMA_TIMEOUT_MS 20U
#define WS2812_UPDATE_MS 40U
#define RPM_MIN 0U
#define RPM_MAX 10000U
#define RPM_PER_LED 1000U
#define RPM_USED_LED_COUNT 10U
#define RPM_REDZONE_START 10000U
#define RPM_LED_BRIGHTNESS 90U
#define RPM_REDZONE_BLINK_MS 80U
#define RPM_LED_MODE_BAR 1U
#define RPM_LED_MODE_DEFAULT RPM_LED_MODE_BAR
#define RACE_NEXTION_DIRTY_RPM 0x01U
#define RACE_NEXTION_DIRTY_GEAR 0x02U
#define RACE_NEXTION_DIRTY_WATER 0x04U
#define RACE_NEXTION_DIRTY_TPS 0x08U
#define RACE_NEXTION_DIRTY_SPEED 0x10U
#define RACE_NEXTION_DIRTY_OIL 0x20U
#define RACE_NEXTION_DIRTY_BATTERY 0x40U
#define RACE_NEXTION_DIRTY_FUEL 0x80U
#define RACE_NEXTION_FIELD_RPM 0U
#define RACE_NEXTION_FIELD_TPS 1U
#define RACE_NEXTION_FIELD_SPEED 2U
#define RACE_NEXTION_FIELD_WATER 3U
#define RACE_NEXTION_FIELD_GEAR 4U
#define RACE_NEXTION_FIELD_OIL 5U
#define RACE_NEXTION_FIELD_BATTERY 6U
#define RACE_NEXTION_FIELD_FUEL 7U
#define RACE_NEXTION_FIELD_COUNT 8U
#define NEXTION_VIS_UNKNOWN 0U
#define NEXTION_VIS_HIDDEN 1U
#define NEXTION_VIS_VISIBLE 2U
#define LED_BLINK_ON_MS 350U
#define LED_BLINK_OFF_MS 350U
#define LED_BLINK_PAUSE_MS 1600U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAYBE_UNUSED __attribute__((unused))
#define CCMRAM_BUFFER __attribute__((section(".ccmram"), aligned(8)))
#define SDIO_ALIGNED_BUFFER __attribute__((aligned(32)))

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
static volatile FRESULT g_sd_last_fresult = FR_OK;
static volatile SD_FaultStage_t g_sd_fault_stage = SD_FAULT_NONE;
static volatile FRESULT g_sd_fault_fresult = FR_OK;
static volatile FRESULT g_sd_fault_log_fresult = FR_OK;
static volatile uint32_t g_sd_fault_detail = 0U;
static volatile uint32_t g_sd_fault_tick = 0U;
static volatile UINT g_sd_fault_buffer_len = 0U;
static volatile uint32_t g_sd_fault_unsynced_count = 0U;
static volatile uint32_t g_sd_fault_telemetry_count = 0U;
static volatile uint32_t g_sd_write_fail_count = 0U;
static volatile uint32_t g_sd_sync_fail_count = 0U;
static volatile FRESULT g_sd_last_sync_fresult = FR_OK;
static volatile uint32_t g_sd_write_call_count = 0U;
static volatile uint32_t g_sd_sync_ok_count = 0U;
static volatile uint32_t g_sd_last_write_size = 0U;
static volatile uint32_t g_sd_max_buffer_len = 0U;
static volatile uint32_t g_sd_last_write_duration_ms = 0U;
static volatile uint32_t g_sd_max_write_duration_ms = 0U;
static volatile uint64_t g_sd_total_bytes_written = 0ULL;
static volatile SD_LogFlushReason_t g_sd_last_flush_reason =
    SD_LOG_FLUSH_REASON_NONE;
static volatile uint8_t g_sd_bringup_status = 0U;
static volatile GPIO_PinState g_sd_detect_state = GPIO_PIN_RESET;
static volatile uint8_t g_sd_hal_stage = 0U;
static volatile uint32_t g_sd_hal_error_code = HAL_SD_ERROR_NONE;
static volatile uint32_t g_sd_ll_last_op = SD_LOWLEVEL_OP_NONE;
static volatile uint32_t g_sd_ll_read_call_count = 0U;
static volatile uint32_t g_sd_ll_write_call_count = 0U;
static volatile uint32_t g_sd_ll_retry_count = 0U;
static volatile uint32_t g_sd_ll_read_fail_count = 0U;
static volatile uint32_t g_sd_ll_write_fail_count = 0U;
static volatile uint32_t g_sd_ll_ready_timeout_count = 0U;
static volatile uint32_t g_sd_ll_last_sector = 0U;
static volatile uint32_t g_sd_ll_last_blocks = 0U;
static volatile uint32_t g_sd_ll_last_buffer_addr = 0U;
static volatile uint32_t g_sd_ll_last_hal_status = HAL_OK;
static volatile uint32_t g_sd_ll_last_hal_error = HAL_SD_ERROR_NONE;
static volatile uint32_t g_sd_ll_last_hal_state = HAL_SD_STATE_RESET;
static volatile uint32_t g_sd_ll_last_card_state = 0U;
static volatile uint32_t g_sd_ll_last_wait_ms = 0U;
static volatile uint32_t g_sd_ll_last_tick = 0U;
static FIL g_emu_can_file;
static FIL *g_sd_active_file = NULL;
static volatile uint32_t g_wheel_log_count = 0U;
static volatile uint32_t g_adc_log_count = 0U;
static volatile uint32_t g_emu_can_log_count = 0U;
static volatile uint32_t g_emu_can_rx_count = 0U;
static volatile uint32_t g_emu_can_rx_overflow_count = 0U;
static volatile uint32_t g_emu_can_hal_error_count = 0U;
static volatile uint8_t g_emu_can_enabled = 0U;
static volatile uint32_t g_telemetry_log_count = 0U;
static volatile uint32_t g_wheel4_log_count = 0U;
static volatile uint32_t g_adc6_log_count = 0U;
static volatile uint32_t g_wheel4_rx_overflow_count = 0U;
static volatile uint32_t g_adc6_rx_overflow_count = 0U;
static volatile uint8_t g_telemetry_queue_enabled = 0U;
static volatile uint8_t g_wheel_legacy_queue_enabled = 0U;
static volatile uint8_t g_adc_legacy_queue_enabled = 0U;
static volatile uint32_t g_adc_rx_overflow_count = 0U;
static volatile uint32_t g_adc_sample_count = 0U;
static volatile uint64_t g_adc_last_timestamp_us = 0ULL;
static volatile uint16_t g_adc_dma_buffer[ADC_DMA_CHANNEL_COUNT];
static volatile uint16_t g_emu_potentiometer4_raw = 0U;
static volatile uint16_t g_emu_potentiometer5_raw = 0U;
static volatile uint16_t g_emu_adc6_raw = 0U;
static volatile uint16_t g_adc_rx_head = 0U;
static volatile uint16_t g_adc_rx_tail = 0U;
static ADC_LogSample_t g_adc_rx_queue[ADC_LOG_QUEUE_LEN];
static volatile uint16_t g_emu_can_rx_head = 0U;
static volatile uint16_t g_emu_can_rx_tail = 0U;
static EMU_CAN_Frame_t g_emu_can_rx_queue[EMU_CAN_RX_QUEUE_LEN];
static volatile uint16_t g_wheel4_rx_head = 0U;
static volatile uint16_t g_wheel4_rx_tail = 0U;
static Wheel4_LogEvent_t g_wheel4_rx_queue[WHEEL4_RX_QUEUE_LEN] CCMRAM_BUFFER;
static volatile uint64_t g_wheel4_last_timestamp_us[WHEEL_CHANNEL_COUNT];
static volatile uint32_t g_wheel4_last_delta_us[WHEEL_CHANNEL_COUNT];
static volatile uint32_t g_wheel4_pulse_count[WHEEL_CHANNEL_COUNT];
static volatile uint32_t g_wheel4_filtered_centi_kmh[WHEEL_CHANNEL_COUNT];
static volatile uint8_t g_wheel4_filter_valid[WHEEL_CHANNEL_COUNT];
static volatile uint32_t g_wheel_rx_overflow_count = 0U;
static volatile uint32_t g_tim1_overflow_count = 0U;
static volatile uint8_t g_tim1_timebase_ready = 0U;
static volatile uint64_t g_wheel_last_timestamp_us = 0ULL;
static volatile uint16_t g_wheel_rx_head = 0U;
static volatile uint16_t g_wheel_rx_tail = 0U;
static Wheel_LogEvent_t g_wheel_rx_queue[WHEEL_RX_QUEUE_LEN];

/* PB10 (TIM2_CH3) hall-effect wheel speed sensor: the input-capture ISR
 * counts falling edges (36 teeth per revolution). Once per second the main
 * loop reads and resets that count and multiplies by 0.14
 * (= 1436mm circumference * 3.6 / 36 teeth) to get whole km/h. */
static volatile uint32_t g_pb10_edge_count = 0U;
static uint16_t g_wheel_speed_kmh = 0U;

/* Raw GNSS UART receive path.
 * The UART ISR only moves one received byte into this ring buffer and
 * immediately arms the next HAL_UART_Receive_IT() call. The main loop drains
 * this queue into the shared SD write buffer. Keeping FATFS out of the
 * interrupt path is important: SD writes can block for milliseconds and would
 * otherwise lose UART bytes or break other interrupt timing.
 */
static volatile uint8_t g_gnss_rx_byte = 0U;
static volatile uint16_t g_gnss_rx_head = 0U;
static volatile uint16_t g_gnss_rx_tail = 0U;
static volatile uint32_t g_gnss_rx_count = 0U;
static volatile uint32_t g_gnss_rx_overflow_count = 0U;
static volatile uint32_t g_gnss_uart_error_count = 0U;
static volatile uint32_t g_gnss_drain_call_count = 0U;
static volatile uint64_t g_gnss_drain_byte_count = 0ULL;
static volatile uint8_t g_gnss_rx_queue[GNSS_RX_QUEUE_LEN] CCMRAM_BUFFER;

static volatile uint32_t g_gnss_rx_byte_count = 0U;
static volatile uint32_t g_gnss_dollar_count = 0U;
static volatile uint32_t g_gnss_0xB5_count = 0U;
static volatile uint32_t g_gnss_fix_update_count = 0U;
static volatile uint64_t g_gnss_last_fix_timestamp_us = 0ULL;

static uint8_t g_imu_rx_dma_buffer[IMU_RX_DMA_BUFFER_SIZE];
static uint16_t g_imu_rx_read_pos = 0U;
static uint8_t g_imu_packet_buffer[IMU_PACKET_SIZE];
static uint8_t g_imu_packet_len = 0U;
static volatile uint32_t g_imu_packet_count = 0U;
static volatile uint32_t g_imu_checksum_error_count = 0U;
static volatile uint32_t g_imu_rx_error_count = 0U;
static volatile uint32_t g_imu_rx_byte_count = 0U;
static volatile uint32_t g_imu_uart_parity_error_count = 0U;
static volatile uint32_t g_imu_uart_noise_error_count = 0U;
static volatile uint32_t g_imu_uart_framing_error_count = 0U;
static volatile uint32_t g_imu_uart_overrun_error_count = 0U;
static volatile uint32_t g_imu_uart_dma_error_count = 0U;
static volatile uint8_t g_imu_rearm_pending = 0U;
static Telemetry_IMULatest_t g_imu_latest;
static Telemetry_IMULatest_t g_imu_history[IMU_HISTORY_QUEUE_LEN];
static uint16_t g_imu_history_head = 0U;
static uint16_t g_imu_history_count = 0U;
static volatile uint16_t g_imu_parse_batch_max = 0U;
static volatile uint64_t g_imu_last_rx_timestamp_us = 0ULL;
static volatile int32_t g_imu_timestamp_error_us = 0;
static volatile uint32_t g_imu_estimated_missing_count = 0U;
static volatile uint32_t g_imu_resync_count = 0U;
static volatile uint32_t g_imu_timeout_count = 0U;
static volatile uint32_t g_imu_recovery_count = 0U;
static volatile uint8_t g_imu_link_valid = 0U;

/* Shared SD staging buffer.
 * Telemetry CSV and raw GNSS logging both reuse this buffer, but only one
 * logger mode is active at a time. g_sd_active_file selects the currently open
 * FATFS file, and SD_PulseLoggerFlush() writes this buffer to that file.
 */
static char g_sd_log_buffer[SD_LOG_BUFFER_SIZE] SDIO_ALIGNED_BUFFER;
static char g_sd_log_path[SD_LOG_PATH_LEN];
static UINT g_sd_log_buffer_len = 0U;
static uint32_t g_sd_log_last_write_tick = 0U;
static uint32_t g_sd_log_last_sync_tick = 0U;
static uint32_t g_sd_log_unsynced_count = 0U;
static Telemetry_CANLatest_t g_telemetry_latest_can[EMU_CAN_FRAME_COUNT];
static Telemetry_WheelLatest_t g_telemetry_latest_wheel[WHEEL_CHANNEL_COUNT];
static uint16_t g_telemetry_latest_adc_raw[ADC_CHANNEL_COUNT];
static uint8_t g_telemetry_adc_seen = 0U;
static uint64_t g_telemetry_next_snapshot_us = 0ULL;
static uint32_t g_telemetry_snapshot_seq = 0U;
static volatile uint32_t g_telemetry_missed_snapshot_count = 0U;
static volatile uint16_t g_telemetry_snapshot_head = 0U;
static volatile uint16_t g_telemetry_snapshot_tail = 0U;
static volatile uint16_t g_telemetry_snapshot_max_depth = 0U;
static volatile uint32_t g_telemetry_snapshot_overflow_count = 0U;
static Telemetry_Snapshot_t
    g_telemetry_snapshot_queue[TELEMETRY_SNAPSHOT_QUEUE_LEN];
static GNSS_ParsedData_t g_gnss_parsed = {.time = "00:00:00.00",
                                          .lat = "0.0",
                                          .lon = "0.0",
                                          .speed = "0.0",
                                          .sat_count = "0",
                                          .fix_qual = "0",
                                          .heading = "0.0"};
volatile uint32_t g_gps_fat_time = 0U;
static RaceCANLatest_t g_race_can_latest;
static uint32_t g_race_nextion_last_rpm_tick = 0U;
static uint32_t g_race_nextion_last_gear_tick = 0U;
static uint32_t g_race_nextion_last_water_tick = 0U;
static uint32_t g_race_nextion_last_tps_tick = 0U;
static uint32_t g_race_nextion_last_speed_tick = 0U;
static uint32_t g_race_nextion_last_oil_tick = 0U;
static uint32_t g_race_nextion_last_battery_tick = 0U;
static uint32_t g_race_nextion_last_fuel_tick = 0U;
static volatile uint16_t g_race_nextion_last_rpm_sent = 0U;
static volatile uint32_t g_race_nextion_rpm_tx_count = 0U;
static volatile uint16_t g_race_nextion_last_tps_sent = 0U;
static volatile uint32_t g_race_nextion_tps_tx_count = 0U;
static volatile uint8_t g_race_nextion_last_gear_sent = 0U;
static volatile uint32_t g_race_nextion_gear_tx_count = 0U;
static volatile int16_t g_race_nextion_last_water_sent = 0;
static volatile uint32_t g_race_nextion_water_tx_count = 0U;
static volatile uint16_t g_race_nextion_last_speed_sent = 0U;
static volatile uint32_t g_race_nextion_speed_tx_count = 0U;
static uint8_t g_race_nextion_rr_index = RACE_NEXTION_FIELD_RPM;
static uint32_t g_race_nextion_next_tx_tick = 0U;
static uint32_t g_race_nextion_last_no_sig_tick = 0U;
static volatile uint8_t g_race_nextion_dirty_mask = 0U;
static uint8_t g_race_can_signal_lost = 1U;
static volatile uint32_t g_race_raw_can_seen_count = 0U;
static volatile uint32_t g_race_can_seen_count = 0U;
static volatile uint32_t g_race_last_raw_can_rx_tick = 0U;
static volatile uint32_t g_race_last_can_rx_tick = 0U;
static uint32_t g_race_last_nextion_tx_tick = 0U;
static volatile uint32_t g_race_log_count = 0U;
static volatile uint32_t g_race_start_trigger_count = 0U;
static volatile uint32_t g_race_nextion_tx_attempt_count = 0U;
static volatile uint32_t g_race_nextion_tx_count = 0U;
static volatile uint32_t g_race_nextion_tx_fail_count = 0U;
static volatile uint32_t g_race_nextion_uart_recover_count = 0U;
static volatile uint32_t g_race_nextion_uart_error_callback_count = 0U;
static volatile uint32_t g_race_nextion_uart_last_error = 0U;
static volatile uint32_t g_race_nextion_uart_last_state = 0U;
static uint8_t g_race_nextion_tx_buffer[64];
static volatile uint16_t g_race_nextion_tx_len = 0U;
static volatile uint16_t g_race_nextion_tx_pos = 0U;
static volatile uint8_t g_race_nextion_tx_active = 0U;
static uint32_t g_race_nextion_tx_start_tick = 0U;
static volatile uint32_t g_race_nextion_tx_busy_count = 0U;
static volatile uint8_t g_emu_can_rx_irq_enabled = 0U;
static uint8_t g_nextion_logok_visibility = NEXTION_VIS_UNKNOWN;
static uint8_t g_nextion_logoff_visibility = NEXTION_VIS_UNKNOWN;
static uint8_t g_nextion_toohot_visibility = NEXTION_VIS_UNKNOWN;
static uint16_t g_nextion_latest_rpm = 0U;
static uint8_t g_nextion_rpm_valid = 0U;
static uint32_t g_nextion_last_tx_tick = 0U;
static uint16_t g_nextion_hmi_rpm = 0U;
static volatile uint32_t g_nextion_hmi_tx_count = 0U;
static volatile uint32_t g_nextion_hmi_tx_fail_count = 0U;
static volatile uint16_t g_current_rpm = 0U;
static uint8_t g_rpm_led_mode = RPM_LED_MODE_DEFAULT;
static uint32_t g_ws2812_last_update_tick = 0U;
static uint16_t g_ws2812_pwm_buffer[WS2812_BUFFER_LEN];
static volatile uint8_t g_ws2812_dma_ready = 1U;
static volatile uint32_t g_ws2812_dma_done_count = 0U;
static volatile uint32_t g_ws2812_dma_start_fail_count = 0U;
static volatile uint32_t g_ws2812_dma_busy_skip_count = 0U;
static uint16_t g_ws2812_pwm_0 = 0U;
static uint16_t g_ws2812_pwm_1 = 0U;

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
static GPIO_PinState SD_IsCardInserted(void);
static FRESULT SD_PulseLoggerFlush(uint8_t force_sync);
static FRESULT SD_LogWriterFlush(uint8_t force_sync,
                                 SD_LogFlushReason_t reason);
static void Wheel_QueuePushFromIsr(const Wheel_LogEvent_t *event);
static uint8_t ADC_LoggerConfigureHardware(void) MAYBE_UNUSED;
static void ADC_QueuePushFromIsr(const uint16_t *raw);
static uint8_t EMU_CANLoggerConfigureHardware(void) MAYBE_UNUSED;
static void EMU_CANLoggerStart(void) MAYBE_UNUSED;
static void USER_CAN1_Apply500Kbps(void);
static void EMU_CANDrainRxFifo(void);
static uint8_t EMU_CANQueuePop(EMU_CAN_Frame_t *frame);
static void EMU_CANQueuePushFromIsr(const CAN_RxHeaderTypeDef *rx_header,
                                    const uint8_t *data);
static void TelemetryLoggerStart(void) MAYBE_UNUSED;
static uint64_t Telemetry_GetTimestampUs(void);
static uint8_t Wheel4_ChannelIndex(const TIM_HandleTypeDef *htim);
static uint32_t Wheel4_HALChannel(uint8_t channel);
static TIM_HandleTypeDef *Wheel4_HALTimer(uint8_t channel);
static void Wheel4_QueuePushFromIsr(const Wheel4_LogEvent_t *event);
static uint8_t Wheel4_QueuePop(Wheel4_LogEvent_t *event);
static void SD_WaitForCardInserted(void) MAYBE_UNUSED;
static FRESULT SD_TelemetryLoggerInit(void) MAYBE_UNUSED;
static FRESULT SD_OpenNextTelemetryLogFile(void);
static FRESULT SD_TelemetryLoggerProcess(void) MAYBE_UNUSED;
static FRESULT SD_TelemetryLoggerDrainInputs(void);
static FRESULT SD_TelemetryLoggerAppendSnapshot(
    const Telemetry_Snapshot_t *snapshot);
static void Telemetry_SnapshotPushFromIsr(void);
static uint8_t Telemetry_SnapshotPeek(Telemetry_Snapshot_t *snapshot);
static uint8_t Telemetry_SnapshotPop(Telemetry_Snapshot_t *snapshot);
static uint8_t Nextion_SendCommand(const char *command);
static uint8_t Nextion_SendTextCommand(const char *component, const char *text);
static uint8_t Nextion_SendVisibility(const char *component, uint8_t visible);
static void Nextion_UARTPump(void);
static void Nextion_RecoverUART4(void);
static void RPMOutputs_Init(void);
static void RPMOutputs_Process(void);
static void RPMOutputs_SetRPM(uint16_t rpm);
static void WS2812_InitTimings(void);
static uint8_t PercentToByte(uint8_t percent);
static void WS2812_SetLedBits(uint32_t *index, uint8_t red, uint8_t green,
                              uint8_t blue);
static void WS2812_ShowRPM(uint16_t rpm, uint8_t mode);
static void WS2812_ShowRPMBar(uint16_t rpm);
static void WS2812_SendBuffer(uint32_t used_len);
static void RaceLoggerUpdateLatestFromCAN(uint32_t std_id, uint8_t dlc,
                                          const uint8_t *data);
static void RaceNextionProcess(void);
static void RaceNextionGetSnapshot(RaceCANLatest_t *latest,
                                   uint8_t *dirty_mask);
static void RaceNextionClearDirty(uint8_t dirty_bit);
static uint8_t RaceNextionSendNumber(const char *component, int32_t value);
static uint8_t RaceNextionSendGear(uint8_t gear);
static void GNSSLoggerStart(void) MAYBE_UNUSED;
void GNSS_QueuePushFromIsr(uint8_t byte);
static void GNSS_ProcessParser(void);
static void IMU_StartReceiver(void);
static void IMU_ProcessReceiver(void);
static void IMU_ParseByte(uint8_t byte);
static uint8_t IMU_HistoryGetAt(uint64_t timestamp_us,
                                Telemetry_IMULatest_t *sample);
static uint8_t GNSS_QueuePop(uint8_t *byte) MAYBE_UNUSED;
static void SD_RecordFault(SD_FaultStage_t stage, FRESULT result,
                           uint32_t detail);
static void SD_WriteFaultLog(void);
static const char *SD_FaultStageName(SD_FaultStage_t stage);
static void SD_RecoverAfterIoError(void);
static uint8_t SD_WaitForTransferReady(uint32_t timeout_ms);
static void LED_Set(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
static void LED_BlinkCode(uint8_t code);
static void LED_BlinkCodeDetail(uint8_t code, uint8_t detail);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void USER_CAN1_Apply500Kbps(void) {
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
static void SD_RecordFault(SD_FaultStage_t stage, FRESULT result,
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

static void SD_WriteFaultLog(void) {
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

static const char *SD_FaultStageName(SD_FaultStage_t stage) {
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

static void SD_RecoverAfterIoError(void) {
  HAL_SD_Abort(&hsd);
  (void)HAL_SD_DeInit(&hsd);
  HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
  (void)HAL_SD_Init(&hsd);

  if (SD_BRINGUP_USE_4BIT_BUS != 0U) {
    (void)HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
  }

  HAL_Delay(SD_LOG_IO_RETRY_DELAY_MS);
}

static uint8_t SD_WaitForTransferReady(uint32_t timeout_ms) {
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

static void RaceLoggerUpdateLatestFromCAN(uint32_t std_id, uint8_t dlc,
                                          const uint8_t *data) {
  // NS26F와 동일하게: 함수 진입 즉시 수신 카운터 및 타임스탬프 갱신
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

static void RaceNextionProcess(void) {
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

  /* Establish a known, safe display state before showing runtime status. */
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

  /* Hide the opposite object first so logok/logoff can never overlap. */
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
    /* Do not overwrite NO SIG with stale values from the last CAN frame. */
    goto transmit_done;
  }

  /* Mark every field dirty only when the CAN signal has actually recovered. */
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

  /* Gear changes must never wait behind continuously changing RPM/TPS/speed.
   * After an urgent gear update, use round-robin service for every field so
   * no high-rate signal can starve another display value. */
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

static void RaceNextionGetSnapshot(RaceCANLatest_t *latest,
                                   uint8_t *dirty_mask) {
  __disable_irq();
  *latest = g_race_can_latest;
  *dirty_mask = g_race_nextion_dirty_mask;
  __enable_irq();
}

static void RaceNextionClearDirty(uint8_t dirty_bit) {
  __disable_irq();
  g_race_nextion_dirty_mask &= (uint8_t)~dirty_bit;
  __enable_irq();
}

static uint8_t RaceNextionSendNumber(const char *component, int32_t value) {
  char text[16];

  (void)snprintf(text, sizeof(text), "%ld", (long)value);
  return Nextion_SendTextCommand(component, text);
}

static uint8_t RaceNextionSendGear(uint8_t gear) {
  if (gear == 0U) {
    return Nextion_SendTextCommand("t_gear", "N");
  }

  return RaceNextionSendNumber("t_gear", (int32_t)gear);
}

static uint8_t EMU_CANLoggerConfigureHardware(void) {
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

static void EMU_CANLoggerStart(void) {
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

static void EMU_CANDrainRxFifo(void) {
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
    /* USART1 is connected to the GNSS module TX pin. Store exactly the byte
     * received by the module, then re-arm reception for the next byte.
     */
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
    /* Framing, parity, noise, or overrun errors can stop byte reception in the
     * HAL state machine. Count the event and immediately try to resume logging
     * so a noisy GNSS line does not permanently stop the file.
     */
    g_gnss_uart_error_count++;
    if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_gnss_rx_byte, 1U) !=
        HAL_OK) {
      g_gnss_uart_error_count++;
    }
  }

  if (huart->Instance == USART6) {
    /* Defer DMA recovery to the foreground logger; only genuine UART/DMA
     * failures reach this callback. Normal circular-buffer completion is
     * handled by HAL_UART_RxCpltCallback and must not restart reception. */
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

static void EMU_CANQueuePushFromIsr(const CAN_RxHeaderTypeDef *rx_header,
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
  /* DMA scans ADC1/4/5/6 only. Preserve the logical ADC channel numbers. */
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

static uint8_t EMU_CANQueuePop(EMU_CAN_Frame_t *frame) {
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

static void TelemetryLoggerStart(void) {
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

  /* Align snapshot scheduling with the active runtime timestamp source. */
  g_telemetry_next_snapshot_us =
      Telemetry_GetTimestampUs() + TELEMETRY_SNAPSHOT_INTERVAL_US;
}

static uint64_t Telemetry_GetTimestampUs(void) {
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

static uint8_t Wheel4_ChannelIndex(const TIM_HandleTypeDef *htim) {
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

static uint32_t Wheel4_HALChannel(uint8_t channel) {
  static const uint32_t hal_channels[WHEEL_CHANNEL_COUNT] = {
      TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};

  return hal_channels[channel];
}

static TIM_HandleTypeDef *Wheel4_HALTimer(uint8_t channel) {
  return (channel == 0U) ? &htim1 : &htim2;
}

static void Wheel4_QueuePushFromIsr(const Wheel4_LogEvent_t *event) {
  uint16_t next_head =
      (uint16_t)((g_wheel4_rx_head + 1U) % WHEEL4_RX_QUEUE_LEN);

  if (next_head == g_wheel4_rx_tail) {
    g_wheel4_rx_overflow_count++;
    return;
  }

  g_wheel4_rx_queue[g_wheel4_rx_head] = *event;
  g_wheel4_rx_head = next_head;
}

static uint8_t Wheel4_QueuePop(Wheel4_LogEvent_t *event) {
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

static int16_t IMU_ReadS16BE(const uint8_t *data) {
  return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static void Telemetry_FormatU64(uint64_t value, char *output,
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

static void IMU_StartReceiver(void) {
  if (HAL_UART_Receive_DMA(&huart6, g_imu_rx_dma_buffer,
                           IMU_RX_DMA_BUFFER_SIZE) != HAL_OK) {
    g_imu_rx_error_count++;
    g_imu_rearm_pending = 1U;
  } else {
    g_imu_rearm_pending = 0U;
  }
}

static void IMU_ParseByte(uint8_t byte) {
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
    /* Packet layout for sof=1, sog=1, soa=2, sob=1, sots=0:
     * SOP(2), channel, id, Euler(3), gyro(3), accel(3), battery, checksum.
     * Euler is intentionally skipped; the logger retains vehicle-dynamics
     * channels only. All 16-bit fields are big-endian. */
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

static void IMU_ProcessReceiver(void) {
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

static uint8_t IMU_HistoryGetAt(uint64_t timestamp_us,
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

static void SD_WaitForCardInserted(void) {
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

static FRESULT SD_TelemetryLoggerInit(void) {
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

static FRESULT SD_OpenNextTelemetryLogFile(void) {
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

static void GNSSLoggerStart(void) {
  /* Reset runtime counters after the file is open, then start interrupt-driven
   * byte reception. The actual SD writing happens in SD_GNSSLoggerProcess().
   */
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

  // Clear flags and enable raw RXNE interrupt bypassing HAL UART receiver
  (void)huart1.Instance->SR;
  (void)huart1.Instance->DR;
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void GNSS_QueuePushFromIsr(uint8_t byte) {
  uint16_t next_head = (uint16_t)((g_gnss_rx_head + 1U) % GNSS_RX_QUEUE_LEN);

  /* Drop the newest byte if the main loop cannot keep up. The overflow counter
   * makes this visible through LED1 and during debugger inspection.
   */
  if (next_head == g_gnss_rx_tail) {
    g_gnss_rx_overflow_count++;
    return;
  }

  g_gnss_rx_queue[g_gnss_rx_head] = byte;
  g_gnss_rx_head = next_head;
  g_gnss_rx_count++;
}

static uint8_t GNSS_QueuePop(uint8_t *byte) {
  uint8_t has_byte = 0U;

  /* Head/tail are shared with the UART ISR. Disable interrupts only for the
   * pointer update, not for SD writes.
   */
  __disable_irq();
  if (g_gnss_rx_tail != g_gnss_rx_head) {
    *byte = g_gnss_rx_queue[g_gnss_rx_tail];
    g_gnss_rx_tail = (uint16_t)((g_gnss_rx_tail + 1U) % GNSS_RX_QUEUE_LEN);
    has_byte = 1U;
  }
  __enable_irq();

  return has_byte;
}

static void NMEA_GetField(const char *line, uint8_t field_idx, char *out_buf,
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

static void GNSS_ParseLine(const char *line) {
  char field[24];

  // 각도 데이터 패킷 유입 전 공란 방지를 위한 기본값 예외처리
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

      // Convert UTC to KST (UTC + 9)
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
          (int32_t)(kmh * 10.0f + 0.5f); // 0.1 km/h 단위로 반올림 변환
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

static void GNSS_ProcessParser(void) {
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

static FRESULT SD_TelemetryLoggerProcess(void) {
  FRESULT res = FR_OK;
  uint32_t now;
  Telemetry_Snapshot_t snapshot;

  /* Service the dashboard before any CSV formatting or blocking SD/UART2
   * operation. UART4 transmission is byte-pumped and must remain responsive. */
  Nextion_UARTPump();
  RaceNextionProcess();
  Nextion_UARTPump();

  IMU_ProcessReceiver();

  // CAN 인터럽트를 비활성화했으므로 루프마다 직접 RxFifo를 긁어와 큐에
  // 채워줍니다.
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
    /* An SD write can block inside the previous append while ADC snapshots and
     * USART6 DMA continue. Parse that accumulated IMU data before selecting
     * the IMU sample for the next queued 100 Hz snapshot. */
    IMU_ProcessReceiver();
    if (Telemetry_SnapshotPeek(&snapshot) == 0U) {
      break;
    }

    /* USART6 receives wireless IMU packets in bursts of up to several
     * samples.  Do not commit a 100 Hz row until the 50 Hz IMU timeline has
     * reached that row.  Otherwise a late burst can contain samples whose
     * timestamps belong to CSV rows that have already been written, causing
     * those intermediate samples to disappear from the log.
     *
     * If the IMU is disconnected, release the snapshot after the normal IMU
     * data timeout.  This bounds the added logging latency and prevents an IMU
     * fault from stopping ADC/CAN/wheel logging. */
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

  // Nextion 디스플레이 비동기 전송 처리 및 레지스터 펌프 가동
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

static FRESULT SD_TelemetryLoggerDrainInputs(void) {
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

      /* Replace the two unwired STM32 ADC inputs with EMU analog inputs.
       * EMU 0x601 bytes 6..7 = AIN4, 0x606 bytes 0..1 = AIN5. */
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

    // Ecumaster EMU Black CAN 파싱 및 디스플레이 데이터 갱신 함수 연동
    RaceLoggerUpdateLatestFromCAN(can_frame.std_id, can_frame.dlc,
                                  can_frame.data);

    // 카운터/타임스탬프는 RaceLoggerUpdateLatestFromCAN() 내부에서 갱신됨

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

static uint8_t Nextion_SendCommand(const char *command) {
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

static uint8_t Nextion_SendTextCommand(const char *component,
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

static uint8_t Nextion_SendVisibility(const char *component, uint8_t visible) {
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

static void Nextion_UARTPump(void) {
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

static void Nextion_RecoverUART4(void) {
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

static void RPMOutputs_Init(void) {
  WS2812_InitTimings();
  g_ws2812_dma_ready = 1U;
  g_ws2812_last_update_tick = 0U;
  RPMOutputs_SetRPM(g_current_rpm);
  WS2812_ShowRPM(g_current_rpm, g_rpm_led_mode);
}

static void RPMOutputs_Process(void) {
  uint32_t now = HAL_GetTick();

  if ((now - g_ws2812_last_update_tick) < WS2812_UPDATE_MS) {
    return;
  }

  g_ws2812_last_update_tick = now;
  WS2812_ShowRPM(g_current_rpm, g_rpm_led_mode);
}

static void RPMOutputs_SetRPM(uint16_t rpm) {
  if (rpm > RPM_MAX) {
    rpm = RPM_MAX;
  }

  g_current_rpm = rpm;
  g_nextion_hmi_rpm = rpm;
  g_nextion_latest_rpm = rpm;
  g_nextion_rpm_valid = 1U;
}

static void WS2812_InitTimings(void) {
  uint32_t timer_period = htim8.Init.Period + 1U;

  g_ws2812_pwm_0 =
      (uint16_t)(((timer_period * WS2812_T0H_NS) + (WS2812_PERIOD_NS / 2U)) /
                 WS2812_PERIOD_NS);
  g_ws2812_pwm_1 =
      (uint16_t)(((timer_period * WS2812_T1H_NS) + (WS2812_PERIOD_NS / 2U)) /
                 WS2812_PERIOD_NS);
}

static uint8_t PercentToByte(uint8_t percent) {
  if (percent > 100U) {
    percent = 100U;
  }

  return (uint8_t)(((uint32_t)255U * percent) / 100U);
}

static void WS2812_SetLedBits(uint32_t *index, uint8_t red, uint8_t green,
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

static void WS2812_ShowRPM(uint16_t rpm, uint8_t mode) {
  (void)mode;
  WS2812_ShowRPMBar(rpm);
}

static void WS2812_ShowRPMBar(uint16_t rpm) {
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
    /* Driver view is left to right: LED 12, 11, ..., 2, 1. */
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

  /* LED 12 reports CAN health independently from the RPM bar. */
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

  /* LED 11 reports coolant temperature: green, orange, then red. */
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

static void WS2812_SendBuffer(uint32_t used_len) {
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

static FRESULT SD_TelemetryLoggerAppendSnapshot(
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

  /* RR is wheel channel 2 and RL is wheel channel 3.  Copy the ISR-owned
   * diagnostic state atomically so every CSV row contains a coherent set. */
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
        /*
         * The row is a catch-up snapshot produced after a short SD stall.
         * Its latest CAN value was received while the main loop was stalled,
         * not lost.  Accept only the bounded look-ahead window so a corrupt
         * timestamp still cannot keep arbitrary data valid forever.
         */
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
        /*
         * A wheel edge can be drained after a short SD stall while the logger
         * is still formatting older scheduled rows.  Treat that bounded
         * look-ahead as valid instead of writing a false zero-speed sample.
         */
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
  /* Optional PC serial monitor output. Disabled in the vehicle build because
   * transmitting a full CSV row at 115200 bps blocks the dashboard service. */
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

static uint8_t ADC_LoggerConfigureHardware(void) {
  static const uint32_t adc_channels[ADC_DMA_CHANNEL_COUNT] = {
      ADC_CHANNEL_10, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15};
  ADC_ChannelConfTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  if (HAL_ADC_DeInit(&hadc1) != HAL_OK) {
    return 0U;
  }

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
    return 0U;
  }

  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  for (uint32_t i = 0U; i < ADC_DMA_CHANNEL_COUNT; i++) {
    sConfig.Channel = adc_channels[i];
    sConfig.Rank = (uint32_t)(i + 1U);
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
      return 0U;
    }
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    return 0U;
  }

  return 1U;
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

static void Telemetry_SnapshotPushFromIsr(void) {
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

static uint8_t Telemetry_SnapshotPop(Telemetry_Snapshot_t *snapshot) {
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

static uint8_t Telemetry_SnapshotPeek(Telemetry_Snapshot_t *snapshot) {
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

static void ADC_QueuePushFromIsr(const uint16_t *raw) {
  ADC_LogSample_t sample;
  uint16_t next_head = (uint16_t)((g_adc_rx_head + 1U) % ADC_LOG_QUEUE_LEN);

  if (next_head == g_adc_rx_tail) {
    g_adc_rx_overflow_count++;
    return;
  }

  sample.timestamp_ms = HAL_GetTick();
  memset(sample.raw, 0, sizeof(sample.raw));
  sample.raw[0] = raw[0];
  sample.raw[1] = g_emu_potentiometer4_raw;
  sample.raw[2] = g_emu_potentiometer5_raw;
  sample.raw[3] = raw[1];
  sample.raw[4] = raw[2];
  sample.raw[5] = raw[3];
  sample.raw[6] = g_emu_adc6_raw;

  g_adc_rx_queue[g_adc_rx_head] = sample;
  g_adc_rx_head = next_head;
}

static void Wheel_QueuePushFromIsr(const Wheel_LogEvent_t *event) {
  uint16_t next_head = (uint16_t)((g_wheel_rx_head + 1U) % WHEEL_RX_QUEUE_LEN);

  if (next_head == g_wheel_rx_tail) {
    g_wheel_rx_overflow_count++;
    return;
  }

  g_wheel_rx_queue[g_wheel_rx_head] = *event;
  g_wheel_rx_head = next_head;
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

static GPIO_PinState SD_IsCardInserted(void) {
  return (HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin) ==
          GPIO_PIN_RESET)
             ? GPIO_PIN_SET
             : GPIO_PIN_RESET;
}

static FRESULT SD_LogWriterFlush(uint8_t force_sync,
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

static FRESULT SD_PulseLoggerFlush(uint8_t force_sync) {
  return SD_LogWriterFlush(force_sync, (force_sync != 0U)
                                           ? SD_LOG_FLUSH_REASON_SYNC
                                           : SD_LOG_FLUSH_REASON_LEGACY);
}

static void LED_Set(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
  HAL_GPIO_WritePin(port, pin, state);
}

static void LED_BlinkCode(uint8_t code) { LED_BlinkCodeDetail(code, 0U); }

static void LED_BlinkCodeDetail(uint8_t code, uint8_t detail) {
  if (code == 0U) {
    code = 1U;
  }

  LED_Set(USER_LED_0_GPIO_Port, USER_LED_0_Pin, GPIO_PIN_RESET);
  LED_Set(USER_LED_1_GPIO_Port, USER_LED_1_Pin, GPIO_PIN_RESET);

  while (1) {
    for (uint8_t i = 0U; i < code; i++) {
      HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
      HAL_Delay(LED_BLINK_ON_MS);
      HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
      HAL_Delay(LED_BLINK_OFF_MS);
    }

    if (detail != 0U) {
      HAL_Delay(LED_BLINK_PAUSE_MS);
      for (uint8_t i = 0U; i < detail; i++) {
        HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
        HAL_Delay(LED_BLINK_ON_MS);
        HAL_GPIO_TogglePin(USER_LED_1_GPIO_Port, USER_LED_1_Pin);
        HAL_Delay(LED_BLINK_OFF_MS);
      }
    }

    HAL_Delay(LED_BLINK_PAUSE_MS);
  }
}

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
