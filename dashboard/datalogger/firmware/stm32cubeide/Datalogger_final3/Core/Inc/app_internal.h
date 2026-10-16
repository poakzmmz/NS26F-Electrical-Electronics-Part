#ifndef APP_INTERNAL_H
#define APP_INTERNAL_H

#include "main.h"
#include "fatfs.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

  "this file on your PC.\r\n"
#define SD_LED_ERR_HAL_INIT 6U
#define SD_LED_ERR_WIDE_BUS 7U
#define SD_LED_ERR_FATFS 8U
#define SD_LED_ERR_MOUNT 11U
#define SD_LED_ERR_WRITE_HEADER 14U
#define SD_LED_ERR_SYNC 15U
#define SD_LED_ERR_WRITE_LOG 16U
#define SD_LED_ERR_ADC_CONFIG 17U
#define SD_LED_ERR_OPEN_EMU 19U
#define SD_FAULT_LOG_FILE "0:/FAULT.TXT"

#define GNSS_RX_QUEUE_LEN 16384U
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
#define SD_LOG_BUFFER_SIZE 16384U
#define SD_LOG_FLUSH_IDLE_MS 5000U
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

#define TELEMETRY_CAN_CATCHUP_TOLERANCE_US 200000ULL
#define TELEMETRY_WHEEL_TIMEOUT_US 500000ULL
#define TELEMETRY_WHEEL_CATCHUP_TOLERANCE_US 200000ULL
#define TELEMETRY_SD_MISSING_BLINK_MS 350U
#define RACE_NEXTION_RPM_UPDATE_MS 50U
#define RACE_NEXTION_TPS_UPDATE_MS 50U
#define RACE_NEXTION_GEAR_UPDATE_MS 250U
#define RACE_NEXTION_DATA_UPDATE_MS 500U
#define RACE_NEXTION_SPEED_UPDATE_MS 40U
#define RACE_NEXTION_RAW_TX_TIMEOUT_MS 250U
#define RACE_NEXTION_TX_SPACING_MS 20U
#define RACE_CAN_SIGNAL_TIMEOUT_MS 2500U
#define RACE_CAN_LED_TIMEOUT_MS 2000U
#define RACE_LOG_WRITE_TIMEOUT_MS 3000U
#define RACE_CLT_TOO_HOT_C 100
#define RACE_NEXTION_NO_SIG_REPEAT_MS 500U
#define NEXTION_RPM_CAN_ID (EMU_CAN_BASE_ID + 0U)
#define WS2812_LED_COUNT 12U
#define WS2812_BITS_PER_LED 24U
#define WS2812_RESET_SLOTS 64U
#define WS2812_BUFFER_LEN                                                      \
  ((WS2812_LED_COUNT * WS2812_BITS_PER_LED) + WS2812_RESET_SLOTS)
#define WS2812_T0H_NS 350U
#define WS2812_T1H_NS 700U
#define WS2812_PERIOD_NS 1250U
#define WS2812_UPDATE_MS 40U
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

#define CCMRAM_BUFFER __attribute__((section(".ccmram"), aligned(8)))
#define SDIO_ALIGNED_BUFFER __attribute__((aligned(32)))

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern SD_HandleTypeDef hsd;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

extern volatile FRESULT g_sd_last_fresult;
extern volatile SD_FaultStage_t g_sd_fault_stage;
extern volatile FRESULT g_sd_fault_fresult;
extern volatile FRESULT g_sd_fault_log_fresult;
extern volatile uint32_t g_sd_fault_detail;
extern volatile uint32_t g_sd_fault_tick;
extern volatile UINT g_sd_fault_buffer_len;
extern volatile uint32_t g_sd_fault_unsynced_count;
extern volatile uint32_t g_sd_fault_telemetry_count;
extern volatile uint32_t g_sd_write_fail_count;
extern volatile uint32_t g_sd_sync_fail_count;
extern volatile FRESULT g_sd_last_sync_fresult;
extern volatile uint32_t g_sd_write_call_count;
extern volatile uint32_t g_sd_sync_ok_count;
extern volatile uint32_t g_sd_last_write_size;
extern volatile uint32_t g_sd_max_buffer_len;
extern volatile uint32_t g_sd_last_write_duration_ms;
extern volatile uint32_t g_sd_max_write_duration_ms;
extern volatile uint64_t g_sd_total_bytes_written;
extern volatile SD_LogFlushReason_t g_sd_last_flush_reason;
extern volatile uint8_t g_sd_bringup_status;
extern volatile GPIO_PinState g_sd_detect_state;
extern volatile uint8_t g_sd_hal_stage;
extern volatile uint32_t g_sd_hal_error_code;
extern volatile uint32_t g_sd_ll_last_op;
extern volatile uint32_t g_sd_ll_read_call_count;
extern volatile uint32_t g_sd_ll_write_call_count;
extern volatile uint32_t g_sd_ll_retry_count;
extern volatile uint32_t g_sd_ll_read_fail_count;
extern volatile uint32_t g_sd_ll_write_fail_count;
extern volatile uint32_t g_sd_ll_ready_timeout_count;
extern volatile uint32_t g_sd_ll_last_sector;
extern volatile uint32_t g_sd_ll_last_blocks;
extern volatile uint32_t g_sd_ll_last_buffer_addr;
extern volatile uint32_t g_sd_ll_last_hal_status;
extern volatile uint32_t g_sd_ll_last_hal_error;
extern volatile uint32_t g_sd_ll_last_hal_state;
extern volatile uint32_t g_sd_ll_last_card_state;
extern volatile uint32_t g_sd_ll_last_wait_ms;
extern volatile uint32_t g_sd_ll_last_tick;
extern FIL g_emu_can_file;
extern FIL *g_sd_active_file;
extern volatile uint32_t g_emu_can_log_count;
extern volatile uint32_t g_emu_can_rx_count;
extern volatile uint32_t g_emu_can_rx_overflow_count;
extern volatile uint32_t g_emu_can_hal_error_count;
extern volatile uint8_t g_emu_can_enabled;
extern volatile uint32_t g_telemetry_log_count;
extern volatile uint32_t g_wheel4_log_count;
extern volatile uint32_t g_adc6_log_count;
extern volatile uint32_t g_wheel4_rx_overflow_count;
extern volatile uint32_t g_adc6_rx_overflow_count;
extern volatile uint8_t g_telemetry_queue_enabled;
extern volatile uint8_t g_wheel_legacy_queue_enabled;
extern volatile uint8_t g_adc_legacy_queue_enabled;
extern volatile uint32_t g_adc_rx_overflow_count;
extern volatile uint32_t g_adc_sample_count;
extern volatile uint64_t g_adc_last_timestamp_us;
extern volatile uint16_t g_adc_dma_buffer[ADC_DMA_CHANNEL_COUNT];
extern volatile uint16_t g_emu_potentiometer4_raw;
extern volatile uint16_t g_emu_potentiometer5_raw;
extern volatile uint16_t g_emu_adc6_raw;
extern volatile uint16_t g_adc_rx_head;
extern volatile uint16_t g_adc_rx_tail;
extern ADC_LogSample_t g_adc_rx_queue[ADC_LOG_QUEUE_LEN];
extern volatile uint16_t g_emu_can_rx_head;
extern volatile uint16_t g_emu_can_rx_tail;
extern EMU_CAN_Frame_t g_emu_can_rx_queue[EMU_CAN_RX_QUEUE_LEN];
extern volatile uint16_t g_wheel4_rx_head;
extern volatile uint16_t g_wheel4_rx_tail;
extern Wheel4_LogEvent_t g_wheel4_rx_queue[WHEEL4_RX_QUEUE_LEN];
extern volatile uint64_t g_wheel4_last_timestamp_us[WHEEL_CHANNEL_COUNT];
extern volatile uint32_t g_wheel4_last_delta_us[WHEEL_CHANNEL_COUNT];
extern volatile uint32_t g_wheel4_pulse_count[WHEEL_CHANNEL_COUNT];
extern volatile uint32_t g_wheel4_filtered_centi_kmh[WHEEL_CHANNEL_COUNT];
extern volatile uint8_t g_wheel4_filter_valid[WHEEL_CHANNEL_COUNT];
extern volatile uint32_t g_wheel_rx_overflow_count;
extern volatile uint32_t g_tim1_overflow_count;
extern volatile uint8_t g_tim1_timebase_ready;
extern volatile uint64_t g_wheel_last_timestamp_us;
extern volatile uint16_t g_wheel_rx_head;
extern volatile uint16_t g_wheel_rx_tail;
extern Wheel_LogEvent_t g_wheel_rx_queue[WHEEL_RX_QUEUE_LEN];
extern volatile uint32_t g_pb10_edge_count;
extern uint16_t g_wheel_speed_kmh;
extern volatile uint8_t g_gnss_rx_byte;
extern volatile uint16_t g_gnss_rx_head;
extern volatile uint16_t g_gnss_rx_tail;
extern volatile uint32_t g_gnss_rx_count;
extern volatile uint32_t g_gnss_rx_overflow_count;
extern volatile uint32_t g_gnss_uart_error_count;
extern volatile uint64_t g_gnss_drain_byte_count;
extern volatile uint8_t g_gnss_rx_queue[GNSS_RX_QUEUE_LEN];
extern volatile uint32_t g_gnss_rx_byte_count;
extern volatile uint32_t g_gnss_dollar_count;
extern volatile uint32_t g_gnss_0xB5_count;
extern volatile uint32_t g_gnss_fix_update_count;
extern volatile uint64_t g_gnss_last_fix_timestamp_us;
extern uint8_t g_imu_rx_dma_buffer[IMU_RX_DMA_BUFFER_SIZE];
extern uint16_t g_imu_rx_read_pos;
extern uint8_t g_imu_packet_buffer[IMU_PACKET_SIZE];
extern uint8_t g_imu_packet_len;
extern volatile uint32_t g_imu_packet_count;
extern volatile uint32_t g_imu_checksum_error_count;
extern volatile uint32_t g_imu_rx_error_count;
extern volatile uint32_t g_imu_rx_byte_count;
extern volatile uint32_t g_imu_uart_parity_error_count;
extern volatile uint32_t g_imu_uart_noise_error_count;
extern volatile uint32_t g_imu_uart_framing_error_count;
extern volatile uint32_t g_imu_uart_overrun_error_count;
extern volatile uint32_t g_imu_uart_dma_error_count;
extern volatile uint8_t g_imu_rearm_pending;
extern Telemetry_IMULatest_t g_imu_latest;
extern Telemetry_IMULatest_t g_imu_history[IMU_HISTORY_QUEUE_LEN];
extern uint16_t g_imu_history_head;
extern uint16_t g_imu_history_count;
extern volatile uint16_t g_imu_parse_batch_max;
extern volatile uint64_t g_imu_last_rx_timestamp_us;
extern volatile int32_t g_imu_timestamp_error_us;
extern volatile uint32_t g_imu_estimated_missing_count;
extern volatile uint32_t g_imu_resync_count;
extern volatile uint32_t g_imu_timeout_count;
extern volatile uint32_t g_imu_recovery_count;
extern volatile uint8_t g_imu_link_valid;
extern char g_sd_log_buffer[SD_LOG_BUFFER_SIZE];
extern char g_sd_log_path[SD_LOG_PATH_LEN];
extern UINT g_sd_log_buffer_len;
extern uint32_t g_sd_log_last_write_tick;
extern uint32_t g_sd_log_last_sync_tick;
extern uint32_t g_sd_log_unsynced_count;
extern Telemetry_CANLatest_t g_telemetry_latest_can[EMU_CAN_FRAME_COUNT];
extern Telemetry_WheelLatest_t g_telemetry_latest_wheel[WHEEL_CHANNEL_COUNT];
extern uint16_t g_telemetry_latest_adc_raw[ADC_CHANNEL_COUNT];
extern uint8_t g_telemetry_adc_seen;
extern uint64_t g_telemetry_next_snapshot_us;
extern uint32_t g_telemetry_snapshot_seq;
extern volatile uint32_t g_telemetry_missed_snapshot_count;
extern volatile uint16_t g_telemetry_snapshot_head;
extern volatile uint16_t g_telemetry_snapshot_tail;
extern volatile uint16_t g_telemetry_snapshot_max_depth;
extern volatile uint32_t g_telemetry_snapshot_overflow_count;
extern Telemetry_Snapshot_t
    g_telemetry_snapshot_queue[TELEMETRY_SNAPSHOT_QUEUE_LEN];
extern GNSS_ParsedData_t g_gnss_parsed;
extern volatile uint32_t g_gps_fat_time;
extern RaceCANLatest_t g_race_can_latest;
extern uint32_t g_race_nextion_last_rpm_tick;
extern uint32_t g_race_nextion_last_gear_tick;
extern uint32_t g_race_nextion_last_water_tick;
extern uint32_t g_race_nextion_last_tps_tick;
extern uint32_t g_race_nextion_last_speed_tick;
extern uint32_t g_race_nextion_last_oil_tick;
extern uint32_t g_race_nextion_last_battery_tick;
extern uint32_t g_race_nextion_last_fuel_tick;
extern volatile uint16_t g_race_nextion_last_rpm_sent;
extern volatile uint32_t g_race_nextion_rpm_tx_count;
extern volatile uint16_t g_race_nextion_last_tps_sent;
extern volatile uint32_t g_race_nextion_tps_tx_count;
extern volatile uint8_t g_race_nextion_last_gear_sent;
extern volatile uint32_t g_race_nextion_gear_tx_count;
extern volatile int16_t g_race_nextion_last_water_sent;
extern volatile uint32_t g_race_nextion_water_tx_count;
extern volatile uint16_t g_race_nextion_last_speed_sent;
extern volatile uint32_t g_race_nextion_speed_tx_count;
extern uint8_t g_race_nextion_rr_index;
extern uint32_t g_race_nextion_next_tx_tick;
extern uint32_t g_race_nextion_last_no_sig_tick;
extern volatile uint8_t g_race_nextion_dirty_mask;
extern uint8_t g_race_can_signal_lost;
extern volatile uint32_t g_race_raw_can_seen_count;
extern volatile uint32_t g_race_can_seen_count;
extern volatile uint32_t g_race_last_raw_can_rx_tick;
extern volatile uint32_t g_race_last_can_rx_tick;
extern uint32_t g_race_last_nextion_tx_tick;
extern volatile uint32_t g_race_nextion_tx_attempt_count;
extern volatile uint32_t g_race_nextion_tx_count;
extern volatile uint32_t g_race_nextion_tx_fail_count;
extern volatile uint32_t g_race_nextion_uart_recover_count;
extern volatile uint32_t g_race_nextion_uart_error_callback_count;
extern volatile uint32_t g_race_nextion_uart_last_error;
extern volatile uint32_t g_race_nextion_uart_last_state;
extern uint8_t g_race_nextion_tx_buffer[64];
extern volatile uint16_t g_race_nextion_tx_len;
extern volatile uint16_t g_race_nextion_tx_pos;
extern volatile uint8_t g_race_nextion_tx_active;
extern uint32_t g_race_nextion_tx_start_tick;
extern volatile uint32_t g_race_nextion_tx_busy_count;
extern volatile uint8_t g_emu_can_rx_irq_enabled;
extern uint8_t g_nextion_logok_visibility;
extern uint8_t g_nextion_logoff_visibility;
extern uint8_t g_nextion_toohot_visibility;
extern uint16_t g_nextion_latest_rpm;
extern uint8_t g_nextion_rpm_valid;
extern uint32_t g_nextion_last_tx_tick;
extern uint16_t g_nextion_hmi_rpm;
extern volatile uint32_t g_nextion_hmi_tx_count;
extern volatile uint32_t g_nextion_hmi_tx_fail_count;
extern volatile uint16_t g_current_rpm;
extern uint8_t g_rpm_led_mode;
extern uint32_t g_ws2812_last_update_tick;
extern uint16_t g_ws2812_pwm_buffer[WS2812_BUFFER_LEN];
extern volatile uint8_t g_ws2812_dma_ready;
extern volatile uint32_t g_ws2812_dma_done_count;
extern volatile uint32_t g_ws2812_dma_start_fail_count;
extern volatile uint32_t g_ws2812_dma_busy_skip_count;
extern uint16_t g_ws2812_pwm_0;
extern uint16_t g_ws2812_pwm_1;

void USER_CAN1_Apply500Kbps(void);
void SD_RecordFault(SD_FaultStage_t stage, FRESULT result,
                           uint32_t detail);
void SD_WriteFaultLog(void);
const char *SD_FaultStageName(SD_FaultStage_t stage);
void SD_RecoverAfterIoError(void);
uint8_t SD_WaitForTransferReady(uint32_t timeout_ms);
void RaceLoggerUpdateLatestFromCAN(uint32_t std_id, uint8_t dlc,
                                          const uint8_t *data);
void RaceNextionProcess(void);
void RaceNextionGetSnapshot(RaceCANLatest_t *latest,
                                   uint8_t *dirty_mask);
void RaceNextionClearDirty(uint8_t dirty_bit);
uint8_t RaceNextionSendNumber(const char *component, int32_t value);
uint8_t RaceNextionSendGear(uint8_t gear);
uint8_t EMU_CANLoggerConfigureHardware(void);
void EMU_CANLoggerStart(void);
void EMU_CANDrainRxFifo(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void EMU_CANQueuePushFromIsr(const CAN_RxHeaderTypeDef *rx_header,
                                    const uint8_t *data);
uint8_t EMU_CANQueuePop(EMU_CAN_Frame_t *frame);
void TelemetryLoggerStart(void);
uint64_t Telemetry_GetTimestampUs(void);
uint8_t Wheel4_ChannelIndex(const TIM_HandleTypeDef *htim);
uint32_t Wheel4_HALChannel(uint8_t channel);
TIM_HandleTypeDef *Wheel4_HALTimer(uint8_t channel);
void Wheel4_QueuePushFromIsr(const Wheel4_LogEvent_t *event);
uint8_t Wheel4_QueuePop(Wheel4_LogEvent_t *event);
int16_t IMU_ReadS16BE(const uint8_t *data);
void Telemetry_FormatU64(uint64_t value, char *output,
                                size_t output_size);
void IMU_StartReceiver(void);
void IMU_ParseByte(uint8_t byte);
void IMU_ProcessReceiver(void);
uint8_t IMU_HistoryGetAt(uint64_t timestamp_us,
                                Telemetry_IMULatest_t *sample);
void SD_WaitForCardInserted(void);
FRESULT SD_TelemetryLoggerInit(void);
FRESULT SD_OpenNextTelemetryLogFile(void);
void GNSSLoggerStart(void);
void GNSS_QueuePushFromIsr(uint8_t byte);
uint8_t GNSS_QueuePop(uint8_t *byte);
void NMEA_GetField(const char *line, uint8_t field_idx, char *out_buf,
                          uint8_t max_len);
void GNSS_ParseLine(const char *line);
void GNSS_ProcessParser(void);
FRESULT SD_TelemetryLoggerProcess(void);
FRESULT SD_TelemetryLoggerDrainInputs(void);
uint8_t Nextion_SendCommand(const char *command);
uint8_t Nextion_SendTextCommand(const char *component,
                                       const char *text);
uint8_t Nextion_SendVisibility(const char *component, uint8_t visible);
void Nextion_UARTPump(void);
void Nextion_RecoverUART4(void);
void RPMOutputs_Init(void);
void RPMOutputs_Process(void);
void RPMOutputs_SetRPM(uint16_t rpm);
void WS2812_InitTimings(void);
uint8_t PercentToByte(uint8_t percent);
void WS2812_SetLedBits(uint32_t *index, uint8_t red, uint8_t green,
                              uint8_t blue);
void WS2812_ShowRPM(uint16_t rpm, uint8_t mode);
void WS2812_ShowRPMBar(uint16_t rpm);
void WS2812_SendBuffer(uint32_t used_len);
FRESULT SD_TelemetryLoggerAppendSnapshot(
    const Telemetry_Snapshot_t *snapshot);
uint8_t ADC_LoggerConfigureHardware(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void Telemetry_SnapshotPushFromIsr(void);
uint8_t Telemetry_SnapshotPop(Telemetry_Snapshot_t *snapshot);
uint8_t Telemetry_SnapshotPeek(Telemetry_Snapshot_t *snapshot);
void ADC_QueuePushFromIsr(const uint16_t *raw);
void Wheel_QueuePushFromIsr(const Wheel_LogEvent_t *event);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
uint8_t BSP_SD_Init(void);
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr,
                          uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr,
                           uint32_t NumOfBlocks, uint32_t Timeout);
GPIO_PinState SD_IsCardInserted(void);
FRESULT SD_LogWriterFlush(uint8_t force_sync,
                                 SD_LogFlushReason_t reason);
FRESULT SD_PulseLoggerFlush(uint8_t force_sync);
void LED_Set(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void LED_BlinkCode(uint8_t code);
void LED_BlinkCodeDetail(uint8_t code, uint8_t detail);

void App_Init(void);
void App_Process(void);

#endif
