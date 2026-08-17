/*
 * ============================================================================
 *  teamNSSUR FSAE 실시간 텔레메트리 송신기 (LoRaSender)
 * ----------------------------------------------------------------------------
 *  보드   : LILYGO/TTGO T3 LoRa32 V1.6.1 (ESP32 + SX1276, 915MHz)
 *  입력   : EMU Black ECU CAN Stream (base ID 0x600, 500kbps Classic CAN)
 *  출력   : LoRa 915MHz -> teamNSSUR 전용 텔레메트리 수신기
 * ============================================================================
 *
 * [무선(PHY) 계층 규약]
 *   주파수       : LORA_FREQUENCY (915MHz)
 *   변조         : SF7 / BW125kHz / CR4:5 (LORA_* 매크로로 조정)
 *   Sync Word    : LORA_SYNC_WORD (0xA5) - teamNSSUR 전용 값.
 *                  송/수신기가 반드시 동일해야 하며, 대회장에서 다른 팀의
 *                  LoRa32/Meshtastic 장비와 겹치지 않도록 라이브러리 기본값
 *                  (0x12)이나 LoRaWAN 예약값(0x34)은 사용하지 않는다.
 *   CRC          : SX1276 하드웨어 CRC 사용 -> 손상된 패킷은 수신측에서 자동 폐기
 *
 * [패킷(링크) 계층 규약] - 멀티바이트 필드는 전부 Little-Endian
 *   offset 0-1 : TEAM_ID     'N','S' (teamNSSUR 식별자, 불일치 시 즉시 폐기)
 *   offset 2   : VEHICLE_ID  차량 구분자 (기본 0x01, 2호차 운용 시 값만 변경)
 *   offset 3   : MSG_TYPE    0x01=FAST, 0x02=SLOW
 *   offset 4   : SEQ         0-255 롤링 시퀀스 (타입별 별도 카운터, 유실 감지용)
 *   offset 5.. : payload     아래 표 참조
 *
 *   MSG_TYPE_FAST (0x01) - 총 10바이트, RPM/TPS/VSS 고정 주기 송신
 *     [5-6] RPM            uint16  rpm 그대로
 *     [7-8] TPS_x10         uint16  0.1% 단위 (예: 453 -> 45.3%)
 *     [9]   VSS             uint8   km/h
 *
 *   MSG_TYPE_SLOW (0x02) - 총 12바이트, 값이 바뀌거나 heartbeat 주기 도달 시 송신
 *     [5]    GEAR           uint8   기어 단수 (0=중립)
 *     [6-7]  CLT            int16   냉각수온 (섭씨)
 *     [8-9]  BATT_MV        uint16  배터리 전압 (mV)
 *     [10-11] FUEL_USED_x100 uint16  0.01L 단위 누적 연료 사용량
 *
 * [EMU Black CAN Stream 매핑] (base 0x600, 500kbps, 전부 Little-Endian)
 *   0x600 : RPM(byte0-1,u16) TPS(byte2,u8*0.5%) IAT(byte3,i8) MAP(4-5,u16) PW(6-7,u16)
 *   0x602 : VSS(byte0-1,u16,1km/h/bit) ... CLT(byte6-7,i16,1'C/bit)
 *   0x604 : GEAR(byte0,u8) ... BATT(byte2-3,u16,0.027V/bit)
 *   0x607 : FUEL_USED(byte6-7,u16,0.01L/bit)
 *
 * [Time-on-Air / 트래픽 예산] (SF7, BW125kHz, CR4:5, preamble 8, 명시 헤더+CRC)
 *   FAST(10B) / SLOW(12B) 패킷 모두 ToA ≈ 41ms.
 *   TX_INTERVAL_FAST_MS=200ms(5Hz) 기준 채널 점유율 ≈ 20% -> SLOW/헤더 여유 충분.
 * ============================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include "driver/twai.h"

// ---------------------------------------------------------------------------
// LoRa(SX1276) SPI/제어 핀 - T3 LoRa32 V1.6.1 고정 배선
// ---------------------------------------------------------------------------
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_SS    18
#define LORA_RST   23
#define LORA_DIO0  26

// ---------------------------------------------------------------------------
// CAN(TWAI) 트랜시버 핀 - 차량 배선 기준 (RX=IO4, TX=IO25), 500kbps
// ---------------------------------------------------------------------------
#define CAN_RX_PIN GPIO_NUM_4
#define CAN_TX_PIN GPIO_NUM_25

// ---------------------------------------------------------------------------
// LoRa 무선 파라미터
// ---------------------------------------------------------------------------
#define LORA_FREQUENCY          915E6
#define LORA_TX_POWER_DBM       17      // SX1276 PA_BOOST 기준 dBm
#define LORA_SPREADING_FACTOR   7       // 트랙 근거리 링크, 빠른 갱신 우선
#define LORA_BANDWIDTH          125E3   // Hz
#define LORA_CODING_RATE        5       // 4/5
#define LORA_PREAMBLE_LENGTH    8
#define LORA_SYNC_WORD          0xA5    // teamNSSUR 전용, 임의 변경 금지(수신기와 항상 일치)

// ---------------------------------------------------------------------------
// 송수신 프로토콜 식별자
// ---------------------------------------------------------------------------
#define TEAM_ID_BYTE0   'N'
#define TEAM_ID_BYTE1   'S'      // teamNSSUR
#define VEHICLE_ID      0x01

#define MSG_TYPE_FAST   0x01     // RPM / TPS / VSS
#define MSG_TYPE_SLOW   0x02     // GEAR / CLT / BATT / FUEL_USED

// ---------------------------------------------------------------------------
// 송신 주기 / 변화 감지 임계값 - 여기 매크로만 바꾸면 됨
// ---------------------------------------------------------------------------
#define TX_INTERVAL_FAST_MS            200   // RPM/TPS/VSS 송신 주기 (5Hz)
#define TX_INTERVAL_SLOW_MIN_MS        100   // SLOW 패킷 최소 송신 간격(연속 변화 시 과도 송신 방지)
#define TX_INTERVAL_SLOW_HEARTBEAT_MS  1000  // 변화가 없어도 최소 이 주기로 강제 송신

#define THRESH_BATT_MV          50   // 배터리 전압 변화 임계값 (0.05V)
#define THRESH_FUEL_USED_X100    5   // 누적 연료 변화 임계값 (0.05L)

#define CAN_HEALTH_CHECK_INTERVAL_MS  1000   // CAN 버스 상태 점검 주기

// ---------------------------------------------------------------------------
// EMU Black CAN Stream 프레임 ID (base 0x600)
// ---------------------------------------------------------------------------
#define EMU_CAN_BASE_ID   0x600
#define EMU_ID_RPM_TPS    (EMU_CAN_BASE_ID + 0)  // RPM, TPS, IAT, MAP, PW
#define EMU_ID_VSS_CLT    (EMU_CAN_BASE_ID + 2)  // VSS, CLT
#define EMU_ID_GEAR_BATT  (EMU_CAN_BASE_ID + 4)  // GEAR, BATT
#define EMU_ID_FUEL       (EMU_CAN_BASE_ID + 7)  // FUEL_USED

// ---------------------------------------------------------------------------
// 최신 파싱 값 (CAN 폴링에서 갱신)
// ---------------------------------------------------------------------------
struct TelemetryState {
  uint16_t rpm          = 0;
  uint16_t tpsX10        = 0;   // 0.1% 단위
  uint8_t  vss           = 0;   // km/h
  uint8_t  gear          = 0;
  int16_t  clt           = 0;   // 'C
  uint16_t battMv        = 0;   // mV
  uint16_t fuelUsedX100  = 0;   // 0.01L 단위
};

TelemetryState g_state;

// SLOW 필드 마지막 송신값 (변화 감지용) - 초기값을 강제로 다르게 하여 첫 패킷은 무조건 전송
uint8_t  g_lastSentGear          = 0xFF;
int16_t  g_lastSentClt           = INT16_MIN;
uint16_t g_lastSentBattMv        = 0xFFFF;
uint16_t g_lastSentFuelUsedX100  = 0xFFFF;

uint8_t g_seqFast = 0;
uint8_t g_seqSlow = 0;

unsigned long g_lastFastTx = 0;
unsigned long g_lastSlowTx = 0;
unsigned long g_lastCanHealthCheck = 0;

// ---------------------------------------------------------------------------
void setupLora() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER_DBM);
  LoRa.enableCrc();

  Serial.println("LoRa started!");
}

void setupCan() {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("CAN driver install failed!");
    while (1);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("CAN start failed!");
    while (1);
  }
  Serial.println("CAN(TWAI) started (500kbps)");
}

void setup() {
  Serial.begin(115200);
  setupLora();
  setupCan();
}

// ---------------------------------------------------------------------------
// CAN 수신: 큐에 쌓인 프레임을 모두 비우면서 최신 상태 갱신 (non-blocking)
// ---------------------------------------------------------------------------
void pollCan() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    if (msg.extd || msg.rtr) continue;  // 표준 11비트 / 데이터 프레임만 처리

    switch (msg.identifier) {
      case EMU_ID_RPM_TPS:
        if (msg.data_length_code >= 3) {
          g_state.rpm = (uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8);
          g_state.tpsX10 = (uint16_t)msg.data[2] * 5;  // raw*0.5% -> x10 = raw*5
        }
        break;

      case EMU_ID_VSS_CLT:
        if (msg.data_length_code >= 8) {
          g_state.vss = (uint16_t)msg.data[0] | ((uint16_t)msg.data[1] << 8);
          g_state.clt = (int16_t)((uint16_t)msg.data[6] | ((uint16_t)msg.data[7] << 8));
        }
        break;

      case EMU_ID_GEAR_BATT:
        if (msg.data_length_code >= 4) {
          g_state.gear = msg.data[0];
          uint16_t battRaw = (uint16_t)msg.data[2] | ((uint16_t)msg.data[3] << 8);
          g_state.battMv = battRaw * 27;  // 0.027V/bit = 27mV/bit (정수 연산)
        }
        break;

      case EMU_ID_FUEL:
        if (msg.data_length_code >= 8) {
          // raw 0.01L/bit == wire format(x100)과 동일 단위라 변환 없이 그대로 사용
          g_state.fuelUsedX100 = (uint16_t)msg.data[6] | ((uint16_t)msg.data[7] << 8);
        }
        break;

      default:
        break;  // 관심 없는 ID는 무시
    }
  }
}

// ---------------------------------------------------------------------------
// CAN 버스 상태 점검: 노이즈로 인한 bus-off 시 자동 복구
// ---------------------------------------------------------------------------
void checkCanHealth() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_BUS_OFF) {
    Serial.println("CAN bus-off detected, recovering...");
    twai_initiate_recovery();
    twai_start();
  }
}

// ---------------------------------------------------------------------------
// 패킷 송신 헬퍼
// ---------------------------------------------------------------------------
void appendHeader(uint8_t *buf, int &idx, uint8_t msgType, uint8_t seq) {
  buf[idx++] = TEAM_ID_BYTE0;
  buf[idx++] = TEAM_ID_BYTE1;
  buf[idx++] = VEHICLE_ID;
  buf[idx++] = msgType;
  buf[idx++] = seq;
}

void appendU16LE(uint8_t *buf, int &idx, uint16_t v) {
  buf[idx++] = (uint8_t)(v & 0xFF);
  buf[idx++] = (uint8_t)(v >> 8);
}

void sendFastPacket() {
  uint8_t buf[16];
  int idx = 0;
  appendHeader(buf, idx, MSG_TYPE_FAST, g_seqFast++);
  appendU16LE(buf, idx, g_state.rpm);
  appendU16LE(buf, idx, g_state.tpsX10);
  buf[idx++] = g_state.vss;

  LoRa.beginPacket();
  LoRa.write(buf, idx);
  LoRa.endPacket();
}

void sendSlowPacket() {
  uint8_t buf[16];
  int idx = 0;
  appendHeader(buf, idx, MSG_TYPE_SLOW, g_seqSlow++);
  buf[idx++] = g_state.gear;
  appendU16LE(buf, idx, (uint16_t)g_state.clt);
  appendU16LE(buf, idx, g_state.battMv);
  appendU16LE(buf, idx, g_state.fuelUsedX100);

  LoRa.beginPacket();
  LoRa.write(buf, idx);
  LoRa.endPacket();

  g_lastSentGear = g_state.gear;
  g_lastSentClt = g_state.clt;
  g_lastSentBattMv = g_state.battMv;
  g_lastSentFuelUsedX100 = g_state.fuelUsedX100;
}

bool slowFieldsChanged() {
  if (g_state.gear != g_lastSentGear) return true;
  if (g_state.clt != g_lastSentClt) return true;
  if (abs((int)g_state.battMv - (int)g_lastSentBattMv) >= THRESH_BATT_MV) return true;
  if (abs((int)g_state.fuelUsedX100 - (int)g_lastSentFuelUsedX100) >= THRESH_FUEL_USED_X100) return true;
  return false;
}

void loop() {
  pollCan();

  unsigned long now = millis();

  if (now - g_lastFastTx >= TX_INTERVAL_FAST_MS) {
    g_lastFastTx = now;
    sendFastPacket();
  }

  bool slowHeartbeatDue = (now - g_lastSlowTx >= TX_INTERVAL_SLOW_HEARTBEAT_MS);
  bool slowMinIntervalOk = (now - g_lastSlowTx >= TX_INTERVAL_SLOW_MIN_MS);

  if ((slowFieldsChanged() && slowMinIntervalOk) || slowHeartbeatDue) {
    g_lastSlowTx = now;
    sendSlowPacket();
  }

  if (now - g_lastCanHealthCheck >= CAN_HEALTH_CHECK_INTERVAL_MS) {
    g_lastCanHealthCheck = now;
    checkCanHealth();
  }
}
