/*
 * ============================================================================
 *  teamNSSUR FSAE 실시간 텔레메트리 수신기 (LoRaReceiver)
 * ----------------------------------------------------------------------------
 *  보드   : LILYGO/TTGO T3 LoRa32 V1.6.1 (ESP32 + SX1276, 915MHz)
 *  입력   : LoRa 915MHz <- LoRaSender (차량 탑재)
 *  출력   : USB Serial(115200bps) -> 관제 PC (CSV 텍스트, 줄 단위)
 *
 *  LoRaSender.ino 와 동일한 PHY/패킷 규약을 그대로 따른다. 이 보드는 PC에
 *  꽂혀 관제 역할을 하며, 수신한 텔레메트리를 파싱해 PC가 읽기 쉬운 CSV
 *  한 줄로 시리얼에 출력한다.
 * ============================================================================
 *
 * [패킷(링크) 계층 규약] - 멀티바이트 필드는 전부 Little-Endian (LoRaSender와 동일)
 *   offset 0-1 : TEAM_ID     'N','S' (불일치 시 즉시 폐기)
 *   offset 2   : VEHICLE_ID  차량 구분자
 *   offset 3   : MSG_TYPE    0x01=FAST, 0x02=SLOW
 *   offset 4   : SEQ         0-255 롤링 시퀀스
 *   offset 5.. : payload
 *
 *   MSG_TYPE_FAST (0x01) - 총 10바이트
 *     [5-6] RPM            uint16
 *     [7-8] TPS_x10        uint16  0.1% 단위
 *     [9]   VSS            uint8   km/h
 *
 *   MSG_TYPE_SLOW (0x02) - 총 12바이트
 *     [5]    GEAR          uint8
 *     [6-7]  CLT           int16   섭씨
 *     [8-9]  BATT_MV       uint16  mV
 *     [10-11] FUEL_USED_x100 uint16 0.01L 단위
 *
 * [PC 출력 포맷] (시리얼, 줄바꿈으로 구분되는 CSV)
 *   FAST,vehicleId,seq,rpm,tpsX10,vss,rssi,snr
 *   SLOW,vehicleId,seq,gear,clt,battMv,fuelUsedX100,rssi,snr
 * ============================================================================
 */

#include <SPI.h>
#include <LoRa.h>

// ---------------------------------------------------------------------------
// LoRa(SX1276) SPI/제어 핀 - T3 LoRa32 V1.6.1 고정 배선 (Sender와 동일)
// ---------------------------------------------------------------------------
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_SS    18
#define LORA_RST   23
#define LORA_DIO0  26

// ---------------------------------------------------------------------------
// LoRa 무선 파라미터 - Sender와 반드시 동일해야 통신 가능
// ---------------------------------------------------------------------------
#define LORA_FREQUENCY          915E6
#define LORA_SPREADING_FACTOR   7
#define LORA_BANDWIDTH          125E3
#define LORA_CODING_RATE        5
#define LORA_PREAMBLE_LENGTH    8
#define LORA_SYNC_WORD          0xA5    // teamNSSUR 전용, Sender와 항상 일치해야 함

// ---------------------------------------------------------------------------
// 송수신 프로토콜 식별자 (Sender와 동일)
// ---------------------------------------------------------------------------
#define TEAM_ID_BYTE0   'N'
#define TEAM_ID_BYTE1   'S'

#define MSG_TYPE_FAST   0x01
#define MSG_TYPE_SLOW   0x02

#define FAST_PACKET_LEN  10   // header(5) + RPM(2)+TPS(2)+VSS(1)
#define SLOW_PACKET_LEN  12   // header(5) + GEAR(1)+CLT(2)+BATT(2)+FUEL(2)

#define RX_BUF_SIZE      32

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
  LoRa.enableCrc();

  Serial.println("LoRa Vehicle Receiver started");
}

void setup() {
  Serial.begin(115200);
  setupLora();
}

// ---------------------------------------------------------------------------
uint16_t readU16LE(const uint8_t *buf, int idx) {
  return (uint16_t)buf[idx] | ((uint16_t)buf[idx + 1] << 8);
}

void handleFastPacket(const uint8_t *buf, int len, uint8_t vehicleId, int rssi, float snr) {
  if (len < FAST_PACKET_LEN) return;  // 손상/불완전 패킷 폐기

  uint8_t  seq    = buf[4];
  uint16_t rpm    = readU16LE(buf, 5);
  uint16_t tpsX10 = readU16LE(buf, 7);
  uint8_t  vss    = buf[9];

  Serial.print("FAST,");
  Serial.print(vehicleId);
  Serial.print(',');
  Serial.print(seq);
  Serial.print(',');
  Serial.print(rpm);
  Serial.print(',');
  Serial.print(tpsX10);
  Serial.print(',');
  Serial.print(vss);
  Serial.print(',');
  Serial.print(rssi);
  Serial.print(',');
  Serial.println(snr, 1);
}

void handleSlowPacket(const uint8_t *buf, int len, uint8_t vehicleId, int rssi, float snr) {
  if (len < SLOW_PACKET_LEN) return;  // 손상/불완전 패킷 폐기

  uint8_t  seq           = buf[4];
  uint8_t  gear          = buf[5];
  int16_t  clt           = (int16_t)readU16LE(buf, 6);
  uint16_t battMv        = readU16LE(buf, 8);
  uint16_t fuelUsedX100  = readU16LE(buf, 10);

  Serial.print("SLOW,");
  Serial.print(vehicleId);
  Serial.print(',');
  Serial.print(seq);
  Serial.print(',');
  Serial.print(gear);
  Serial.print(',');
  Serial.print(clt);
  Serial.print(',');
  Serial.print(battMv);
  Serial.print(',');
  Serial.print(fuelUsedX100);
  Serial.print(',');
  Serial.print(rssi);
  Serial.print(',');
  Serial.println(snr, 1);
}

// ---------------------------------------------------------------------------
void loop() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  uint8_t buf[RX_BUF_SIZE];
  int len = 0;
  while (LoRa.available() && len < RX_BUF_SIZE) {
    buf[len++] = (uint8_t)LoRa.read();
  }

  if (len < 5) return;                                              // header보다 짧으면 폐기
  if (buf[0] != TEAM_ID_BYTE0 || buf[1] != TEAM_ID_BYTE1) return;    // 팀 ID 불일치 -> 폐기

  uint8_t vehicleId = buf[2];
  uint8_t msgType    = buf[3];
  int     rssi       = LoRa.packetRssi();
  float   snr        = LoRa.packetSnr();

  switch (msgType) {
    case MSG_TYPE_FAST:
      handleFastPacket(buf, len, vehicleId, rssi, snr);
      break;
    case MSG_TYPE_SLOW:
      handleSlowPacket(buf, len, vehicleId, rssi, snr);
      break;
    default:
      break;  // 알 수 없는 MSG_TYPE 무시
  }
}
