# NS26F Telemetry

TTGO T3 LoRa32 V1.6.1 보드와 SX1276을 사용한 915 MHz 무선통신 체계.

## 구성

- `vehicle-node/firmware/LoRaSender/`: EMU Black CAN 데이터 송신기
- `receiver/firmware/LoRaReceiver/`: LoRa 수신 및 USB Serial 출력
- `protocol/`: LoRa 패킷과 Serial CSV 형식
- `viewer/lora-monitor/`: Matplotlib 로컬 뷰어 및 CSV 저장
- `viewer/telemetry-dashboard/`: CSV 및 실시간 데이터 뷰어
