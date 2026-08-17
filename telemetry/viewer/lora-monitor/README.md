# LoRa Monitor

LoRa 수신기의 USB Serial 데이터를 표시하고 CSV로 저장하는 로컬 뷰어.

## 실행

```bash
python3 -m pip install -r requirements.txt
python3 lora_monitor.py --port COM4
```

- 기본 baud rate: 115200
- 기본 Serial port: COM4
- 로그 위치: `logs/telemetry_YYYYMMDD_HHMMSS.csv`
- `--no-web`: 웹 브리지 비활성화
- `--web-port`: 웹 브리지 포트 변경

웹 브리지는 `../telemetry-dashboard/tools/rf_bridge.py`를 사용한다.

