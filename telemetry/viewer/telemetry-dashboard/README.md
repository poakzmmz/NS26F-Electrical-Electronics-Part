# NS26F Telemetry Dashboard

NS26F 데이터로거 CSV 로그와 실시간 수신 데이터를 브라우저에서 보는 정적 웹 프로젝트.

## 실행

```bash
python3 -m http.server 8081
```

macOS에서는 `대시보드_실행.command`로도 실행할 수 있다.

## 파일

- `index.html`: 페이지
- `style.css`: 스타일
- `app.js`: CSV 파싱, CAN 데이터 해석, 차트, GPS
- `filters.js`: 채널 필터
- `steering.js`: 조향 영점 보정
- `suspension.js`: 서스펜션 화면
- `realtime.js`: 실시간 데이터 화면
- `tools/rf_bridge.py`: Serial 데이터를 브라우저로 전달하는 중계 서버
- `vercel.json`: Vercel 설정

CSV는 브라우저에서 파싱한다.

## 실시간 중계

Serial 장치:

```bash
python3 tools/rf_bridge.py --serial /dev/tty.usbserial-0001 --baud 115200
```

시뮬레이션:

```bash
python3 tools/rf_bridge.py --sim
```

CSV 재생:

```bash
python3 tools/rf_bridge.py --replay telemetry.csv
```

Serial 입력에는 `pyserial`이 필요하다. 중계 서버는 WebSocket, SSE, HTTP polling을 지원한다.

## 입력 형식

JSON line:

```text
{"msg_type":"FAST","seq":12,"rpm":5200,"tps_pct":31.5,"rssi":-72,"snr":9.4}
```

CSV line:

```text
#msg_type,seq,rpm,tps_pct,rssi,snr
FAST,12,5200,31.5,-72,9.4
```

`seq`로 패킷 손실을 계산하고 `rssi`, `snr`을 수신 상태 표시에 사용한다.

