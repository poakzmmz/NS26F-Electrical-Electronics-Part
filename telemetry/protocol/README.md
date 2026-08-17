# Telemetry Protocol

## LoRa PHY

- Frequency: 915 MHz
- Spreading Factor: 7
- Bandwidth: 125 kHz
- Coding Rate: 4/5
- Preamble: 8
- Sync Word: `0xA5`
- CRC 사용

## 공통 헤더

멀티바이트 값은 Little-Endian이다.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | TEAM ID `NS` |
| 2 | 1 | Vehicle ID |
| 3 | 1 | Message Type |
| 4 | 1 | Sequence |

## FAST (`0x01`)

전체 길이 10 bytes.

| Offset | Size | Field | Unit |
|---:|---:|---|---|
| 5 | 2 | RPM | rpm |
| 7 | 2 | TPS × 10 | 0.1% |
| 9 | 1 | VSS | km/h |

## SLOW (`0x02`)

전체 길이 12 bytes.

| Offset | Size | Field | Unit |
|---:|---:|---|---|
| 5 | 1 | Gear | - |
| 6 | 2 | CLT | °C |
| 8 | 2 | Battery | mV |
| 10 | 2 | Fuel Used × 100 | 0.01 L |

## Receiver Serial

```text
FAST,vehicleId,seq,rpm,tpsX10,vss,rssi,snr
SLOW,vehicleId,seq,gear,clt,battMv,fuelUsedX100,rssi,snr
```
