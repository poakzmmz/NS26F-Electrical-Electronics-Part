# NS26F Datalogger

STM32F405RGT6 기반 차량 탑재형 데이터 수집장치입니다.

차량에서 발생하는 CAN, Analog Sensor, Wheel Speed, GNSS/IMU 데이터를 수집하고 공통 시간축으로 정리하여 microSD에 기록합니다. 동시에 운전자용 Dashboard와 Shift Light에 필요한 주요 신호를 제공합니다.

## Purpose

기존 차량에서는 각 파트의 설계 목표와 실제 주행 성능을 정량적으로 비교하기 어려웠습니다.

이를 해결하기 위해 다음 요구사항을 기준으로 Data Logger를 구성했습니다.

- 차량 ECU/CAN 데이터 수집
- Analog Sensor 다채널 입력
- Wheel Speed Pulse 측정
- GNSS/IMU UART 입력
- 장시간 주행 중 microSD Logging
- 운전자 표시계와 Shift Light 출력
- 실제 차량 환경에서 반복 검증 가능한 구조

## Hardware

| 항목 | 구성 |
|---|---|
| MCU | STM32F405RGT6 |
| Clock | 8 MHz External Crystal |
| Regulator | AMS1117-3.3 |
| CAN Transceiver | SN65HVD230 |
| CAN | 500 kbps runtime configuration |
| Analog Input | ADC 6ch (board interface 기준) |
| Timer Input | Wheel Speed Input Capture |
| UART | GNSS / IMU / HMI 등 주변장치 연결 |
| Storage | microSD + FATFS |
| Debug | SWD |
| USB | USB Type-C connector |

> 현재 펌웨어는 SD 안정성을 위해 1-bit SDIO 운용 경로를 사용합니다. KiCad/초기 CubeMX 구성에는 4-bit SDIO 관련 설정이 남아 있을 수 있습니다.

## Firmware Architecture

현재 공개 펌웨어에는 다음 기능이 통합되어 있습니다.

- CAN frame reception and signal extraction
- ADC DMA acquisition
- Wheel speed input capture
- GNSS serial reception
- IMU DMA reception and parsing
- microSD CSV logging
- Nextion HMI update
- WS2812B shift-light control
- Ring buffer / queue based input handling
- SD write buffering, retry and fault logging

펌웨어에서는 blocking SD/FATFS 동작이 통신 및 센서 수집을 방해하지 않도록 ISR에서는 데이터를 Queue에 저장하고, 메인 루프에서 SD 기록을 처리하는 구조를 사용했습니다.

## CAN Configuration Note

CubeMX `.ioc` 파일의 CAN 계산값과 실제 런타임 설정이 다릅니다.

현재 펌웨어에서는 `USER_CAN1_Apply500Kbps()`에서 CAN1을 재초기화하여 **500 kbps**로 운용합니다. 따라서 포트폴리오 검토 시 실제 동작 기준은 런타임 설정을 기준으로 봐주시면 됩니다.

## Verification

개발 과정은 다음 순서로 진행했습니다.

1. Interface 단위 동작 확인
2. CAN / ADC / Timer / UART 개별 데이터 확인
3. SD Logging 및 파일 무결성 확인
4. 차량 탑재 후 센서/ECU와 통합
5. 반복 주행 중 Data Loss 및 통신 오류 확인
6. Fault/Buffer 처리 개선 후 재검증

완성된 Data Logger는 Formula Student 차량에 탑재되어 약 200 km의 누적 테스트 주행 및 대회 환경에서 사용했습니다.

## Files

- `hardware/imported-source/NS26F-Datalogger-HW/`: KiCad 및 생산 파일
- `firmware/stm32cubeide/Datalogger_final3/`: STM32CubeIDE 프로젝트
- `hardware/imported-source/NS26F-Datalogger-HW/DLTM.pdf`: Schematic PDF

## Portfolio Notes

이 저장소는 학생 Formula Student 프로젝트의 공개 가능한 산출물을 정리한 것입니다.

- 실제 양산 Automotive Qualification을 수행한 제품이 아닙니다.
- 설계 변경 과정의 일부 초기 파일도 보존되어 있습니다.
- 차량/팀 운영에 민감한 정보와 개인정보는 공개하지 않습니다.
