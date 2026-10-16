# Datalogger Firmware

STM32F405RGTx 기반 차량용 Data Logger 펌웨어입니다.

## Structure

```text
Core/
├── Inc/
│   └── app_internal.h
└── Src/
    ├── main.c
    ├── app.c
    ├── app_state.c
    ├── can_bus.c
    ├── adc_input.c
    ├── wheel_speed.c
    ├── gnss.c
    ├── imu.c
    ├── telemetry.c
    ├── sd_logger.c
    ├── dashboard.c
    ├── shiftlight.c
    ├── app_callbacks.c
    └── led_diag.c
```

기존에는 약 4,700줄의 `main.c`에 대부분의 기능이 집중되어 있었습니다. 현재는 기능별 책임을 분리하고 `main.c`에는 CubeMX 초기화와 애플리케이션 실행 진입점만 남겼습니다.

## Module Responsibility

| Module | Responsibility |
|---|---|
| `app.c` | Application 초기화 순서 및 main-loop scheduling |
| `app_state.c` | 공유 runtime state |
| `can_bus.c` | CAN 500 kbps 설정, RX queue, frame processing |
| `adc_input.c` | ADC DMA acquisition 및 queue |
| `wheel_speed.c` | Timer Input Capture 기반 wheel speed |
| `gnss.c` | GNSS UART queue 및 NMEA parsing |
| `imu.c` | IMU DMA reception 및 packet parsing |
| `telemetry.c` | 공통 timestamp/snapshot aggregation |
| `sd_logger.c` | FATFS logging, buffering, retry, fault handling |
| `dashboard.c` | Nextion HMI 및 race display state |
| `shiftlight.c` | WS2812B RPM shift-light |
| `app_callbacks.c` | STM32 HAL callback dispatch |
| `led_diag.c` | 상태 LED 및 error blink code |

## Main Functions

- CAN 500 kbps 수신 및 차량 신호 처리
- ADC DMA 기반 Analog Sensor 수집
- Timer Input Capture 기반 Wheel Speed 측정
- GNSS UART 수신 및 Parsing
- IMU UART/DMA 수신
- microSD + FATFS 기반 CSV Logging
- Nextion HMI 출력
- WS2812B Shift Light 제어
- SD I/O Fault 진단 및 Recovery

## Runtime Design

센서/통신 입력과 SD Logging이 동시에 수행되므로 ISR에서는 긴 파일 I/O를 수행하지 않고 Queue/Ring Buffer에 데이터를 적재한 뒤 foreground loop에서 처리합니다.

SD/FATFS Write가 수 ms 이상 Blocking될 수 있기 때문에 CAN, UART, Wheel Speed 입력 처리와 저장 처리를 분리했습니다.

## Configuration

### CAN

실제 runtime에서는 `USER_CAN1_Apply500Kbps()`에서 CAN1을 재초기화하여 **500 kbps**로 운용합니다.

### SDIO

현재 firmware는 주행 환경에서의 안정성을 우선해 **1-bit SDIO**로 운용합니다.

### USB MSC

Logging 중 Host와 SD Card가 동시에 접근하는 상황을 방지하기 위해 runtime에서는 USB Mass Storage를 사용하지 않습니다.

## Cleanup

Refactor 과정에서 사용되지 않는 bring-up/test 상수와 더 이상 참조되지 않는 diagnostic counter를 제거했습니다. 기능 구현부에 남아 있던 과거 개발 단계의 장문 주석도 정리하고, 코드 구조 자체로 책임이 드러나도록 구성했습니다.

CubeMX가 관리하는 `main.c`의 생성 코드와 USER CODE marker는 유지합니다.

## Validation Status

Refactor branch에서는 다음 정적 검사를 수행했습니다.

- module delimiter balance
- shared global-state declaration coverage
- duplicate function definition check
- unused legacy macro check
- STM32CubeIDE `Core` source-path 자동 포함 확인

실차 동작 검증 기준은 기존 firmware와 동일합니다. 최종 배포 전에는 기존 regression 항목(CAN, ADC, wheel speed, GNSS, IMU, SD logging, HMI, shift light)을 bench/vehicle 환경에서 재확인해야 합니다.

## Requirements & Design Decisions

초기 요구사항과 실차 검증 과정에서의 변경 사항은 [REQUIREMENTS.md](REQUIREMENTS.md)에 정리되어 있습니다.

## Project

- STM32CubeIDE project: `stm32cubeide/Datalogger_final3/`
- CubeMX configuration: `Datalogger_test.ioc`
- FATFS
- Build script: `tools/stm32_build.sh`
- SWD upload script: `tools/stm32_upload.sh`

## Build

```bash
cd dashboard/datalogger/firmware/stm32cubeide/Datalogger_final3
./tools/stm32_build.sh
./tools/stm32_upload.sh
```

스크립트는 macOS의 `/Applications/STM32CubeIDE.app` 설치 경로를 기준으로 작성되어 있습니다. `Debug/` 빌드 산출물은 Git에서 제외됩니다.
