# Datalogger Firmware

STM32F405RGTx 기반 차량용 Data Logger 펌웨어입니다.

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

센서/통신 입력과 SD Logging이 동시에 수행되므로, ISR에서 긴 파일 I/O를 수행하지 않고 Queue/Ring Buffer에 데이터를 적재한 뒤 메인 루프에서 처리하는 구조를 사용합니다.

특히 SD/FATFS Write가 수 ms 이상 Blocking될 수 있기 때문에 CAN, UART, Wheel Speed 입력 손실을 줄이기 위해 입력 처리와 저장 처리를 분리했습니다.

## Configuration Notes

### CAN

CubeMX `.ioc`의 CAN 계산값은 현재 런타임 운용값과 다릅니다.

실제 펌웨어에서는 `USER_CAN1_Apply500Kbps()`에서 CAN1을 재초기화하여 **500 kbps**로 운용합니다.

### SDIO

초기 하드웨어/설정은 4-bit SDIO를 고려했으나, 현재 공개 펌웨어는 주행 환경에서의 안정성을 우선해 **1-bit SDIO** 운용 경로를 사용합니다.

### USB MSC

활성 Logging 중 Host와 SD Card를 동시에 접근하는 상황에서 파일시스템 충돌 가능성을 줄이기 위해 현재 런타임에서는 USB Mass Storage를 비활성화했습니다.

## Project

- STM32CubeIDE project: `stm32cubeide/Datalogger_final3/`
- CubeMX configuration: `Datalogger_test.ioc`
- FATFS
- USB Device source retained in project
- Build script: `tools/stm32_build.sh`
- SWD upload script: `tools/stm32_upload.sh`

## Build

```bash
cd dashboard/datalogger/firmware/stm32cubeide/Datalogger_final3
./tools/stm32_build.sh
./tools/stm32_upload.sh
```

현재 스크립트는 macOS의 `/Applications/STM32CubeIDE.app` 설치 경로를 기준으로 작성되어 있습니다. `Debug/` 빌드 산출물은 Git에서 제외됩니다.

## Validation

펌웨어는 Bench Test 후 실제 Formula Student 차량에 통합하여 CAN, Sensor Input, Data Logging, Dashboard Output을 반복 검증했습니다.

양산 자동차용 소프트웨어 인증을 수행한 코드는 아니며, 학생 Formula Student 차량의 개발/대회 환경을 기준으로 작성되었습니다.
