# NS26F Datalogger

CAN, ADC, 타이머 입력, UART 데이터를 수집해 SD 카드에 기록하는 데이터로거.

## 하드웨어

- STM32F405RGT6
- 8 MHz 외부 크리스털
- AMS1117-3.3
- SN65HVD230
- microSD / SDIO 4-bit
- USB Type-C
- SWD
- ADC 6채널
- Input Capture 4채널
- 상태 LED 2채널

## 경로

- `hardware/imported-source/NS26F-Datalogger-HW/`: KiCad 및 생산 파일
- `firmware/stm32cubeide/Datalogger_final3/`: STM32CubeIDE 프로젝트

