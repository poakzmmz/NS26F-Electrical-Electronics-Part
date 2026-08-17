# NS26F Datalogger HW

NS26F 데이터로거 KiCad 프로젝트.

## 하드웨어

- MCU: STM32F405RGT6
- 외부 크리스털: 8 MHz
- 3.3 V 레귤레이터: AMS1117-3.3
- CAN 트랜시버: SN65HVD230
- microSD: SDIO 4-bit
- USB Type-C
- SWD
- ADC 입력 6채널
- Input Capture 4채널
- 상태 LED 2채널

## 파일

- `DLTM.kicad_sch`: 회로도
- `DLTM.kicad_pcb`: PCB
- `DLTM.kicad_pro`: 프로젝트 설정
- `DLTM.pdf`: 회로도 PDF
- `production/`: BOM, 실장 좌표, 생산 파일
