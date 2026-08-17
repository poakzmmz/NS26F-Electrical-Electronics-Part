# Datalogger Firmware

STM32F405RGTx용 STM32CubeIDE 프로젝트.

- 프로젝트: `stm32cubeide/Datalogger_final3/`
- CubeMX 설정: `Datalogger_test.ioc`
- FATFS
- USB Device
- 빌드 스크립트: `tools/stm32_build.sh`
- SWD 업로드 스크립트: `tools/stm32_upload.sh`

```bash
cd dashboard/datalogger/firmware/stm32cubeide/Datalogger_final3
./tools/stm32_build.sh
./tools/stm32_upload.sh
```

두 스크립트는 macOS의 `/Applications/STM32CubeIDE.app` 경로를 사용한다. `Debug/`는 Git에서 제외된다.

