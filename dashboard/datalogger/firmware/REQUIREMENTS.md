# Datalogger Software Requirements & Design Decisions

이 문서는 NS26F Data Logger 개발 초기에 정의한 요구사항과 실제 차량 검증 과정에서 변경된 구현 사항을 정리합니다.

원본 요구사항은 `hardware/imported-source/NS26F-Datalogger-HW/sw요구사항.txt`에 보존되어 있습니다.

## Initial Requirements

| Area | Initial Requirement |
|---|---|
| CAN | 500 kbps, RX interrupt + ring buffer |
| ADC | 6-channel sensor acquisition |
| Wheel Speed | 4-channel Timer Input Capture |
| UART | GNSS / external serial devices |
| Synchronization | CAN / ADC / TIM / UART data on a common timestamp |
| Logging | FATFS-based CSV or binary logging |
| Storage | SDIO DMA / buffered SD write |
| USB | VBUS detection + USB Mass Storage |
| Priority | CAN RX → sensor packing → SD write → USB/status |

## Implemented Runtime Architecture

### Data Acquisition

- CAN 500 kbps RX interrupt and queue processing
- ADC DMA acquisition
- 4-channel wheel-speed input capture
- GNSS UART ring-buffer reception and NMEA parsing
- IMU UART DMA reception and packet parsing

### Data Synchronization

CAN, ADC, wheel-speed and IMU samples are aggregated against a common MCU timestamp before being serialized to the log stream.

Different input rates are therefore handled as independent producers rather than forcing all peripherals into one blocking sampling loop.

### Storage

SD/FATFS access is performed outside interrupt context.

ISR paths enqueue data and return quickly; foreground processing performs parsing, aggregation and file I/O. This design was introduced because SD writes can block long enough to interfere with continuous CAN/UART acquisition.

The logger also maintains retry/fault state for write and synchronization failures.

## Requirement Changes from Vehicle Testing

| Initial Requirement | Current Implementation | Reason |
|---|---|---|
| 4-bit SDIO | 1-bit SDIO | Prioritized signal/stability margin in the vehicle environment |
| USB Mass Storage during operation | Runtime disabled | Avoid simultaneous Host/FATFS access to the same SD card |
| Single integrated source file | Functional modules | Improved maintainability and responsibility separation |
| Basic logging | Buffered logging + fault/retry handling | Required by intermittent storage latency/error conditions |

These changes reflect implementation trade-offs made after integration and vehicle testing rather than simply preserving the first design specification.

## Current Firmware Modules

- `can_bus.c`
- `adc_input.c`
- `wheel_speed.c`
- `gnss.c`
- `imu.c`
- `telemetry.c`
- `sd_logger.c`
- `dashboard.c`
- `shiftlight.c`
- `app_callbacks.c`
- `led_diag.c`

See [Firmware README](README.md) for the full structure.
