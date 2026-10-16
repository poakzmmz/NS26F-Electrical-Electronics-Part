# Datalogger Firmware Refactor Plan

> Preparation only. This document defines the target structure and cleanup rules before modifying the working firmware.

## Current State

- Target: `STM32F405RGT6`
- Main application file: `Core/Src/main.c`
- Current size: approximately 4,700 lines
- Application functions: approximately 80+
- Major responsibilities currently mixed in `main.c`:
  - CAN reception / signal extraction
  - ADC DMA acquisition
  - Wheel-speed input capture
  - GNSS UART reception / parsing
  - IMU UART DMA / parsing
  - SD/FATFS logging and recovery
  - Telemetry snapshot aggregation
  - Nextion dashboard output
  - WS2812B shift-light output
  - LED diagnostics
  - HAL callbacks

The refactor goal is not to redesign behavior. The first pass should preserve runtime behavior while separating responsibilities and removing legacy/debug residue.

## Target Structure

```text
Core/
├── Inc/
│   ├── app.h
│   ├── can_bus.h
│   ├── adc_input.h
│   ├── wheel_speed.h
│   ├── gnss.h
│   ├── imu.h
│   ├── telemetry.h
│   ├── sd_logger.h
│   ├── nextion.h
│   ├── shiftlight.h
│   └── led_diag.h
└── Src/
    ├── main.c
    ├── app.c
    ├── can_bus.c
    ├── adc_input.c
    ├── wheel_speed.c
    ├── gnss.c
    ├── imu.c
    ├── telemetry.c
    ├── sd_logger.c
    ├── nextion.c
    ├── shiftlight.c
    └── led_diag.c
```

Names may be shortened later (for example `adc.c` instead of `adc_input.c`) after collision checks with STM32Cube-generated files.

## Responsibility Split

### `main.c`

Keep only:

- HAL/CubeMX initialization
- `SystemClock_Config()`
- generated `MX_*_Init()` functions
- `App_Init()`
- `App_Process()`
- CubeMX-required USER CODE markers

Target shape:

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SDIO_SD_Init();
    MX_FATFS_Init();
    MX_CAN1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();
    MX_UART4_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM8_Init();

    App_Init();

    while (1)
    {
        App_Process();
    }
}
```

### `app.c`

Own top-level application sequencing only.

Proposed responsibilities:

- runtime CAN reconfiguration to 500 kbps
- module initialization order
- main-loop scheduling
- high-level state / fatal startup handling

No sensor parsing or FATFS implementation should remain here.

### `can_bus.c`

Move:

- `USER_CAN1_Apply500Kbps()`
- CAN hardware start/configuration
- RX queue
- FIFO draining
- frame decoding / latest-signal state
- CAN health/error counters

HAL callback handling should call module entry points rather than contain CAN logic directly.

Example:

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CanBus_OnRxPending(hcan);
}
```

### `adc_input.c`

Move:

- ADC DMA startup/configuration
- ADC queue / latest samples
- ADC conversion callback processing
- channel mapping

Keep conversion units and channel meaning explicit in the header.

### `wheel_speed.c`

Move:

- TIM input-capture processing
- per-wheel timestamps
- speed calculation
- filtering
- pulse counters
- wheel queue

Do not keep legacy single-wheel and four-wheel paths together unless both are still used.

### `gnss.c`

Move:

- UART receive queue
- NMEA line assembly
- parser
- GNSS latest-state structure
- UART recovery specific to GNSS

### `imu.c`

Move:

- USART6 DMA buffer handling
- packet framing/checksum
- IMU parsing
- sample history / timestamp alignment
- IMU-specific error counters

### `telemetry.c`

Move:

- `Telemetry_Snapshot_t`
- snapshot scheduling
- snapshot queue
- cross-source timestamp selection
- aggregation of CAN / ADC / wheel / IMU into a common row

This module should not write files.

### `sd_logger.c`

Move:

- file open/indexing
- FATFS buffer
- flush/sync
- retry logic
- SD fault state
- write diagnostics
- card detect / card wait if application-specific
- CSV serialization only if it is tightly coupled to storage

Consider splitting CSV formatting from SD transport later only if the module remains too large.

### `nextion.c`

Move:

- UART4 HMI transmission
- dirty-field scheduling
- visibility control
- gear/text/number update
- UART4 recovery

### `shiftlight.c`

Move:

- RPM output handling
- WS2812 timing
- PWM DMA buffer
- LED pattern generation

### `led_diag.c`

Move:

- user status LEDs
- blink codes
- fatal startup indication

## HAL Callback Rule

STM32 HAL allows one callback symbol for each callback type. To avoid ownership conflicts, keep the public HAL callbacks in one place and dispatch them to modules.

Suggested location: `app_callbacks.c` or `main.c`.

Example:

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Gnss_OnUartError(huart);
    Imu_OnUartError(huart);
    Nextion_OnUartError(huart);
}
```

Do not define the same HAL callback independently in multiple modules.

## Legacy Removal Candidates

The refactor pass should explicitly verify and remove unused paths rather than merely move them.

Current source contains indicators of historical/bring-up code:

- `MAYBE_UNUSED`
- `SD_BRINGUP_*`
- `USB_MSC_ENABLE`
- legacy wheel logger paths
- old USB MSC bring-up logic/comments
- test-mode constants
- diagnostic counters that may no longer be consumed
- duplicated current-vs-legacy queue structures
- old single-wheel and current four-wheel logic living together

Each candidate should be classified as:

1. Used in current runtime
2. Debug-only but intentionally retained
3. Dead/legacy -> delete

No behavior-affecting code should be removed solely because its name contains `test`, `old`, or `legacy`.

## Comment Cleanup Policy

The current file contains many historical and explanatory comments. Cleanup should be aggressive, but not blind.

### Keep

- hardware constraints not obvious from code
- timing/unit assumptions
- concurrency / ISR safety rationale
- protocol packet layout
- non-obvious workaround rationale
- CubeMX `USER CODE BEGIN/END` markers while CubeMX regeneration is still used

### Delete or shorten

- comments that merely restate the next line
- historical narratives after the design has stabilized
- obsolete bring-up notes
- commented-out legacy behavior
- generated boilerplate that adds no value, except markers required by CubeMX
- repeated comments describing the same design choice
- stale comments that disagree with runtime behavior

Important current mismatches that should be resolved in code/config, not only comments:

- CubeMX CAN calculation vs runtime 500 kbps override
- 4-bit SDIO project configuration vs current 1-bit runtime path
- retained USB MSC project source vs runtime-disabled USB MSC

## Refactor Order

Behavior-preserving sequence:

1. Build current `main` and record baseline warnings/errors.
2. Tag or branch the current known-working state.
3. Extract pure utility/data modules first:
   - LED diagnostics
   - shift light
   - GNSS parser
   - IMU parser
4. Extract input modules:
   - ADC
   - wheel speed
   - CAN
5. Extract telemetry aggregation.
6. Extract SD logger last because it is the most stateful/high-risk module.
7. Reduce `main.c` to init + application scheduler.
8. Remove dead legacy code only after successful build.
9. Run bench regression.
10. Run vehicle regression before treating the refactor as production-ready.

## Regression Checklist

After each extraction:

- project compiles with zero new warnings
- CAN remains 500 kbps
- CAN RX counters increment
- ADC DMA updates expected channels
- wheel capture updates all active channels
- GNSS continues receiving after UART error
- IMU DMA parsing remains stable
- SD creates and flushes log file correctly
- no queue overflow regression
- Nextion updates without starvation
- Shift Light DMA completes
- fault LED codes still work

Before final merge:

- bench logging session
- power-cycle test
- SD remove/reinsert behavior as applicable
- CAN disconnect/reconnect test
- GNSS/IMU disconnect test
- vehicle integration test

## Git History Cleanup

Current public history is already small, but the initial public release is a large import commit.

Before rewriting history, first verify:

- no secrets or personal information in old commits
- no external links depend on existing commit SHAs
- no collaborators have branches based on the existing history
- no release/tag must preserve the old ancestry

A preliminary scan found no obvious committed email addresses, API-key assignments, or private keys in the current five-commit history. The initial commit contains `.env` only as ignore-rule mentions.

If a portfolio-clean history is still desired, use a local orphan-branch rewrite after the firmware refactor is complete. Do not rewrite history before the code structure stabilizes, or the history will need to be rewritten twice.

Recommended final state:

- one clean initial/import commit containing the final public repository snapshot
- subsequent meaningful engineering commits, e.g.
  - `refactor: split datalogger firmware into modules`
  - `cleanup: remove legacy logger paths and stale comments`
  - `docs: add hardware validation evidence`

History rewriting is intentionally NOT performed in this preparation branch.

## Scope of This Preparation

No runtime firmware code should be changed in this preparation step.

The next execution step should begin by building a baseline and extracting one low-risk module at a time.
