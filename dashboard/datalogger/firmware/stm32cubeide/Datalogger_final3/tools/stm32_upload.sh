#!/bin/zsh
set -euo pipefail

project_dir="${0:A:h:h}"
ide_plugins="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins"
programmers=($ide_plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_*/tools/bin/STM32_Programmer_CLI(N))
firmware="$project_dir/Debug/Datalogger_final3.elf"

if (( ${#programmers} == 0 )); then
  print -u2 "STM32CubeProgrammer CLI를 찾을 수 없습니다."
  exit 1
fi

if [[ ! -f "$firmware" ]]; then
  print -u2 "빌드 결과가 없습니다: $firmware"
  print -u2 "먼저 VS Code의 'STM32: Build' 작업을 실행하세요."
  exit 1
fi

"${programmers[-1]}" -c port=SWD -w "$firmware" -v -rst
