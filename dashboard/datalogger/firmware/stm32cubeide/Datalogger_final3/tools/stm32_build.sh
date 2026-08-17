#!/bin/zsh
set -euo pipefail

project_dir="${0:A:h:h}"
ide_plugins="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins"
toolchain_bins=($ide_plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*.macos64_*/tools/bin(N))

if (( ${#toolchain_bins} == 0 )); then
  print -u2 "STM32CubeIDE GNU toolchain을 찾을 수 없습니다."
  exit 1
fi

export PATH="${toolchain_bins[-1]}:$PATH"
make -C "$project_dir/Debug" all -j4

if [[ ! -f "$project_dir/Debug/Datalogger_final3.elf" ]]; then
  print -u2 "빌드 결과가 없습니다: Debug/Datalogger_final3.elf"
  exit 1
fi
print "업로드용 펌웨어 생성: Debug/Datalogger_final3.elf"
