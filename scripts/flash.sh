#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Explicit invocation only. Writes the application directly at 0x08000000.
firmware=${1:-build/bustepper.elf}
if [[ ! -f "$firmware" ]]; then
    echo "Build firmware first: cmake --build build" >&2
    exit 1
fi
adapter_args=()
if [[ -n ${STLINK_USB_LOCATION:-} ]]; then
    adapter_args=(-c "adapter usb location $STLINK_USB_LOCATION")
fi
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
    "${adapter_args[@]}" -c 'adapter speed 1000' \
    -c "program {$firmware} verify reset exit"
