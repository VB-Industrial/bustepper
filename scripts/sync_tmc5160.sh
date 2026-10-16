#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source_repo=${1:-/home/robot/projects/STM32_TMC5160_SPI}
source_header="$source_repo/Core/Inc/tmc5160.h"
[[ -f "$source_header" ]] || { echo "Missing header: $source_header" >&2; exit 1; }
cp "$source_header" Core/Inc/tmc5160.h
{
    echo "Source: $source_header"
    echo "Commit: $(git -C "$source_repo" rev-parse HEAD)"
    echo "SHA256: $(sha256sum Core/Inc/tmc5160.h | cut -d ' ' -f 1)"
    echo "Source working tree (header and configuration):"
    git -C "$source_repo" status --short -- Core/Inc/tmc5160.h Core/Inc/robot_config.h
    echo "Copied verbatim. robot_config.h remains owned by the bench."
} > TMC5160_SOURCE.txt
./scripts/test.sh
